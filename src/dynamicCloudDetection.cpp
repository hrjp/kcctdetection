/**
* @file dynamicCloudDetector.cpp
* @brief
* @author Akiro Harada
* @date
* @details 動的な点群の検出
*/

// ROS
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/utils.h>

// ROS <-> PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <omp.h>

ros::Publisher grid_pub;
ros::Publisher dynamic_pub;
ros::Publisher static_pub;

std::string map_frameId = "map";
std::string base_link_frameId = "base_link";

double resolution = 0.5;
double width = 20.0;
double occupancyThreshold = 0.5;
double beam_num = 720;
double logOdds_inc = 0.15;
double logOdds_dec = 0.4;

int grid_width = width/resolution;
int grid_num = grid_width*grid_width;
double width_2 = width/2;
int grid_width_2 = grid_width/2;

class GridCell{
    public:
        GridCell(void){
            logOdds = 0;
        }

        double get_occupancy(void){
            return 1.0/(1 + exp(-logOdds));
        }

        double get_logOdds(void){
            return logOdds;
        }

        void add_logOdds(double lo){
            logOdds += lo;
            //if(logOdds>10.0)
            //    logOdds = 10.0;
            //else if(logOdds<-10.0)
            //    logOdds = 10.0;
        }

        double logOdds;
    private:
}; // class GridCell

std::vector<GridCell> occupancyGridMap;

std::string remove_firstSlash(std::string frame_id){
    const int slash_pos = frame_id.find('/');
    if(slash_pos == 0)
        frame_id.erase(0, 1);
    return frame_id;
} // remove_firstSlash()

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*cloud_filtered); 

    return cloud_filtered;
} // downsample_cloud()

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*cloud);    

    return cloud;
} // crop_cloud()

pcl::PointCloud<pcl::PointXYZ>::Ptr remove_roof(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    std::vector<int> indices;

    pcl::CropBox<pcl::PointXYZ> roof(true);

    roof.setMin(minPoint);
    roof.setMax(maxPoint);
    roof.setInputCloud(cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices){
    	inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    return cloud;
} // remove_roof()

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::SacModel sm){

    pcl::PointCloud<pcl::PointXYZ>::Ptr road_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointIndices::Ptr inliers_seg{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<pcl::PointXYZ> segmenter;

    segmenter.setOptimizeCoefficients(true);
    segmenter.setModelType(sm);
    segmenter.setMethodType(pcl::SAC_RANSAC);

    segmenter.setDistanceThreshold(0.1);
    // segmenter.setMaxIterations(100);

    segmenter.setInputCloud(cloud);
    segmenter.segment (*inliers_seg, *coefficients);

	if (inliers_seg->indices.size () == 0)
	{
        // Failure
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract_seg;
	extract_seg.setInputCloud(cloud);
	extract_seg.setIndices(inliers_seg);
    
	extract_seg.setNegative(false);
	extract_seg.filter(*road_cloud);

    extract_seg.setNegative(true);
    extract_seg.filter(*obstacle_cloud);

    return std::make_tuple(road_cloud, obstacle_cloud);

} // segment_clouds()

int get_x_index_from_index(const int index){
    return index % grid_width;
} // get_x_index_from_index()

int get_y_index_from_index(const int index){
    return index / grid_width;
} // get_y_index_from_index()

int get_index_from_xy(const double x, const double y)
{
    const int _x = floor(x / resolution + 0.5) + grid_width_2;
    const int _y = floor(y / resolution + 0.5) + grid_width_2;
    return _y * grid_width + _x;
} // get_index_fromxy()

bool is_validPoint(double x, double y)
{
    const int index = get_index_from_xy(x, y);
    if(x < -width_2 || x > width_2 || y < -width_2 || y > width_2){
        return false;
    }else if(index < 0 || grid_num <= index){
        return false;
    }else{
        return true;
    }
} // is_validPoint()

void transform_occupancyGridMap(const Eigen::Vector2d& translation, double diff_odom_yaw, std::vector<GridCell>& map){
    
    const double dx = translation(0);
    const double dy = translation(1);
    const double c_yaw = cos(diff_odom_yaw);
    const double s_yaw = sin(diff_odom_yaw);

    const double dx_grid = dx / resolution;
    const double dy_grid = dy / resolution;

    Eigen::Matrix3d affine;
    affine <<   c_yaw, -s_yaw,  dx_grid,
                s_yaw,  c_yaw,  dy_grid,
                0,          0,        1;
    
    const Eigen::Matrix3d affine_inv = affine.inverse();

    std::vector<GridCell> ogm(grid_num); // Occupancy Grid Map

    for(int i = 0; i < grid_num; i++){
        
        const double x_i = get_x_index_from_index(i) - grid_width_2;
        const double y_i = get_y_index_from_index(i) - grid_width_2;
        
        Eigen::Vector3d ogm_i(x_i, y_i, 1);
        Eigen::Vector3d map_i = affine_inv * ogm_i;
        
        const int x_0 = std::floor(map_i(0));
        const int x_1 = x_0 + 1;
        const int y_0 = std::floor(map_i(1));
        const int y_1 = y_0 + 1;

        if(x_0 < -grid_width_2 || grid_width_2 <= x_1)
            continue;
        if(y_0 < -grid_width_2 || grid_width_2 <= y_1)
            continue;
        
        const int index_0_0 = (y_0 + grid_width_2) * grid_width + x_0 + grid_width_2;
        const int index_0_1 = (y_1 + grid_width_2) * grid_width + x_0 + grid_width_2;
        const int index_1_0 = (y_0 + grid_width_2) * grid_width + x_1 + grid_width_2;
        const int index_1_1 = (y_1 + grid_width_2) * grid_width + x_1 + grid_width_2;

        const Eigen::Vector2d y_vec(y_1 - map_i(1), map_i(1) - y_0);
        const Eigen::Vector2d x_vec(x_1 - map_i(0), map_i(0) - x_0);
        Eigen::Matrix2d value_mat;
        value_mat << map[index_0_0].get_occupancy(), map[index_1_0].get_occupancy(),
                     map[index_0_1].get_occupancy(), map[index_1_1].get_occupancy();

        const double ogm_value = y_vec.transpose() * value_mat * x_vec;
        ogm[i].logOdds = std::log(ogm_value / (1 - ogm_value));
    } // for i
    map.clear();
    map = ogm;
} // transform_occupancyGridMap()

void set_clear_GridCells(const std::vector<double>& beam_list, const std::vector<bool>& obstacle_indices, std::vector<GridCell>& map){
    std::vector<bool> clear_indices(grid_num, false);
    const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num;

    for(int i=0;i<beam_num;i++){
        double direction = i * beam_angle_resolution - M_PI;
        direction = atan2(sin(direction), cos(direction));
        const double c = cos(direction);
        const double s = sin(direction);

        for(double range=0.0;range<beam_list[i];range+=resolution){
            const double x = range * c;
            const double y = range * s;
            if(is_validPoint(x, y)){
                const int index = get_index_from_xy(x, y);
                if(!obstacle_indices[index]){
                    clear_indices[index] = true;
                }else{
                    break;
                }
            } // if
            else
            {
                break;
            } // else
        } // for rage
    } // for i
    for(int i = 0; i < grid_num; ++i){
        if(clear_indices[i]){
            map[i].add_logOdds(-logOdds_dec);
        }
    } // for i
} // set_clear_GridCells()


void input_cloud_to_occupancyGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr){
    std::vector<double>beam_list(beam_num, sqrt(2) * width_2);
    const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num;

    // occupancyGridMap.clear();
    // occupancyGridMp.resize(grid_num);
    const int cloud_size = cloud_ptr->points.size();
    std::vector<bool> obstacle_indices(grid_num, false);

    #pragma omp parallel for
    for(int i=0;i<cloud_size;i++){
        const auto& p = cloud_ptr->points[i];
        if(!is_validPoint(p.x, p.y)){
            continue;
        } // if
        // occupancyGridmap[get_index_from_xy(p.x, p.y)].add_logOdds(0.01);
        const double distance = sqrt(p.x * p.x + p.y * p.y);
        const double direction = atan2(p.y, p.x);
        const int beam_index = (direction + M_PI) / beam_angle_resolution;
        if(0 <= beam_index && beam_index < beam_num){
            beam_list[beam_index] = std::min(beam_list[beam_index], distance);
        } // if
        const int index = get_index_from_xy(p.x, p.y);
        if(index < 0 || grid_num <= index){
            continue;
        } // if
        obstacle_indices[get_index_from_xy(p.x, p.y)] = true;
    } // for i

    for(int i=0;i<grid_num;i++){
        if(obstacle_indices[i]){
            occupancyGridMap[i].add_logOdds(logOdds_inc);
        } // if
    } // for i

    set_clear_GridCells(beam_list, obstacle_indices, occupancyGridMap);
} // input_cloud_to_occupancyGridMap()

void publish_occupancyGridMap(const ros::Time& stamp, const std::string& frame_id)
{
    nav_msgs::OccupancyGrid og;
    og.header.stamp = stamp;
    og.header.frame_id = frame_id;
    og.info.resolution = resolution;
    og.info.width = grid_width;
    og.info.height = grid_width;
    og.info.origin.position.x = -width_2;
    og.info.origin.position.y = -width_2;
    og.info.origin.orientation.w = 1.0;
    og.data.resize(grid_num);
    for(int i=0;i<grid_num;i++){
        og.data[i] = occupancyGridMap[i].get_occupancy() * 100;
    } // for i
    og.header.stamp = ros::Time::now();
    grid_pub.publish(og);
} // public_OccupancyGridMap()

void divide_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& static_cloud)
{
    dynamic_cloud->points.clear();
    static_cloud->points.clear();

    for(const auto& pt : cloud->points){
        if(-width_2 <= pt.x && pt.x <= width_2 && -width_2 <= pt.y && pt.y <= width_2){
            const int index = get_index_from_xy(pt.x, pt.y);
            if(0 <= index && index < grid_num){
                const double occupancy = occupancyGridMap[index].get_occupancy();
                if(occupancy < occupancyThreshold){
                    dynamic_cloud->points.push_back(pt);
                } // if
                else
                {
                    static_cloud->points.push_back(pt);
                } // else
            } //if
        } // if
    } // for pt
} // divide_cloud()

geometry_msgs::TransformStamped transform;

void callback(const sensor_msgs::PointCloud2ConstPtr& ros_pc_input, const nav_msgs::OdometryConstPtr& odom_input){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_input, *pc_input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    pc_filtered = downsample_cloud(pc_input, 0.1f);
    pc_filtered = crop_cloud(pc_filtered, Eigen::Vector4f(-10, -10, -1, 1), Eigen::Vector4f(10, 10, 1.2, 1));
    pc_filtered = remove_roof(pc_filtered, Eigen::Vector4f(-1, -1, -1, 1), Eigen::Vector4f(1, 1, 1, 1));

    std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_segmented;
    pc_segmented = segment_clouds(pc_filtered, pcl::SACMODEL_PLANE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_road = std::get<0>(pc_segmented);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_obstacle = std::get<1>(pc_segmented);

    pc_obstacle = crop_cloud(pc_obstacle, Eigen::Vector4f(-10, -10, -0.6, 1), Eigen::Vector4f(10, 10, 1.2, 1));
    
    static Eigen::Vector2d last_odom_pos(odom_input->pose.pose.position.x, odom_input->pose.pose.position.y);
    static double last_odom_yaw = tf2::getYaw(odom_input->pose.pose.orientation);
    /*
    //const std::string sensor_frame_id = ros_pc_input.header.frame_id;//remove_firstSlash(pc_obstacle->header.frame_id);
    //const std::string base_frame_id = odom_input->header.frame_id;//remove_firstSlash(odom_input->header.frame_id);

    try{
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer.lookupTransform(base_frame_id, sensor_frame_id, ros::Time(0));
        const Eigen::Matrix4d mat = tf2::transformToEigen(transform.transform).matrix().cast<double>();
        pcl::transformPointCloud(*pc_obstacle, *pc_obstacle, mat);
        pc_obstacle->header.frame_id = base_frame_id;
    }catch(tf2::TransformException& ex){
        std::cout << ex.what() << std::endl;
        return;
    }
    */

    const Eigen::Vector2d odom_pos(odom_input->pose.pose.position.x, odom_input->pose.pose.position.y);
    const double odom_yaw = tf2::getYaw(odom_input->pose.pose.orientation);
    const Eigen::Vector2d diff_odom_pos = Eigen::Rotation2Dd(-last_odom_yaw).toRotationMatrix() * (odom_pos - last_odom_pos);
    double diff_odom_yaw = odom_yaw - last_odom_yaw;
    diff_odom_yaw = atan2(sin(diff_odom_yaw), cos(diff_odom_yaw));
    transform_occupancyGridMap(-diff_odom_pos, -diff_odom_yaw, occupancyGridMap);
    input_cloud_to_occupancyGridMap(pc_obstacle);

    publish_occupancyGridMap(odom_input->header.stamp, base_link_frameId);

    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    dynamic_cloud->header = pc_input->header;
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    static_cloud->header = pc_input->header;
    divide_cloud(pc_obstacle, dynamic_cloud, static_cloud);

    sensor_msgs::PointCloud2 ros_pc_dynamic, ros_pc_static;
    pcl::toROSMsg(*dynamic_cloud, ros_pc_dynamic);
    pcl::toROSMsg(*static_cloud, ros_pc_static);

    ros_pc_dynamic.header = ros_pc_static.header = ros_pc_input->header;

    dynamic_pub.publish(ros_pc_dynamic);
    static_pub.publish(ros_pc_static);

    last_odom_pos = odom_pos;
    last_odom_yaw = odom_yaw;
} // callback()

int main(int argc, char **argv){
    
    ros::init(argc, argv, "dynamicCloudTracker");
    
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>> sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>(10), cloud_sub, odom_sub);

    cloud_sub.subscribe(nh, "/velodyne_points", 10);
    odom_sub.subscribe(nh, "/hdl_localization/odom", 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
    dynamic_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/dynamic", 1);
    static_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/static", 1);

    occupancyGridMap.resize(grid_num);

    ros::spin();

    return 0;
} // main()
