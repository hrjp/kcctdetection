/**
* @file pcl_clustering.cpp
* @brief
* @author Akiro Harada
* @date
* @details PCLクラスタリング+トラッキング
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>

// ROS<->PCL
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <pcl/people/ground_based_people_detection_app.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*cloud_filtered); 

    return cloud_filtered;
} // pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud()

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*cloud);    

    return cloud;
} // pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud()

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
} // pcl::PointCloud<pcl::PointXYZ>::Ptr remove_roof()

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
        /* Failure*/
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract_seg;
	extract_seg.setInputCloud(cloud);
	extract_seg.setIndices(inliers_seg);
    
	extract_seg.setNegative(false);
	extract_seg.filter(*road_cloud);


    extract_seg.setNegative(true);
    extract_seg.filter(*obstacle_cloud);

    return std::make_tuple(road_cloud, obstacle_cloud);

} // std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_clouds()

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(0.8);
	ec.setMinClusterSize(25);
	ec.setMaxClusterSize(200);
	ec.setSearchMethod(kd_tree);
	ec.setInputCloud(cloud); 

    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		
	//Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.001);
	normalEstimation.setSearchMethod(kd_tree);
	normalEstimation.compute(*normals);

	// Region growing clustering object.
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> clustering;
	clustering.setMinClusterSize(20);
	clustering.setMaxClusterSize(1000);
	clustering.setSearchMethod(kd_tree);
	clustering.setNumberOfNeighbours(10);
	clustering.setInputCloud(cloud);
	clustering.setInputNormals(normals);
	// Set the angle in radians that will be the smoothness threshold
	// (the maximum allowable deviation of the normals).
	clustering.setSmoothnessThreshold(4.0 / 180.0 * M_PI); // 4 degrees.
	// Set the curvature threshold. The disparity between curvatures will be
	// tested after the normal deviation check has passed.
	clustering.setCurvatureThreshold(1.0);
	clustering.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            //cloud->points[*pit].z = 0.0f;
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    return clusters;
} // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_clusters()

nav_msgs::Odometry odom;
nav_msgs::Odometry odom_old;
void odom_callback(const nav_msgs::Odometry& odom_message)
{
    odom = odom_message;
}

ros::Publisher pc_road_pub;
ros::Publisher pc_obstacle_pub;
ros::Publisher marker_array_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_old (new pcl::PointCloud<pcl::PointXYZ>);

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& ros_pc2){
    
    // ROS->PCL Conversion
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc2, *pc_input);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //pc_filtered = downsample_cloud(pc_input, 0.1f);
    pc_filtered = pc_input;
    pc_filtered = crop_cloud(pc_filtered, Eigen::Vector4f(-10, -10, -1, 1), Eigen::Vector4f(10, 10, 1.5, 1));
    pc_filtered = remove_roof(pc_filtered, Eigen::Vector4f(-1, 0, -1, 1), Eigen::Vector4f(1, 1, 1, 1));

    std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_segmented;
    pc_segmented = segment_clouds(pc_filtered, pcl::SACMODEL_PLANE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_road = std::get<0>(pc_segmented);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_obstacle = std::get<1>(pc_segmented);

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (.8f);

    octree.setInputCloud (pc_obstacle);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers();
    octree.setInputCloud (pc_obstacle);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels (newPointIdxVector);
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_obstacle_diff (new pcl::PointCloud<pcl::PointXYZ> );

    pc_obstacle_diff->width = pc_old->points.size() + pc_obstacle->points.size();
    pc_obstacle_diff->height = 1;
    pc_obstacle_diff->points.resize (pc_obstacle_diff->width * pc_obstacle_diff->height);

    int n = 0;
    for(size_t i = 0; i < newPointIdxVector.size (); i++)
    {
        pc_obstacle_diff->points[i].x = pc_obstacle->points[newPointIdxVector[i]].x;
        pc_obstacle_diff->points[i].y = pc_obstacle->points[newPointIdxVector[i]].y;
        pc_obstacle_diff->points[i].z = pc_obstacle->points[newPointIdxVector[i]].z;
        n++;
    }

    pc_obstacle_diff->width = n;
    pc_obstacle_diff->height = 1;
    pc_obstacle_diff->points.resize (pc_obstacle_diff->width * pc_obstacle_diff->height);

    pcl::copyPointCloud(*pc_filtered, *pc_old);

    sensor_msgs::PointCloud2 ros_pc2_road, ros_pc2_obstacle_diff;
    pcl::toROSMsg(*pc_road, ros_pc2_road);
    pcl::toROSMsg(*pc_obstacle_diff, ros_pc2_obstacle_diff);
    
    ros_pc2_road.header = ros_pc2_obstacle_diff.header = ros_pc2->header;

    pc_road_pub.publish(ros_pc2_road);
    pc_obstacle_pub.publish(ros_pc2_obstacle_diff);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = get_clusters(pc_old);

    geometry_msgs::PoseArray pose_array;
    visualization_msgs::MarkerArray marker_array;

    for(int i = 0; i < clusters.size(); i++){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);

        /*
        geometry_msgs::Pose pose;
        pose.position.x = centroid[0];
        pose.position.y = centroid[1];
        pose.position.z = centroid[2];
        pose.orientation.w = 1;
        pose_array.poses.push_back(pose);

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*clusters[i], pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*clusters[i], pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(clusters[i]);
        pca.project(*clusters[i], *cloudPCAprojection);

        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*clusters[i], *cloudPointsProjected, projectionTransform);

        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
        */

        visualization_msgs::Marker marker;
        marker.header = ros_pc2->header;
        marker.ns = "clustering_and_tracking";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;

        marker.pose.position.x = centroid[0];//bboxTransform.x();
        marker.pose.position.y = centroid[1];//bboxTransform.y();
        marker.pose.position.z = centroid[2];//bboxTransform.z();

        marker.pose.orientation.x = 0;//bboxQuaternion.x();
        marker.pose.orientation.y = 0;//bboxQuaternion.y();
        marker.pose.orientation.z = 0;//bboxQuaternion.z();
        marker.pose.orientation.w = 1;//bboxQuaternion.w();

        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*clusters[i], minPoint, maxPoint);

        marker.scale.x = maxPoint.x - minPoint.x;
        marker.scale.y = maxPoint.y - minPoint.y;
        marker.scale.z = maxPoint.z - minPoint.z;

        marker.color.a = 0.9;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(0.2);
        //float obstacleSize = marker.scale.x*marker.scale.y*marker.scale.z; 
        //if(fabs(marker.scale.x - 0.6) < 0.3 && fabs(marker.scale.y - 0.6) < 0.3 && fabs(marker.scale.z - 1.7) < 0.3)
        //if(marker.scale.x < 1.8 && marker.scale.y  < 1.8 && (marker.scale.x > 0.5 || marker.scale.y > 0.5) && marker.scale.z < 2.0 && minPoint.z <0.3)
            marker_array.markers.push_back(marker);
    } // for i
    marker_array_pub.publish(marker_array);

} // void pointCloudCallback()

int main(int argc, char **argv){

    ros::init(argc, argv, "clustering_and_tracking");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber pc_input_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pointCloudCallback);
    ros::Subscriber odom_sub = nh.subscribe("/hdl_localization/odom", 10, odom_callback);

    // Publishers
    pc_road_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_road", 100);
    pc_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_obstacle", 100);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("markers_obstacle", 100);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/share/pcd/PointCloudMap2022-4-5-6-10.pcd", *pc_map) == -1){
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    ros::spin();
    return 0;
} // int main()