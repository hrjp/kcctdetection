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

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>

ros::Publisher cloud_filtered_pub;
ros::Publisher pose_array_pub;
ros::Publisher marker_array_pub;

float height_min = -0.8f;
float height_max = 2.0f;

const int area_max = 5;
int areas[100];

int cluster_size_min = 3;
int cluster_size_max = 22000;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    // Conversions
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc2, *pcl_pc2);

    // Z-Axis Clipping
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> pt;
    pt.setInputCloud(pcl_pc2);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(height_min, height_max);
    pt.filter(*pc_indices);

    // Divide PCL
    boost::array<std::vector<int>, area_max> indices_array;
    for(int i = 0; i < pc_indices->size(); i++){
        
        float range = 0.0f;
        
        for(int j = 0; j < area_max; j++){
            const float x_2 = pcl_pc2->points[(*pc_indices)[i]].x*pcl_pc2->points[(*pc_indices)[i]].x;
            const float y_2 = pcl_pc2->points[(*pc_indices)[i]].y*pcl_pc2->points[(*pc_indices)[i]].y;
            const float z_2 = pcl_pc2->points[(*pc_indices)[i]].z*pcl_pc2->points[(*pc_indices)[i]].z;
            const float xyz_2 = x_2 + y_2 + z_2;
            if(xyz_2 > range*range && xyz_2 <= (range+areas[j])*(range+areas[j])){
                indices_array[j].push_back((*pc_indices)[i]);
                break;
            }
            range+=areas[j];
        }
    }

    // Clustering
    float tolerance = 0.0f;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ>::Ptr > > clusters;

    for(int i = 0; i < area_max; i++){
        tolerance += 0.2;
        if(indices_array[i].size() > cluster_size_min){
            boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(pcl_pc2, indices_array_ptr);
      
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(tolerance);
            ec.setMinClusterSize(cluster_size_min);
            ec.setMaxClusterSize(cluster_size_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcl_pc2);
            ec.setIndices(indices_array_ptr);
            ec.extract(cluster_indices);
      
            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
      	        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      	        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                    pcl_pc2->points[*pit].z = 0.0f;
      	            cluster->points.push_back(pcl_pc2->points[*pit]);
  	            }
      	        cluster->width = cluster->size();
      	        cluster->height = 1;
      	        cluster->is_dense = true;
	            clusters.push_back(cluster);
            } // for
        } // if
    } // for

    // Output
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc2_output(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ros_pc2_output;
    pcl::copyPointCloud(*pcl_pc2, *pc_indices, *pcl_pc2_output);
    pcl::toROSMsg(*pcl_pc2_output, ros_pc2_output);
    cloud_filtered_pub.publish(ros_pc2_output);

    geometry_msgs::PoseArray pose_array;
    visualization_msgs::MarkerArray marker_array;

    for(int i = 0; i < clusters.size(); i++){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);

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

        visualization_msgs::Marker marker;
        marker.header = pc2->header;
        marker.ns = "clustering_and_tracking";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;

        marker.pose.position.x = bboxTransform.x();
        marker.pose.position.y = bboxTransform.y();
        marker.pose.position.z = bboxTransform.z();

        marker.pose.orientation.x = bboxQuaternion.x();
        marker.pose.orientation.y = bboxQuaternion.y();
        marker.pose.orientation.z = bboxQuaternion.z();
        marker.pose.orientation.w = bboxQuaternion.w();

        marker.scale.x = 2;//maxPoint.x - minPoint.x;
        marker.scale.y = maxPoint.y - minPoint.y;
        marker.scale.z = maxPoint.z - minPoint.z;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(marker);
    } // for i

    if(pose_array.poses.size()) {
        pose_array.header = pc2->header;
        pose_array_pub.publish(pose_array);
    } // if
  
    if(marker_array.markers.size()) {
        marker_array_pub.publish(marker_array);
    } // if

} // pointCloudCallback

int main(int argc, char **argv){
    
    ros::init(argc, argv, "clustering_and_tracking");
    
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_diff", 1, pointCloudCallback);
    
    // Publishers
    cloud_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 100);
    pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array", 100);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("markers_array", 100);


    areas[0] = 4; areas[1] = 5; areas[2] = 4; areas[3] = 5; areas[4] = 4;
    areas[5] = 5; areas[6] = 5; areas[7] = 4; areas[8] = 5; areas[9] = 4;
    areas[10]= 5; areas[11]= 5; areas[12]= 4; areas[13]= 5;

    ros::spin();

    return 0;
} // main
