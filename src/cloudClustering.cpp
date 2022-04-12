/**
* @file clusterTracker.cpp
* @brief
* @author Akiro Harada
* @date
* @details 点群のクラスタリングと追跡
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

// ROS<->PCL
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

ros::Publisher clusterMarker_pub;

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

void callback(const sensor_msgs::PointCloud2ConstPtr& ros_pc_input){
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_input, *pc_input);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = get_clusters(pc_input);

    geometry_msgs::PoseArray pose_array;
    visualization_msgs::MarkerArray marker_array;

    for(int i = 0; i < clusters.size(); i++){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);

        visualization_msgs::Marker marker;
        marker.header = ros_pc_input->header;
        marker.ns = "";
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

        marker.scale.x = 0.3;//maxPoint.x - minPoint.x;
        marker.scale.y = 0.3;//maxPoint.y - minPoint.y;
        marker.scale.z = 0.3;//maxPoint.z - minPoint.z;

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
    clusterMarker_pub.publish(marker_array);
    return;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "cloudClustering");
    
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber pc_input_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud/dynamic", 1, callback);

    // Publishers
    clusterMarker_pub = nh.advertise<visualization_msgs::MarkerArray>( "/marker/clusterMarker", 0 );

    ros::spin();

    return 0;
} // main()
