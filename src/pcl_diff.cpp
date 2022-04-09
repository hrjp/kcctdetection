/**
* @file pcl_diff.cpp
* @brief
* @author Akiro Harada
* @date
* @details PCL差分
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <nav_msgs/Odometry.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <string>

ros::Publisher cloud_diff_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr map_pcl (new pcl::PointCloud<pcl::PointXYZ>);

nav_msgs::Odometry odom;
void odom_callback(const nav_msgs::Odometry& odom_message)
{
    odom = odom_message;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc2){

    //sensor_msgs::PointCloud2 pc2_output = *pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc2, *pc2_input);

/*
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (pc2_input);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.8, 2);
    pass.filter (*pc2_input);

    pass.setInputCloud (map_pcl);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.8, 2);
    pass.filter (*map_pcl);

    pass.setInputCloud (pc2_input);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-odom.pose.pose.position.y-10.0, -odom.pose.pose.position.y+10.0);
    pass.filter (*pc2_input);
*/  

    Eigen::Matrix3f mat3 = Eigen::Quaternionf(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block(0,0,3,3) = mat3;

    pcl::transformPointCloud( *pc2_input, *pc2_input, mat4 );

    for(int i = 0;i < pc2_input->points.size();i++){
        pc2_input->points[i].x += odom.pose.pose.position.x;
        pc2_input->points[i].y += odom.pose.pose.position.y;
        pc2_input->points[i].z += odom.pose.pose.position.z + 0.8;
    }
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (1.f);
    octree.setInputCloud (map_pcl);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers();
    octree.setInputCloud (pc2_input);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels (newPointIdxVector);//比較の結果差分と判断された点郡の情報を保管
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ> );//出力先

    cloud_diff->width = map_pcl->points.size() + pc2_input->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);   

    int n = 0;
    for(size_t i = 0; i < newPointIdxVector.size (); i++)
    {
        cloud_diff->points[i].x = pc2_input->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = pc2_input->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = pc2_input->points[newPointIdxVector[i]].z;
        n++;
    }

    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_diff_ (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud_diff, *cloud_diff_);

    // Output
    sensor_msgs::PointCloud2 ros_pc2_output;
    pcl::toROSMsg(*cloud_diff_, ros_pc2_output);
    ros_pc2_output.header.frame_id = "map";
    cloud_diff_pub.publish(ros_pc2_output);
    return;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "pcl_diff");
    
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pointCloudCallback);
    ros::Subscriber odom_sub = nh.subscribe("/hdl_localization/odom", 10, odom_callback);
    
    // Publishers
    cloud_diff_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_diff", 100);

    // Map Import
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/share/pcd/autoware_world.pcd", *map_pcl) == -1){
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    ros::spin();

    return 0;
} // main
