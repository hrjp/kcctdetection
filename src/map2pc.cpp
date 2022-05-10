/**
* @file map2pc.cpp
* @brief
* @author Akiro Harada
* @date
* @details マップデータから現在のOdometry値周辺のPointCloudを切り出しPublishする
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <iostream>
#include <string>

#include "pcUtils/groundRemoval.h"

ros::Publisher pc_map_pub;
ros::Publisher pc_map_obstacle_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map_all (new pcl::PointCloud<pcl::PointXYZ>);

nav_msgs::Odometry odom;

pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*cloud);

    return cloud;
} // crop_cloud()

void callback(const nav_msgs::Odometry& odom)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map_obstacle (new pcl::PointCloud<pcl::PointXYZ>);

    *pc_map = *pc_map_all;
    pc_map = crop_cloud(pc_map, Eigen::Vector4f(odom.pose.pose.position.x-10, odom.pose.pose.position.y-10, odom.pose.pose.position.z-1, 1), 
                                Eigen::Vector4f(odom.pose.pose.position.x+10, odom.pose.pose.position.y+10, odom.pose.pose.position.z+2, 1));
    for(int i = 0;i < pc_map->points.size();i++){
        pc_map->points[i].x -= odom.pose.pose.position.x;
        pc_map->points[i].y -= odom.pose.pose.position.y;
        pc_map->points[i].z -= odom.pose.pose.position.z+0.8;
    }
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block(0,0,3,3) = mat3;
    pcl::transformPointCloud(*pc_map, *pc_map, mat4.inverse());

    //*pc_map_obstacle = *pc_map;
    //pc_map_obstacle = crop_cloud(pc_map_obstacle,   Eigen::Vector4f(-10, -10, 0.5, 1), 
    //                                                Eigen::Vector4f(+10, +10, 1, 1));

    groundRemove(*pc_map, pc_map_obstacle);

    sensor_msgs::PointCloud2 ros_pc_map, ros_pc_map_obstacle;
    toROSMsg(*pc_map, ros_pc_map);
    toROSMsg(*pc_map_obstacle, ros_pc_map_obstacle);
    ros_pc_map.header.frame_id = ros_pc_map_obstacle.header.frame_id = "base_link";
    ros_pc_map.header.stamp = ros_pc_map_obstacle.header.stamp = ros::Time::now();
    pc_map_pub.publish(ros_pc_map);
    pc_map_obstacle_pub.publish(ros_pc_map_obstacle);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "map2pc");
    
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/hdl_localization/odom", 10, callback);
    
    // Publishers
    pc_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map2pc/pc_map", 1);
    pc_map_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/map2pc/pc_map_obstacle", 1);

    // Map Import
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/share/pcd/y1.pcd", *pc_map_all) == -1){
        return (-1);
    }

    ros::spin();

    return 0;
} // main
