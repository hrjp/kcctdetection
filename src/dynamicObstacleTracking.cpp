#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcUtils/groundRemoval.h"
#include "pcUtils/componentClustering.h"
#include "pcUtils/cloudSegmentation.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

ros::Publisher pc_ground_pub;
ros::Publisher pc_obstacle_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& ros_pc_input){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*ros_pc_input, *pc_input);

    PointCloud<pcl::PointXYZ>::Ptr pc_obstacle (new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud<pcl::PointXYZ>::Ptr pc_ground   (new pcl::PointCloud<pcl::PointXYZ>());

    groundRemove(*pc_input, pc_obstacle, pc_ground);
/*
    int numCluster = 0;
    array<array<int, numGrid>, numGrid> cartesianData{};
    componentClustering(pc_obstacle, cartesianData, numCluster);

    PointCloud<pcl::PointXYZRGB>::Ptr pc_clustered (new pcl::PointCloud<pcl::PointXYZRGB>);
    makeClusteredCloud(pc_obstacle, cartesianData, pc_clustered);
    pc_clustered->header.frame_id = pc_input->header.frame_id;
    sensor_msgs::PointCloud2 output;
    toROSMsg(*pc_clustered, output);
*/
    
    pc_obstacle->header = pc_ground->header = pc_input->header;
    sensor_msgs::PointCloud2 ros_pc_obstacle, ros_pc_ground;
    toROSMsg(*pc_obstacle, ros_pc_obstacle);
    toROSMsg(*pc_ground, ros_pc_ground);
    pc_obstacle_pub.publish(ros_pc_obstacle);
    pc_ground_pub.publish(ros_pc_ground);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "dynamicObstacleTracking");
    
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, callback);
    
    pc_ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/dynamicObstacleTracking/cloud/pc_ground", 1);
    pc_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/dynamicObstacleTracking/cloud/pc_obstacle", 1);
    ros::spin();

    return 0;
} // main()
