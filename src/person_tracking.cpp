/**
* @file person_tracking.cpp
* @brief path
* @author Shunya Hara
* @date 2021.3.8
* @details pathを受けっとってそれをなぞって走行する
*/


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <kcctdetection/waypoint_type.h>

using namespace std;

ros::Publisher cmd_pub;

double pcl_dis;
void pcl_callback(const std_msgs::Float32& pcl_dis_){
    pcl_dis=pcl_dis_.data;
}

std_msgs::String now_type;
void now_type_callback(const std_msgs::String& now_type_)
{
    now_type=now_type_;
}


double double_constrain(double val,double down_limit,double up_limit){
  if(val>up_limit){
    return up_limit;
  }
  if(val<down_limit){
    return down_limit;
  }
  return val;
}

geometry_msgs::Twist person_tracking(double front_angle,double vel_max){
    geometry_msgs::Twist calc_vel;
    //param
     ros::Time start_time;
     const double angle_p=0.005;
     double angle_max=0.2;
     const double front_ditect_dis=5.0;
     static bool ditect_once=false;
        calc_vel.linear.x=vel_max;
        calc_vel.angular.z=-front_angle;
        calc_vel.angular.z*=angle_p;
    calc_vel.angular.z=double_constrain(calc_vel.angular.z,-angle_max,angle_max);
    if(true/*wp_type.at(now_wp)==waypoint_type_camera*/){
      if(pcl_dis<0.9){
        calc_vel.linear.x=0;
        if(ros::Time::now().sec-start_time.sec>3){
          //ditect_once=true;
        }
      }
      else{
        start_time=ros::Time::now();
      }
      cout<<ros::Time::now().sec-start_time.sec<<endl;
      if(ditect_once){
        calc_vel.linear.x=0;
        calc_vel.angular.z=0;
      }
    }
    else{
      ditect_once=false;
    }
    //calc_vel.linear.x=0;//*=(abs(calc_vel.angular.z)<curve_stop_angle);
    return calc_vel;
}


void camera_callback(const std_msgs::Float32MultiArray& camera_){
    auto cmd_vel=person_tracking(camera_.data.at(0),2.5/3.6);
    cmd_pub.publish(cmd_vel);
}




int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle n;


    //param setting
    ros::NodeHandle pn("~");
    double looprate=10.0;
    pn.param<double>("loop_rate",looprate,100.0);
        //制御周期 Hz
    ros::Rate loop_rate(looprate);

    ros::NodeHandle lSubscriber("");

    //camera subscliber
    ros::Subscriber camera_sub = lSubscriber.subscribe("result", 50, camera_callback);
    //waypoint type
    ros::Subscriber now_type_sub = lSubscriber.subscribe("waypoint/now_type", 10, now_type_callback);
    //pcl_handler subscliber
    ros::Subscriber pcl_sub = lSubscriber.subscribe("/pcl_handler/front_dist", 50, pcl_callback);

    //cmd_vel publisher
    cmd_pub=n.advertise<geometry_msgs::Twist>("camera_cmd_vel", 1);

    ros::spin();//subsucriberの割り込み関数はこの段階で実装される
    
    return 0;
}
