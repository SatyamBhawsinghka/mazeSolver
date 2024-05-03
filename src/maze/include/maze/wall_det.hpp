#ifndef WALL_DET_H
#define WALL_DET_H

#pragma once
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <string>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <typeinfo>
#include <math.h>
#include <ros/package.h>

//msgs
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

//srvs
#include <maze/det_srv.h>

class WallDetect
{
private:
    typedef boost::shared_mutex SharedMutex;
    typedef boost::unique_lock< SharedMutex > WriteLock;
    typedef boost::shared_lock< SharedMutex > ReadLock;
    
private:
    ros::NodeHandle _nh;
    ros::AsyncSpinner _spinner;

    
    ros::Subscriber _laser_sub;
    ros::Subscriber _odom_sub;
    
    ros::Publisher _vel_pub;
    
    ros::ServiceServer _wall_det_service;
    
    std::mutex _det_mutex, _rot_mutex;
    SharedMutex _laser_mutex, _odom_mutex;

    geometry_msgs::Twist _vel;
    nav_msgs::OdometryConstPtr _odom_ptr;
    sensor_msgs::LaserScanConstPtr _laser_ptr;
    
    float _minLaserValues, _intervalEpsilon, _distanceToWall, _angle;
    bool _flag;
    
private:
    void laserCb(const sensor_msgs::LaserScan::ConstPtr &laser);
    void odomCb(const nav_msgs::Odometry::ConstPtr &odom);
    bool serviceCb(maze::det_srv::Request  &req,  maze::det_srv::Response &res);
    void initParams();
    bool wallDetection(float actMinLaserValue, float turnSpeed);
    void rotate_angle(float angle, float speed);       

public:
    WallDetect();
    ~WallDetect();
    
    
};

#endif
