#ifndef Simclass_H
#define Simclass_H
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <random>
#include <Eigen/Dense>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

typedef struct inParam_sim{
  std::string sensor_type;
  Eigen::Matrix2f Q_search;
  Eigen::Matrix2f Q_tracking;
}inParam_sim;


//rbt class
class Sim{
public:
  // member
  std::string sensor_type;
  Eigen::Matrix2f Q_search;
  Eigen::Matrix2f Q_tracking;

  nav_msgs::OccupancyGrid map;

  // functions
  void Initialization(inParam_sim);
};

#endif