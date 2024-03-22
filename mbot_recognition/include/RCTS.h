#ifndef RCTS_H
#define RCTS_H
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <time.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <random>
#include <Eigen/Dense>
#include <algorithm>
#include "Simclass.h"

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
#include <flann/flann.hpp>

typedef struct Belief{
  Eigen::MatrixXf particles;
  Eigen::RowVectorXf weights;
}Belief;

class Robot;

//rbt class
class Node{
public:
  // member
  int num; // 标号
  Eigen::Vector3f state; // robot state
  Belief B; // belief state
  std::vector<std::vector<float>> hist; // history
  std::vector<std::vector<float>> a; // 未选择的动作
  int a_num; // originated from which action
  int N; // The number of times node has been visited
  double R; // the immediate reward
  double Q; // the sum reward
  double r; // the reward for the rollout
  int parent; // the index of the parent node
  std::vector<int> children; // the children node

};
// functions
  
float simulate(Robot&, Sim, int, int&, int, double, std::vector<std::vector<float>>, Eigen::MatrixXf, Eigen::RowVectorXf);

void backup(Robot&, int, double, double);

int best_child(Robot&, int, float);

int expand_spec(Robot&, int, int, Eigen::Vector3f, std::vector<float>, std::vector<std::vector<float>>);

int expand(Robot&, int, int, std::vector<float>, int, std::vector<std::vector<float>>);

float rollOut(Robot, Sim, Node&, double, int, Eigen::MatrixXf, Eigen::RowVectorXf);

std::vector<float> EPFT_planner(Robot&, Sim);

std::vector<std::vector<float>> vec2mat(std::vector<float>, int);

//gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &,std::vector<float>);

#endif