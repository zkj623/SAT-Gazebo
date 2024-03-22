#ifndef Robotclass_H
#define Robotclass_H
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
#include "RCTS.h"

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

typedef struct hpar{
  Eigen::Vector2f first_particle;
  float first_weight;
  std::vector<std::vector<float>> third_particles;
  std::vector<float> third_weights;
}hpar;

typedef struct inParam_rbt{
  // motion specs
  std::vector<std::vector<float>> traj;
  std::vector<std::vector<float>> planned_traj;
  Eigen::Vector3f pose; //[x;y;theta]

  // sensor specs
  float r_max;
  float r_min;
  float theta_max;
  Eigen::Matrix2f R;

  sensor_msgs::LaserScan scan;

  std::vector<bool> inFOV_hist; 
  bool is_tracking;
  bool first;

  // particles
  float numParticles;
  Belief bel;
  std::vector<float> est_pose;
  std::vector<std::vector<float>> est_pose_hist;

  // first_particles
  std::vector<hpar> h_par;

  Eigen::MatrixXf loc_par;
  Eigen::RowVectorXf loc_w;
  Eigen::Vector2f vir_tar;

  // map specs
  nav_msgs::OccupancyGrid map;
  float map_size;

  // action space
  std::vector<std::vector<float>> a_S; // search
  std::vector<std::vector<float>> a_T; // tracking
  std::vector<Eigen::MatrixXf> path_S;
  std::vector<Eigen::MatrixXf> path_T;

  int a_hist;
  float max_value;

  Eigen::MatrixXf cache;

  std::vector<Node> tree;
}inParam_rbt;


//rbt class
class Robot{
public:
  // member
  // motion specs
  std::vector<std::vector<float>> traj;
  std::vector<std::vector<float>> planned_traj;
  Eigen::Vector3f pose; //[x;y;theta]

  // sensor specs
  float r_max;
  float r_min;
  float theta_max;
  Eigen::Matrix2f R;

  sensor_msgs::LaserScan scan;

  std::vector<bool> inFOV_hist; 
  bool is_tracking;
  bool first;

  // particles
  float numParticles;
  Belief bel;
  std::vector<float> est_pose;
  std::vector<std::vector<float>> est_pose_hist;

  // first_particles
  std::vector<hpar> h_par;
  
  Eigen::MatrixXf loc_par;
  Eigen::RowVectorXf loc_w;
  Eigen::Vector2f vir_tar;

  // map specs
  nav_msgs::OccupancyGrid map;
  float map_size;

  // action space
  std::vector<std::vector<float>> a_S; // search
  std::vector<std::vector<float>> a_T; // tracking
  std::vector<Eigen::MatrixXf> path_S;
  std::vector<Eigen::MatrixXf> path_T;

  int a_hist;
  float max_value;

  Eigen::MatrixXf cache;

  std::vector<Node> tree;

  // functions
  void Initialization(inParam_rbt);
};



//Norm function prototype
float Norm( std::vector<float> , std::vector<float> );

Belief particle_initialization(int);

visualization_msgs::Marker marker_initialization();

visualization_msgs::Marker hparticle_initialization();

visualization_msgs::Marker goal_particle_initialization();

visualization_msgs::Marker local_particle_initialization();

visualization_msgs::Marker FOV_marker_initialization();

visualization_msgs::Marker rbt_traj_marker_initialization();

visualization_msgs::Marker planned_traj_marker_initialization();

visualization_msgs::Marker particle_visualization(Eigen::MatrixXf, visualization_msgs::Marker);

visualization_msgs::Marker hparticle_visualization(Robot, visualization_msgs::Marker);

visualization_msgs::Marker goal_visualization(Robot, visualization_msgs::Marker);

visualization_msgs::Marker loc_particle_visualization(Robot, visualization_msgs::Marker);

visualization_msgs::Marker FOV_visualization(Robot, visualization_msgs::Marker);

visualization_msgs::Marker rbt_traj_visualization(Robot, visualization_msgs::Marker);

visualization_msgs::Marker planned_traj_visualization(Robot, visualization_msgs::Marker);

float normalize_angle(float);

Belief particleFilter(Robot, Eigen::VectorXf, Belief, std::vector<float> );

std::vector<bool> inFOV(Robot, nav_msgs::OccupancyGrid&, Eigen::VectorXf, Eigen::MatrixXf, int);

std::vector<bool> inFOV_red(Robot, nav_msgs::OccupancyGrid&, Eigen::VectorXf, Eigen::MatrixXf, int);

std::vector<float> test_planner(Robot, Sim);

Eigen::MatrixXf normpdf_mat(float, Eigen::MatrixXf, float);

float normpdf(float, float, float);

float MI_reward(Robot, Sim, Belief, Eigen::VectorXf, nav_msgs::OccupancyGrid);

nav_msgs::OccupancyGrid map_inflation(nav_msgs::OccupancyGrid, float);

void particle_hierarchy(Robot&, Belief);
#endif