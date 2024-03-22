#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <math.h>
#include <string>
#include <vector>
#include <random>
#include "Robotclass.h"
#include "Simclass.h"
#include "RCTS.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

// global variables
nav_msgs::OccupancyGrid mapData;
std::vector<float> target_pose(2);

inParam_rbt rbt_param;
inParam_sim sim_param;

Robot rbt;
Sim sim;

// Perception module
void objectDetectCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // ROS_INFO("Target Pose: x:%0.6f, y:%0.6f", msg->point.x, msg->point.y);
    target_pose[0] = msg->point.x;
    target_pose[1] = msg->point.y;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Received map message");
    sim.map = *msg;
}

void inflatedmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Received inflated map message");
    rbt.map = *msg;
    // robot map inflation
    rbt.map = map_inflation(rbt.map, 0.1);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    rbt.pose[0] = msg->pose.pose.position.x;
    rbt.pose[1] = msg->pose.pose.position.y;
    rbt.pose[2] = tf::getYaw(msg->pose.pose.orientation);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    rbt.scan = *msg;
}


// Estimation module
void estimationModule()
{
    // TODO: Implement estimation logic here
    std::cout << "---------------------------Estimation module---------------------------" << std::endl;
}

Eigen::MatrixXf generatePrimitive(float v, float w, float dt, int steps) {
    Eigen::MatrixXf primitive(steps+1, 3);

    primitive(0, 0) = 0;  // x
    primitive(0, 1) = 0;  // y
    primitive(0, 2) = M_PI/2;  // theta

    for (int i = 0; i < steps; ++i) {
        primitive(i+1, 0) = primitive(i, 0) + v * dt / steps * cos(primitive(i, 2));  // x
        primitive(i+1, 1) = primitive(i, 1) + v * dt / steps * sin(primitive(i, 2));  // y
        primitive(i+1, 2) = primitive(i, 2) + w * dt / steps;  // theta
    }

    return primitive;
}


int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "EPFT_planner");
    
    // Robot robot("robot_name");

    // 创建节点句柄
    ros::NodeHandle n;

    // ros::Subscriber inflated_map_sub = n.subscribe("/move_base/global_costmap/costmap", 10000, inflatedmapCallback);
    ros::Subscriber inflated_map_sub = n.subscribe("/map", 100, inflatedmapCallback);

    ros::Subscriber map_sub = n.subscribe("/map", 10000, mapCallback);

    ros::Subscriber robot_sub = n.subscribe("/odom", 10, odomCallback);

    ros::Subscriber target_sub = n.subscribe("/object_detect_pose", 10, objectDetectCallback);

    ros::Subscriber laser_sub = n.subscribe("/limo/scan", 10, laserCallback);

    // 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    ros::Publisher robot_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Publisher particles_marker_pub = n.advertise<visualization_msgs::Marker>("/particles_marker", 10);

    ros::Publisher hparticles_marker_pub = n.advertise<visualization_msgs::Marker>("/hparticles_marker", 10);

    ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>("/goal_marker", 10);

    ros::Publisher loc_particles_marker_pub = n.advertise<visualization_msgs::Marker>("/loc_particles_marker", 10);

    ros::Publisher FOV_marker_pub = n.advertise<visualization_msgs::Marker>("/FOV_marker", 10);

    ros::Publisher rbt_traj_pub = n.advertise<visualization_msgs::Marker>("/rbt_traj_marker", 10);

    ros::Publisher planned_traj_pub = n.advertise<visualization_msgs::Marker>("/planned_traj_marker", 10);

    ros::Publisher inflated_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 10);
    
    

    // Game Simulation

    // Initialization

    ros::Rate rate(1.0);  // 设置频率为1Hz
    for (int i = 0; i < 3; ++i) {
        ros::spinOnce();  // 处理一次消息队列
        rate.sleep();     // 等待直到达到1Hz的频率
    }

    // std::cout << rbt.map.data.size() << std::endl;

    // 初始化机器人参数

    // rbt_param.pose = {0, 0, 0};

    rbt_param.r_max = 4;
    rbt_param.r_min = 0.5;
    rbt_param.theta_max = M_PI/2;
    rbt_param.R = Eigen::Matrix2f::Identity();

    rbt_param.is_tracking = 0;
    rbt_param.first = 1;

    rbt_param.numParticles = 500;
    rbt_param.map_size = 10;

    // 初始化粒子
    Belief bel_sim;
    bel_sim = particle_initialization(rbt_param.numParticles);
    rbt_param.bel = bel_sim;

    std::vector<std::vector<float>> input_S; // search
    std::vector<std::vector<float>> input_T; // tracking
    std::vector<std::vector<float>> a_S; // search
    std::vector<std::vector<float>> a_T; // tracking
    std::vector<Eigen::MatrixXf> path_S;
    std::vector<Eigen::MatrixXf> path_T;

    float v = 0.4;

    input_S = {{v, M_PI/4}, {v, M_PI/8}, {v, -M_PI/4}, {v, -M_PI/8}, {v, 0},{0, M_PI/4},{0, -M_PI/4}};
    input_T = {{0.15, M_PI/16}, 
            {0.10, M_PI/16},
            {0.05, M_PI/16},
            {0.15, M_PI/8}, 
            {0.10, M_PI/8},
            {0.05, M_PI/8},
            {0.15, -M_PI/16}, 
            {0.10, -M_PI/16},
            {0.05, -M_PI/16},
            {0.15, -M_PI/8}, 
            {0.10, -M_PI/8},
            {0.05, -M_PI/8},
            {0.15, 0},
            {0.10, 0},
            {0.05, 0},
            {0, M_PI/16},
            {0, M_PI/4},
            {0, -M_PI/16},
            {0, -M_PI/4}};

    int ii = 0;
    int steps = 20;
    float dt = 1;
    // 对于a_S中的每个元素
    for (const auto& a : input_S) {
        // 使用运动模型来生成一个motion primitive
        Eigen::MatrixXf primitive = generatePrimitive(a[0], a[1], dt, steps);

        // 将生成的motion primitive添加到path向量中
        path_S.push_back(primitive);

        std::vector<float> action = {primitive(steps, 0), primitive(steps, 1), primitive(steps, 2)-static_cast<float>(M_PI/2),static_cast<float>(ii),static_cast<float>(a[0]),static_cast<float>(a[1]*2)};
        a_S.push_back(action);
        ii++;
    }

    ii = 0;
    // 对于a_T中的每个元素
    for (const auto& a : input_T) {
        // 使用运动模型来生成一个motion primitive
        Eigen::MatrixXf primitive = generatePrimitive(a[0], a[1], dt, steps);

        // 将生成的motion primitive添加到path向量中
        path_T.push_back(primitive);

        std::vector<float> action = {primitive(steps, 0), primitive(steps, 1), primitive(steps, 2)-static_cast<float>(M_PI/2),static_cast<float>(ii),static_cast<float>(a[0]),static_cast<float>(a[1]*2)};
        a_T.push_back(action);
        ii++;
    }

    // std::cout << path_S[0] << std::endl;

    // std::cout << "a_S: " << a_S[0][0] << std::endl;
    // std::cout << "a_S: " << a_S[0][1] << std::endl;
    // std::cout << "a_S: " << a_S[0][2] << std::endl;
    // std::cout << "a_S: " << a_S[0][3] << std::endl;
    // std::cout << "a_S: " << a_S[0][4] << std::endl;

    rbt_param.a_S = a_S;
    rbt_param.a_T = a_T;
    rbt_param.path_S = path_S;
    rbt_param.path_T = path_T;
    
    
    rbt.Initialization(rbt_param);

    sim_param.sensor_type = "rb";
    sim_param.Q_search = Eigen::Matrix2f::Identity();
    sim_param.Q_tracking = Eigen::Matrix2f::Identity();

    sim.Initialization(sim_param);


    float ctrl[2] = {0, 0};


    // 初始可视化
    visualization_msgs::Marker points;
    points = marker_initialization();

    visualization_msgs::Marker hparticles;
    hparticles = hparticle_initialization();

    visualization_msgs::Marker goal_particle;
    goal_particle = goal_particle_initialization();

    visualization_msgs::Marker loc_particles;
    loc_particles = local_particle_initialization();

    visualization_msgs::Marker FOV_marker;
    FOV_marker = FOV_marker_initialization();

    visualization_msgs::Marker rbt_traj_marker;
    rbt_traj_marker = rbt_traj_marker_initialization();

    visualization_msgs::Marker planned_traj_marker;
    planned_traj_marker = planned_traj_marker_initialization();

    // ----------------Iteration--------------------

    // 设置循环的频率
    ros::Rate loop_rate(2);
 
    while (ros::ok())
    {
        auto start = std::chrono::high_resolution_clock::now();

        target_pose[0] = -100;
        target_pose[1] = -100;

        ros::spinOnce();  // 处理一次消息队列

        // Call the perception module

        Eigen::MatrixXf tarpos(2,1);
        tarpos(0,0) = target_pose[0];  
        tarpos(1,0) = target_pose[1];
        std::vector<bool> flg = inFOV(rbt, sim.map, rbt.pose, tarpos, 0);
        // std::cout << "visibility: " << flg[0] << std::endl;

        // range and bearing observation
        std::vector<float> obs(2);
        if (flg[0] == 1) {
            obs[0] = std::sqrt((target_pose[0]-rbt.pose[0])*(target_pose[0]-rbt.pose[0])+(target_pose[1]-rbt.pose[1])*(target_pose[1]-rbt.pose[1]));
            obs[1] = std::atan2(target_pose[1]-rbt.pose[1], target_pose[0]-rbt.pose[0]) - rbt.pose[2];
            obs[1] = normalize_angle(obs[1]);

            rbt.is_tracking = 1;
        } else {
            obs[0] = -100;
            obs[1] = -100;

            rbt.is_tracking = 0;
        }
        // std::cout << "rb sensor:" << obs[0] << "," << obs[1] << std::endl;

        // Call the estimation module
        // Particle Filter
        bel_sim = particleFilter(rbt, rbt.pose, bel_sim, obs);
        rbt.bel = bel_sim;

        // Particles Visualization
        points = particle_visualization(bel_sim.particles, points);
        particles_marker_pub.publish(points);

        FOV_marker = FOV_visualization(rbt, FOV_marker);
        FOV_marker_pub.publish(FOV_marker);

        inflated_map_pub.publish(rbt.map);

        // hierarchical_particles
        particle_hierarchy(rbt, bel_sim);

        hparticles = hparticle_visualization(rbt, hparticles);
        hparticles_marker_pub.publish(hparticles);

        goal_marker_pub.publish(goal_visualization(rbt, goal_particle));

        loc_particles_marker_pub.publish(loc_particle_visualization(rbt, loc_particles));

        // auto start = std::chrono::high_resolution_clock::now();

        // Call the planning module
        float reward;
        reward = MI_reward(rbt, sim, bel_sim, rbt.pose, rbt.map);
        // std::cout << "reward: " << reward << std::endl;

        // std::vector<float> ctrl = test_planner(rbt, sim);
        std::vector<float> ctrl = EPFT_planner(rbt, sim);
        std::cout << "---------------------------Perception module---------------------------" << std::endl;
        std::cout << "Observed Target Pose: x:" << target_pose[0] << " y:" << target_pose[1] << std::endl;

        std::cout << "Robot Pose: x:" << rbt.pose[0] << " y:" << rbt.pose[1] << " theta:" << rbt.pose[2] << std::endl;

        std::cout << "visibility: " << flg[0] << std::endl;

        std::cout << "rb sensor:" << obs[0] << "," << obs[1] << std::endl;

        std::cout << "reward: " << reward << std::endl;

        std::cout << rbt.scan.ranges.size() << std::endl;

        std::cout << "---------------------------EPFT planner---------------------------" << std::endl;

        std::cout << "v: " << ctrl[0] << std::endl;
        std::cout << "w: " << ctrl[1] << std::endl;
        std::cout << "value: " << rbt.max_value << std::endl;

        // 初始化geometry_msgs::Twist类型的消息
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = ctrl[0];
        vel_msg.angular.z = ctrl[1];
 
        // 发布消息
        robot_vel_pub.publish(vel_msg);
        // ROS_INFO("Publsh robot velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);

        // rbt_traj_marker = rbt_traj_visualization(rbt, rbt_traj_marker);
        // rbt_traj_pub.publish(rbt_traj_marker);

        planned_traj_marker = planned_traj_visualization(rbt, planned_traj_marker);
        planned_traj_pub.publish(planned_traj_marker);


        // 记时
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> diff = end - start;
        std::cout << "Time to run the code: " << diff.count() << " s\n";
 
        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
