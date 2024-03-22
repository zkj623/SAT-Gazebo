#include "Robotclass.h"
// #include "RCTS.h"

void Robot::Initialization(inParam_rbt inParam) {
    this->pose = inParam.pose;

    this->r_max = inParam.r_max;
    this->r_min = inParam.r_min;
    this->theta_max = inParam.theta_max;
    this->R = inParam.R;

    this->is_tracking = inParam.is_tracking;
    this->first = inParam.first;

    this->numParticles = inParam.numParticles;
    this->bel = inParam.bel;

    this->map_size = inParam.map_size;

    this->a_S = inParam.a_S;
    this->a_T = inParam.a_T;
    this->path_S = inParam.path_S;
    this->path_T = inParam.path_T;
}

//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}


 
Belief particle_initialization(int num) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0, 1);

    int numParticles = num;

    Eigen::MatrixXf particles_sim(2,numParticles);
    Eigen::RowVectorXf weights_sim(numParticles);

    for (int i = 0; i < numParticles; ++i)
    {
        particles_sim(0,i) = 1.0 + dist(gen);
        particles_sim(1,i) = 8.0 + dist(gen);
    }
    for (int j = 0; j < numParticles; j++) {
        weights_sim(j) = 1.0 / numParticles;
    }

    Belief bel_sim;
    bel_sim.particles = particles_sim;
    bel_sim.weights = weights_sim;
    return bel_sim;
}

visualization_msgs::Marker marker_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "particles";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::POINTS;

    // 设置粒子的大小
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0.05;

    // 设置粒子的颜色
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker hparticle_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "hparticles";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::POINTS;

    // 设置粒子的大小
    points.scale.x = 0.15;
    points.scale.y = 0.15;
    points.scale.z = 0.15;

    // 设置粒子的颜色
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker goal_particle_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "goal_particles";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::POINTS;

    // 设置粒子的大小
    points.scale.x = 0.15;
    points.scale.y = 0.15;
    points.scale.z = 0.15;

    // 设置粒子的颜色
    points.color.r = 0.0f;
    points.color.g = 0.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker local_particle_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "local_particles";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::POINTS;

    // 设置粒子的大小
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.scale.z = 0.1;

    // 设置粒子的颜色
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker FOV_marker_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "FOV";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::LINE_STRIP;

    // 设置粒子的大小
    points.scale.x = 0.1;

    // 设置粒子的颜色
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker rbt_traj_marker_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "rbt_traj";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::LINE_STRIP;

    // 设置粒子的大小
    points.scale.x = 0.05;

    // 设置粒子的颜色
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker planned_traj_marker_initialization(){
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.ns = "rbt_traj";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    // points.type = visualization_msgs::Marker::SPHERE;
    points.type = visualization_msgs::Marker::LINE_STRIP;

    // 设置粒子的大小
    points.scale.x = 0.05;

    // 设置粒子的颜色
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;

    return points;
}

visualization_msgs::Marker particle_visualization(Eigen::MatrixXf particles, visualization_msgs::Marker points) {
    points.points.clear();
    int numParticles = particles.cols();
    for (int i = 0; i < numParticles; ++i)
    {
        geometry_msgs::Point p;
        p.x = particles(0,i);
        p.y = particles(1,i);
        p.z = 0;

        points.points.push_back(p);
    }
    return points;
}

visualization_msgs::Marker hparticle_visualization(Robot rbt, visualization_msgs::Marker points) {
    points.points.clear();
    int numParticles = rbt.h_par.size();
    for (int i = 0; i < numParticles; ++i)
    {
        if (rbt.h_par[i].first_weight < 0.05) {
            continue;
        }
        geometry_msgs::Point p;
        p.x = rbt.h_par[i].first_particle[0];
        p.y = rbt.h_par[i].first_particle[1];
        p.z = 0;

        // float size = rbt.loc_w(i) * 0.1;
        // p.scale.x = size;
        // p.scale.y = size;
        // p.scale.z = size;

        points.points.push_back(p);
    }
    return points;
}

visualization_msgs::Marker goal_visualization(Robot rbt, visualization_msgs::Marker points) {
    points.points.clear();

    geometry_msgs::Point p;
    p.x = rbt.vir_tar[0];
    p.y = rbt.vir_tar[1];
    p.z = 0;

    points.points.push_back(p);

    return points;
}

visualization_msgs::Marker loc_particle_visualization(Robot rbt, visualization_msgs::Marker points) {
    points.points.clear();
    int numParticles = rbt.loc_par.cols();
    for (int i = 0; i < numParticles; ++i)
    {
        geometry_msgs::Point p;
        p.x = rbt.loc_par(0,i);
        p.y = rbt.loc_par(1,i);
        p.z = 0;

        points.points.push_back(p);
    }
    return points;
}

visualization_msgs::Marker FOV_visualization(Robot rbt, visualization_msgs::Marker points) {
    points.points.clear();
    points.type = visualization_msgs::Marker::LINE_STRIP;

    // 添加扇形的起点
    geometry_msgs::Point start;
    start.x = rbt.pose[0];
    start.y = rbt.pose[1];
    start.z = 0;
    points.points.push_back(start);

    // 添加扇形的边缘点
    for (float angle = -rbt.theta_max / 2; angle <= rbt.theta_max / 2; angle += 0.05) {
        geometry_msgs::Point p;
        p.x = rbt.pose[0] + rbt.r_max * cos(rbt.pose[2] + angle);
        p.y = rbt.pose[1] + rbt.r_max * sin(rbt.pose[2] + angle);
        p.z = 0;
        points.points.push_back(p);
    }

    // 添加扇形的终点
    points.points.push_back(start);

    return points;
}

visualization_msgs::Marker rbt_traj_visualization(Robot rbt, visualization_msgs::Marker points) {
    points.points.clear();
    points.type = visualization_msgs::Marker::LINE_STRIP;

    // 添加扇形的起点
    geometry_msgs::Point start;
    start.x = rbt.pose[0];
    start.y = rbt.pose[1];
    start.z = 0;
    points.points.push_back(start);

    for (int i = 0; i < rbt.traj.size(); i++) {
        geometry_msgs::Point p;
        p.x = rbt.traj[i][0];
        p.y = rbt.traj[i][1];
        p.z = 0;
        points.points.push_back(p);
    }

    return points;
}

visualization_msgs::Marker planned_traj_visualization(Robot rbt, visualization_msgs::Marker points) {
    points.points.clear();
    points.type = visualization_msgs::Marker::LINE_STRIP;

    // 添加扇形的起点
    geometry_msgs::Point start;
    start.x = rbt.pose[0];
    start.y = rbt.pose[1];
    start.z = 0;
    points.points.push_back(start);

    // 添加扇形的边缘点
    for (int i = 0; i < rbt.planned_traj.size(); i++) {
        geometry_msgs::Point p;
        p.x = rbt.planned_traj[i][0];
        p.y = rbt.planned_traj[i][1];
        p.z = 0;
        points.points.push_back(p);
    }

    return points;
}

float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}


Belief particleFilter(Robot rbt, Eigen::VectorXf rbt_pose, Belief bel, std::vector<float> obs){
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> norm_dist(0.0, 0.1);

    int numParticles = bel.particles.cols();
    Eigen::MatrixXf particles(2, numParticles);
    Eigen::RowVectorXf weights(numParticles);
    particles = bel.particles;
    weights = bel.weights;

    // Update particles based on observed target pose
    float sumWeights = 0.0;

    std::vector<bool> FOV = inFOV(rbt, rbt.map, rbt_pose, particles, 1);
    
    for (int j = 0; j < numParticles; j++) {
        // Prediction based on target motion model

        particles(0,j) += norm_dist(gen);
        particles(1,j) += norm_dist(gen);

        if (particles(0,j) < 0 || particles(0,j) > 10 || particles(1,j) < 0 || particles(1,j) > 10) {
            weights(j) = 0;
            continue;
        }
        std::vector<float> par = {particles(0,j), particles(1,j)};
        if (gridValue(rbt.map, par) > 50) {
            weights(j) = 0;
            continue;
        }
        if (obs[0] == -100 && obs[1] == -100){
            if (FOV[j]) {
                weights(j) = 0;
            }
            else {
                weights(j) = 1;
            }
        }
        else{
            if (j == 0){
                particles(0,j) = obs[0] * cos(obs[1] + rbt_pose[2]) + rbt_pose[0];
                particles(1,j) = obs[0] * sin(obs[1] + rbt_pose[2]) + rbt_pose[1];
            }
            if(FOV[j]) {
                // Calculate weight based on distance to observed target pose
                float dx = particles(0,j) - rbt_pose[0];
                float dy = particles(1,j) - rbt_pose[1];
                float range = std::sqrt(dx * dx + dy * dy);
                float range_diff = std::abs(range - obs[0]);
                // std::cout << "range_diff:" << range_diff << std::endl;
                float bearing = normalize_angle(std::atan2(dy, dx)- rbt_pose[2]);
                float bearing_diff = normalize_angle(bearing - obs[1]);

                weights(j) = std::exp(-0.5 * (range_diff * range_diff / 0.1 + bearing_diff * bearing_diff / 0.01));
            }
            else {
                weights(j) = 0;
            }
        }
        sumWeights += weights(j);
    }
    
    // Normalize weights
    for (int j = 0; j < numParticles; j++) {
        weights[j] /= sumWeights;
    }

    // Resampling
    int N = numParticles;
    int flag = 1;
    float M = 1.0 / N;
    float U = rand() / (RAND_MAX + 1.0) * M;
    Eigen::MatrixXf new_particles(2,numParticles);
    float tmp_w = weights[0];
    int i = 0;
    int jj = 0;
    
    while (jj < N) {
        while (tmp_w < U + jj * M) {
            i++;
            tmp_w += weights(i);
        }
        for (int k = 0; k < 2; k++) {
            new_particles(k,jj) = particles(k,i);
        }
        jj++;
    }
    for (int j = 0; j < N; j++) {
        particles(0,j) = new_particles(0,j);
        particles(1,j) = new_particles(1,j);
        // std::cout << "particles:" << particles(0,j) << std::endl;
        // std::cout << "particles:" << particles(1,j) << std::endl;
    }

    for (int j = 0; j < N; j++) {
        weights(j) = 1.0 / N;
    }

    Belief bel_tmp;

    bel_tmp.particles = particles;
    bel_tmp.weights = weights;

    return bel_tmp;
}


std::vector<bool> inFOV(Robot rbt, nav_msgs::OccupancyGrid& map, Eigen::VectorXf rbt_pose, Eigen::MatrixXf tar_pos, int id) {
    std::vector<bool> flag(tar_pos.cols());

    for (int i = 0; i < tar_pos.cols(); i++) {
        std::vector<float> tmp = {tar_pos(0,i) - rbt_pose[0], tar_pos(1,i) - rbt_pose[1]};
        float ran = std::sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1]);
        bool flag1 = ran < rbt.r_max;
        bool flag2 = ran > rbt.r_min;
        bool flag3 = (tmp[0] * std::cos(rbt_pose[2]) + tmp[1] * std::sin(rbt_pose[2])) / ran > std::cos(rbt.theta_max/2);

        flag[i] = flag1 && flag2 && flag3;

        // std::cout << "flg1: " << flag1 << std::endl;
        // std::cout << "flg2: " << flag2 << std::endl;
        // std::cout << "flg3: " << flag3 << std::endl;

        float angle_max = rbt.scan.angle_max;
        float angle_increment = rbt.scan.angle_increment;

        if (id && flag[i]) {
            float angle = std::atan2(tar_pos(1,i) - rbt_pose[1], tar_pos(0,i) - rbt_pose[0]) - rbt_pose[2];
            int idx = std::round((angle + rbt.scan.angle_max) / angle_increment);
            float obs_ran_val = rbt.scan.ranges[idx];
            bool flag4;
            if (std::isinf(obs_ran_val)) {
                flag4 = true;
            }
            else {
                flag4 = ran < obs_ran_val;
            }
            flag[i] = flag4;
        }
    }
    return flag;
}


std::vector<bool> inFOV_red(Robot rbt, nav_msgs::OccupancyGrid& map, Eigen::VectorXf rbt_pose, Eigen::MatrixXf tar_pos, int id) {
    std::vector<bool> flag(tar_pos.cols());

    for (int i = 0; i < tar_pos.cols(); i++) {
        std::vector<float> tmp = {tar_pos(0,i) - rbt_pose[0], tar_pos(1,i) - rbt_pose[1]};
        float ran = std::sqrt(tmp[0] * tmp[0] + tmp[1] * tmp[1]);
        bool flag1 = ran < rbt.r_max-0.5;
        bool flag2 = ran > rbt.r_min+0.2;
        bool flag3 = (tmp[0] * std::cos(rbt_pose[2]) + tmp[1] * std::sin(rbt_pose[2])) / ran > std::cos(rbt.theta_max/2);

        flag[i] = flag1 && flag2 && flag3;

        float angle_max = rbt.scan.angle_max;
        float angle_increment = rbt.scan.angle_increment;

        if (id && flag[i]) {
            float angle = std::atan2(tar_pos(1,i) - rbt_pose[1], tar_pos(0,i) - rbt_pose[0]) - rbt_pose[2];
            int idx = std::round((angle + rbt.scan.angle_max) / angle_increment);
            float obs_ran_val = rbt.scan.ranges[idx];
            bool flag4;
            if (std::isinf(obs_ran_val)) {
                flag4 = true;
            }
            else {
                flag4 = ran < obs_ran_val;
            }
            flag[i] = flag4;
        }
    }

    return flag;
}


// Planning module
std::vector<float> test_planner(Robot rbt, Sim sim)
{
    std::cout << "---------------------------Planning module---------------------------" << std::endl;
    std::vector<float> ctrl(2);

    float linear_velocity; // m/s
    float angular_velocity; // rad/s

    linear_velocity = 0.1; 
    angular_velocity = 0.8;

    linear_velocity = 0; 
    angular_velocity = 0;

    ctrl[0] = linear_velocity;
    ctrl[1] = angular_velocity;

    std::cout << "Linear Velocity: " << ctrl[0] << " m/s" << std::endl;
    std::cout << "Angular Velocity: " << ctrl[1] << " rad/s" << std::endl;

    return ctrl;
}



Eigen::MatrixXf normpdf_mat(float x, Eigen::MatrixXf mu, float sigma) {
    static const float inv_sqrt_2pi = 1/sqrt(2*M_PI);

    int size = mu.size();

    // std::cout << "size:" << size << std::endl;

    float tmp;
    Eigen::MatrixXf result(size,1);
    for (int i = 0; i < mu.size(); i++) {
        tmp = (x - mu(i, 0)) / sigma;
        result(i,0) = inv_sqrt_2pi / sigma * std::exp(-0.5f * tmp * tmp);
    }

    return result;
}

float normpdf(float x, float mu, float sigma) {
    static const float inv_sqrt_2pi = 1/sqrt(2*M_PI);
    float tmp = (x - mu) / sigma;
    return inv_sqrt_2pi / sigma * std::exp(-0.5f * tmp * tmp);
}


float MI_reward(Robot rbt, Sim sim, Belief bel, Eigen::VectorXf rbt_pose, nav_msgs::OccupancyGrid rbt_map){
    float reward = 0;
    if ((0 >= rbt_pose[0] && 0 >= rbt_pose[1]) || (10 <= rbt_pose[0] && 10 <= rbt_pose[1])) //|| this->map.region_exp(ceil(rbt_pose[0] * 5), ceil(rbt_pose[1] * 5)) < 0.45)
    {
        reward = -1000;
        return reward;
    }

    Eigen::MatrixXf R = rbt.R;
    float det_R = R.determinant();
    float H_cond = 0;
    float H0 = 0.5 * 2 * (log(2 * M_PI) + 1) + 0.5 * log(det_R);

    // std::cout << "H0:" << H0 << std::endl;
    // std::cout << "detR:" << det_R << std::endl;

    Eigen::MatrixXf ori_particles = bel.particles;
    Eigen::RowVectorXf ori_w = bel.weights;

    int N = ori_particles.cols();

    // std::cout << "N:" << N << std::endl;

    std::vector<bool> ori_FOV(N);
    if (rbt.is_tracking)
    {
        ori_FOV = inFOV_red(rbt, rbt_map, rbt_pose, ori_particles, 0);
    }
    else
    {
        ori_FOV = inFOV(rbt, rbt_map, rbt_pose, ori_particles, 0);
    }

    if (std::none_of(ori_FOV.begin(), ori_FOV.end(), [](bool v) { return v; })) {
    // FOV中的所有元素都为false
        reward = 0;
        return reward;
    }

    float ratio = 1;

    std::vector<std::vector<int>> Cidx(N, std::vector<int>(2));
    std::vector<std::vector<int>> flag(500, std::vector<int>(500));
    int new_N = 0;
    float grid_size;
    if (rbt.is_tracking)
    {
        grid_size = 0.2;
    }
    else
    {
        grid_size = 0.5;
    }

    int id1;
    int id2;
    for (int mm = 0; mm < N; mm++)
    {

        id1 = ceil(ori_particles(0,mm) / grid_size) + 10;
        Cidx[mm][0] = id1;
        id2 = ceil(ori_particles(1,mm) / grid_size) + 10;
        Cidx[mm][1] = id2;

        // std::cout << "id1:" << id1 << std::endl;

        // assert(mm < particles.size() && "mm is out of range");
        // assert(2 <= particles[mm].size() && "particles[mm] is too small");
        // assert(mm < Cidx.size() && "mm is out of range");
        // assert(2 <= Cidx[mm].size() && "Cidx[mm] is too small");
        // assert(id1 < flag.size() && "id1 is out of range");

        if (flag[id1][id2] == 0)
        {
            new_N++;
            flag[id1][id2] = new_N;
        }
    }

    Eigen::MatrixXf particles_tmp = ori_particles;
    Eigen::RowVectorXf w_tmp = ori_w;

    Eigen::MatrixXf particles = Eigen::MatrixXf::Zero(2, new_N);
    Eigen::RowVectorXf w = Eigen::RowVectorXf::Zero(new_N);

    for (int mm = 0; mm < N; mm++)
    {
        w[flag[Cidx[mm][0]][Cidx[mm][1]] - 1] += w_tmp[mm];
        // std::cout << "w1:" << w[flag[Cidx[mm][0]][Cidx[mm][1]] - 1] << std::endl;
        // std::cout << "w0:" << w_tmp[mm] << std::endl;
    }
    for (int mm = 0; mm < N; mm++)
    {
        particles(0,flag[Cidx[mm][0]][Cidx[mm][1]] - 1) += particles_tmp(0,mm) * w_tmp[mm] / w[flag[Cidx[mm][0]][Cidx[mm][1]] - 1];
        particles(1,flag[Cidx[mm][0]][Cidx[mm][1]] - 1) += particles_tmp(1,mm) * w_tmp[mm] / w[flag[Cidx[mm][0]][Cidx[mm][1]] - 1];
    }

    std::vector<bool> FOV(new_N);
    if (rbt.is_tracking)
    {
        FOV = inFOV_red(rbt, rbt_map, rbt_pose, particles, 1);
    }
    else
    {
        FOV = inFOV(rbt, rbt_map, rbt_pose, particles, 1);
    }

    Eigen::MatrixXf FOV_matrix(new_N,1);
    for (int jj = 0; jj < new_N; jj++)
    {
        // FOV_matrix(jj,0) = static_cast<float>(FOV[jj]);
        FOV_matrix(jj,0) = FOV[jj];
    }
    Eigen::MatrixXf w_matrix(1,new_N);
    for (int jj = 0; jj < new_N; jj++)
    {
        w_matrix(0,jj) = w[jj];
    }

    for (int jj = 0; jj < new_N; jj++)
    {
        float H_temp = w[jj] * FOV[jj];
        H_cond += H_temp;
    }
    H_cond = H0 * H_cond;

    // std::vector<std::vector<float>> mu(N, std::vector<float>(2));
    Eigen::MatrixXf mu(new_N,2);
    for (int jj = 0; jj < new_N; jj++)
    {
        mu(jj,0) = sqrt(pow(rbt_pose[0] - particles(0,jj), 2) + pow(rbt_pose[1] - particles(1,jj), 2) + 0.1);
        mu(jj,1) = atan2(particles(1,jj) - rbt_pose[1], particles(0,jj) - rbt_pose[0])-rbt_pose[2];
        mu(jj,1) = normalize_angle(mu(jj,1));
    }


    // for (int i = 0; i < N; i++)
    // {
    //     if (range(fmod(mu[i][1], 2 * M_PI)) > range(fmod(mu[i][1], 2 * M_PI)))
    //     {
    //         mu[i][1] = fmod(mu[i][1], 2 * M_PI);
    //     }
    //     else
    //     {
    //         mu[i][1] = fmod(mu[i][1], 2 * M_PI);
    //     }
    // }

    const int nx = 2;

    Eigen::MatrixXf V(nx,nx);
    V = R;

    Eigen::MatrixXf X(nx,nx);
    X = V; //待修改
    // X = V.sqrt().matrix();
    
    std::vector<float> ws(2 * nx + 1);
    float lambda = 2;
    ws[0] = lambda / (lambda + nx);
    for (int i = 1; i < 2 * nx + 1; i++)
    {
        ws[i] = 1 / (2 * (lambda + nx));
    }

    float tmp1 = 0;
    for (int jj = 0; jj < new_N; jj++)
    {
        std::vector<std::vector<float>> sigma(2 * nx + 1,std::vector<float>(nx));
        sigma[0][0] = mu(jj,0);
        sigma[0][1] = mu(jj,1);
        for (int ss = 0; ss < nx; ss++)
        {
            sigma[2 * ss][0] = mu(jj,0) + sqrt(lambda + nx) * X(ss,0);
            sigma[2 * ss][1] = mu(jj,1) + sqrt(lambda + nx) * X(ss,1);
            sigma[2 * ss + 1][0] = mu(jj,0) - sqrt(lambda + nx) * X(ss,0);
            sigma[2 * ss + 1][1] = mu(jj,1) - sqrt(lambda + nx) * X(ss,1);
        }
        float tmp2 = 0;
        for (int ll = 0; ll < 2 * nx + 1; ll++)
        {
            float tmp3 = 0;
            for (int ss = 0; ss < new_N; ss++)
            {
                float tmp4 = 1;
                if (FOV[jj] != FOV[ss])
                {
                    tmp4 = 0;
                }
                if (FOV[jj] == 1 && FOV[ss] == 1)
                {
                    if (sim.sensor_type == "rb")
                    {
                        tmp4 = (FOV[ss] == 1) * normpdf(sigma[ll][0], mu(ss,0), sqrt(R(0,0))) * normpdf(sigma[ll][1], mu(ss,1), sqrt(R(1,1)));
                    }
                }
                tmp3 += w[ss] * tmp4;
                // std::cout << "tmp4:" << tmp4 << std::endl;
            }

            // Eigen::MatrixXf tmp4(new_N,1);
            // if (FOV[jj] == 1)
            // {
            //     // std::cout << mu << std::endl;
            //     Eigen::MatrixXf mat1 = normpdf_mat(sigma[ll][0], mu.col(0), sqrt(R(0,0)));
            //     Eigen::MatrixXf mat2 = normpdf_mat(sigma[ll][1], mu.col(1), sqrt(R(1,1)));
            //     tmp4 = FOV_matrix.array() * mat1.array() * mat2.array();
            //     // tmp4 = 1 - FOV_matrix.array();
            // }
            // else
            // {
            //     // tmp4 = (FOV_matrix.array() == 0);
            //     tmp4 = 1 - FOV_matrix.array();
            // }
            // tmp3 = (w_matrix * tmp4)(0,0);


            tmp2 += ws[ll] * log(tmp3);
        }
        tmp1 += w[jj] * tmp2;
    }

    // std::cout << "tmp1:" << tmp1 << std::endl;
    // std::cout << "H_cond:" << H_cond << std::endl;

    reward = -tmp1 - H_cond;
    reward = reward * ratio;
    if (reward < -pow(10, 1))
    {
        throw std::runtime_error("negative reward!");
    }
    else if (reward < 1e-10)
    {
        reward = 0;
    }

    return reward;
}

nav_msgs::OccupancyGrid map_inflation(nav_msgs::OccupancyGrid map, float inflationRadius)
{
    nav_msgs::OccupancyGrid map_inflated;
    map_inflated = map;
    int width = map.info.width;
    int height = map.info.height;
    float resolution = map.info.resolution;
    std::vector<signed char> data = map.data;
    std::vector<signed char> data_inflated = map.data;
    std::vector<float> distance(width * height, 1000);
    // 第一遍，从左上角到右下角
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (data[y * width + x] == 100) {
                distance[y * width + x] = 0;
            } else {
                int tmp = ceil(inflationRadius/resolution)+1;
                for (int y1 = y-tmp; y1 < y+tmp; ++y1) {
                    for (int x1 = x-tmp; x1 < x+tmp; ++x1) {
                        if (data[y1 * width + x1] == 100) {
                            float d = std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y)) * resolution * resolution;
                            if (d < distance[y * width + x]) {
                                distance[y * width + x] = d;
                            }
                        }
                    }
                }
            }
        }
    }

    // 标记膨胀区域
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (distance[y * width + x] <= inflationRadius) {
                data_inflated[y * width + x] = 70;
            }
            if (distance[y * width + x] == 0) {
                data_inflated[y * width + x] = 100;
            }
        }
    }

    map_inflated.data = data_inflated;

    return map_inflated;
}

void particle_hierarchy(Robot& rbt, Belief bel_sim){
    Belief loc_bel;

    int N = bel_sim.particles.cols();

    std::vector<std::vector<int>> Cidx(N, std::vector<int>(2));
    std::vector<std::vector<int>> flag(4, std::vector<int>(4));
    int new_N = 0;
    float grid_size = rbt.map_size / 4;

    Eigen::MatrixXf particles_tmp = bel_sim.particles;
    Eigen::RowVectorXf w_tmp = bel_sim.weights;

    int id1;
    int id2;
    for (int mm = 0; mm < N; mm++)
    {
        id1 = ceil(particles_tmp(0,mm) / grid_size)-1;
        Cidx[mm][0] = id1;
        id2 = ceil(particles_tmp(1,mm) / grid_size)-1;
        Cidx[mm][1] = id2;

        // std::cout << "id1:" << id1 << std::endl;
        // std::cout << "id2:" << id2 << std::endl;

        if (flag[id1][id2] == 0)
        {
            new_N++;
            flag[id1][id2] = new_N;
        }
    }

    std::vector<hpar> h_par(new_N);

    Eigen::MatrixXf particles = Eigen::MatrixXf::Zero(2, new_N);
    Eigen::RowVectorXf w = Eigen::RowVectorXf::Zero(new_N);

    int num;

    for (int mm = 0; mm < N; mm++)
    {
        num = flag[Cidx[mm][0]][Cidx[mm][1]] - 1;
        w[num] += w_tmp[mm];
    }
    for (int mm = 0; mm < N; mm++)
    {
        num = flag[Cidx[mm][0]][Cidx[mm][1]] - 1;
        particles(0,num) += particles_tmp(0,mm) * w_tmp[mm] / w[num];
        particles(1,num) += particles_tmp(1,mm) * w_tmp[mm] / w[num];

        std::vector<float> par_tmp = {particles_tmp(0,mm), particles_tmp(1,mm)};
        h_par[num].third_particles.push_back(par_tmp);
        h_par[num].third_weights.push_back(w_tmp(mm));
    }

    for (int mm = 0; mm < new_N; mm++)
    {
        h_par[mm].first_particle = particles.col(mm);
        h_par[mm].first_weight = w(mm);
    }

    rbt.h_par = h_par;

    // choose the first particle as the goal

    int id;

    int flg = 0;
    int idx = ceil(rbt.vir_tar(0)/grid_size);
    int idy = ceil(rbt.vir_tar(1)/grid_size);
    for (int jj = 0; jj < h_par.size(); jj++)
    {
        int idx_tmp = ceil(h_par[jj].first_particle(0)/grid_size);
        int idy_tmp = ceil(h_par[jj].first_particle(1)/grid_size);
        if (idx_tmp == idx && idy_tmp == idy && h_par[jj].first_weight > 0.02) // ablation:Q小,0.15; comparison:Q大,0.02
        {
            flg = 1;
            id = jj;
            break;
        }
    }

    if (flg == 0)
    {
        float minDistance = 100;
        for (int jj = 0; jj < h_par.size(); jj++)
        {
            if (h_par[jj].first_weight > 0.05)
            {
                float distance = sqrt(pow(rbt.pose(0) - h_par[jj].first_particle(0), 2) + pow(rbt.pose(1) - h_par[jj].first_particle(1), 2));
                if (distance < minDistance)
                {
                    minDistance = distance;
                    id = jj;
                }
            }
        }
    }

    rbt.vir_tar = h_par[id].first_particle;

    // Convert to Eigen library
    std::vector<std::vector<float>> vec = h_par[id].third_particles;
    Eigen::MatrixXf mat(vec.size(), vec[0].size());
    for (int i = 0; i < vec.size(); ++i) {
        mat.row(i) = Eigen::VectorXf::Map(&vec[i][0], vec[i].size());
    }
    mat.transposeInPlace();
    rbt.loc_par = mat;

    std::vector<float> vec_w = h_par[id].third_weights;
    Eigen::RowVectorXf mat_w(vec_w.size());
    for (int i = 0; i < vec_w.size(); ++i) {
        mat_w(i) = vec_w[i];
    }
    rbt.loc_w = mat_w;

}

  





