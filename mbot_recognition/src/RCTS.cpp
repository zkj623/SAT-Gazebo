#include "RCTS.h"
#include "Robotclass.h"

std::vector<std::vector<float>> vec2mat(std::vector<float> vec, int n) {
    int m = vec.size() / n;
    std::vector<std::vector<float>> mat(m, std::vector<float>(n));
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            mat[i][j] = vec[i * m + j];
        }
    }
    return mat;
}

//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp){

float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;
float width=mapData.info.width;

std::vector<signed char> Data=mapData.data;

// std::cout << "Xp: " << Xp[0] << " " << Xp[1] << std::endl;

//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free
float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
// float indx=(  floor((Xp[0]-Xstarty)/resolution)*width)+( floor((Xp[1]-Xstartx)/resolution) );

// std::cout << "indx: " << indx << std::endl;
// std::cout << "Data_size: " << Data.size() << std::endl;
int out;
out=Data[int(indx)];
return out;
}


std::vector<float> EPFT_planner(Robot& rbt, Sim sim)
{
    // std::cout << "---------------------------EPFT planner---------------------------" << std::endl;

    bool is_tracking = rbt.is_tracking;
    bool first = rbt.first;

    std::vector<std::vector<float>> a;
    if (!is_tracking)
    {
        a = rbt.a_S;
    }
    else
    {
        a = rbt.a_T;
    }

    std::vector<Node> tree;
    Node root;
    // Initialization
    root.num = 1;
    root.state = rbt.pose;
    root.hist = {};
    root.a = a;

    // if (!is_tracking)
    // {
    //     int id = rbt.a_hist;
    //     if (id == 5)
    //     {
    //         root.a.erase(root.a.begin() + 6);
    //     }
    //     else if (id == 6)
    //     {
    //         root.a.erase(root.a.begin() + 5);
    //     }
    // }

    // std::cout << root.a.size() << std::endl;

    root.N = 0;
    root.Q = 0;
    root.parent = 0;
    root.children = {};
    tree.push_back(root);
    rbt.cache = {};

    int max_depth = is_tracking ? 4 : 10;

    float eta = is_tracking ? 0.95 : 0.95;

    int planlen = is_tracking ? 30 : 50;

    Eigen::MatrixXf B = rbt.bel.particles;
    Eigen::RowVectorXf w = rbt.bel.weights;

    if (!rbt.is_tracking)
    {
        B = rbt.loc_par;
        w = rbt.loc_w;
    }

    int num = 0;
    rbt.tree = tree;
    while (num < 100)
    {
        float Reward;
        Reward = simulate(rbt, sim, 0, num, max_depth, eta, a, B, w);
        // std::cout << "num: " << num << std::endl;
        // num
    }

    tree = rbt.tree;
    rbt.planned_traj = {};

    std::vector<float> opt_action(2);

    float max_value = -10000;

    // std::cout << tree[0].children.empty() << std::endl;
    if (tree[0].children.empty())
    {
        opt_action = {0, 0};
    }
    else
    {
        // Planning horizon 1
        std::vector<float> val(tree[0].children.size());
        for (int jj = 0; jj < tree[0].children.size(); jj++)
        {
            val[jj] = tree[tree[0].children[jj]].Q;
        }
        int maxid;
        auto max_id = std::max_element(val.begin(), val.end());

        max_value = *max_id;
        maxid = std::distance(val.begin(), max_id);

        // std::cout << "maxvalue: " << max_value << std::endl;

        int opt = tree[0].children[maxid];
        int id = tree[opt].a_num;

        opt_action[0] = a[id][4];
        opt_action[1] = a[id][5];

        Eigen::VectorXf z = rbt.pose;

        Eigen::MatrixXf p;
        if (is_tracking){
            p = rbt.path_T[id];
        }
        else{
            p = rbt.path_S[id];
        }

        for (int ii = 0; ii < p.rows(); ii++)
        {
            // rbt.traj.push_back({p(ii, 0) * sin(z[2]) + p(ii, 1) * cos(z[2]) + z[0], -p(ii, 0) * cos(z[2]) + p(ii, 1) * sin(z[2]) + z[1], z[2] + p(ii, 2) - static_cast<float>(M_PI / 2)});
            rbt.planned_traj.push_back({p(ii, 0) * sin(z[2]) + p(ii, 1) * cos(z[2]) + z[0], -p(ii, 0) * cos(z[2]) + p(ii, 1) * sin(z[2]) + z[1], z[2] + p(ii, 2) - static_cast<float>(M_PI / 2)});
        }
        // rbt.traj.push_back({p(0,0) * sin(z[2]) + p(0,1) * cos(z[2]) + z[0], -p(0,0) * cos(z[2]) + p(0,1) * sin(z[2]) + z[1], z[2] + p(0,2) - static_cast<float>(M_PI / 2)});
        rbt.a_hist = id;

        // Planning horizon
        for (int hor = 0; hor < 1; ++hor)
        {
            if (!tree[opt].children.empty())
            {
                opt = tree[opt].children[0];
                if (!tree[opt].children.empty())
                {
                    std::vector<float> val(tree[opt].children.size());
                    for (int jj = 0; jj < tree[opt].children.size(); jj++)
                    {
                        val[jj] = tree[tree[opt].children[jj]].Q;
                    }
                    z = tree[opt].state;

                    int maxid;
                    auto max_id = std::max_element(val.begin(), val.end());
                    maxid = std::distance(val.begin(), max_id);

                    opt = tree[opt].children[maxid];

                    int id = tree[opt].a_num;
                    Eigen::MatrixXf p;
                    if (is_tracking){
                        p = rbt.path_T[id];
                    }
                    else{
                        p = rbt.path_S[id];
                    }
                    for (int ii = 0; ii < p.rows(); ii++)
                    {
                        rbt.planned_traj.push_back({p(ii, 0) * sin(z[2]) + p(ii, 1) * cos(z[2]) + z[0], -p(ii, 0) * cos(z[2]) + p(ii, 1) * sin(z[2]) + z[1], z[2] + p(ii, 2) - static_cast<float>(M_PI / 2)});
                    }
                }
            }
        }
    }
    rbt.max_value = max_value;

    // opt_action = {0,M_PI/2};
    // opt_action = {0,0};

    return opt_action;
}

// policy tree construction
float simulate(Robot& rbt, Sim sim, int begin, int& num, int depth, double eta, std::vector<std::vector<float>> a, Eigen::MatrixXf B, Eigen::RowVectorXf w) {
    double K;

    // std::cout << "depth: " << depth << std::endl;

    if (rbt.is_tracking) {
        K = 0.5;
    } else {
        K = 0.5;
    }
    double alpha = 0.1;

    float Reward;

    if (depth == 0) {
        Reward = 0;
        return Reward;
    } else {
        Eigen::Vector3f z = rbt.tree[begin].state;
        int num_a;

        rbt.tree[begin].N = rbt.tree[begin].N + 1;

        if (rbt.tree[begin].a.size() == 0) {
            begin = best_child(rbt, begin, 0.732);
        } else {
            num = num + 1;

            begin = expand(rbt, begin, num, {0,0}, 1, a);
            
            num_a = begin;

            int id = rbt.tree[num_a].a_num;

            Eigen::MatrixXf p;
            if (rbt.is_tracking){
                p = rbt.path_T[id];
            }
            else{
                p = rbt.path_S[id];
            }
            
            std::vector<std::vector<float>> tmp(p.rows(), std::vector<float>(3));
            for (int jj = 0; jj < p.rows(); jj++) {
                tmp[jj][0] = p(jj,0) * sin(z[2]) + p(jj,1) * cos(z[2]) + z[0];
                tmp[jj][1] = -p(jj,0) * cos(z[2]) + p(jj,1) * sin(z[2]) + z[1];
                tmp[jj][2] = z[2] + p(jj,2);
            }

            int wrong = 0;
            for (int jj = 0; jj < tmp.size(); jj++) {
                if ((0.2 >= tmp[jj][0]) || (0.2 >= tmp[jj][1]) || (9.8 <= tmp[jj][0]) || (9.8 <= tmp[jj][1]) || gridValue(rbt.map, {tmp[jj][0], tmp[jj][1]}) > 50){
                    wrong = 1;
                    break;
                }
            }

            if (wrong) {
                rbt.tree[num_a].N = rbt.tree[num_a].N + 1;
                // need to be modified
                rbt.tree[num_a].Q = -5;
                Reward = -5;
                return Reward;
            }
        }
        
        num_a = begin;

        rbt.tree[num_a].N = rbt.tree[num_a].N + 1;

        Eigen::Matrix2f Q_noise;
        if (rbt.is_tracking) {
            Q_noise = sim.Q_tracking;
        } else {
            Q_noise = sim.Q_search;
        }
        // B = mvnrnd(B, Q_noise).transpose(); // ------------------------------------not completed

        // feasible particles
        for (int jj = 0; jj < B.cols(); jj++) {
            if ((0.2 > B(0, jj)) || (0.2 > B(1, jj)) || (9.8 < B(0, jj)) || (9.8 < B(1, jj)) || gridValue(rbt.map, {B(0, jj), B(1, jj)}) > 50) {
                w(jj) = 0;
            }
        }
        w /= w.sum();
        int N = B.cols();

        double M = 1.0 / N;
        double U = rand() / (RAND_MAX + 1.0) * M;
        Eigen::MatrixXf new_particles(2, N);
        double tmp_w = w(0);
        int i = 0;
        int jj = 0;
        while (jj < N) {
            while (tmp_w < U + jj * M) {
                i++;
                tmp_w += w(i);
            }
            new_particles.col(jj) = B.col(i);
            jj++;
        }
        B = new_particles;
        w = Eigen::MatrixXf::Ones(1, N) / N;

        Belief bel;
        bel.particles = B;
        bel.weights = w;
        Eigen::Vector3f state = rbt.tree[num_a].state;
        float reward = MI_reward(rbt, sim, bel, state, rbt.map);
 
        // immediate reward (to be modified)
        rbt.tree[num_a].R = reward;

        std::vector<float> o(2);  
        int flag;

        // std::cout << "error 0"<< std::endl;
        

        if (rbt.tree[begin].children.size() <= K * pow(rbt.tree[begin].N, alpha)) {
            if (N == 0) {
                o = {-100, -100};
            } else {
                int randnum = rand() % N;
                Eigen::VectorXf B_tmp = B.block(0, randnum, 2, 1);
                std::vector<bool> in = inFOV(rbt, rbt.map, state, B_tmp, 1);
                if (in[0]) {
                    o = {-100, -100};
                } else {
                    o[0] = sqrt(pow(B_tmp[0] - state[0], 2) + pow(B_tmp[1] - state[1], 2));
                    o[1] = normalize_angle(atan2(B_tmp[1] - state[1], B_tmp[0] - state[0]) - state[2]);
                }
            }

            num = num + 1;
            begin = expand(rbt, begin, num, o, 2, a);
            flag = 1;

        } else {
            begin = rbt.tree[begin].children[rand() % rbt.tree[begin].children.size()];
    
            std::vector<std::vector<float>> mat = rbt.tree[begin].hist;
            o[0] = mat[mat.size() - 1][mat[mat.size() - 1].size() - 2];
            o[1] = mat[mat.size() - 1][mat[mat.size() - 1].size() - 1];
            flag = 0;
        }

        int num_o = begin;

        Eigen::MatrixXf B_pre = B;
        if (o[0] != -100) { // If not taking PF, infeasible particles may occur
            bel = particleFilter(rbt, state, bel, o);
        }
        B = bel.particles;
        w = bel.weights;
        
        if (flag == 1) {
            Node node = rbt.tree[begin];
            float rollout;

            // no reuse
            // rollout = rollOut(rbt, sim, node, eta, depth - 1, B, w);

            // std::cout << "cache_cols: " << rbt.cache.cols() << std::endl;

            // reuse
            if (rbt.cache.cols() == 0) {
                rollout = rollOut(rbt, sim, node, eta, depth - 1, B, w);

                rbt.cache.conservativeResize(4, 1);

                Eigen::VectorXf vec(4);
                vec.segment(0, 3) = node.state;
                vec(3) = begin;

                rbt.cache.col(0) = vec;
                // rbt.cache.push_back({node.state.head(3), begin});
                rbt.tree[begin].r = rollout;
            }
            else{
                int Idx;
                double D;

                Eigen::MatrixXf dataset = rbt.cache.topRows(2);
                dataset.transposeInPlace();

                // 创建一个FLANN索引
                flann::Index<flann::L2<float>> index(flann::Matrix<float>(dataset.data(), dataset.rows(), dataset.cols()), flann::KDTreeIndexParams(4));
                index.buildIndex();

                // 执行k-NN搜索
                std::vector<std::vector<int>> indices;
                std::vector<std::vector<float>> dists;
                Eigen::VectorXf query = node.state.head(2);

                // std::cout << "knn" << std::endl;  

                index.knnSearch(flann::Matrix<float>(query.data(), 1, 2), indices, dists, 1, flann::SearchParams(128));

                Idx = indices[0][0];
                D = dists[0][0];

                // std::cout << Idx << std::endl;
                // std::cout << D << std::endl;

                double D_value;
                double theta_value;
                if (rbt.is_tracking) {
                    D_value = 3;
                    theta_value = M_PI;
                } else {
                    D_value = 1;
                    theta_value = M_PI / 4;
                }

                if (D < D_value && std::abs(rbt.cache(2,Idx) - node.state(2)) < theta_value) {
                    rollout = rbt.tree[rbt.cache(3, Idx)].r;
                    rbt.tree[begin].r = rollout;
                } else {
                    rollout = rollOut(rbt, sim, node, eta, depth - 1, B, w);
                    rbt.cache.conservativeResize(4, rbt.cache.cols() + 1);
                    rbt.cache.col(rbt.cache.cols() - 1) << node.state.head(3), begin;
                    rbt.tree[begin].r = rollout;

                    int T = rbt.tree[begin].hist.size();
                    for (int ii = 1; ii < rbt.tree.size(); ii++) {
                        if (rbt.tree[ii].a_num == 0 && rbt.tree[ii].hist.size() + 1 == T) {
                            int kk = 0;
                            for (int jj = 0; jj < rbt.tree[ii].a.size(); jj++) {
                                std::vector<float> action = rbt.tree[ii].a[kk];
                                int id = action[3];
                                Eigen::MatrixXf p;
                                if (rbt.is_tracking){
                                    p = rbt.path_T[id];
                                }
                                else{
                                    p = rbt.path_S[id];
                                }

                                Eigen::Vector3f z = rbt.tree[ii].state;
                                Eigen::Vector3f tmp;

                                int wrong = 0;
                                for (int ll = 0; ll < p.rows() ; ll++) {
                                    if (ll % 5 == 0) {
                                        tmp[0] = p(ll,0) * sin(z[2]) + p(ll,1) * cos(z[2]) + z[0];
                                        tmp[1] = -p(ll,0) * cos(z[2]) + p(ll,1) * sin(z[2]) + z[1];
                                        tmp[2] = z[2] + p(ll,2);
                                        if ((0.2 >= tmp[0]) || (0.2 >= tmp[1]) || (9.8 <= tmp[0]) || (9.8 <= tmp[1])||(gridValue(rbt.map, {tmp[0], tmp[1]}) > 50)) {
                                            wrong = 1;
                                            break;
                                        }
                                    }
                                }

                                if (wrong == 0) {
                                    int p_row = p.rows()-1;
                                    tmp[0] = p(p_row,0) * sin(z[2]) + p(p_row,1) * cos(z[2]) + z[0];
                                    tmp[1] = -p(p_row,0) * cos(z[2]) + p(p_row,1) * sin(z[2]) + z[1];
                                    tmp[2] = z[2] + p(p_row,2);
  
                                    if (sqrt(pow(tmp[0]-rbt.cache(0,rbt.cache.cols() - 1),2)+pow(tmp[1]-rbt.cache(1,rbt.cache.cols() - 1),2)) &&
                                        std::abs(rbt.cache(2, rbt.cache.cols() - 1) - tmp[2]) < M_PI) {
                                        Eigen::Vector3f state = tmp;
                                        num = num + 1;
                                        num = expand_spec(rbt, ii, num, state, action, a);

                                        Belief bel_pre;
                                        bel_pre.particles = B_pre;
                                        bel_pre.weights = w;
                                        double r = eta * rollout + MI_reward(rbt, sim, bel_pre, state, rbt.map);
                                        backup(rbt, num, r, eta);

                                        rbt.tree[ii].a.erase(rbt.tree[ii].a.begin() + kk);
                                        kk = kk - 1;
                                    }
                                }

                                kk = kk + 1;
                            }
                        }
                    }
                }
            }

            // rollout = 0;
            Reward = reward + eta*rollout;
        }
        else {
            Reward = simulate(rbt, sim, begin, num, depth - 1, eta, a, B, w);
            Reward = reward + eta * Reward;
        }

        // std::cout << "error 2"<< std::endl;

        // std::cout << "tree_size: " << rbt.tree.size() << std::endl;
        // std::cout << "num_o: " << num_o << std::endl;
        // std::cout << "num_a: " << num_a << std::endl;
        rbt.tree[num_o].N = rbt.tree[num_o].N + 1;
        rbt.tree[num_a].Q = rbt.tree[num_a].Q + (Reward - rbt.tree[num_a].Q) / rbt.tree[num_a].N;
    }
    return Reward;
}

            
void backup(Robot& rbt, int begin, double r, double eta) {
    int idx = begin;
    while (idx != 0) {
        if (rbt.tree[idx].a_num == 0) {
            rbt.tree[idx].N = rbt.tree[idx].N + 1;
            idx = rbt.tree[idx].parent;
        } else {
            rbt.tree[idx].N = rbt.tree[idx].N + 1;
            rbt.tree[idx].Q = rbt.tree[idx].Q + (r - rbt.tree[idx].Q) / rbt.tree[idx].N;
            idx = rbt.tree[idx].parent;
            r = rbt.tree[idx].R + eta * r;
        }
    }
}

int best_child(Robot& rbt, int begin, float c) {
    float max_val = -10000;
    int v;
    for (int jj = 0; jj < rbt.tree[begin].children.size(); jj++) {
        Node node = rbt.tree[begin];
        int tmp = node.children[jj];
        float val = rbt.tree[tmp].Q + 2 * c * sqrt(log(node.N) / rbt.tree[tmp].N);
        if (val > max_val) {
            max_val = val;
            v = tmp;
        }
    }
    return v;
}

int expand_spec(Robot& rbt, int begin, int num, Eigen::Vector3f state, std::vector<float> action, std::vector<std::vector<float>> a) {
    Node node = rbt.tree[begin];

    // action node
    Node new_node;
    new_node.num = num;
    new_node.a_num = action[3];
    new_node.state = state;

    std::vector<float> hist = action;
    hist.push_back(0);
    hist.push_back(0);

    new_node.hist = node.hist;
    new_node.hist.push_back(hist);

    new_node.a = a;
    new_node.N = 1;
    new_node.R = 0;
    new_node.Q = 0;
    new_node.r = 0;
    new_node.parent = begin;
    new_node.children = {};
    rbt.tree[begin].children.push_back(new_node.num);
    rbt.tree.push_back(new_node);
    begin = num;

    num = num + 1;

    // observation node
    Node obs_node = new_node;
    obs_node.num = num;
    obs_node.a_num = 0;
    obs_node.parent = begin;
    obs_node.children = {};
    std::vector<float> o = {-100, -100};
    for (int i = 0; i < o.size(); i++) {
        new_node.hist[new_node.hist.size() - 1].push_back(o[i]);
    }
    rbt.tree[begin].children.push_back(obs_node.num);
    rbt.tree.push_back(obs_node);
    begin = num;

    return num;
}

int expand(Robot& rbt, int begin, int num, std::vector<float> o, int tmp, std::vector<std::vector<float>> a) {
    int t = 1;
    Node node = rbt.tree[begin];
    Eigen::Vector3f state; 
    state.setZero(); 
    if (tmp == 1) { // action
        int ii = rand() % rbt.tree[begin].a.size();
        std::vector<float> action = rbt.tree[begin].a[ii];
        rbt.tree[begin].a.erase(rbt.tree[begin].a.begin() + ii);

        state[0] = node.state[0] + action[0] * sin(node.state[2]) * t + action[1] * cos(node.state[2]) * t;
        state[1] = node.state[1] - action[0] * cos(node.state[2]) * t + action[1] * sin(node.state[2]) * t;
        state[2] = node.state[2] + action[2] * t;

        Node new_node;
        new_node.num = num;
        new_node.a_num = action[3];
        new_node.state = state;

        std::vector<float> hist = action;
        new_node.hist = node.hist;
        new_node.hist.push_back(hist);

        new_node.a = a;

        new_node.N = 0;
        new_node.R = 0;
        new_node.Q = 0;
        new_node.r = 0;
        new_node.parent = begin;
        new_node.children = {};
        rbt.tree[begin].children.push_back(new_node.num);
        rbt.tree.push_back(new_node);
        begin = num;
    }
    else { // observation
        Node new_node = node;
        new_node.a = a;

        new_node.num = num;
        new_node.a_num = 0;
        for (int i = 0; i < o.size(); i++) {
            new_node.hist[new_node.hist.size() - 1].push_back(o[i]);
        }
        rbt.tree[begin].children.push_back(new_node.num);
        rbt.tree.push_back(new_node);
        begin = num;
    }

    // std::cout << num << std::endl;

    return begin;
}

float rollOut(Robot rbt, Sim sim, Node& node, double eta, int depth, Eigen::MatrixXf B, Eigen::RowVectorXf w) {
    int t = 1;

    if (depth == 0) {
        return 0;
    } 

    Eigen::Matrix2f Q_noise;
    if (rbt.is_tracking) {
        Q_noise = sim.Q_tracking;
    } else {
        Q_noise = sim.Q_search;
    }
    // B = mvnrnd(B, Q_noise).transpose(); // ------------------------------------not completed

    Eigen::MatrixXf B_tmp1 = B;
    int N = B.cols();

    int j = 0;
    for (int i = 0; i < B.cols(); i++) {
        if (B_tmp1(0, j) < 0 || B_tmp1(0, j) > 10 || B_tmp1(1, j) < 0 || B_tmp1(1, j) > 10) {
            B_tmp1.col(j) = B_tmp1.col(N - 1);
            B_tmp1.conservativeResize(Eigen::NoChange, N - 1);
            N--;
            j--;
        }
        j++;
    }
    
    if (B_tmp1.cols() == 0) {
        return 0;
    }

    Eigen::MatrixXf B_tmp2(2, N);
    B_tmp2.block(0, 0, 2, B_tmp1.cols()) = B_tmp1;
    for (int jj = B_tmp1.cols(); jj < N; jj++) {
        int randIdx = rand() % B_tmp1.cols();
        B_tmp2.col(jj) = B_tmp1.col(randIdx);
    }
    B = B_tmp2;
    w = Eigen::RowVectorXf::Constant(N, 1.0 / N);

    std::vector<float> action_opt;
    Eigen::Vector2f target;
    Eigen::Vector2f vec_tmp;

    if (!rbt.is_tracking) {
        // target = rbt.vir_tar;
        target = B * w.transpose();
    }
    else{
        target = B * w.transpose();
    }

    double mindist = 10000;
    double max_rew = -10000;
    for (int jj = 0; jj < node.a.size(); jj++) {
        std::vector<float> action = node.a[jj];
        Eigen::VectorXf state = node.state;
        state(0) = node.state(0) + action[0] * sin(node.state(2)) * t + action[1] * cos(node.state(2)) * t;
        state(1) = node.state(1) - action[0] * cos(node.state(2)) * t + action[1] * sin(node.state(2)) * t;
        state(2) = node.state(2) + action[2] * t;

        int id = action[3];

        Eigen::MatrixXf p;
        if (rbt.is_tracking){
            p = rbt.path_T[id];
        }
        else{
            p = rbt.path_S[id];
        }

        Eigen::Vector3f z = node.state;
        Eigen::VectorXf tmp(3);

        bool wrong = false;
        for (int kk = 0; kk < p.rows(); kk++) {
            if (kk % 5 == 0) {
                tmp(0) = p(kk,0) * sin(z(2)) + p(kk,1) * cos(z(2)) + z(0);
                tmp(1) = -p(kk,0) * cos(z(2)) + p(kk,1) * sin(z(2)) + z(1);
                tmp(2) = z(2) + p(kk,2);
                if ((0.2 >= tmp(0)) || (0.2 >= tmp(1)) || (9.8 <= tmp(0)) || (9.8 <= tmp(1)) || gridValue(rbt.map, {tmp(0), tmp(1)}) > 50) {
                    wrong = true;
                    break;
                }
            }
        }

        if (wrong) {
            continue;
        }

        vec_tmp = state.head(2) - target;
        float dist = vec_tmp.norm();
        if (dist < mindist) {
            mindist = dist;
            action_opt = action;
        }
    }

    float reward; 

    if (action_opt.size() == 0) {
        reward = 0;
    } else {
        node.state(0) = node.state(0) + action_opt[0] * sin(node.state(2)) + action_opt[1] * cos(node.state(2));
        node.state(1) = node.state(1) - action_opt[0] * cos(node.state(2)) + action_opt[1] * sin(node.state(2));
        node.state(2) = node.state(2) + action_opt[2];

        Belief bel;
        bel.particles = B;
        bel.weights = w;

        reward = MI_reward(rbt, sim, bel, node.state, rbt.map);
        vec_tmp = node.state.head(2) - target;
        reward = 5 * exp(-vec_tmp.norm());

        reward += eta * rollOut(rbt, sim, node, eta, depth - 1, B, w);
    }

    return reward;
}




  
 
 
 
 





























