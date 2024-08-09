#include "detector_test.hpp"
#include <signal.h>

#include <thread>
#include <chrono>

// Carla connection parameters
std::string FRAME_ID = "hero/LIDAR";
std::string CLOUD_PUB_NAME = "filtered_cloud";
std::string CLOUD_SUB_NAME = "/carla/hero/LIDAR";
std::string NODE_NAME = "detector_node";
std::string BOX_PUB_NAME = "bounding_box";

// Algorithm parameters
float VOXEL_SIZE = 0.3;
int VOXEL_MINPTS = 20;
float RANSAC_THRESHOLD = 0.2;
float ROI = -0.4;
float EPS = 0.5;
float EPS_RATIO = 0.03;
int MIN_PTS = 5;
float CLOUD_HEIGHT = 3;
float BOX_DIST = 500;


namespace plt = matplotlibcpp;
static int iter = 0;


void Detector::get_param() {
    nh.getParam("/detector/VOXEL_SIZE", VOXEL_SIZE);
    nh.getParam("/detector/VOXEL_MINPTS", VOXEL_MINPTS);
    nh.getParam("/detector/RANSAC_THRESHOLD", RANSAC_THRESHOLD);
    nh.getParam("/detector/ROI", ROI);
    nh.getParam("/detector/EPS", EPS);
    nh.getParam("/detector/EPS_RATIO", EPS_RATIO);
    nh.getParam("/detector/MIN_PTS", MIN_PTS);
    nh.getParam("/detector/CLOUD_HEIGHT", CLOUD_HEIGHT);
    nh.getParam("/detector/BOX_DIST", BOX_DIST);

}

Detector::Detector() {
    // Update parameter
    get_param();
    
    // ROS base
    pub = nh.advertise<sensor_msgs::PointCloud2>(CLOUD_PUB_NAME, 100);
    pub_box = nh.advertise<vision_msgs::BoundingBox2DArray>(BOX_PUB_NAME,100);
    sub = nh.subscribe(CLOUD_SUB_NAME, 100, &Detector::callback, this);

    // Downsampling
    cloud_original.reset(new pcl::PCLPointCloud2());
    cloud_downsampled.reset(new pcl::PCLPointCloud2());
    cloud_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    vg.setMinimumPointsNumberPerVoxel(VOXEL_MINPTS);

    // ROI
    cloud_roi.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // RANSAC
    cloud_non_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ransac_inliers.reset(new pcl::PointIndices);
    ransac_coefficients.reset(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(RANSAC_THRESHOLD);
    Eigen::Vector3f v1(0.0f, 0.0f, -1.0f);
    seg.setAxis(v1);

    // Clustering
    cluster_vec.reserve(sizeof(Cluster) * 20);
    cloud_clustered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    indices_temp.reset(new pcl::PointIndices);

    // Tracking
    F << 1,0,dt,0,0,0,
         0,1,0,dt,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;

    H << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;
    
    Q << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;

    R << 1,0,0,0,
         0,1,0,0,
         0,0,1,0,
         0,0,0,1;
    
    I << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0,
         0,0,0,1,0,0,
         0,0,0,0,1,0,
         0,0,0,0,0,1;    
}

Detector::~Detector() {}

void Detector::callback(const sensor_msgs::PointCloud2& msg) {
    pcl_conversions::toPCL(msg, *cloud_original);
}

void Detector::downsample() {

    vg.setInputCloud(cloud_original);
    vg.filter(*cloud_original);
    pcl::fromPCLPointCloud2(*cloud_original, *cloud_downsampled_);

    roi_filter.setFilterFieldName("x");
    roi_filter.setInputCloud(cloud_downsampled_);
    roi_filter.setNegative(true);
    roi_filter.setFilterLimits(-100, -15); // 
    roi_filter.filter(*cloud_downsampled_);

}

void Detector::ground_extract() {

    roi_filter.setFilterFieldName("z");
    roi_filter.setInputCloud(cloud_downsampled_);
    roi_filter.setNegative(false);
    roi_filter.setFilterLimits(ROI, 20); // non ground roi
    roi_filter.filter(*cloud_non_ground);

    roi_filter.setNegative(true);
    roi_filter.filter(*cloud_roi); // ground roi

    seg.setInputCloud(cloud_roi);
    seg.segment(*ransac_inliers, *ransac_coefficients);
    
    extractor.setInputCloud(cloud_roi);
    extractor.setIndices(ransac_inliers);
    extractor.setNegative(true);
    extractor.filter(*cloud_roi);

    *cloud_non_ground += *cloud_roi;

}

void Detector::reset_dbscan() {

    if (visit.size() != 0) {
        visit.clear();
    }
    if (cluster_vec.size() != 0) {
        cluster_vec.clear();
    }
    visit.resize(cloud_non_ground->points.size(), -1);

}

std::vector<int> Detector::region_query(int query_point_idx) {

    std::vector<int> neighbor_vec;
    std::vector<float> searched_dist_vec;
    std::vector<int> searched_idx_vec;
    pcl::PointXYZ query_point = cloud_non_ground->points[query_point_idx];
    
    float dist = std::sqrt(std::pow(query_point.x,2) + std::pow(query_point.y,2) + std::pow(query_point.z,2));

    if (kdtree.radiusSearch(query_point, EPS + (dist * EPS_RATIO), searched_idx_vec, searched_dist_vec) > 0) {
        for (size_t i = 0; i < searched_idx_vec.size(); i++) {
            neighbor_vec.push_back(searched_idx_vec[i]);
        }
    }

    visit[query_point_idx] = 1;
    
    return neighbor_vec;
}

void Detector::expand_cluster(std::vector<int>& neighbor_vec) {
    size_t i = 0;
    while (i < neighbor_vec.size()) {
        int neighbor_idx = neighbor_vec[i];

        if (visit[neighbor_idx] == -1) {

            std::vector<int> new_neighbor_vec = region_query(neighbor_idx);
            if (new_neighbor_vec.size() >= MIN_PTS) {
                neighbor_vec.insert(neighbor_vec.end(), new_neighbor_vec.begin(), new_neighbor_vec.end());
                visit[neighbor_idx] = 1;
            } else {
                visit[neighbor_idx] = 0;
            }

        }
        ++i;
    }
    
}

void Detector::create_cluster(int cluster_id, std::vector<int>& neighbor_vec) {

    std::set<int> indices_set(neighbor_vec.begin(), neighbor_vec.end());
    indices_temp->indices.assign(indices_set.begin(), indices_set.end());

    if (neighbor_vec.size() == 0) {return;}
    extractor.setIndices(indices_temp);
    extractor.filter(*cloud_temp);
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cloud_temp, min_point, max_point);
    float height = max_point.z - min_point.z;

    if (height > CLOUD_HEIGHT) {return;}

    cluster_vec.emplace_back(cluster_id, indices_set, min_point, max_point);

}

void Detector::dbscan() {

    reset_dbscan();
    kdtree.setInputCloud(cloud_non_ground);
    int cluster_id = 0;
    int noise_id = 0;
    extractor.setInputCloud(cloud_non_ground);
    extractor.setNegative(false);

    for (size_t i = 0; i < cloud_non_ground->points.size(); i++) {
        if (visit[i] != -1) {continue;}
        float height = -1;

        std::vector<int> neighbor_vec = region_query(i);
        if (neighbor_vec.size() < MIN_PTS) {
            visit[i] = 0;
            continue;
        } else {
            ++cluster_id;
            expand_cluster(neighbor_vec);
            create_cluster(cluster_id, neighbor_vec);
        }
    }

}

void Detector::bounding_box() {
    if (!msg_box.boxes.empty()) {
        msg_box.boxes.clear();
    }
    if (cluster_vec.empty()) {return;}
    for (const auto& elem : cluster_vec) {
        vision_msgs::BoundingBox2D box;
        float x = (elem.min_point.x + elem.max_point.x) / 2;
        float y = (elem.min_point.y + elem.max_point.y) / 2;
        float x_length = elem.max_point.x - elem.min_point.x;
        float y_length = elem.max_point.y - elem.min_point.y;
        box.center.x = x;
        box.center.y = y;
        box.center.theta = 0;
        box.size_x = x_length;
        box.size_y = y_length;
        msg_box.boxes.push_back(box);
    }
}

void Detector::association() {
    // Solve Assignment problem
    if (track_vec.empty() && cluster_vec.empty()) {
        return;
    }

    if (track_vec.empty() && !cluster_vec.empty()) {
        for (size_t i = 0; i < cluster_vec.size(); i++) {
            create_track(cluster_vec[i]);
        }
        return;
    }

    if (cluster_vec.empty()) {
        std::vector<int> assignment(track_vec.size(), -1);
        for (int i = 0; i < assignment.size(); i++) {
            track_vec[i].not_detected++;
            track_vec[i].associated = false;
        }
    } else {
        std::vector<std::vector<double>> cost_matrix = create_cost_matrix();
        std::vector<int> assignment(track_vec.size(), -1);
        double cost = hungarian.Solve(cost_matrix, assignment);
        for (int i = 0; i < assignment.size(); i++) {
            int cluster_idx = assignment[i];
            if (cluster_idx == -1) {
                track_vec[i].not_detected++;
                track_vec[i].associated = false;
                continue;
            }
            track_vec[i].meas << (cluster_vec[cluster_idx].min_point.x + cluster_vec[cluster_idx].max_point.x) / 2,
                                (cluster_vec[cluster_idx].min_point.y + cluster_vec[cluster_idx].max_point.y) / 2,
                                (cluster_vec[cluster_idx].max_point.x - cluster_vec[cluster_idx].min_point.x),
                                (cluster_vec[cluster_idx].max_point.y - cluster_vec[cluster_idx].min_point.y);
            track_vec[i].confirmed_count++;
            track_vec[i].not_detected = 0;
            track_vec[i].associated = true;
            if (track_vec[i].confirmed_count >= 2) {track_vec[i].confirmed = true;}
            cluster_vec[cluster_idx].associated = true;
        }

        for (size_t i = 0; i < cluster_vec.size(); i++) {
            if (!cluster_vec[i].associated) {
                create_track(cluster_vec[i]);
            }
        }
    }
}

std::vector<std::vector<double>> Detector::create_cost_matrix() {
    std::vector<std::vector<double>> cost_matrix(track_vec.size(), std::vector<double>(cluster_vec.size(), 0.0));
    for (size_t i = 0; i < cluster_vec.size(); i++) {
        float cluster_x = (cluster_vec[i].min_point.x + cluster_vec[i].max_point.x) / 2;
        float cluster_y = (cluster_vec[i].min_point.y + cluster_vec[i].max_point.y) / 2;
        for (size_t j = 0; j < track_vec.size(); j++) {
            float dx = cluster_x - track_vec[j].state_post(0);
            float dy = cluster_y - track_vec[j].state_post(1);
            cost_matrix[j][i] = std::sqrt(dx*dx*5 + dy*dy);
        }
    }

    return cost_matrix;
}

void Detector::create_track(const Cluster& cluster) {
    int id = ++track_id;
    Eigen::VectorXf state(6);
    state << (cluster.min_point.x + cluster.max_point.x) / 2,
             (cluster.min_point.y + cluster.max_point.y) / 2,
             0.0,
             0.0,
             (cluster.max_point.x - cluster.min_point.x),
             (cluster.max_point.y - cluster.min_point.y);
    Eigen::Matrix<float,6,6> P = Eigen::MatrixXf::Identity(6,6);

    track_vec.emplace_back(id, state, P);
}

void Detector::predict() {
    for (size_t i = 0; i < track_vec.size(); i++) {
        track_vec[i].state_pri = F * track_vec[i].state_post;
        track_vec[i].P_pri = F * track_vec[i].P_post * F.transpose() + Q;
    }
}

void Detector::update() {
    for (size_t i = 0; i < track_vec.size(); i++) {
        if (!track_vec[i].associated) {
            track_vec[i].state_post = track_vec[i].state_pri;
            track_vec[i].P_post = track_vec[i].P_pri;
            continue;
        }
        Eigen::Matrix<float,6,4> K;
        K = track_vec[i].P_pri * H.transpose() * (H * track_vec[i].P_pri * H.transpose() + R).inverse();
        track_vec[i].state_post = track_vec[i].state_pri + K * (track_vec[i].meas - H * track_vec[i].state_pri);
        track_vec[i].P_post = (I - K * H) * track_vec[i].P_pri * (I - K * H).transpose() + K * R * K.transpose();
    }
}



void Detector::track_maintenance() {
    std::remove_if(track_vec.begin(), track_vec.end(), track_checker());
}

void Detector::tracking() {
    association();
    if (track_vec.empty()) {return;}
    track_maintenance();
    update();
    predict();
    std::cout << "track_vec.size() : " << track_vec.size() << std::endl;
}

void Detector::publish() {

    pcl::toROSMsg(*cloud_non_ground, cloud_output_msg);
    cloud_output_msg.header.stamp = ros::Time::now();
    cloud_output_msg.header.frame_id = FRAME_ID;
    pub.publish(cloud_output_msg);
    // msg_box.header.stamp = ros::Time::now();
    // msg_box.header.frame_id = FRAME_ID;
    // pub_box.publish(msg_box);
}

void Detector::process() {
    // plt_x.clear();
    // plt_y.clear();

    auto start = std::chrono::high_resolution_clock::now();

    downsample();
    ground_extract();
    dbscan();
    // bounding_box();
    // publish();

    // tracking();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> dt = end - start;

    std::cout << "end - start" << std::to_string(dt.count()) << std::endl;
    plt_x = {static_cast<double>(cluster_vec.size())};
    plt_y = {dt.count()};
    // if (iter > 750) {
    //     plt_x.push_back(static_cast<float>(cluster_vec.size()));
    //     plt_y.push_back(end-start);
    //     data[cluster_vec.size()].push_back(end-start);

    //     std::cout << "cluster_vec.size() : " << cluster_vec.size() << std::endl;

    //     std::cout << "Processing time : " << std::to_string(end-start) << std::endl;
    // }
    plt::scatter(plt_x, plt_y, 3.0, {{"color", "blue"}});
    plt::pause(0.1);
}

void Detector::sigintHandler() {
   
    // std::vector<double> avg;
    // float temp = 0;
    // for (const auto& elem : data) {
    //     double sum = std::accumulate(elem.second.begin(), elem.second.end(), 0.0);
    //     avg.push_back(sum / elem.second.size());
    // }
    // // Shut down ROS node

    // int i = 0;
    // std::vector<double> bar_x;
    // for (const auto& elem : data) {
    //    bar_x.push_back(static_cast<double>(elem.first));
    // }
    // plt::bar(bar_x, avg, "r", "-", 1.0, {{"label", "average"}});
    // plt::scatter(plt_x, plt_y, 10.0, {{"label", "Data Points"}, {"color", "blue"}});
    // plt::xlim(0.0,10.0);
    // plt::ylim(0.0,12.0);
    // plt::xlabel("Number of cluster");
    // plt::ylabel("time(s)");
    // plt::legend();
    // plt::title("Processing time for each number of cluster");
    // plt::pause(0.1);

    // double total_avg = 0;
    // for (const auto& elem : avg) {
    //     total_avg += elem;
    // }
    // std::cout << "total_avg : " << std::to_string(total_avg/avg.size()) << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    Detector detector;

    // ros::Rate loop_rate(10);
    plt::figure_size(1200,800);
    plt::xlim(-0.5,10.5);
    // plt::ylim(0.0,1.0);
    std::vector<int> x_ticks = {0,1,2,3,4,5,6,7,8,9,10};
    plt::xticks(x_ticks);
    plt::xlabel("Number of clusters");
    plt::ylabel("time(ms)");
    plt::title("Processing time for each number of cluster");

    while(ros::ok()) {
        detector.process();
        // loop_rate.sleep();
        ros::spinOnce();
    }

    
}