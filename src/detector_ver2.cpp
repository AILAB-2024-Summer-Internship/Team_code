#include "detector_ver2.hpp"

// Carla connection parameters
std::string FRAME_ID = "hero/front";
std::string CLOUD_PUB_NAME = "filtered_cloud";
std::string CLOUD_SUB_NAME = "/carla/hero/front";
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
    // box_pub = nh.advertise<visualization_msgs::MarkerArray>(BOX_PUB_NAME,100);
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

    // Projection
    cloud_2d.reset(new pcl::PointCloud<pcl::PointXY>);
    plt_x.reserve(sizeof(double) * 40000);
    plt_y.reserve(sizeof(double) * 40000);

    // Clustering
    cluster_vec.reserve(sizeof(Cluster) * 20);
    cloud_clustered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    indices_temp.reset(new pcl::PointIndices);


    // Bounding box
    box_x.reserve(5);
    box_y.reserve(5);

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

void Detector::projection() {
    cloud_2d->width = cloud_non_ground->width;
    cloud_2d->height = cloud_non_ground->height;
    cloud_2d->is_dense = cloud_non_ground->is_dense;
    cloud_2d->points.resize(cloud_non_ground->points.size());

    if (!plt_x.empty()) {
        plt_x.clear();
    }
    if (!plt_y.empty()) {
        plt_y.clear();
    }

    for (size_t i = 0; i < cloud_non_ground->points.size(); i++) {
        cloud_2d->points[i].x = cloud_non_ground->points[i].x;
        cloud_2d->points[i].y = cloud_non_ground->points[i].y;

        plt_x.push_back(cloud_non_ground->points[i].x);
        plt_y.push_back(cloud_non_ground->points[i].y);
    }

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
    if (!cluster_vec.size() > 0) {return;}
    plt::clf();  // Clear the current plot
    plt::scatter(plt_x, plt_y, 1.0);  // Plot the points
    plt::xlim(-10,50);
    plt::ylim(-10,10);
    
    for (size_t i = 0; i < cluster_vec.size(); i++) {
        box_x = {cluster_vec[i].min_point.x, cluster_vec[i].max_point.x, cluster_vec[i].max_point.x, cluster_vec[i].min_point.x, cluster_vec[i].min_point.x};
        box_y = {cluster_vec[i].min_point.y, cluster_vec[i].min_point.y, cluster_vec[i].max_point.y, cluster_vec[i].max_point.y, cluster_vec[i].min_point.y};
        plt::plot(box_x,box_y,"r--");

      

    }
    plt::pause(0.01);
}


void Detector::color_cloud() {

    if (!cloud_clustered->points.empty()) {
        cloud_clustered->points.clear();
    }

    if (cluster_vec.size() == 0) {return;}

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, 255);
    for (size_t i = 0; i < cluster_vec.size(); i++) {

        uint8_t r = distribution(generator);
        uint8_t g = distribution(generator);
        uint8_t b = distribution(generator);

        for (const auto& indices : cluster_vec[i].indices->indices) {
            
            pcl::PointXYZRGB colored_point;
            colored_point.x = (*cloud_non_ground).points[indices].x;
            colored_point.y = (*cloud_non_ground).points[indices].y;
            colored_point.z = (*cloud_non_ground).points[indices].z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;
            cloud_clustered->points.emplace_back(colored_point);
        }

    }

}

void Detector::publish() {

    pcl::toROSMsg(*cloud_clustered, cloud_output_msg);
    cloud_output_msg.header.stamp = ros::Time::now();
    cloud_output_msg.header.frame_id = FRAME_ID;
    pub.publish(cloud_output_msg);
}

void Detector::process() {

    double start = ros::Time::now().toSec();

    downsample();
    ground_extract();
    projection();
    dbscan();
    // bounding_box();
    double end = ros::Time::now().toSec();
    color_cloud();
    // publish();

    std::cout << "Processing time : " << std::to_string(end-start) << std::endl;

    std::cout << "cluster_vec.size() : " << cluster_vec.size() << std::endl;
}

// ----------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    Detector detector;
    plt::figure_size(1200, 600);
    // ros::Rate loop_rate(10);
    while(ros::ok()) {
        detector.process();
        // loop_rate.sleep();
        ros::spinOnce();
    }
}