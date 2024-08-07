#include "filter_ver_2.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>

// Parameters
float roi_upper = 0.1;
float roi_lower = -1.35;
float roi_front = 2.0;
float roi_back = -2.0;
float roi_left = -1.0;
float roi_right = 1.0;
float ransac_threshold = 0.2;
float leaf_size_x = 0.3;
float leaf_size_y = 0.3;
float leaf_size_z = 0.3;

float min_cluster_vol = 3;
float max_cluster_vol = 100;
float max_cluster_dist = 20;
float eps = 0.8;
int minPts = 7;
int maxPts = 100;
float bbox_min = 0.01;
float bbox_max = 20;
float loop_rate_ = 10;


class cluster_check {

public:
    bool operator()(PointCloudProcess::ClusterStruct& cluster_struct) const {
        float dist = std::sqrt(std::pow(cluster_struct.center[0], 2) + std::pow(cluster_struct.center[1], 2) + std::pow(cluster_struct.center[2], 2));
        float volume = cluster_struct.size[0] * cluster_struct.size[1] * cluster_struct.size[2];

        if (dist > max_cluster_dist) {return true;}
        else if (volume > max_cluster_vol || volume < min_cluster_vol) {return true;}
        return false;
    }
};


PointCloudProcess::PointCloudProcess() {

    // cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 100);
    cloud_2_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_2_topic", 100);
    cloud_3_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_3_topic", 100);
    cloud_4_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_4_topic", 100);
    cloud_5_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_5_topic", 100);
    // box_publisher = nh.advertise<visualization_msgs::MarkerArray>("box_topic", 100);
    ego_box_publisher = nh.advertise<visualization_msgs::MarkerArray>("ego_box_topic", 100);
    sub = nh.subscribe("/carla/hero/LIDAR", 1000, &PointCloudProcess::pcl_msg_callback, this); // front -> LIDAR
    nh.getParam("/filter/loop_rate_", loop_rate_);

    // For downsampling
    cloud_original.reset(new pcl::PCLPointCloud2());
    cloud_1.reset(new pcl::PCLPointCloud2());

    cloud_2.reset(new pcl::PointCloud<pcl::PointXYZ>); 
    cloud_3.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_4.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_5.reset(new pcl::PointCloud<pcl::PointXYZ>);
    hero_filter_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // For ground segmentation
    coefficients.reset(new pcl::ModelCoefficients());
    inliers.reset(new pcl::PointIndices());

    // For cluster struct
    // cluster_vec.reserve(sizeof(ClusterStruct) * 20);
    cluster_vec_ptr.reset(new std::vector<ClusterStruct>);

    // DBSCAN
    colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    points_ptr.reset(new std::vector<PointXYZCluster>);

    // For center sphere
    cloud_markers.markers.reserve(sizeof(visualization_msgs::Marker) * 20);

    // For rviz
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("hero/front", "/box_topic"));
}

void PointCloudProcess::downsample() {
    ROS_INFO("DOWNSAMPLE START");
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_original);
    sor.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    sor.filter(*cloud_1);
    pcl::fromPCLPointCloud2(*cloud_1, *cloud_2);
    ROS_INFO("DOWNMSAMPLE OK");
}

void PointCloudProcess::pcl_msg_callback(const sensor_msgs::PointCloud2& msg) {
    pcl_conversions::toPCL(msg, *cloud_original);
}

void PointCloudProcess::get_info() {
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cloud_2, min_point, max_point);
    std::cout << "min_point : " << min_point.x << ", " << min_point.y << ", " << min_point.z << std::endl;
    std::cout << "max_point : " << max_point.x << ", " << max_point.y << ", " << max_point.z << std::endl;
}

void PointCloudProcess::roi() {

    ROS_INFO("ROI START");
    roi_filter.setInputCloud(cloud_3);
    roi_filter.setFilterFieldName("z");
    roi_filter.setFilterLimits(roi_lower,roi_upper);
    roi_filter.setNegative(false);
    roi_filter.filter(*cloud_4);

    hero_filter.setInputCloud(cloud_4);
    hero_filter.setFilterFieldName("x");
    hero_filter.setFilterLimits(-2.0,2.0);
    hero_filter.setNegative(true);
    hero_filter.filter(*hero_filter_cloud);

    hero_filter.setInputCloud(hero_filter_cloud);
    hero_filter.setFilterFieldName("y");
    hero_filter.setFilterLimits(-1.0,1.0);
    hero_filter.setNegative(true);
    hero_filter.filter(*cloud_5);
    ROS_INFO("ROI OK");

}

void PointCloudProcess::ground_extract() {

    ROS_INFO("GROUND EXTRACT START");
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_threshold);
    seg.setInputCloud(cloud_2);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_2);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_3);
    ROS_INFO("GROUND EXTRACT OK");
}


void PointCloudProcess::create_cluster(std::pair<std::vector<int>, std::vector<float>>& result, int clusterId) {

    ROS_INFO("CREATE_CLUSTER START");
    if (result.first.empty() || result.second.empty()) {return;}
    std::vector<float> center = {((result.second)[0] + (result.second)[1]) / 2, ((result.second)[2] + (result.second)[3]) / 2, ((result.second)[4] + (result.second)[5]) / 2};
    std::vector<float> size = {((result.second)[1] - (result.second)[0]), ((result.second)[3] - (result.second)[2]), ((result.second)[5] - (result.second)[4])};
    cluster_vec_ptr->emplace_back(clusterId, result.first, center, size);
    ROS_INFO("CREATE_CLUSTER OK");
}


void PointCloudProcess::dbscan() {

    ROS_INFO("DBSCAN START");
    points_ptr.reset(new std::vector<PointXYZCluster>);
    cluster_vec_ptr.reset(new std::vector<ClusterStruct>);

    for (const auto &p : cloud_5->points) {
        points_ptr->emplace_back(p);
    }

    // Set up KDTree for region query
    kdtree.setInputCloud(cloud_5);

    int clusterId = 0;

    for (size_t i = 0; i < points_ptr->size(); ++i) {
        if ((*points_ptr)[i].clusterID != -1) {
            continue;
        }

        std::pair<std::vector<int>, std::vector<float>> neighbors = region_query(i);
        if (neighbors.first.size() < minPts) {
            (*points_ptr)[i].clusterID = 0; // mark as noise
        } else {
            ++clusterId;
            std::pair<std::vector<int>, std::vector<float>> result = expand_cluster(i, neighbors, clusterId);

            create_cluster(result, clusterId);
        }
    }
    ROS_INFO("DBSCAN OK");
}

std::pair<std::vector<int>, std::vector<float>> PointCloudProcess::region_query(int pointIdx) {
    ROS_INFO("REGION QUERY START");
    std::pair<std::vector<int>, std::vector<float>> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ searchPoint = (*points_ptr)[pointIdx].point;
    float x_min = 1000;
    float x_max = -1000;
    float y_min = 1000;
    float y_max = -1000;
    float z_min = 1000;
    float z_max = -1000;

    // Use KDTree to find neighbors within the eps radius
    std::vector<int> pointIdxRadiusSearch;
    if (kdtree.radiusSearch(searchPoint, eps, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            neighbors.first.push_back(pointIdxRadiusSearch[i]);
            x_min = std::min(x_min, (*points_ptr)[pointIdxRadiusSearch[i]].point.x);
            x_max = std::max(x_max, (*points_ptr)[pointIdxRadiusSearch[i]].point.x);
            y_min = std::min(y_min, (*points_ptr)[pointIdxRadiusSearch[i]].point.y);
            y_max = std::max(y_max, (*points_ptr)[pointIdxRadiusSearch[i]].point.y);
            z_min = std::min(z_min, (*points_ptr)[pointIdxRadiusSearch[i]].point.z);
            z_max = std::max(z_max, (*points_ptr)[pointIdxRadiusSearch[i]].point.z);
        }
    }
    std::vector<float> min_max = {x_min, x_max, y_min, y_max, z_min, z_max};
    neighbors.second  = min_max;
    ROS_INFO("REGION QUERY OK");
    return neighbors;
}

std::pair<std::vector<int>, std::vector<float>> PointCloudProcess::region_query(int pointIdx, std::vector<float>& min_max) {
    ROS_INFO("REGION QUERY MIN MAX START");
    std::pair<std::vector<int>, std::vector<float>> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ searchPoint = (*points_ptr)[pointIdx].point;
    float x_min = min_max[0];
    float x_max = min_max[1];
    float y_min = min_max[2];
    float y_max = min_max[3];
    float z_min = min_max[4];
    float z_max = min_max[5];

    // Use KDTree to find neighbors within the eps radius
    std::vector<int> pointIdxRadiusSearch;
    if (kdtree.radiusSearch(searchPoint, eps, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            neighbors.first.push_back(pointIdxRadiusSearch[i]);
            x_max = std::max(x_max, (*points_ptr)[pointIdxRadiusSearch[i]].point.x);
            x_min = std::min(x_min, (*points_ptr)[pointIdxRadiusSearch[i]].point.x);
            y_max = std::max(y_max, (*points_ptr)[pointIdxRadiusSearch[i]].point.y);
            y_min = std::min(y_min, (*points_ptr)[pointIdxRadiusSearch[i]].point.y);
            z_max = std::max(z_max, (*points_ptr)[pointIdxRadiusSearch[i]].point.z);
            z_min = std::min(z_min, (*points_ptr)[pointIdxRadiusSearch[i]].point.z);
        }
    }
    std::vector<float> min_max_ = {x_min, x_max, y_min, y_max, z_min, z_max};
    neighbors.second  = min_max_;

    ROS_INFO("REGION QUERY MIN MAX OK");
    return neighbors;
}

std::pair<std::vector<int>, std::vector<float>> PointCloudProcess::expand_cluster(int pointIdx, std::pair<std::vector<int>, std::vector<float>>& neighbors, int clusterId) {
    ROS_INFO("EXPAND CLUSTER START");
    std::pair<std::vector<int>, std::vector<float>> result;
    (*points_ptr)[pointIdx].clusterID = clusterId;

    size_t i = 0;
    while (i < neighbors.first.size()) {
        int neighborIdx = neighbors.first[i];

        if ((*points_ptr)[neighborIdx].clusterID == -1) { // if neighbor is not yet visited
            (*points_ptr)[neighborIdx].clusterID = clusterId;

            std::pair<std::vector<int>, std::vector<float>> newNeighbors = region_query(neighborIdx, neighbors.second);
            if (newNeighbors.first.size() >= minPts) {
                neighbors.first.insert(neighbors.first.end(), newNeighbors.first.begin(), newNeighbors.first.end());
            }
            result.second = newNeighbors.second;
        }
        ++i;
    }

    result.first = neighbors.first;

    ROS_INFO("EXPAND CLUSTER OK");
    return result;
}


void PointCloudProcess::clear_marker_array() {

    visualization_msgs::Marker clear_marker;
    clear_marker.id = 0;
    clear_marker.ns = "bounding_boxes";
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    cloud_markers.markers.push_back(clear_marker);
}


void PointCloudProcess::reassign_cluster() {

    cluster_vec_ptr->erase(std::remove_if(cluster_vec_ptr->begin(), cluster_vec_ptr->end(), cluster_check()), cluster_vec_ptr->end());
    int new_id = 1;
    for (auto& elem : *cluster_vec_ptr) {
        elem.ID = new_id;
        ++new_id;
    }
}

void PointCloudProcess::center_sphere() {
    // markers.markers.clear();
    ROS_INFO("CENTER SPHERE START");
    // markers_ptr.reset(new visualization_msgs::MarkerArray);
    int id = 0;

    for (const auto& elem: *cluster_vec_ptr) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = "hero/LIDAR";
        marker.header.stamp = ros::Time::now();
        marker.ns = "bounding_boxes";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Compute the center of the bounding box
        marker.pose.position.x = elem.center[0];
        marker.pose.position.y = elem.center[1];
        marker.pose.position.z = elem.center[2];
        marker.pose.orientation.w = 1.0;

        // Set the color and transparency
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // Compute the dimensions of the bounding box
        marker.scale.x = 0.5f;
        marker.scale.y = 0.5f;
        marker.scale.z = 0.5f;

        marker.frame_locked = true;

        cloud_markers.markers.push_back(marker);

    }
    ROS_INFO("CENTER SPHERE OK");
    clear_marker_array();


}


void PointCloudProcess::ego_box() {


    visualization_msgs::Marker marker1;
    marker1.header.frame_id = "hero/LIDAR";
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "ego_boxes";
    marker1.id = 1;
    marker1.type = visualization_msgs::Marker::CUBE;
    marker1.action = visualization_msgs::Marker::ADD;

    // Compute the center of the bounding box
    marker1.pose.position.x = 0.0;
    marker1.pose.position.y = 0.0;
    marker1.pose.position.z = 0.0;
    marker1.pose.orientation.w = 1.0;

    // Set the color and transparency
    marker1.color.r = 1.0f;
    marker1.color.g = 0.0f;
    marker1.color.b = 0.0f;
    marker1.color.a = 1.0f;

    // Compute the dimensions of the bounding box
    marker1.scale.x = roi_front - roi_back;
    marker1.scale.y = roi_right - roi_left;
    marker1.scale.z = 0.5f;

    marker1.frame_locked = true;

    ego_markers.markers.push_back(marker1);
}

void PointCloudProcess::convert_and_publish() {
    // pcl::toPCLPointCloud2(*cloud_extracted, *cloud_final);
    // pcl::toROSMsg(*cloud_clustered, cloud_msg);
    // cloud_msg.header.frame_id = "hero/front";
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
    std::cout << "cluster_vec.size() : " << cluster_vec_ptr->size() << std::endl;
    pcl::toROSMsg(*cloud_2, cloud_2_msg);
    pcl::toROSMsg(*cloud_3, cloud_3_msg);
    pcl::toROSMsg(*cloud_4, cloud_4_msg);
    pcl::toROSMsg(*cloud_5, cloud_5_msg);
    cloud_2_msg.header.frame_id = "hero/LIDAR";
    cloud_3_msg.header.frame_id = "hero/LIDAR";
    cloud_4_msg.header.frame_id = "hero/LIDAR";
    cloud_5_msg.header.frame_id = "hero/LIDAR";
    
    // box_publisher.publish(cloud_markers);
    ego_box_publisher.publish(ego_markers);
    // markers.markers.clear();
    // cloud_publisher.publish(cloud_msg);

    cloud_2_publisher.publish(cloud_2_msg);
    cloud_3_publisher.publish(cloud_3_msg);
    cloud_4_publisher.publish(cloud_4_msg);
    cloud_5_publisher.publish(cloud_5_msg);
    visual_tools->publishMarkers(cloud_markers);
    visual_tools->trigger();
}

void PointCloudProcess::clear_vec() {
    cloud_markers.markers.clear();
}


void PointCloudProcess::process() {
    nh.getParam("filter_test_2/roi_upper", roi_upper);
    nh.getParam("filter_test_2/roi_lower", roi_lower);
    nh.getParam("filter_test_2/roi_front", roi_front);
    nh.getParam("filter_test_2/roi_back", roi_back);
    nh.getParam("filter_test_2/roi_left", roi_left);
    nh.getParam("filter_test_2/roi_right", roi_right);

    nh.getParam("filter_test_2/ransac_threshold", ransac_threshold);
    nh.getParam("filter_test_2/leaf_size_x", leaf_size_x);
    nh.getParam("filter_test_2/leaf_size_y", leaf_size_y);
    nh.getParam("filter_test_2/leaf_size_z", leaf_size_z);

    nh.getParam("filter_test_2/min_cluster_vol", min_cluster_vol);
    nh.getParam("filter_test_2/max_cluster_vol", max_cluster_vol);
    nh.getParam("filter_test_2/max_cluster_dist", max_cluster_dist);
    nh.getParam("filter_test_2/eps", eps);
    nh.getParam("filter_test_2/minPts", minPts);
    nh.getParam("filter_test_2/maxPts", maxPts);
    nh.getParam("filter_test_2/bbox_min", bbox_min);
    nh.getParam("filter_test_2/bbox_max", bbox_max);
    
    downsample();
    ground_extract();
    roi();
    dbscan();
    reassign_cluster();
    center_sphere();
    ego_box();
    convert_and_publish();
    clear_vec();

    ROS_INFO("CLEARED!");

}


int main (int argc, char** argv) {

    ros::init(argc, argv, "filter_node");

    PointCloudProcess process_node;
    ros::Rate loop_rate(loop_rate_);

    while(ros::ok()) {
        process_node.process();
        // loop_rate.sleep();
        ros::spinOnce();
    }

  return 0;
}