#include "filter.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

float roi_upper = -1.5;
float roi_lower = -3.0;
float ransac_threshold = 0.5;
float leaf_size_x = 0.3;
float leaf_size_y = 0.3;
float leaf_size_z = 0.3;
float outlier_mean = 0.7;
float outlier_std = 0.5;
float cluster_tolerance = 20;
float min_cluster_size = 0.5;
float max_cluster_size = 100;
double eps = 0.9;
int minPts = 10;
int maxPts = 100;
float bbox_min = 0.0001;
float bbox_max = 20;
float loop_rate_ = 0.5;

PointCloudProcess::PointCloudProcess() {
    cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 100);
    box_publisher = nh.advertise<visualization_msgs::MarkerArray>("box_topic", 100);
    debug_publisher = nh.advertise<std_msgs::String>("debug_topic", 100);
    debug_msg.data = "";
    sub = nh.subscribe("/carla/hero/LIDAR", 1000, &PointCloudProcess::pcl_msg_callback, this); // front -> LIDAR
    nh.getParam("/filter/loop_rate_", loop_rate_);

    // For downsampling
    cloud_original.reset(new pcl::PCLPointCloud2());
    cloud_downsampled.reset(new pcl::PCLPointCloud2());
    cloud_final.reset(new pcl::PCLPointCloud2());

    cloud_converted.reset(new pcl::PointCloud<pcl::PointXYZ>); 
    cloud_extracted.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_extracted_ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_roi.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // For ground segmentation
    coefficients.reset(new pcl::ModelCoefficients());
    inliers.reset(new pcl::PointIndices());

    // For outlier removal
    cloud_outlier_removed.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // For clustering
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    cloud_clustered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // DBSCAN
    colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // For bounding box
    bbox_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void PointCloudProcess::downsample() {
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_original);
    sor.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    sor.filter(*cloud_downsampled);
    pcl::fromPCLPointCloud2(*cloud_downsampled, *cloud_converted);
}

float getMinZValue(const pcl::PCLPointCloud2::Ptr& cloud) {
    // Ensure the point cloud is valid
    if (cloud->data.empty()) {
        throw std::runtime_error("Point cloud data is empty.");
    }

    // Get the point fields to locate the Z field
    int z_offset = -1;
    for (const auto& field : cloud->fields) {
        if (field.name == "z") {
            z_offset = field.offset;
            break;
        }
    }
    if (z_offset == -1) {
        throw std::runtime_error("Z field not found in point cloud.");
    }

    // Find the minimum Z value
    float min_z = std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud->width * cloud->height; ++i) {
        // Point offset in the data array
        size_t point_offset = i * cloud->point_step;

        // Extract the Z value
        float z;
        std::memcpy(&z, &cloud->data[point_offset + z_offset], sizeof(float));

        // Update the minimum Z value
        if (z < min_z) {
            min_z = z;
        }
    }

    return min_z;
}

float getMinYValue(const pcl::PCLPointCloud2::Ptr& cloud) {
    // Ensure the point cloud is valid
    if (cloud->data.empty()) {
        throw std::runtime_error("Point cloud data is empty.");
    }

    // Get the point fields to locate the y field
    int y_offset = -1;
    for (const auto& field : cloud->fields) {
        if (field.name == "y") {
            y_offset = field.offset;
            break;
        }
    }
    if (y_offset == -1) {
        throw std::runtime_error("y field not found in point cloud.");
    }

    // Find the minimum y value
    float min_y = std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud->width * cloud->height; ++i) {
        // Point offset in the data array
        size_t point_offset = i * cloud->point_step;

        // Extract the Z value
        float y;
        std::memcpy(&y, &cloud->data[point_offset + y_offset], sizeof(float));

        // Update the minimum Z value
        if (y < min_y) {
            min_y = y;
        }
    }

    return min_y;
}

float getMinXValue(const pcl::PCLPointCloud2::Ptr& cloud) {
    // Ensure the point cloud is valid
    if (cloud->data.empty()) {
        throw std::runtime_error("Point cloud data is empty.");
    }

    // Get the point fields to locate the x field
    int x_offset = -1;
    for (const auto& field : cloud->fields) {
        if (field.name == "x") {
            x_offset = field.offset;
            break;
        }
    }
    if (x_offset == -1) {
        throw std::runtime_error("x field not found in point cloud.");
    }

    // Find the minimum x value
    float min_x = std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud->width * cloud->height; ++i) {
        // Point offset in the data array
        size_t point_offset = i * cloud->point_step;

        // Extract the Z value
        float x;
        std::memcpy(&x, &cloud->data[point_offset + x_offset], sizeof(float));

        // Update the minimum Z value
        if (x < min_x) {
            min_x = x;
        }
    }

    return min_x;
}

void PointCloudProcess::pcl_msg_callback(const sensor_msgs::PointCloud2& msg) {
    pcl_conversions::toPCL(msg, *cloud_original);
    // float min_z = getMinZValue(cloud_original);   
    // float min_y = getMinYValue(cloud_original);
    // float min_x = getMinXValue(cloud_original);
    // std::cout << "Minimum Z value: " << min_z << std::endl;
    // std::cout << "Minimum y value: " << min_z << std::endl;
    // std::cout << "Minimum x value: " << min_z << std::endl;
}

void PointCloudProcess::roi() {
    ground_pass.setInputCloud(cloud_converted);
    ground_pass.setFilterFieldName("z");
    ground_pass.setFilterLimits(roi_lower,roi_upper);
    ground_pass.setNegative(false);
    ground_pass.filter(*ground_roi);
    non_ground_pass.setInputCloud(cloud_converted);
    non_ground_pass.setFilterFieldName("z");
    non_ground_pass.setFilterLimits(roi_lower,roi_upper);
    non_ground_pass.setNegative(true);
    non_ground_pass.filter(*cloud_extracted);
}

void PointCloudProcess::ground_extract() {

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_threshold);
    seg.setInputCloud(ground_roi);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(ground_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_extracted_ground);
    *cloud_extracted += (*cloud_extracted_ground);
}

void PointCloudProcess::outlier_removal() {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_extracted);
    sor.setMeanK(outlier_mean);
    sor.setStddevMulThresh(outlier_std);
    sor.filter(*cloud_outlier_removed);

    debug_msg.data += "outlier removed : " + std::to_string(cloud_extracted->points.size() - cloud_outlier_removed->points.size()) + " // ";
}

void PointCloudProcess::cluster() {
    tree->setInputCloud(cloud_outlier_removed);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance); // 2cm
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_outlier_removed);
    ec.extract(cluster_indices1);

    // std::cout << "Clustered number : " << cluster_indices1.size() << std::endl;
    cloud_clustered->points.clear();
    int color_index = 0;
    std::vector<int> color;
    for (const auto& indices : cluster_indices1) {
        // Generate a random color for each cluster
        // auto color = generateRandomColor();
        if (remainder(color_index,3) == 0) {
            color = {255,0,0};
        } else if (remainder(color_index,3) == 1) {
            color = {0,0,255};
        }
        else {
            color = {0,255,0};
        }

        for (const auto& idx : indices.indices) {
            pcl::PointXYZRGB point;
            point.x = cloud_extracted->points[idx].x;
            point.y = cloud_extracted->points[idx].y;
            point.z = cloud_extracted->points[idx].z;
            point.r = color[0];
            point.g = color[1];
            point.b = color[2];
            cloud_clustered->points.emplace_back(point);
        }
        color_index++;
        
    }
    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;
    cloud_clustered->is_dense = true;
}

void PointCloudProcess::dbscan() {
    points.clear();
    for (const auto &p : cloud_outlier_removed->points) {
        points.emplace_back(p);
    }

    // Set up KDTree for region query
    kdtree.setInputCloud(cloud_outlier_removed);

    int clusterId = 0;

    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].clusterID != -1) {
            continue;
        }

        std::vector<int> neighbors = region_query(i);
        if (neighbors.size() < minPts) {
            points[i].clusterID = 0; // mark as noise
        } else {
            ++clusterId;
            expand_cluster(i, neighbors, clusterId);
        }
    }
    create_colored_pcl();
}

std::vector<int> PointCloudProcess::region_query(int pointIdx) {
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ searchPoint = points[pointIdx].point;

    // Use KDTree to find neighbors within the eps radius
    std::vector<int> pointIdxRadiusSearch;
    if (kdtree.radiusSearch(searchPoint, eps, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            neighbors.push_back(pointIdxRadiusSearch[i]);
        }
    }
    return neighbors;
}

void PointCloudProcess::expand_cluster(int pointIdx, std::vector<int> &neighbors, int clusterId) {
    points[pointIdx].clusterID = clusterId;

    size_t i = 0;
    while (i < neighbors.size()) {
        int neighborIdx = neighbors[i];

        if (points[neighborIdx].clusterID == -1) { // if neighbor is not yet visited
            points[neighborIdx].clusterID = clusterId;

            std::vector<int> newNeighbors = region_query(neighborIdx);
            if (newNeighbors.size() >= minPts) {
                neighbors.insert(neighbors.end(), newNeighbors.begin(), newNeighbors.end());
            }
        }
        ++i;
    }
}

void PointCloudProcess::create_colored_pcl() {
    cluster_indices2.clear();
    (colored_cloud->points).clear();

    std::vector<std::vector<int>> clusters;
    int maxClusterID = 0;
    for (const auto &p : points) {
        if (p.clusterID > maxClusterID) {
            maxClusterID = p.clusterID;
        }
    }
    debug_msg.data += " cluster : " + std::to_string(maxClusterID) + "---";
    clusters.resize(maxClusterID + 1);

    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].clusterID != 0 && clusters[points[i].clusterID].size() < maxPts) {
            clusters[points[i].clusterID].push_back(i);
        }
    }

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, 255);
    // std::cout << "clusters.size() : "<< clusters.size() << std::endl;
    for (size_t i = 1; i < clusters.size(); ++i) {
        pcl::PointIndices cluster;
        uint8_t r = distribution(generator);
        uint8_t g = distribution(generator);
        uint8_t b = distribution(generator);

        float max_point_x = -1000;
        float min_point_x = 1000;
        float max_point_y = -1000;
        float min_point_y = 1000;
        float max_point_z = -1000;
        float min_point_z = 1000;

        for (const auto &idx : clusters[i]) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = points[idx].point.x;
            colored_point.y = points[idx].point.y;
            colored_point.z = points[idx].point.z;
            colored_point.r = r;
            colored_point.g = g;
            colored_point.b = b;

            colored_cloud->points.emplace_back(colored_point);
            cluster.indices.push_back(idx);

            max_point_x = std::max(max_point_x, colored_point.x);
            min_point_x = std::min(min_point_x, colored_point.x);
            max_point_y = std::max(max_point_y, colored_point.y);
            min_point_y = std::min(min_point_y, colored_point.y);
            min_point_z = std::min(min_point_z, colored_point.z);
            max_point_z = std::max(max_point_z, colored_point.z);

        }
        cluster_indices2.push_back(cluster);

        center_sphere(min_point_x, max_point_x, min_point_y, max_point_y, min_point_z, max_point_z, static_cast<int>(i));

    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;

}

void PointCloudProcess::bounding_box() {
    markers.markers.clear();
    int id = 0;

    for (const auto& indices : cluster_indices2) {
        if (indices.indices.empty()) {
            continue;
        }

        // Compute the bounding box
        pcl::PointXYZ min_point, max_point;
        pcl::copyPointCloud(*colored_cloud, indices.indices, *bbox_cloud);
        pcl::getMinMax3D(*bbox_cloud, min_point, max_point);

        // Compute the dimensions of the bounding box
        float scale_x = max_point.x - min_point.x;
        float scale_y = max_point.y - min_point.y;
        float scale_z = max_point.z - min_point.z;

        if (std::min({scale_x, scale_y, scale_z}) < bbox_min || std::max({scale_x, scale_y, scale_z}) > bbox_max) {continue;}

        // Create a marker for the bounding box
        visualization_msgs::Marker marker;
        marker.header.frame_id = "hero/LIDAR";
        marker.header.stamp = ros::Time::now();
        marker.ns = "bounding_boxes";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Compute the center of the bounding box
        marker.pose.position.x = (min_point.x + max_point.x) / 2.0;
        marker.pose.position.y = (min_point.y + max_point.y) / 2.0;
        marker.pose.position.z = (min_point.z + max_point.z) / 2.0;
        marker.pose.orientation.w = 1.0;

        // Set the color and transparency
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;

        // Compute the dimensions of the bounding box
        marker.scale.x = max_point.x - min_point.x;
        marker.scale.y = max_point.y - min_point.y;
        marker.scale.z = max_point.z - min_point.z;

        marker.frame_locked = true;

        markers.markers.push_back(marker);
    }
}

void PointCloudProcess::center_sphere(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, int id) {
    float scale_x = max_x - min_x;
    float scale_y = max_y - min_y;
    float scale_z = max_z - min_z;

    if (std::min({scale_x, scale_y, scale_z}) < bbox_min || std::max({scale_x, scale_y, scale_z}) > bbox_max) {return;}

    // Create a marker for the bounding box
    visualization_msgs::Marker marker;
    marker.header.frame_id = "hero/LIDAR";
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_boxes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // Compute the center of the bounding box
    marker.pose.position.x = (min_x + max_x) / 2.0;
    marker.pose.position.y = (min_y + max_y) / 2.0;
    marker.pose.position.z = (min_z + max_z) / 2.0;
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

    markers.markers.push_back(marker);
    
}

void PointCloudProcess::center_sphere() {
    markers.markers.clear();
    int id = 0;

    for (const auto& indices : cluster_indices2) {
        if (indices.indices.empty()) {
            continue;
        }

        // Compute the bounding box
        pcl::PointXYZ min_point, max_point;
        pcl::copyPointCloud(*colored_cloud, indices.indices, *bbox_cloud);
        pcl::getMinMax3D(*bbox_cloud, min_point, max_point);

        // Compute the dimensions of the bounding box
        float scale_x = max_point.x - min_point.x;
        float scale_y = max_point.y - min_point.y;
        float scale_z = max_point.z - min_point.z;

        if (std::min({scale_x, scale_y, scale_z}) < bbox_min || std::max({scale_x, scale_y, scale_z}) > bbox_max) {continue;}

        // Create a marker for the bounding box
        visualization_msgs::Marker marker;
        marker.header.frame_id = "hero/LIDAR";
        marker.header.stamp = ros::Time::now();
        marker.ns = "bounding_boxes";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Compute the center of the bounding box
        marker.pose.position.x = (min_point.x + max_point.x) / 2.0;
        marker.pose.position.y = (min_point.y + max_point.y) / 2.0;
        marker.pose.position.z = (min_point.z + max_point.z) / 2.0;
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

        markers.markers.push_back(marker);
    }

    
}

void PointCloudProcess::convert_and_publish() {
    // pcl::toPCLPointCloud2(*cloud_extracted, *cloud_final);
    pcl::toROSMsg(*colored_cloud, cloud_msg);
    cloud_msg.header.frame_id = "hero/LIDAR";
    
    box_publisher.publish(markers);
    cloud_publisher.publish(cloud_msg);
    debug_publisher.publish(debug_msg);
    markers.markers.clear();
}

void PointCloudProcess::process() {
    nh.getParam("filter/roi_upper", roi_upper);
    nh.getParam("filter/roi_lower", roi_lower);
    nh.getParam("filter/ransac_threshold", ransac_threshold);
    nh.getParam("filter/leaf_size_x", leaf_size_x);
    nh.getParam("filter/leaf_size_y", leaf_size_y);
    nh.getParam("filter/leaf_size_z", leaf_size_z);
    nh.getParam("filter/outlier_mean", outlier_mean);
    nh.getParam("filter/outlier_std", outlier_std);
    nh.getParam("filter/cluster_tolerance", cluster_tolerance);
    nh.getParam("filter/min_cluster_size", min_cluster_size);
    nh.getParam("filter/max_cluster_size", max_cluster_size);
    nh.getParam("filter/eps", eps);
    nh.getParam("filter/minPts", minPts);
    nh.getParam("filter/maxPts", maxPts);
    nh.getParam("filter/bbox_min", bbox_min);
    nh.getParam("filter/bbox_max", bbox_max);
    downsample();
    roi();
    ground_extract();
    outlier_removal();
    dbscan();
    // bounding_box();
    // center_sphere();
    convert_and_publish();

}


int main (int argc, char** argv) {

    ros::init(argc, argv, "filter_node");

    PointCloudProcess process_node;
    ros::Rate loop_rate(loop_rate_);


    while(ros::ok()) {
        process_node.process();
        ros::spinOnce();
        loop_rate.sleep();
  }

  return 0;
}