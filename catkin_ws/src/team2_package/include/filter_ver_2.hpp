#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


class PointCloudProcess {
public:
    PointCloudProcess();
private:

    // Ros setup
    ros::NodeHandle nh;
    ros::Publisher cloud_publisher;
    ros::Publisher cloud_2_publisher;
    ros::Publisher cloud_3_publisher;
    ros::Publisher cloud_4_publisher;
    ros::Publisher cloud_5_publisher;
    // ros::Publisher box_publisher;
    ros::Publisher ego_box_publisher;
    ros::Subscriber sub;

    // Downsampling
    pcl::PCLPointCloud2::Ptr cloud_original;
    pcl::PCLPointCloud2::Ptr cloud_1;

    // ROI + Ground extracting
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hero_filter_cloud;
  

    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PassThrough<pcl::PointXYZ> roi_filter;
    pcl::PassThrough<pcl::PointXYZ> hero_filter;
    pcl::PassThrough<pcl::PointXYZ> non_ground_pass;

    // Cluster struct
    struct ClusterStruct {
        int ID;
        std::vector<int> indices;
        std::vector<float> center;
        std::vector<float> size;

        ClusterStruct(int id, std::vector<int> indices, std::vector<float> center, std::vector<float> size) : ID(id), indices(indices), center(center), size(size) {}
    };
    std::unique_ptr<std::vector<ClusterStruct>> cluster_vec_ptr;
    friend class cluster_check;



    // DBSCAN
    struct PointXYZCluster {
        pcl::PointXYZ point;
        int clusterID;

        PointXYZCluster(const pcl::PointXYZ &p) : point(p), clusterID(-1) {}
    };

    std::unique_ptr<std::vector<PointXYZCluster>> points_ptr;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    std::vector<pcl::PointIndices> cluster_indices2;

    // Bounding box


    // Publishing
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2 cloud_2_msg;
    sensor_msgs::PointCloud2 cloud_3_msg;
    sensor_msgs::PointCloud2 cloud_4_msg;
    sensor_msgs::PointCloud2 cloud_5_msg;
    visualization_msgs::MarkerArray cloud_markers;
    // std::unique_ptr<visualization_msgs::MarkerArray> markers_ptr;
    visualization_msgs::MarkerArray ego_markers;

    // For rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools;

public:
    void pcl_msg_callback(const sensor_msgs::PointCloud2& msg);
    void downsample();
    void get_info();
    void roi();
    void ground_extract();
    void create_cluster(std::pair<std::vector<int>, std::vector<float>>& result, int clusterId);
    std::pair<std::vector<int>, std::vector<float>> region_query(int pointIdx);
    std::pair<std::vector<int>, std::vector<float>> region_query(int pointIdx, std::vector<float>& min_max);
    void create_colored_pcl(int maxClusterID);
    std::pair<std::vector<int>, std::vector<float>> expand_cluster(int pointIdx, std::pair<std::vector<int>, std::vector<float>> &neighbors, int clusterId);
    void dbscan();
    void reassign_cluster();
    void center_sphere();
    void center_sphere(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, int id);
    void ego_box();
    void clear_vec();
    void clear_marker_array();
    void convert_and_publish();
    void process();
};

