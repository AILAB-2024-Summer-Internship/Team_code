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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>


class PointCloudProcess {
public:
    PointCloudProcess();
private:

    // Ros setup
    ros::NodeHandle nh;
    ros::Publisher cloud_publisher;
    ros::Publisher box_publisher;
    ros::Publisher debug_publisher;
    ros::Subscriber sub;

    // Downsampling
    pcl::PCLPointCloud2::Ptr cloud_original;
    pcl::PCLPointCloud2::Ptr cloud_downsampled;

    // Ground extracting
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_converted;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_roi;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted_ground;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PassThrough<pcl::PointXYZ> ground_pass;
    pcl::PassThrough<pcl::PointXYZ> non_ground_pass;

    // Outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier_removed;

    // Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    std::vector<pcl::PointIndices> cluster_indices1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered;

    // DBSCAN
    struct PointXYZCluster {
        pcl::PointXYZ point;
        int clusterID;

        PointXYZCluster(const pcl::PointXYZ &p) : point(p), clusterID(-1) {}
    };
    double eps;
    int minPts;
    std::vector<PointXYZCluster> points;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    std::vector<pcl::PointIndices> cluster_indices2;

    // Bounding box
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud;

    // Publishing
    pcl::PCLPointCloud2::Ptr cloud_final;
    sensor_msgs::PointCloud2 cloud_msg;
    visualization_msgs::MarkerArray markers;
    std_msgs::String debug_msg;

public:
    void pcl_msg_callback(const sensor_msgs::PointCloud2& msg);
    void downsample();
    void roi();
    void ground_extract();
    void outlier_removal();
    void cluster();
    std::vector<int> region_query(int pointIdx);
    void create_colored_pcl();
    void expand_cluster(int pointIdx, std::vector<int> &neighbors, int clusterId);
    void dbscan();
    void bounding_box();
    void center_sphere();
    void center_sphere(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, int id);
    void convert_and_publish();
    void process();
};