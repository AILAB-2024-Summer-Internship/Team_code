#include <Eigen/Dense>
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
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "matplotlibcpp.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <rviz_visual_tools/rviz_visual_tools.h>



class Detector {
public:
    Detector();
    ~Detector();
private:

    // ROS base
    ros::NodeHandle nh;
    ros::Publisher pub;
    // ros::Publisher box_pub;
    ros::Subscriber sub;

    // ROS msgs
    sensor_msgs::PointCloud2 cloud_output_msg;

    // PointCloud extractor (indices -> pointcloud)
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    pcl::ExtractIndices<pcl::PointXY> extractor_2d;

    // Downsampling
    pcl::PCLPointCloud2::Ptr cloud_original;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original_; // Input
    pcl::PCLPointCloud2::Ptr cloud_downsampled; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_; // Output
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    
    // ROI
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi;
    pcl::PassThrough<pcl::PointXYZ> roi_filter;

    // RANSAC
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ModelCoefficients::Ptr ransac_coefficients;
    pcl::PointIndices::Ptr ransac_inliers;

    // 2D projection
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d;
    std::vector<double> plt_x, plt_y;

    // Clustering
    struct Cluster {
        int id;
        pcl::PointIndices::Ptr indices;
        pcl::PointXYZ min_point, max_point, position;
        Eigen::Matrix3f rotation_matrix;
        Cluster(int id_, std::set<int>& indices_set, pcl::PointXYZ& min_point_, pcl::PointXYZ& max_point_, pcl::PointXYZ& position_, Eigen::Matrix3f& rotation_matrix_) {
            id = id_;
            indices.reset(new pcl::PointIndices);
            indices->indices.assign(indices_set.begin(), indices_set.end());
            min_point = min_point_;
            max_point = max_point_;
            position = position_;
            rotation_matrix = rotation_matrix_;
        }
    };
    std::vector<Cluster> cluster_vec;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered;
    std::vector<int> visit;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp;
    pcl::PointIndices::Ptr indices_temp;

    // Moment of inertia
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;

    // PCA
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_temp_2d;
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_temp_2d_;
    Eigen::Vector2f mean;
    Eigen::Matrix2f covariance;
    Eigen::Vector2f eigen_values;
    Eigen::Matrix2f eigen_vectors;

    // Bounding box
    Eigen::Vector2f p1;
    Eigen::Vector2f p2;
    Eigen::Vector2f p3;
    Eigen::Vector2f p4;
    std::vector<double> box_x;
    std::vector<double> box_y;


public:
    void get_param();
    void callback(const sensor_msgs::PointCloud2& msg);
    void downsample();
    void ground_extract();

    void projection();
    void get_mean();
    void get_covariance();
    void pca();

    void reset_dbscan();
    void create_cluster(int cluster_id, std::vector<int>& neighbor_vec);
    std::vector<int> region_query(int query_point_idx);
    void expand_cluster(std::vector<int>& neighbor_vec);
    void dbscan();

    void bounding_box();
    void color_cloud();
    void publish();
    void process();
};
