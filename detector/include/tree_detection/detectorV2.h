#ifndef DETECTORV2_H_
#define DETECTORV2_H_

#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <thread>

class detectorV2
{
private:
    ros::Subscriber cloudSub_;
    ros::Publisher cloudPub_;
    ros::Publisher boundingBoxPub_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeNaNs_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);
    void callback_(const sensor_msgs::PointCloud2::ConstPtr& msg);
    std::pair<float, float> computeThreshold_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);
    std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> splitCloud_(const unsigned int cores, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, std::pair<float, float> limits);
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeNoise_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);
    std::pair<pcl::PointCloud<pcl::PointXYZL>::Ptr, const int> instanceSegmentation_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);
    pcl::PointCloud<pcl::PointXYZL>::Ptr instanceExtractor_(const pcl::PointCloud<pcl::PointXYZL>::Ptr& inputCloud, const int ID);
    std::pair<pcl::PointXYZL, pcl::PointXYZL> computeAABB_(const pcl::PointCloud<pcl::PointXYZL>::Ptr& inputCloud);
    visualization_msgs::Marker genMarker_(pcl::PointXYZL minPt, pcl::PointXYZL maxPt, const unsigned int id, const ros::Time stamp);
public:
    detectorV2(ros::NodeHandle *nh);
    ~detectorV2();
};

#endif