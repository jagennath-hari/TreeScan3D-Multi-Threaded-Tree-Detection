#include "detectorV2.h"

//----------Class----------//

detectorV2::detectorV2(ros::NodeHandle *nh)
{
    this->cloudSub_ = nh->subscribe("/cam660_node/points", 1, &detectorV2::callback_, this);
    this->cloudPub_ = nh->advertise<sensor_msgs::PointCloud2>("/tree_cloud", 1);
    this->boundingBoxPub_ = nh->advertise<visualization_msgs::Marker>("/bounding_box", 1);
}

detectorV2::~detectorV2()
{
}

pcl::PointCloud<pcl::PointXYZI>::Ptr detectorV2::removeNaNs_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::removeNaNFromPointCloud(*inputCloud, *filtered_cloud, indices);
    
    return filtered_cloud;
}

std::pair<float, float> detectorV2::computeThreshold_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D(*inputCloud, minPt, maxPt);

    return std::make_pair((maxPt.x + minPt.x)/2, (maxPt.z + minPt.z)/2);
}

std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> detectorV2::splitCloud_(const unsigned int cores, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, std::pair<float, float> limits)
{
    //TODO 
    // 1) Automate #cores, variables and loop
    // 2) Optimize code
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud4(new pcl::PointCloud<pcl::PointXYZI>);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> sub_clouds = {sub_cloud1, sub_cloud2, sub_cloud3, sub_cloud4};

    for (const auto& point : inputCloud->points)
    {
        int index = 0;
        if (point.x >= limits.first) index += 1;
        if (point.z >= limits.second) index += 2;
        sub_clouds[index]->points.push_back(point);
    }

    return std::make_tuple(sub_clouds[0], sub_clouds[1], sub_clouds[2], sub_clouds[3]);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr detectorV2::removeNoise_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    return cloud_filtered;
} 


std::pair<pcl::PointCloud<pcl::PointXYZL>::Ptr, const int> detectorV2::instanceSegmentation_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (inputCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.1);
    ec.setMinClusterSize (75);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (inputCloud);
    ec.extract (cluster_indices);

    unsigned int instances = 0;
    pcl::PointCloud<pcl::PointXYZL>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZL>);
    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_cluster_labelled (new pcl::PointCloud<pcl::PointXYZL>);
        for (const auto& idx : cluster.indices)
        {
            cloud_cluster->push_back((*inputCloud)[idx]);
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            pcl::copyPointCloud(*cloud_cluster , *cloud_cluster_labelled);
            for (pcl::PointCloud<pcl::PointXYZL>::iterator it = cloud_cluster_labelled->begin(); it != cloud_cluster_labelled->end(); ++it) it->label = instances;
        }
        *output_cloud =  *output_cloud + *cloud_cluster_labelled;
        instances++;
    }

    return std::make_pair(output_cloud, instances);
}

pcl::PointCloud<pcl::PointXYZL>::Ptr detectorV2::instanceExtractor_(const pcl::PointCloud<pcl::PointXYZL>::Ptr& inputCloud, const int ID)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr instance_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::ConditionAnd<pcl::PointXYZL>::Ptr label_cond (new pcl::ConditionAnd<pcl::PointXYZL> ());
    label_cond->addComparison (pcl::FieldComparison<pcl::PointXYZL>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZL> ("label", pcl::ComparisonOps::EQ, ID)));
    pcl::ConditionalRemoval<pcl::PointXYZL> label_filter;
    label_filter.setCondition (label_cond);
    label_filter.setInputCloud (inputCloud);
    label_filter.setKeepOrganized (true);
    label_filter.filter (*instance_cloud);

    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::removeNaNFromPointCloud(*instance_cloud, *filtered_cloud, indices);

    return filtered_cloud;
}

std::pair<pcl::PointXYZL, pcl::PointXYZL> detectorV2::computeAABB_(const pcl::PointCloud<pcl::PointXYZL>::Ptr& inputCloud)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZL> feature_extractor;
    feature_extractor.setInputCloud (inputCloud);
    feature_extractor.compute ();

    pcl::PointXYZL min_point_AABB, max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);

    return std::make_pair(min_point_AABB, max_point_AABB);
}

visualization_msgs::Marker detectorV2::genMarker_(pcl::PointXYZL minPt, pcl::PointXYZL maxPt, const unsigned int id, const ros::Time stamp)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "sensor_frame";
    marker.header.stamp = stamp;
    marker.ns = "bounding_box";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // set the scale of the lines
    marker.scale.x = marker.scale.y = marker.scale.z = 0.01;

    // set the color of the lines
    marker.color.a = marker.pose.orientation.w = 1.0;
    marker.color.r = marker.color.g = marker.color.b = 1.0;
    //marker.lifetime = ros::Duration(0.2);

    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

    p1.x = minPt.x; p1.y = minPt.y; p1.z = minPt.z;
    p2.x = maxPt.x; p2.y = minPt.y; p2.z = minPt.z;
    p3.x = maxPt.x; p3.y = minPt.y; p3.z = maxPt.z;
    p4.x = minPt.x; p4.y = minPt.y; p4.z = maxPt.z;
    p5.x = minPt.x; p5.y = maxPt.y; p5.z = minPt.z;
    p6.x = maxPt.x; p6.y = maxPt.y; p6.z = minPt.z;
    p7.x = maxPt.x; p7.y = maxPt.y; p7.z = maxPt.z;
    p8.x = minPt.x; p8.y = maxPt.y; p8.z = maxPt.z;

    // add the vertices to the Marker message
    marker.points.push_back(p1); marker.points.push_back(p2);
    marker.points.push_back(p2); marker.points.push_back(p3);
    marker.points.push_back(p3); marker.points.push_back(p4);
    marker.points.push_back(p4); marker.points.push_back(p1);
    marker.points.push_back(p5); marker.points.push_back(p6);
    marker.points.push_back(p6); marker.points.push_back(p7);
    marker.points.push_back(p7); marker.points.push_back(p8);
    marker.points.push_back(p8); marker.points.push_back(p5);
    marker.points.push_back(p1); marker.points.push_back(p5);
    marker.points.push_back(p2); marker.points.push_back(p6);
    marker.points.push_back(p3); marker.points.push_back(p7);
    marker.points.push_back(p4); marker.points.push_back(p8);

    return marker;
}

//----------Class----------//




//----------Static Functions----------//

pcl::PointCloud<pcl::PointXYZI>::Ptr removeGround_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; 
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());

    ne.setSearchMethod (tree);
    ne.setInputCloud (inputCloud);
    ne.setKSearch (10);
    ne.compute (*cloud_normals);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10);
    seg.setDistanceThreshold (0.3);
    seg.setInputCloud (inputCloud);
    seg.setInputNormals (cloud_normals);
    seg.segment (*inliers_plane, *coefficients_plane);

    extract.setInputCloud (inputCloud);
    extract.setIndices (inliers_plane);
    extract.setNegative (true);
    extract.filter (*cloud_plane);

    return cloud_plane;
}

static void startCore_(const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& outputCloud)
{
    outputCloud = removeGround_(inputCloud);
}

static std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> threadedGroundRemoval_(std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output4(new pcl::PointCloud<pcl::PointXYZI>);
    std::thread first(startCore_, std::ref(std::get<0>(clouds)), std::ref(output1));
    std::thread second(startCore_, std::ref(std::get<1>(clouds)), std::ref(output2));
    std::thread third(startCore_, std::ref(std::get<2>(clouds)), std::ref(output3));
    std::thread fourth(startCore_, std::ref(std::get<3>(clouds)), std::ref(output4));

    first.join();
    second.join();
    third.join();
    fourth.join();

    return std::make_tuple(output1, output2, output3, output4);
}

//----------Static Functions----------//




//----------Class Callback----------//

void detectorV2::callback_(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //Reading cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *raw_cloud);

    //Timer start
    std::chrono::_V2::system_clock::time_point start = std::chrono::high_resolution_clock::now();

    //Process Cloud
    // 1) Remove Nans
    // 2) split cloud
    // 3) Detect ground using 4 cores
    // 4) Refuse all the sub clouds to a single cloud
    // 5) Instance segmentation
    // 6) Bounding Box detector
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = this->removeNaNs_(raw_cloud);
    std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> groundRemovedClouds = threadedGroundRemoval_(this->splitCloud_(4, input_cloud, this->computeThreshold_(input_cloud)));
    pcl::PointCloud<pcl::PointXYZI>::Ptr refused_cloud = this->removeNoise_((*(std::get<0>(groundRemovedClouds)) + *(std::get<1>(groundRemovedClouds)) + *(std::get<2>(groundRemovedClouds)) + *(std::get<3>(groundRemovedClouds))).makeShared());
    std::pair<pcl::PointCloud<pcl::PointXYZL>::Ptr, const int> instanceCloud = this->instanceSegmentation_(refused_cloud);
    std::pair<pcl::PointXYZL, pcl::PointXYZL> AABB = this->computeAABB_(this->instanceExtractor_(instanceCloud.first, 0));
    //ROS_INFO("Min: %f, %f, %f Max: %f, %f, %f", (AABB.first).x, (AABB.first).y, (AABB.first).z, (AABB.second).x, (AABB.second).y, (AABB.second).z);

    /* TEST BLOCK FOR MULTI DETECTOR
    TODO
    1) Split instances to cores
    2) Perform 3D Bounding Box detector 
    3) Estimate and publish msg 


    std::pair<pcl::PointXYZL, pcl::PointXYZL> AABB2;
    if (instanceCloud.second == 1)
    {
        AABB2 = this->computeAABB_(this->instanceExtractor_(instanceCloud.first, 1));
        ROS_INFO("Min: %f, %f, %f Max: %f, %f, %f", (AABB2.first).x, (AABB2.first).y, (AABB2.first).z, (AABB2.second).x, (AABB2.second).y, (AABB2.second).z);
    }
    */

    //Timer stop plus durations in ms
    std::chrono::_V2::system_clock::time_point stop = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    //Console log
    static std::vector<int> times;
    times.push_back(duration.count());
    ROS_INFO("Time: %i ms, Average Time: %f ms, Highest: %i ms, Lowest: %i ms", times.back(), std::accumulate(times.begin(), times.end(), 0.0) / times.size(), *std::max_element(times.begin(), times.end()), *std::min_element(times.begin(), times.end()));

    //Segmented Point Cloud msg
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*(instanceCloud.first), cloud_msg);
    cloud_msg.header.frame_id = "sensor_frame";
    cloud_msg.header.stamp = ros::Time::now();

    //ROS Publishers
    this->cloudPub_.publish(cloud_msg);
    this->boundingBoxPub_.publish(this->genMarker_(AABB.first, AABB.second, 0, cloud_msg.header.stamp));
    //if (instanceCloud.second == 1) this->boundingBoxPub_.publish(this->genMarker_(AABB2.first, AABB2.second, 1, cloud_msg.header.stamp));
}

//----------Class Callback----------//



//----------Main----------//

int main(int argc, char** argv)
{
	ros::init(argc, argv, "detector_v2");
	ros::NodeHandle nh;
	detectorV2* detectorV2_ = new detectorV2(&nh);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
	return 0;
}

//----------Main----------//