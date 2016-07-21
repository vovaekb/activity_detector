#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZRGB PointInT;

ros::Publisher result_pub;

pcl::octree::OctreePointCloudChangeDetector<PointInT> *octree;

int fp = 1;

// Octree resolution - side length of octree voxels
float resolution = 0.01; // 0.01 - default in openni_change_viewer
int noise_filter = 10; // 7 - default in openni_change_viewer

int sor_mean_k = 70;
float sor_stddev_mul_th = 1.0;

float ror_rad = 0.01;
int ror_min_heighbors = 150;

int min_size_thresh = 1500;

void parseCommandLine()
{
    ros::NodeHandle private_node_handle_("~");

    private_node_handle_.getParam("fp", fp);
    private_node_handle_.getParam("res", resolution);
    private_node_handle_.getParam("noise", noise_filter);
    private_node_handle_.getParam("rad", ror_rad);
    private_node_handle_.getParam("min_neighbors", ror_min_heighbors);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2 result_cloud_msg;

    pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);

    pcl::fromROSMsg(*input, *cloud);

    // Filtering out noise
//    pcl::StatisticalOutlierRemoval<PointInT> sor;
//    sor.setInputCloud (cloud);
//    sor.setMeanK (sor_mean_k); // 50 - default
//    sor.setStddevMulThresh (sor_stddev_mul_th);
//    sor.filter (*cloud);

    octree->setInputCloud (cloud);
    octree->addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    octree->getPointIndicesFromNewVoxels (newPointIdxVector, noise_filter);

    if(!newPointIdxVector.size())
    {
        ROS_INFO("Activity wasn't detected\n");
        return;
    }

    pcl::PointCloud<PointInT>::Ptr person_cloud (new pcl::PointCloud<PointInT>);

    pcl::copyPointCloud(*cloud, newPointIdxVector, *person_cloud);

    // Remove NaN from point cloud
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*person_cloud, *person_cloud, mapping);

    // Some noise removal procedure

    pcl::RadiusOutlierRemoval<PointInT> outrem;
    outrem.setInputCloud(person_cloud);
    outrem.setRadiusSearch(ror_rad);
    outrem.setMinNeighborsInRadius (ror_min_heighbors);
    outrem.filter (*person_cloud);

    // Reject too small cloud
    if(person_cloud->points.size() < min_size_thresh)
    {
        ROS_INFO("Activity wasn't detected\n");
        return;
    }

    ROS_INFO("Activity cluster has %d points\n", (int)person_cloud->points.size());

    pcl::toROSMsg(*person_cloud, result_cloud_msg);

    result_cloud_msg.header.frame_id = input->header.frame_id;
    result_pub.publish(result_cloud_msg);


//    ROS_INFO("Person pose was published\n");

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree->switchBuffers ();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "activity_detector");

    ros::NodeHandle nh;

    parseCommandLine();

    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    result_pub = nh.advertise<sensor_msgs::PointCloud2>("activity", 1);

    octree = new pcl::octree::OctreePointCloudChangeDetector<PointInT> (resolution);

    ros::Rate loop_rate(fp);

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
