#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>

class rosSegmentationNode : public rclcpp::Node
{
public:
    rosSegmentationNode() : Node("ros_segmentation_node")
    {
        // subscribe to topic "/input_point_cloud" and define callback function
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
           "/marus_boat/lidar", 10, std::bind(&rosSegmentationNode::pointCloudCallback, this, std::placeholders::_1)); // subscribe to ros2 topic 
    }
   // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // convert message to point cloud 
        pcl::fromROSMsg(*msg, *cloud);

        
        // insert segmentation algorithm
        //region growing algorithm 
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);

        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::removeNaNFromPointCloud(*cloud, *indices);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (50);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (60);
        reg.setInputCloud (cloud);
        reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

        std::string filename_orig = "point_cloud_" + std::to_string(counter++) + ".pcd";
        std::string filename = "plane_segmentation_" + std::to_string(counter++) + ".pcd";
        // write pcd file 
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>(filename_orig, *cloud);
        writer.write<pcl::PointXYZRGB>(filename, *colored_cloud);



        RCLCPP_INFO(this->get_logger(), "Segmented plane saved in plane_segmentation.pcd");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int counter = 1; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rosSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}


