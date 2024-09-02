#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

class RansacNode : public rclcpp::Node
{
public:
    RansacNode() : Node("ransac_node")
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //load pcd file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("original.pcd", *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Nije moguće učitati PCD datoteku.");
            return;
        }
        pointCloudCallback(cloud);
    }
    struct Color {
    int r, g, b;

    Color(int red, int green, int blue) : r(red), g(green), b(blue) {}
};

private:
    void pointCloudCallback( pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        auto start = std::chrono::high_resolution_clock::now(); // start time measurement

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
       
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        // adjust parameters
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int nr_points = (int) cloud->size ();
        while (cloud->size () > 0.9 * nr_points) // reduce points from point cloud using ransac
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud = *cloud_f;
        }

        // after ransac fitration comes standard DBSCAN segmentation
        
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (20); 
        ec.setMinClusterSize (1);
        ec.setMaxClusterSize (40000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        int cluster_id = 0;

        std::vector<Color> colors = {
            Color(0, 255, 0),    // green  
            Color(0, 255, 255), // cyan
            Color(255, 0, 255), // magenta
            Color(255, 0, 0),   // red         
            Color(0, 0, 255),   // blue
            Color(255, 255, 0), // yellow
        };
   

        for (const auto& indices : cluster_indices) {
            int r = rand() % 256;
            int g = rand() % 256;
            int b = rand() % 256;
            for (const auto& index : indices.indices) {
                pcl::PointXYZRGB point;
                point.x = cloud->points[index].x;
                point.y = cloud->points[index].y;
                point.z = cloud->points[index].z;
                point.r = r;
                point.g = g;
                point.b = b; //colors[cluster_id].b -> visualize point cloud with same colors
                segmented_cloud->points.push_back(point);
            }
            std::cout << cluster_id +1 <<" cluster has " << indices.indices.size () << " points." << std::endl;

            ++cluster_id;
        }
        

        std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
        std::cout << "First cluster has " << cluster_indices[0].indices.size () << " points." << std::endl;
        std::cout << "New point cloud has " << segmented_cloud->size()<< " points." << std::endl;

        segmented_cloud->width = segmented_cloud->points.size(); // number of points in point cloud 
        segmented_cloud->height = 1;
        pcl::PCDWriter writer;

        std::string filename = "ransac_point_cloud.pcd" ;
        writer.write<pcl::PointXYZRGB>(filename, *segmented_cloud);

        // calculate execution time 
        auto end = std::chrono::high_resolution_clock::now(); // end time measurement
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Execution time: " << elapsed.count() << " seconds" << std::endl;

        // visualize point cloud using pcl viewer 
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGB>(segmented_cloud, "segmented_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented_cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        while (!viewer->wasStopped()) {
            viewer->spinOnce(1000000);
            std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
        }
        
        RCLCPP_INFO(this->get_logger(), "Segmented plane saved in ransac_point_cloud.pcd");
        
        
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int counter = 1; 
};



int main(int argc, char *argv[])
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RansacNode>());
    rclcpp::shutdown();
    return 0;
}


