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
#include <pcl/io/pcd_io.h> // Dodajte uključivanje za čitanje PCD datoteke
#include <pcl/common/centroid.h>
#include <iomanip> // for setw, setfill
#include <chrono>



class RegionGrowingNode : public rclcpp::Node
{
public:
    RegionGrowingNode() : Node("region_growing_node")
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("original.pcd", *cloud) == -1) //load pcd file
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

        //calculate normals 
        pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);

        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::removeNaNFromPointCloud(*cloud, *indices);

        // region growing algorithm 
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize (75); // 1 for min
        reg.setMaxClusterSize (40000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (600); // 30 for min 

        reg.setInputCloud (cloud);
        reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (1000 / 180.0 * M_PI/40); // 500 for min 
        reg.setCurvatureThreshold (500.0); // 100 for min  
        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
        
        //color cloud 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->width =   cloud->size ();
        colored_cloud->height = cloud->height;
        colored_cloud->is_dense = cloud->is_dense;
        // add colors to clusters and add to segmented point cloud - specific for this case 
        int cluster_id = 0;
       
        std::vector<Color> colors = {
            Color(0, 0, 255),   // Plava
            Color(0, 255, 0),    // Zelena
            Color(255, 0, 0),   // Crvena
            Color(255, 255, 0), // Žuta
            Color(255, 0, 255), // Magenta
            Color(0, 255, 255), // Cijan
        };
         for (const auto& i_point : *cloud) {
                pcl::PointXYZRGB point;
                point.x = *(i_point.data);
                point.y = *(i_point.data + 1);
                point.z = *(i_point.data + 2);
                point.r = colors[cluster_id].r;
                point.g = colors[cluster_id].g;
                point.b = colors[cluster_id].b;
                colored_cloud->points.push_back(point);
                
        }
         ++cluster_id;

        for (const auto& indices : clusters) {
            for (const auto& index : indices.indices) {
                (*colored_cloud)[index].r = colors[cluster_id].r;
                (*colored_cloud)[index].g = colors[cluster_id].g;
                (*colored_cloud)[index].b = colors[cluster_id].b;
            }
            std::cout << cluster_id +1 <<" cluster has " << indices.indices.size () << " points." << std::endl;
            ++cluster_id;
        }

        // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud(); use in general - automatically add colors
       
        std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
        std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
        std::cout << "New point cloud has " << colored_cloud->size()<< " points." << std::endl;

        std::string filename = "region_growing_segmentation.pcd";

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGB>(filename, *colored_cloud);

        // calculate execution time 
        auto end = std::chrono::high_resolution_clock::now(); // end time measurement
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Execution time: " << elapsed.count() << " seconds" << std::endl;

        // visualize point cloud 
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "colored_cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(1000000);
            std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
        }
        
        RCLCPP_INFO(this->get_logger(), "region_growing_segmentation.pcd");
    }


 };
  
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionGrowingNode>());
    rclcpp::shutdown();
    return 0;
}


