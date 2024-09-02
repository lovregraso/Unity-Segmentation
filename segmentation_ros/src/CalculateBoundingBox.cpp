#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>



class CalculateBoundingBoxNode : public rclcpp::Node
{
public:
    CalculateBoundingBoxNode() : Node("calculate_bounding_box_node")
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("add_pcd_file.pcd", *cloud) == -1) // add pcd file 
        {
            RCLCPP_ERROR(this->get_logger(), "Nije moguće učitati PCD datoteku.");
            return;
        }
        pointCloudCallback(cloud);
    }
   // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
   struct Color {
    int r, g, b;

    Color(int red, int green, int blue) : r(red), g(green), b(blue) {}
    };


private:
    void pointCloudCallback( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        

        std::vector<Color> colors = {
            Color(0, 255, 0),    // Zelena  isto
            Color(0, 255, 255), // Cijan
            Color(0, 0, 255),   // Plava
            Color(255, 0, 255), // Magenta
            Color(255, 255, 0), // Žuta
            Color(255, 0, 0),   // Crvena
        };
        std::vector<std::vector<pcl::PointXYZRGB>> list_of_regions(6);
        int index = 0;
        for(const auto& color: colors){
            for (const auto& pt : cloud->points) {
                if (pt.r == color.r && pt.g == color.g && pt.b == color.b) {
                    list_of_regions[index].push_back(pt);
                    
                }
            }
             index++;
                
        }
        std::vector<std::array<float, 3>> list_of_BoundingBoxes(6);

        for (int i = 0; i < 6; ++i) {
                float x = 0; float y=0; float z=0; 
                for (int j = 0; j < list_of_regions[i].size(); ++j) {
                    x += list_of_regions[i][j].x;
                    y += list_of_regions[i][j].y;
                    z += list_of_regions[i][j].z;
                }
                x = x/list_of_regions[i].size();
                y = y/list_of_regions[i].size();
                z = z/list_of_regions[i].size();

                list_of_BoundingBoxes[i][0] = x;
                list_of_BoundingBoxes[i][1] = y;
                list_of_BoundingBoxes[i][2] = z;

        }

        for (int k = 0; k < 6; ++k) {
            
            std::cout << k +1 << " bounding box-> x: " << list_of_BoundingBoxes[k][0] << " y: " << list_of_BoundingBoxes[k][1] 
             << " z: " << list_of_BoundingBoxes[k][2]  << std::endl;

        } 

        // visualize point cloud 
        /*
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(1000000);
            std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
        }

        */
         
    }
};

int main(int argc, char *argv[])
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalculateBoundingBoxNode>());
    rclcpp::shutdown();
    return 0;
}


