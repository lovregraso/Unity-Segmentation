#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

#include <unordered_set>

typedef pcl::PointXYZRGB PointT;


void removeDuplicates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Uklanjanje NaN točaka
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Uklanjanje duplikata
    std::sort(cloud->points.begin(), cloud->points.end(), [](const pcl::PointXYZRGB &a, const pcl::PointXYZRGB &b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y) || (a.x == b.x && a.y == b.y && a.z < b.z);
    });
    auto last = std::unique(cloud->points.begin(), cloud->points.end(), [](const pcl::PointXYZRGB &a, const pcl::PointXYZRGB &b) {
        return a.x == b.x && a.y == b.y && a.z == b.z ;//&& a.rgb == b.rgb;
    });
    cloud->points.erase(last, cloud->points.end());
}

class PointCloudComparator : public rclcpp::Node {
public:
    PointCloudComparator() : Node("compare_point_clouds") {

        this->compare_point_clouds();
    }

private:
    void compare_point_clouds() {
       
        std::string pcd_orig = "ground_truth_colored.pcd"; // add first pcd file 
        std::string pcd_to_compare = "dbscan_point_cloud_obojen.pcd"; // add second pcd file

        pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);

        if (pcl::io::loadPCDFile<PointT>(pcd_orig, *cloud1) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", pcd_orig.c_str());
            return;
        }

        if (pcl::io::loadPCDFile<PointT>(pcd_to_compare, *cloud2) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", pcd_orig.c_str());
            return;
        }

        removeDuplicates(cloud1);
        removeDuplicates(cloud2);

        int same_color_points = 0;


        std::string output_path = "all_points.txt";
            std::ofstream output(output_path);
        /*
        for (const auto& pt1 : cloud1->points) {
            //output <<"(" << pt1.x << ", " << pt1.y << ", " << pt1.z <<  ", " << pt1.r << ", " << pt1.g << ", " << pt1.b << ")\n";
            output << ", Point: (" << pt1.x << ", " << pt1.y << ", " << pt1.z << ")\n";

        }
        */
        int matching_points = 0;
        int total_comparisons = 0;

        for (const auto& pt1 : cloud1->points) {
            for (const auto& pt2 : cloud2->points) {
                if (pt1.x == pt2.x && pt1.y == pt2.y && pt1.z == pt2.z &&   
                    pt1.r == pt2.r && pt1.g == pt2.g && pt1.b == pt2.b) {
                    matching_points++;
                }
                total_comparisons++;
            }

        }
     
        std::cout << "Original point cloud has " << cloud1->size()<< " points." << std::endl;
        std::cout << "Segmented point cloud has " << cloud2->size()<< " points." << std::endl;
        RCLCPP_INFO(this->get_logger(), "Number of points with the same coordinates and color: %d", matching_points);

        /*
         pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        viewer2->setBackgroundColor(0, 0, 0);
        viewer2->addPointCloud<pcl::PointXYZRGB>(cloud2, "colored_cloud");
        viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "colored_cloud");
        viewer2->addCoordinateSystem(1.0);
        viewer2->initCameraParameters();

        while (!viewer2->wasStopped()) {
            viewer2->spinOnce(1000000);
            std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
        }
        */
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudComparator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
