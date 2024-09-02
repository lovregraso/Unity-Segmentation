#include <fstream>
#include <iostream>
#include <string>
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
#include <iomanip> // for setw, setfill
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <map>
#include <random>
#include <cmath>  // Za std::pow


// Helper function to convert uint16_t to int16_t
int16_t convert_to_signed(uint16_t value) {
    if (value & 0x8000) { // if the highest bit is set
        return -static_cast<int16_t>((~value + 1) & 0xFFFF);
    } else {
        return static_cast<int16_t>(value);
    }
}


void read_labels(const std::string& file_path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    std::ifstream input(file_path, std::ios::binary);
    std::string output_path = "ground_truth.txt";
    std::ofstream output(output_path);

    if (!input.is_open()) {
        std::cerr << "Error opening input file: " << file_path << std::endl;
        return;
    }


    while (true) {
        char data[8];
        input.read(data, 8);
        if (input.gcount() < 8) {
            break;
        }

        uint16_t class_id = *reinterpret_cast<uint16_t*>(data);
       // uint16_t inst_id = *reinterpret_cast<uint16_t*>(data + 2);
        uint16_t point_x = *reinterpret_cast<uint16_t*>(data + 2);
        uint16_t point_y = *reinterpret_cast<uint16_t*>(data + 4);
        uint16_t point_z = *reinterpret_cast<uint16_t*>(data + 6);

        if (class_id <= 6 && class_id > 0) {
            pcl::PointXYZRGB point;
            pcl::PointXYZRGB point_uncolored;
            point.x = convert_to_signed(point_z);
            point.y = convert_to_signed(point_y);
            point.z = convert_to_signed(point_x);

            point_uncolored.x = convert_to_signed(point_z);
            point_uncolored.y = convert_to_signed(point_y);
            point_uncolored.z = convert_to_signed(point_x);
            point_uncolored.r = 0; point_uncolored.g = 255; point_uncolored.b = 0;   // zelena

            // Assign color based on class_id
            switch (class_id) {
                case 1: point.r = 0; point.g = 255; point.b = 0; break;   // zelena
                case 2: point.r = 255; point.g = 0; point.b = 0; break;   // crvena
                case 3: point.r = 0; point.g = 0; point.b = 255; break; // plava
                case 4: point.r = 255; point.g = 255; point.b = 0; break; // zuta
                case 5: point.r = 255; point.g = 0; point.b = 255; break; // magenta
                case 6: point.r = 0; point.g = 255; point.b = 255; break;   // cijan
                //case 6: point.r = 128; point.g = 128; point.b = 128; break; // Grey
                default: point.r = 255; point.g = 255; point.b = 255; break; // White for any other class_id
            }

            if (point.y > 0 ) {
                cloud->points.push_back(point);
                cloud_orig->points.push_back(point_uncolored);
            }
        }
       
        if (class_id <= 7) {
            output << "ClassId: " << class_id << ", Point: ("
                   << point_x << ", " << point_y << ", " << point_z << ")\n";
        }


    }
    //CalculateBoundingBoxes(cloud);
    input.close();
    output.close();

   
}

void  FogFilteredPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
            int fogAttenuationDistance = 40;
            //int rainIntensity = 10;
            double p = 0.00032 * std::pow(fogAttenuationDistance, 2) - 0.0164 * fogAttenuationDistance + 0.167;
            //double p = (270 - 12.384*rainIntensity - 1.361* std::pow(rainIntensity, 2))/270;

            if (p > 1){
                return;
            }
            if (p <0){
                p = 0;
            }
            int n = cloud->points.size();
            int m = static_cast<int>(std::round(n * p));

             // Koristi std::random_device i std::mt19937 za generiranje nasumičnih brojeva
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, cloud->points.size() - 1);

            std::vector<int> indices;
            while (indices.size() < n-m) {
                int index = dis(gen);
                if (std::find(indices.begin(), indices.end(), index) == indices.end()) {
                    indices.push_back(index);
                }
            }

            // Sortiraj indekse u silaznom redoslijedu
            std::sort(indices.rbegin(), indices.rend());

            // Ukloni točke s odabranim indeksima
            for (int index : indices) {
                cloud->points.erase(cloud->points.begin() + index);
            }

            std::cout << "Novi broj točaka: " << cloud->points.size() << std::endl;

           
        }


int main(int argc, char** argv) {
  
    std::string label_file_path = "lidar_000046.label";
    std::string pcd_file_path_orig = "original_magla.pcd";
    std::string pcd_file_path = "ground_truth_colored.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    read_labels(label_file_path, cloud_orig, cloud);

    FogFilteredPoints(cloud_orig);

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    cloud_orig->width = cloud_orig->points.size();
    cloud_orig->height = 1;
    cloud_orig->is_dense = true;
    pcl::io::savePCDFileASCII(pcd_file_path_orig,*cloud_orig);
    pcl::io::savePCDFileASCII(pcd_file_path,*cloud);



///*
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud_orig, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(1000000);
            std::this_thread::sleep_for(std::chrono::milliseconds(10000000));
        }
   //     */
    std::cout << "Saved " << cloud->points.size() << " data points to " << pcd_file_path << std::endl;

    return 0;
}
