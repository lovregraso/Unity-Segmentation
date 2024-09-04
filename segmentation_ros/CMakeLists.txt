cmake_minimum_required(VERSION 3.8)
project(segmentation_ros)

# Postavite C++ standard
set(CMAKE_CXX_STANDARD 14)

# Pronađite potrebne pakete
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common visualization) # Dodajte PCL
find_package(PCL REQUIRED COMPONENTS common io kdtree) # Dodajte io komponentu


link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})



include_directories(include ${roscpp_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})



# Dodajte izvršnu datoteku
add_executable(ros_segmentation_node src/ros_segmentation_node.cpp)
add_executable(DBSCAN_node src/euclideanSegmentation.cpp)
add_executable(region_growing_node src/regionGrowing.cpp)
add_executable(ransac_node src/ransac.cpp)
add_executable(compare_point_clouds src/cloudCompare.cpp)
add_executable(calculate_bounding_box_node src/CalculateBoundingBox.cpp)





# Navedite poveznice na potrebne biblioteke
ament_target_dependencies(ros_segmentation_node
    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
    std_msgs
)

ament_target_dependencies(DBSCAN_node
    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
    std_msgs
)

ament_target_dependencies(region_growing_node
    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
    std_msgs
)

ament_target_dependencies(ransac_node
    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
    std_msgs
)


ament_target_dependencies(compare_point_clouds
    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
   std_msgs
)
ament_target_dependencies(calculate_bounding_box_node

    rclcpp 
    pcl_conversions 
    pcl_ros 
    sensor_msgs 
   std_msgs
)

target_include_directories(ros_segmentation_node PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(DBSCAN_node PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(region_growing_node PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(ransac_node PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(compare_point_clouds PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(calculate_bounding_box_node
PRIVATE ${PCL_INCLUDE_DIRS})


target_link_libraries(ros_segmentation_node ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries(DBSCAN_node ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries(region_growing_node ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries(ransac_node ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries(compare_point_clouds ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries(calculate_bounding_box_node
${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})




# Instalirajte izvršnu datoteku
install(TARGETS
  ros_segmentation_node
  DBSCAN_node
  region_growing_node
  ransac_node
  read_labels
  compare_point_clouds
  calculate_bounding_box_node

  DESTINATION lib/${PROJECT_NAME})

# Instalirajte C++ datoteke
install(
  DIRECTORY include/
  DESTINATION include
)

# Instalirajte package.xml datoteku
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
