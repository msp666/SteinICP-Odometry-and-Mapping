cmake_minimum_required(VERSION 3.15)
project(steinicp_om)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

#find third party dependencies
set(CMAKE_PREFIX_PATH "/home/haoming/Desktop/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
find_package(PCL REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(GTSAMCMakeTools)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(tsl-robin-map REQUIRED)

# find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(boreas_msgs REQUIRED)
find_package(stein_particle_msgs REQUIRED)

include_directories(
include
include/stein_icp
include/stein_icp/knn
include/stein_icp/utils
include/data
include/fgo_interface
include/utils
include/ScanContext
        include/Segmentation

${TORCH_LIBRARY_DIRS}
${PCL_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${GTSAM_INCLUDE_DIRS}
${TBB_INCLUDE_DIR}
${tsl-robin-map_INCLUDE_DIR}
)

### build SteinICP class
add_library(stein_icp SHARED
src/stein_icp/SteinICP.cpp
src/stein_icp/BetaSteinICP.cpp
src/stein_icp/steinicp_utils.cpp
src/stein_icp/knn/knn.cu
src/stein_icp/knn/knn_cpu.cpp
src/stein_icp/knn/knn.cpp
src/stein_icp/VoxelHashMap.cpp
)

target_link_libraries(stein_icp
${TORCH_LIBRARIES}
${PCL_LIBRARIES}
${PYTHON_LIBRARIES}
${Boost_LIBRARIES}
${GTSAM_LIBRARIES}
#gtsam
gtsam_unstable
)



#### build a steinicp top level in ros2
#add_library(steinicp_ros2
#            src/stein_icp/steinicp_ros2.cpp
#            )
#
#target_link_libraries(steinicp_ros2
#                      stein_icp
#                      )
#
#
#ament_target_dependencies(steinicp_ros2
#                         "rclcpp"
#                         "sensor_msgs"
#                         "std_msgs"
#                         "pcl_msgs"
#                         "PCL"
#                         "pcl_conversions")
#
#
#### build a simple node in which instance a stein_icp odometry(steinicp_ros2)
add_library(onlineFGO_test  SHARED
src/fgo_interface/onlineFGO_test.cpp
src/stein_icp/steinicp_ros2.cpp
src/ScanContext/ScanContext.cpp
src/stein_icp/VoxelHashMap.cpp
)

target_link_libraries(onlineFGO_test
stein_icp
${TBB_LIBRARIES}
${tsl-robin-map_LIBRARIES}
${OPENCV_LIBRARY_DIRS}
)

ament_target_dependencies(onlineFGO_test
"rclcpp"
"sensor_msgs"
"std_msgs"
"pcl_msgs"
"PCL"
"pcl_conversions"
"geometry_msgs"
"nav_msgs"
#        "boreas_msgs"
"stein_particle_msgs"
)


# generate executable and link to steinicp_ros2 library
add_executable(steinicp_om_main
src/node/stein_icp_node.cpp
)

target_link_libraries(steinicp_om_main
onlineFGO_test
)

install(TARGETS steinicp_om_main
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")



if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()
