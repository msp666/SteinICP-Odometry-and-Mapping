//
// Created by haoming on 27.06.23.
//

#ifndef STEINICP_STEINICP_ROS2_H
#define STEINICP_STEINICP_ROS2_H

#pragma once

//general
#include <iostream>
#include <vector>
#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>


//third party
#include <torch/torch.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/transforms.h>
#include <gtsam/geometry/Pose3.h>
#include <tbb/parallel_for.h>
#include <torch/torch.h>
#include <Eigen/Eigen>


//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <stein_particle_msgs/msg/stein_particle.hpp>
#include <stein_particle_msgs/msg/stein_parameters.hpp>
#include <stein_particle_msgs/msg/runtime.hpp>
//#include <grid_map_msgs/grid_map_msgs/msg/grid_map.hpp>

//self defined
#include "Buffer.h"
#include "DataTypes.h"

#include "SteinICP.h"
#include "BetaSteinICP.h"
#include "steinicp_utils.h"
#include "ScanContext.h"
#include "types.h"
#include "VoxelHashMap.h"
#include "imageProjection.h"


namespace stein_icp
{
class SteinICPOdometry
            {
            private:
                rclcpp::Node &node_;
                rclcpp::Time timestamp_cloud_;
                std::atomic_bool is_firstScan_ = true;
                std::atomic_bool is_firstOdom_ = true;
                std::atomic_bool skip_scan_ = false;
                std::atomic_bool last_optim_finished_ = true;
                std::atomic_bool new_scan_ = false;
                std::atomic_bool odom_finished_ = false;
                std::mutex variable_mutex_;

                //Buffer
                fgo::buffer::CircularDataBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_msg_buffer_;
                fgo::buffer::CircularDataBuffer<fgo::data_types::Odom> odom_buffer_;
                fgo::buffer::CircularDataBuffer<std::pair<std::vector<double>,std::vector<double>>> stein_particle_weight_buffer_;

                rclcpp::Time timestamp_last_;
                rclcpp::Time timestamp_current_;
                rclcpp::Time timestamp_odom_;
                double msg_buffer_gap_;

                //SteinICP odom
                sensor_msgs::msg::PointCloud2 original_msg_;
                sensor_msgs::msg::PointCloud2 deskewed_msg_;
                bool deskew_cloud_;
                std::string cloud_topic_;
                std::unique_ptr<stein_icp::BetaSteinICP> steinicp_odom_;
                stein_icp::SteinICPParam steinicp_param_;
                torch::Tensor init_pose_;
                torch::Tensor source_tr_;
                torch::Tensor target_tr_;
                pcl::PointCloud<Point_t>::Ptr source_pcl_;
                pcl::PointCloud<Point_t>::Ptr target_pcl_;
                pcl::PointCloud<Point_t>::Ptr total_map_;
                std::vector<std::pair<rclcpp::Time, pcl::PointCloud<Point_t>>> deskewed_cloud_;
                std::vector<std::pair<rclcpp::Time, pcl::PointCloud<Point_t>>> scds_cloud_;
                std::vector<std::pair<rclcpp::Time, pcl::PointCloud<Point_t>>> source_cloud_vec_;
                std::vector<std::pair<rclcpp::Time, pcl::PointCloud<Point_t>>> target_cloud_vec_;
                int particle_count_ = 100;
                gtsam::Matrix44 T_;
                std::chrono::duration<float> align_duration_;
                fgo::buffer::CircularDataBuffer<gtsam::Pose3> poses3_;
                std::vector<gtsam::Pose3> covariance_;
                std::vector<gtsam::Pose3> delta_pose_;
                double steinicp_runtime_, preprocessing_runtime_;
                stein_icp::VoxelHashMap local_map_;
                double max_range_;
                double min_range_;

                bool use_Segmentation_ = true;

                bool use_BetaSteinICP_ = true;
                stein_icp::BetaSteinICPOpt beta_stein_opt_;

                bool voxelization_ = false;
                double voxel_size_ = 0.01;
                pcl::VoxelGrid<Point_t> voxel_grid_;
                pcl::UniformSampling<Point_t> uniform_sampling_;


                //ScanContext
                ScanContext::ScanContext sc_generator_;
                ScanContext::ScanContextParam sc_param_;
                std::vector<std::pair<rclcpp::Time,Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>> scan_context_;
                bool use_sc_;
                bool save_particle_info_;

                //Cloud Segmentation
                std::shared_ptr<ImageProjection> cloud_segmentation_;

                //save log

                std::string integrator_name_;

                //Subscriber & Publisher
                rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
                rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr odom_publisher_;
                rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_publisher_;
                rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_publisher_;
                nav_msgs::msg::Path path_msg_;
                rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_publisher_;
                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr odom_statistic_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    sc_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    ds_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    source_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    original_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    deskewed_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    localmap_publisher_;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    neighbourmap_publisher_;
                rclcpp::Publisher<stein_particle_msgs::msg::SteinParticle>::SharedPtr stein_particle_publisher_;
                rclcpp::Publisher<stein_particle_msgs::msg::SteinParameters>::SharedPtr stein_param_publisher_;
                rclcpp::Publisher<stein_particle_msgs::msg::Runtime>::SharedPtr runtime_publisher_;

                bool ds_published_;

                rclcpp::TimerBase::SharedPtr odom_timer_;
                rclcpp::TimerBase::SharedPtr odom_statistic_timer_;

                std::shared_ptr<std::thread> steinicp_thread_;
                std::shared_ptr<std::thread> publish_thread_;

            private:


                /**
                 * @brief load SteinICP parameters from .yaml file
                 */
                void load_param();

                /**
                 * @brief initialize variables, initialize pointer
                 */
                void allocateMemory();


                /**
                 * @brief receive point cloud msg form topic "velodyne_points"
                 *        and buffer them if time step is bigger than 0.15s, buffer per 0.2s
                 * @param msg
                 */
                void msg_handler_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


                /**
                 * @brief deskew point clouds
                 */
                pcl::PointCloud<Point_t> deskew_pointcloud(const sensor_msgs::msg::PointCloud2 msg);

                /**
                 *@brief process the SteinICP if all condition is fullfilled,
                 *       and buffer the odometry in a buffer
                 */
                void ICP_processing();

                /**
                 *@brief  voxelization downsample
                 */
                pcl::PointCloud<Point_t> downsample_voxel(const pcl::PointCloud<Point_t>::Ptr &cloud, double voxel_size);

                pcl::PointCloud<Point_t> downsample_uniform(const pcl::PointCloud<Point_t>::Ptr &cloud, double voxel_size);

                pcl::PointCloud<Point_t> crop_pointcloud(const pcl::PointCloud<Point_t>::Ptr &pointcloud);

                gtsam::Pose3 position_prediction(const rclcpp::Time &new_time);

                double constraint_value(double value, double limit);

                gtsam::Pose3 constraint_transform(const gtsam::Pose3 &pose);

                void publish_stein_param();

                /**
                 * @brief publish the position
                 */
                void publish_odometry();

                void publish_scancontext();

                void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr,
                                   std::vector<std::pair<rclcpp::Time, pcl::PointCloud<Point_t>>>&);

                void publish_cloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &,
                                   pcl::PointCloud<Point_t>, rclcpp::Time);

                void publish_particle_info();

                void publish_runtime();

                void publish_thread_cb();



            public:
                explicit SteinICPOdometry(rclcpp::Node &node,
                                      const rclcpp::CallbackGroup::SharedPtr SteinICP_group,
                                      const rclcpp::CallbackGroup::SharedPtr visualization_group,
                                      const std::string& integratorName
                                      );

                ~SteinICPOdometry(){
                    steinicp_thread_->join();
                    publish_thread_->join();
                }

                /**
                 * @brief show whether there is already computed odmometry
                 * @return
                 */
                bool has_odom();

                /**
                 * @return the odometry from buffer
                 */
                std::vector<fgo::data_types::Odom> get_odom();

                void set_initpose();

                void set_initpose(const torch::Tensor &delta_last);

                //TODO: Some update API function should be declared and defined. Here is only an example
                void update_opt_state();







            };
}



#endif //STEINICP_STEINICP_ROS2_H
