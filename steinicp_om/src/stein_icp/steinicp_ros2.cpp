//
// Created by haoming on 26.06.23.
//



#include "steinicp_ros2.h"

namespace stein_icp
{


    SteinICPOdometry::SteinICPOdometry(rclcpp::Node &node,
                               const rclcpp::CallbackGroup::SharedPtr SteinICP_group,
                               const rclcpp::CallbackGroup::SharedPtr visualization_group,
                               const std::string& integratorName
                               )
                               :node_(node)
    {
        integrator_name_ = integratorName;

        this->load_param();

        this->allocateMemory();

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = SteinICP_group;
        cloud_subscriber_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_,
                                                                                     rclcpp::SystemDefaultsQoS(),
                                                                                     [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)->void
                                                                                     {
                                                                                            this->msg_handler_cb(msg);
                                                                                     },
                                                                                     sub_opt);

        odom_publisher_ = node_.create_publisher<geometry_msgs::msg::PointStamped>("/stein_icp/odom_visualization", 10);
        pose_publisher_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>("/stein_icp/pose_visualization", 10);
        path_publisher_ = node_.create_publisher<nav_msgs::msg::Path>("/stein_icp/trajectories", 10);
        pose_cov_publisher_ =  node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/stein_icp/pose_with_covariance", 10);
        sc_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/scan_context", 10);
        ds_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/downsampled_cloud", 10);
        source_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/source_cloud", 10);
        original_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/original_cloud", 10);
        deskewed_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/deskewed_cloud", 10);
        localmap_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/localmap_cloud", 10);
        neighbourmap_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/stein_icp/neighbourmap_cloud", 10);
        stein_particle_publisher_ = node_.create_publisher<stein_particle_msgs::msg::SteinParticle>("/stein_icp/particles", 10);
        stein_param_publisher_ = node_.create_publisher<stein_particle_msgs::msg::SteinParameters>("/stein_icp/parameters",10);
        runtime_publisher_ = node_.create_publisher<stein_particle_msgs::msg::Runtime>("/stein_icp/runtime", 10);
//        odom_timer_ = node_.create_wall_timer(std::chrono::milliseconds(500),
//                                              [this]()->void
//                                              {
//                                                   this->publish_odometry();
//                                                  this->publish_scancontext();
//                                                  this->publish_OriginalCloud();
//                                                   if (deskew_cloud_)
//                                                       this->publish_DeskewedCloud();
//                                              });

        odom_statistic_publisher_ = node_.create_publisher<std_msgs::msg::Float64MultiArray>("/odom_statistic", 10);

        steinicp_thread_ = std::make_shared<std::thread>(
                [this]()->void{
                    this->ICP_processing();
                }
                );

        publish_thread_ = std::make_shared<std::thread>(
                [this]()->void{
                    this->publish_thread_cb();
                }
                );
    }

    void SteinICPOdometry::load_param()
    {
        //        RCLCPP_INFO(node_.get_logger(), "Loading parameters");

        //SteinICP parameters
        node_.declare_parameter("SteinICP_parameters.optimizer", "Adam");
        node_.get_parameter("SteinICP_parameters.optimizer", steinicp_param_.optimizer);
        node_.declare_parameter("SteinICP_parameters.iterations", 50);
        node_.get_parameter("SteinICP_parameters.iterations", steinicp_param_.iterations);
        node_.declare_parameter("SteinICP_parameters.batch_size", 50);
        node_.get_parameter("SteinICP_parameters.batch_size", steinicp_param_.batch_size);
        node_.declare_parameter("SteinICP_parameters.normalize", true);
        node_.get_parameter("SteinICP_parameters.normalize", steinicp_param_.normalize);
        node_.declare_parameter("SteinICP_parameters.lr", 0.02);
        node_.get_parameter("SteinICP_parameters.lr", steinicp_param_.lr);
        node_.declare_parameter("SteinICP_parameters.max_dist", 2.8);
        node_.get_parameter("SteinICP_parameters.max_dist", steinicp_param_.max_dist);
        node_.declare_parameter("SteinICP_parameters.particle_size", 100);
        node_.get_parameter("SteinICP_parameters.particle_size", particle_count_);
        node_.declare_parameter("SteinICP_parameters.msg_buffer_gap_seconds", 0.35);
        node_.get_parameter("SteinICP_parameters.msg_buffer_gap_seconds", msg_buffer_gap_);
        node_.declare_parameter("SteinICP_parameters.using_EarlyStop", false);
        node_.get_parameter("SteinICP_parameters.using_EarlyStop", steinicp_param_.using_EarlyStop);
        node_.declare_parameter("SteinICP_parameters.convergence_steps", 5);
        node_.get_parameter("SteinICP_parameters.convergence_steps", steinicp_param_.convergence_steps);
        node_.declare_parameter("SteinICP_parameters.convergence_threshold", 1e-5);
        node_.get_parameter("SteinICP_parameters.convergence_threshold", steinicp_param_.convergence_threshold);
        node_.declare_parameter("SteinICP_parameters.sub_topic", "/velodyne_points");
        node_.get_parameter("SteinICP_parameters.sub_topic", cloud_topic_);
        node_.declare_parameter("SteinICP_parameters.deskew_cloud", false);
        node_.get_parameter("SteinICP_parameters.deskew_cloud", deskew_cloud_);
        node_.declare_parameter("SteinICP_parameters.save_particle_info", false);
        node_.get_parameter("SteinICP_parameters.save_particle_info", save_particle_info_);

        node_.declare_parameter("SteinICP_parameters.max_range", 80.0);
        node_.get_parameter("SteinICP_parameters.max_range", max_range_);
        node_.declare_parameter("SteinICP_parameters.min_range", 0.0);
        node_.get_parameter("SteinICP_parameters.min_range", min_range_);

        node_.declare_parameter("SteinICP_parameters.map_voxel_max_points", 10);
        node_.get_parameter("SteinICP_parameters.map_voxel_max_points", local_map_.max_pointscount_);
        local_map_.max_range_ = max_range_;
        node_.declare_parameter("SteinICP_parameters.map_voxel_size", 1.0);
        node_.get_parameter("SteinICP_parameters.map_voxel_size", local_map_.voxel_size_);

        node_.declare_parameter("SteinICP_parameters.voxelization", false);
        node_.get_parameter("SteinICP_parameters.voxelization", voxelization_);
        if(voxelization_)
        {
            node_.declare_parameter("SteinICP_parameters.voxel_size", 0.01);
            node_.get_parameter("SteinICP_parameters.voxel_size", voxel_size_);
        }

        node_.declare_parameter("SteinICP_parameters.USE_BetaSteinICP", false);
        node_.get_parameter("SteinICP_parameters.USE_BetaSteinICP", use_BetaSteinICP_);
        node_.declare_parameter("SteinICP_parameters.BetaSteinICPOptions.weight_update_loops", 10);
        node_.get_parameter("SteinICP_parameters.BetaSteinICPOptions.weight_update_loops", beta_stein_opt_.weight_update_loop);
        node_.declare_parameter("SteinICP_parameters.USE_Segmentation", true);
        node_.get_parameter("SteinICP_parameters.USE_Segmentation", use_Segmentation_);
        if (cloud_topic_ == "/rslidar_points")
            use_Segmentation_ = false;
        if(use_BetaSteinICP_)
        {
            node_.declare_parameter("SteinICP_parameters.BetaSteinICPOptions.r", 0.2);
            node_.get_parameter("SteinICP_parameters.BetaSteinICPOptions.r", beta_stein_opt_.r);
            node_.declare_parameter("SteinICP_parameters.BetaSteinICPOptions.tau", 0.01);
            node_.get_parameter("SteinICP_parameters.BetaSteinICPOptions.tau", beta_stein_opt_.tau);
            node_.declare_parameter("SteinICP_parameters.BetaSteinICPOptions.beta", 0.0);
            node_.get_parameter("SteinICP_parameters.BetaSteinICPOptions.beta", beta_stein_opt_.beta);
            node_.declare_parameter("SteinICP_parameters.BetaSteinICPOptions.md_iterations", 20);
            node_.get_parameter("SteinICP_parameters.BetaSteinICPOptions.md_iterations", beta_stein_opt_.md_iter);
            node_.declare_parameter("SteinICP_parameters.BetaSteinICPOptions.use_weight_mean", false);
            node_.get_parameter("SteinICP_parameters.BetaSteinICPOptions.use_weight_mean", beta_stein_opt_.use_weight_mean);

        }

        //Scan Context parameters
        node_.declare_parameter("ScanContext_parameters.USE_SC", false);
        node_.get_parameter("ScanContext_parameters.USE_SC", use_sc_);
        node_.declare_parameter("ScanContext_parameters.PC_NUM_RING", 20);
        node_.get_parameter("ScanContext_parameters.PC_NUM_RING", sc_param_.PC_NUM_RING_);
        node_.declare_parameter("ScanContext_parameters.PC_NUM_SECTOR", 60);
        node_.get_parameter("ScanContext_parameters.PC_NUM_SECTOR", sc_param_.PC_NUM_SECTOR_);
        node_.declare_parameter("ScanContext_parameters.PC_MAX_RADIUS", 80.0);
        node_.get_parameter("ScanContext_parameters.PC_MAX_RADIUS", sc_param_.PC_MAX_RADIUS_);


        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: SteinICP parameter loaded\n");
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Particle Count: " << particle_count_
                                               << " Iterations: " << steinicp_param_.iterations
                                               << " Batch Size: " << steinicp_param_.batch_size
                                               << " Learning Rate: " << steinicp_param_.lr);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Subscribe topic " << cloud_topic_);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Use Scan Context " << use_sc_);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Deskew pointcloud " << deskew_cloud_);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Use Voxelization " << voxelization_);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Voxel Size " << voxel_size_);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Use Beta-Stein-ICP " << use_BetaSteinICP_);
        RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: step size: " << beta_stein_opt_.r
                                             <<" mirror descent iterations: " << beta_stein_opt_.md_iter
                                             <<" use weight mean: "<< beta_stein_opt_.use_weight_mean );



//        RCLCPP_INFO(node_.get_logger(), "Loaded parameters");
    }

    void SteinICPOdometry::allocateMemory()
    {
//        std::string steinicp_optimizer = "Adam";
//        steinicp_param_ = std::make_unique<stein_icp::SteinICPParam>(50, 50, 0.02, 2.8, steinicp_optimizer, true);
        this->set_initpose();
        steinicp_odom_ = std::make_unique<stein_icp::BetaSteinICP>(steinicp_param_, init_pose_, beta_stein_opt_);

        source_pcl_.reset(new pcl::PointCloud<Point_t>());
        target_pcl_.reset(new pcl::PointCloud<Point_t>());
        total_map_.reset(new pcl::PointCloud<Point_t>());

        cloud_msg_buffer_.resize_buffer(1000);
        odom_buffer_.resize_buffer(1000);
        poses3_.resize_buffer(100);
        stein_particle_weight_buffer_.resize_buffer(100);

        sc_generator_.set_param(sc_param_);
        cloud_segmentation_ = std::make_shared<ImageProjection>();
//        poses3_.resize(1000);

    }


    void SteinICPOdometry::msg_handler_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        /* here we should also query the optimized state. But before we integrate SteinICP odometry
           in onlineFGO, we do not do that
         */
        //TODO: Add function for query here
        timestamp_current_ = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        new_scan_ = true;
        sensor_msgs::msg::PointCloud2::SharedPtr segmented_msg;
        segmented_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        if(use_Segmentation_)
        {
            cloud_segmentation_->cloudHandler(msg);
            pcl::PointCloud<Point_t>::Ptr segmented_cloud;
            segmented_cloud.reset(new pcl::PointCloud<Point_t>());
            segmented_cloud = cloud_segmentation_->GetSegmentedCloudPure();
            std::cout << "Segmented Cloud Points Number:" << segmented_cloud->size() << std::endl;
            pcl::toROSMsg(*segmented_cloud, *segmented_msg);
            segmented_msg->header = msg->header;
        }
        else
        {
            segmented_msg = msg;
        }


        if(is_firstScan_)
        {
            RCLCPP_INFO(node_.get_logger(), "Received the first scan");
            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Received new scan at " << timestamp_current_.nanoseconds());
            is_firstScan_ = false;
            cloud_msg_buffer_.update_buffer(segmented_msg, timestamp_current_);
            timestamp_last_ = timestamp_current_;
        }
        else if(!skip_scan_)
        {
            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Received new scan at " << timestamp_current_.nanoseconds());
            cloud_msg_buffer_.update_buffer(segmented_msg, timestamp_current_);
            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Msg buffer size: " << cloud_msg_buffer_.size());
            timestamp_last_ = timestamp_current_;

        }
    }


    pcl::PointCloud<Point_t> SteinICPOdometry::deskew_pointcloud(const sensor_msgs::msg::PointCloud2 msg)
    {

        sensor_msgs::msg::PointField timestamp_field;
        for(auto field : msg.fields)
        {
            if(field.name == "t" || field.name == "timestamp" || field.name == "time")
                timestamp_field = field;
        }

        const size_t point_number = msg.width * msg.height;
        const auto new_frame_timestamp = rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec, RCL_ROS_TIME);

        auto extract_timestamps = [&]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> &&it)->std::vector<double>{
            std::vector<double> timestamps;

            timestamps.reserve(point_number);
            for(size_t i = 0; i < point_number; ++it, ++i)
            {
                timestamps.emplace_back(static_cast<double>(*it));
            }

            return timestamps;
        };

        std::vector<double> extracted_timestamps;
        if(timestamp_field.datatype == sensor_msgs::msg::PointField::UINT32)
            extracted_timestamps = extract_timestamps(sensor_msgs::PointCloud2ConstIterator<uint32_t>(msg, timestamp_field.name));
        else if(timestamp_field.datatype == sensor_msgs::msg::PointField::FLOAT32)
            extracted_timestamps = extract_timestamps(sensor_msgs::PointCloud2ConstIterator<float>(msg, timestamp_field.name));
        else if(timestamp_field.datatype == sensor_msgs::msg::PointField::FLOAT64)
            extracted_timestamps = extract_timestamps(sensor_msgs::PointCloud2ConstIterator<double>(msg, timestamp_field.name));

        const auto [min_time_it, max_time_it] = std::minmax_element(extracted_timestamps.begin(), extracted_timestamps.end());
        const double min_timestamp = *min_time_it;
        const double max_timestamp = *max_time_it;
        std::vector<double> timestamps(extracted_timestamps.size());
        std::transform(extracted_timestamps.begin(), extracted_timestamps.end(), timestamps.begin(),
                       [&](const auto &timestamp){
            return (timestamp-min_timestamp) / (max_timestamp-min_timestamp);
        });

        auto pose_number = poses3_.size();
        auto start_frame = poses3_.get_buffer_from_id(pose_number-2);
        auto start_time = poses3_.time_buffer[pose_number-2];
        auto finish_frame = poses3_.get_last_buffer();
        auto finish_time = poses3_.get_last_time();
        auto delta_pose = gtsam::Pose3::Logmap(start_frame.inverse()*finish_frame);
        auto delta_time_ratio = (new_frame_timestamp.seconds()-finish_time.seconds()) / (finish_time.seconds()-start_time.seconds());
        auto new_time_normalized = (new_frame_timestamp.seconds()-min_timestamp) / (max_timestamp-min_timestamp);

        pcl::PointCloud<Point_t>::Ptr frame;
        frame.reset(new pcl::PointCloud<Point_t>());
        pcl::fromROSMsg(msg, *frame);
        auto frame_points = frame->points;

        pcl::PointCloud<Point_t>::Ptr deskewed_pcl;
        deskewed_pcl.reset(new pcl::PointCloud<Point_t>());
        deskewed_pcl->resize(point_number);

        tbb::parallel_for(size_t(0), point_number, [&](size_t i){
           const auto motion = gtsam::Pose3::Expmap((timestamps[i]-new_time_normalized) * delta_pose * delta_time_ratio);
           auto deskewed_point = motion.transformFrom(gtsam::Point3(frame_points[i].x, frame_points[i].y,frame_points[i].z));
           Point_t pt;
           pt.x = deskewed_point.x();
           pt.y = deskewed_point.y();
           pt.z = deskewed_point.z();
           deskewed_pcl->points[i] = pt;
        });

        return *deskewed_pcl;

    }

    void SteinICPOdometry::ICP_processing()
    {
        torch::Tensor odom_tr_last = torch::zeros({6}).to(torch::kCUDA);
        pcl::PointCloud<Point_t>::Ptr deskewed_cloud;
        deskewed_cloud.reset(new pcl::PointCloud<Point_t>());
        pcl::PointCloud<Point_t>::Ptr voxelized_cloud;
        voxelized_cloud.reset(new pcl::PointCloud<Point_t>());
        pcl::PointCloud<Point_t>::Ptr voxelized_cloud_toMap;
        voxelized_cloud_toMap.reset(new pcl::PointCloud<Point_t>());
        pcl::PointCloud<Point_t>::Ptr scds_cloud;
        scds_cloud.reset(new pcl::PointCloud<Point_t>());
        pcl::PointCloud<Point_t>::Ptr cropped_cloud;
        cropped_cloud.reset(new pcl::PointCloud<Point_t>());
        pcl::PointCloud<Point_t>::Ptr target_cloud_lidar_frame;
        target_cloud_lidar_frame.reset(new pcl::PointCloud<Point_t>());
        pcl::PointCloud<Point_t>::Ptr source_cloud_map_searching;
        source_cloud_map_searching.reset(new pcl::PointCloud<Point_t>());


        bool parameter_published = false;
        torch::Tensor cov_N_2 = torch::zeros({6}).to(torch::kCUDA);
        torch::Tensor cov_N_1 = torch::ones({6}).to(torch::kCUDA);

        auto source_msg = sensor_msgs::msg::PointCloud2();
        while(rclcpp::ok())
        {
//            std::unique_lock<std::mutex> uniquelk(variable_mutex_);

            if(is_firstScan_)
            {
                continue;
            }

            if(!parameter_published)
            {
                this->publish_stein_param();
                parameter_published = true;
            }

//            if(is_firstOdom_)
//            {
//                if(cloud_msg_buffer_.size() != 0)
//                {
//                    auto [target_msg, target_timestamp] = cloud_msg_buffer_.get_first_buffer_time_pair_and_pop();
//                    pcl::fromROSMsg(target_msg, *target_pcl_);
//                    target_tr_ = stein_icp::vector2tensor(stein_icp::pcl2vector(target_pcl_), torch::kCUDA);
//                    is_firstOdom_ = false;
//                }
//                continue;
//            }


            if(skip_scan_)
            {
                continue;
            }

            if(!last_optim_finished_)
                continue;

            if(cloud_msg_buffer_.size()==0)
                continue;

            RCLCPP_INFO(node_.get_logger(), "[SteinICP]: Start Reading Buffer");

            //Msg to
            auto [origin_msg, source_timestamp] = cloud_msg_buffer_.get_first_buffer_time_pair_and_pop();
//            uniquelk.lock()

            if (source_timestamp.seconds() - timestamp_odom_.seconds() < msg_buffer_gap_)
                continue;

            auto preprocessing_start = std::chrono::steady_clock::now();
            if(deskew_cloud_ && poses3_.size()>=2)
            {
                auto deskew_start = std::chrono::steady_clock::now();
                *deskewed_cloud = this->deskew_pointcloud(*origin_msg);
                auto deskew_end = std::chrono::steady_clock::now();
                std::chrono::duration<double> deskew_duration = deskew_end - deskew_start;
                std::cout << "Deskew time: " << deskew_duration.count() << "sec" << std::endl;
            }
            else
            {
                source_msg = *origin_msg;
                pcl::fromROSMsg(source_msg, *deskewed_cloud);
            }

            *cropped_cloud = this->crop_pointcloud(deskewed_cloud);

            *voxelized_cloud = *cropped_cloud;

            RCLCPP_INFO_STREAM(node_.get_logger(), "Points number: "<< cropped_cloud->points.size());
            auto voxel_start = std::chrono::steady_clock::now();
            if(voxelization_)
            {
                *voxelized_cloud_toMap = this->downsample_uniform(cropped_cloud, 0.5*voxel_size_);
                *voxelized_cloud = this->downsample_uniform(voxelized_cloud_toMap, 1.5*voxel_size_);
                RCLCPP_INFO_STREAM(node_.get_logger(),
                                   "Points number after voxelization: " << voxelized_cloud->points.size());
            }

            auto voxel_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> voxel_duration = voxel_end - voxel_start;

            //Scan context
            *scds_cloud = *voxelized_cloud;
            auto sc_start = std::chrono::steady_clock::now();
            if(use_sc_)
            {
                auto sc_start = std::chrono::steady_clock::now();
                auto [scan_context, bin_idx] = sc_generator_.makeScancontext(*voxelized_cloud);
                scan_context_.emplace_back(source_timestamp, scan_context);
                *scds_cloud = sc_generator_.DownsampleCloud();
                scds_cloud_.emplace_back(source_timestamp, *scds_cloud);
                RCLCPP_INFO_STREAM(node_.get_logger(),
                                   "Scan Context Downsample Points Number : " << scds_cloud->points.size());
                auto sc_end = std::chrono::steady_clock::now();
                std::chrono::duration<double> sc_duration = sc_end-sc_start;
                RCLCPP_INFO_STREAM(node_.get_logger(), "[ScanContext]: Finished in " << sc_duration.count() <<"s");

//                steinicp_odom_->add_scanContext(scan_context, bin_idx);
            }
            auto sc_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> sc_duration = sc_end - sc_start;

            auto initial_guess = this->position_prediction(source_timestamp);
//            pcl::transformPointCloud(*scds_cloud, *source_pcl_, initial_guess.matrix());
            *source_pcl_ = *scds_cloud;
            pcl::transformPointCloud(*source_pcl_, *source_cloud_map_searching, initial_guess.matrix());

            //Align Process
            RCLCPP_INFO(node_.get_logger(), "[SteinICP]: --------------------Start Align Process----------------");
            this->set_initpose();
            RCLCPP_INFO_STREAM(node_.get_logger(),
                               "Source Cloud Points Number : " << source_pcl_->points.size());

            source_tr_ = stein_icp::vector2tensor(stein_icp::pcl2vector(source_pcl_), torch::kCUDA);
            if (!local_map_.Empty())
            {
                *target_pcl_ = local_map_.GetNeighbourMap(*source_cloud_map_searching);
//                *target_pcl_ = local_map_.GetMap();
                if(target_pcl_->size() == 0)
                {
                    *target_pcl_ = local_map_.GetMap();
                }
                RCLCPP_INFO_STREAM(node_.get_logger(),
                                   "Local Map Points Number : " << target_pcl_->size());
                pcl::transformPointCloud(*target_pcl_, *target_cloud_lidar_frame, initial_guess.inverse().matrix());
//                *target_pcl_ = this->downsample_voxel(target_pcl_, voxel_size_);
                target_tr_ = stein_icp::vector2tensor(stein_icp::pcl2vector(target_cloud_lidar_frame), torch::kCUDA);
                steinicp_odom_->add_cloud(source_tr_, target_tr_, init_pose_.clone());

            }
            else
            {
//                steinicp_odom_->add_cloud(source_tr_, init_pose_);
                local_map_.AddPointCloud(*voxelized_cloud_toMap, initial_guess);
                poses3_.update_buffer(initial_guess, source_timestamp);
            }
            auto preprocessing_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> preprocessing_duration = preprocessing_end-preprocessing_start;
            std::cout << "Preprocessing Time: " << preprocessing_duration.count() << std::endl;

            auto align_start = std::chrono::steady_clock::now();
            stein_icp::SteinICP_State align_state = steinicp_odom_->stein_align();
            auto align_end = std::chrono::steady_clock::now();
            align_duration_ = align_end - align_start;
            RCLCPP_INFO_STREAM(node_.get_logger(),
                               "Align Process Time : " << align_duration_.count());

            //check whether aligning is successed
            if(align_state != SteinICP_State::ALIGN_SUCCESS)
            {
                continue;
            }

            auto correction_tr = steinicp_odom_->get_transformation();
            auto cov_tr = steinicp_odom_->get_distribution();
            auto stein_particles = steinicp_odom_->get_particles();
            auto stein_weights = steinicp_odom_->get_stein_weight();

//            correction_tr = (( 2*cov_N_1 + cov_N_2 ).div(2*cov_N_1+cov_N_2+cov_tr)).mul(correction_tr);
//            cov_N_2 = cov_N_1.clone();
//            cov_N_1 = (torch::ones({6}).to(torch::kCUDA) - (( 2*cov_N_1 + cov_N_2 ).div(2*cov_N_1+cov_N_2+cov_tr)))
//                       * ( 2*cov_N_1 + cov_N_2 );

            odom_tr_last = correction_tr.clone();

            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: x:" << correction_tr[0].item<float>()
                                                                    << ", y:" << correction_tr[1].item<float>()
                                                                    << ", z:" << correction_tr[2].item<float>());
            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: var(x):" << cov_tr[0].item<float>()
                                                                         << ", var(y):" << cov_tr[1].item<float>()
                                                                            << ", var(z):" << cov_tr[2].item<float>());
            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Finish Align Process for timestamp " << source_timestamp.nanoseconds() << "s");
            RCLCPP_INFO_STREAM(node_.get_logger(), "[SteinICP]: Align Process is "
                                                    << timestamp_current_.seconds() - source_timestamp.seconds()
                                                    << "s delayed compared to the received msg");

            //Odometry
            auto cov_gtsam = stein_icp::tensor2gtsamPose3(cov_tr);
//            auto pose3_current = gtsam::Pose3(stein_icp::tensor2Matrix(correction_tr))*initial_guess;
            auto pose3_current = initial_guess * stein_icp::tensor2gtsamPose3(correction_tr) ;
//            pose3_current = this->constraint_transform(pose3_current);
            if(cov_gtsam.x()  + cov_gtsam.y() > 15)
            {
//                pcl::PointCloud<Point_t> transformed_map;
//                pcl::transformPointCloud(*target_pcl_, transformed_map, pose3_current.inverse().matrix());
//                *voxelized_cloud = *voxelized_cloud + transformed_map;
//                this->downsample_voxel(voxelized_cloud_toMap, voxel_size_*2);
                local_map_.Clear();
                poses3_.clean();
                steinicp_odom_.reset(new stein_icp::BetaSteinICP(steinicp_param_, init_pose_.clone(), beta_stein_opt_));
                local_map_.AddPointCloud(*voxelized_cloud_toMap, pose3_current);

            }
            else
            {
                local_map_.AddPointCloud(*voxelized_cloud_toMap, pose3_current);
            }
            poses3_.update_buffer(pose3_current, source_timestamp);
            stein_particle_weight_buffer_.update_buffer({stein_particles,stein_weights},source_timestamp) ;
            covariance_.push_back(cov_gtsam);
            original_msg_ = *origin_msg;
            deskewed_cloud_.emplace_back(source_timestamp, *deskewed_cloud);
            timestamp_odom_ = source_timestamp;
            source_cloud_vec_.emplace_back(source_timestamp, *source_cloud_map_searching);
            target_cloud_vec_.emplace_back(source_timestamp, *target_pcl_);
            steinicp_runtime_ = align_duration_.count();
            preprocessing_runtime_ = preprocessing_duration.count();
            odom_finished_ = true;
//            fgo::data_types::Odom odom;
        }
    }


    void SteinICPOdometry::set_initpose()
    {
        torch::Tensor lb = torch::tensor({-0.2, -0.1, -0.001, -0.02, -0.02, -0.1}).to(torch::kCUDA);
        torch::Tensor ub = torch::tensor({0.2, 0.1, 0.001, 0.02, 0.02, 0.1}).to(torch::kCUDA);
        init_pose_ = stein_icp::initialize_particles(particle_count_, torch::kCUDA, ub, lb);
    }


    void SteinICPOdometry::set_initpose(const torch::Tensor &delta_last)
    {
        torch::Tensor lb = delta_last.view({6}) + torch::tensor({-0.8, -0.1, -0.05, -0.1753/4, -0.1753/4, -0.1753}).to(torch::kCUDA);
        torch::Tensor ub = delta_last.view({6}) + torch::tensor({0.8, 0.1, 0.05, 0.1753/4, 0.1753/4, 0.1753}).to(torch::kCUDA);
        init_pose_ = stein_icp::initialize_particles(particle_count_, torch::kCUDA, ub, lb);
        RCLCPP_INFO_STREAM(node_.get_logger(), "init pose mean: " << torch::mean(init_pose_,1)[0].item<float>());
    }

    pcl::PointCloud<Point_t> SteinICPOdometry::downsample_voxel(const pcl::PointCloud<Point_t>::Ptr &cloud, double voxel_size)
    {
        voxel_grid_.setInputCloud(cloud);
        voxel_grid_.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_grid_.filter(*cloud);
        return *cloud;
    }

    pcl::PointCloud<Point_t> SteinICPOdometry::downsample_uniform(const pcl::PointCloud<Point_t>::Ptr &cloud, double voxel_size)
    {
        uniform_sampling_.setInputCloud(cloud);
        uniform_sampling_.setRadiusSearch(voxel_size);
        uniform_sampling_.filter(*cloud);
        return *cloud;
    }

    pcl::PointCloud<Point_t> SteinICPOdometry::crop_pointcloud(
            const pcl::PointCloud<Point_t>::Ptr &pointcloud)
    {
        pcl::PointCloud<Point_t> cropped_cloud;
        std::copy_if(pointcloud->points.begin(),
                         pointcloud->points.end(),
                         std::back_inserter(cropped_cloud.points),
                         [&](const auto &pt){
                        auto norm = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
                        return norm < max_range_*max_range_ && norm > min_range_*min_range_;
        });
        return cropped_cloud;
    }

    gtsam::Pose3 SteinICPOdometry::position_prediction(const rclcpp::Time &new_time)
    {
        const size_t N = poses3_.size();
        auto delta_pose = gtsam::Pose3();

        if (N < 2)
        {
//            delta_pose_.push_back(delta_pose);
            if (poses3_.size() == 0)
                return gtsam::Pose3();
            else
                return poses3_.get_last_buffer() * gtsam::Pose3();
        }

        auto start_frame = poses3_.get_buffer_from_id(N-2);
        auto start_time = poses3_.time_buffer[N-2];
        auto finish_frame = poses3_.get_last_buffer();
        auto finish_time = poses3_.get_last_time();
        auto delta_time = finish_time.seconds()-start_time.seconds();
        auto new_delta_time = new_time.seconds() - finish_time.seconds();
//        delta_pose = start_frame.inverse()*finish_frame;
        delta_pose = gtsam::Pose3(start_frame.matrix().inverse() * finish_frame.matrix());
        auto time_ratio = new_delta_time/delta_time;
        delta_pose = gtsam::Pose3::Expmap(time_ratio * gtsam::Pose3::Logmap(delta_pose));

//        delta_pose_.push_back(delta_pose);
//        gtsam::Pose3 delta_sum;
//        if (delta_pose_.size() < 10)
//        {
//            for (auto delta : delta_pose_)
//            {
//                  delta_sum = delta_sum * delta;
//            }
//            delta_pose = gtsam::Pose3::Expmap(gtsam::Pose3::Logmap(delta_sum) / delta_pose_.size());
//        }
//        else
//        {
//            for (auto it = delta_pose_.end()-10; it != delta_pose_.end(); it++)
//            {
//                delta_sum = delta_sum * (*it);
//            }
//            delta_pose = gtsam::Pose3::Expmap(gtsam::Pose3::Logmap(delta_sum) / 10);
//        }

//        auto initial_guess = finish_frame * delta_pose;
        auto initial_guess = gtsam::Pose3(finish_frame.matrix() * delta_pose.matrix());
        return initial_guess;
    }

    double SteinICPOdometry::constraint_value(double value, double limit)
    {
        if (value < -limit)
            value = -limit;
        else if (value > limit)
            value = limit;

        return value;
    }

    gtsam::Pose3 SteinICPOdometry::constraint_transform(const gtsam::Pose3 &pose)
    {
//        double rx = this->constraint_value(pose.rotation().roll(), 0.05);
//        double ry = this->constraint_value(pose.rotation().pitch(), 0.05);
        double tz = this->constraint_value(pose.z(), 0.1);

        double rz = pose.rotation().yaw();
        double rx = pose.rotation().roll();
        double ry = pose.rotation().pitch();
        double tx = pose.x();
        double ty = pose.y();

        return gtsam::Pose3(gtsam::Rot3::RzRyRx(rx, ry, rz), gtsam::Point3(tx, ty, tz));

    }

    void SteinICPOdometry::publish_stein_param()
    {
         stein_particle_msgs::msg::SteinParameters stein_param_msg;
         stein_param_msg.optimizer = steinicp_param_.optimizer;
         stein_param_msg.learning_rate = steinicp_param_.lr;
         stein_param_msg.iterations = steinicp_param_.iterations;
         stein_param_msg.batch_size = steinicp_param_.batch_size;
         stein_param_msg.particle_count = particle_count_;
         stein_param_msg.early_stop = steinicp_param_.using_EarlyStop;
         stein_param_msg.converge_steps = steinicp_param_.convergence_steps;
         stein_param_msg.converge_threshold = steinicp_param_.convergence_threshold;
         stein_param_msg.correspondence_distance = steinicp_param_.max_dist;

         stein_param_msg.point_range = {min_range_, max_range_};
         stein_param_msg.voxelization = voxelization_;
         stein_param_msg.voxel_size = voxel_size_;
         stein_param_msg.map_voxel_size = local_map_.voxel_size_;
         stein_param_msg.map_voxel_max_points = local_map_.max_pointscount_;
         stein_param_msg.weight_mean = beta_stein_opt_.use_weight_mean;
         stein_param_msg.md_iterations = beta_stein_opt_.md_iter;
         stein_param_msg.md_learning_rate = beta_stein_opt_.r;

         stein_param_publisher_->publish(stein_param_msg);

    }



    void SteinICPOdometry::publish_odometry()
    {
         auto odom_msg = geometry_msgs::msg::PointStamped();
         const auto last_pose = poses3_.get_last_buffer();
         auto timestamp = poses3_.get_last_time();
         odom_msg.header.frame_id = "odom_steinicp";
         odom_msg.header.stamp = timestamp;
         odom_msg.point.x = last_pose.x();
         odom_msg.point.y = last_pose.y();

         auto pose_msg = geometry_msgs::msg::PoseStamped();
         pose_msg.header.frame_id = "odom_steinicp";
         pose_msg.header.stamp = timestamp;

//         odom_msg.point.z = position_current_(2,0);

         pose_msg.pose.position.x = last_pose.x();
         pose_msg.pose.position.y = last_pose.y();
         pose_msg.pose.position.z = last_pose.z();
//         pose_msg.pose.position.z = position_current_(2,0);
         pose_msg.pose.orientation.x = last_pose.rotation().toQuaternion().x();
         pose_msg.pose.orientation.y = last_pose.rotation().toQuaternion().y();
         pose_msg.pose.orientation.z = last_pose.rotation().toQuaternion().z();
         pose_msg.pose.orientation.w = last_pose.rotation().toQuaternion().w();
//         pose_msg.pose.orientation.w = ;

         path_msg_.poses.push_back(pose_msg);
         path_msg_.header.frame_id = "odom_steinicp";

         auto pose_cov_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
         pose_cov_msg.header.frame_id = "odom_steinicp";
         pose_cov_msg.header.stamp = timestamp;
         pose_cov_msg.pose.pose = pose_msg.pose;
         pose_cov_msg.pose.covariance[0] = (covariance_.end()-1)->x();
         pose_cov_msg.pose.covariance[1] = (covariance_.end()-1)->y();
         pose_cov_msg.pose.covariance[2] = (covariance_.end()-1)->z();
         pose_cov_msg.pose.covariance[3] = (covariance_.end()-1)->rotation().roll();
         pose_cov_msg.pose.covariance[4] = (covariance_.end()-1)->rotation().pitch();
         pose_cov_msg.pose.covariance[5] = (covariance_.end()-1)->rotation().yaw();

         odom_publisher_->publish(odom_msg);
         pose_publisher_->publish(pose_msg);
         path_publisher_->publish(path_msg_);
         pose_cov_publisher_->publish(pose_cov_msg);


//         if (poses3_.size() > 100)
//             poses3_.erase(poses3_.begin(), poses3_.begin()+60);

    }


    void SteinICPOdometry::publish_scancontext()
    {
       auto sc_msg = sensor_msgs::msg::PointCloud2();
       pcl::PointCloud<Point_t>::Ptr sc_pcl;
       sc_pcl.reset(new pcl::PointCloud<Point_t>());

       for (auto scan_context_pair : scan_context_)
       {
           auto scan_context = scan_context_pair.second;
           for(int row = 0; row<scan_context.rows(); row++)
           {
               for(int col = 0; col<scan_context.cols(); col++)
               {
                   int idx = row*scan_context.cols()+col;
                   Point_t pt;
                   pt.x = col-scan_context.cols()/2;
                   pt.y = row;
                   pt.z = scan_context(row, col);
                   sc_pcl->points.push_back(pt);
               }
           }
           pcl::toROSMsg(*sc_pcl, sc_msg);
           sc_msg.header.frame_id = "odom_steinicp";
           sc_msg.header.stamp = scan_context_pair.first;
           sc_publisher_->publish(sc_msg);
       }
       scan_context_.clear();

    }


    void SteinICPOdometry::publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                                         std::vector<std::pair<rclcpp::Time, pcl::PointCloud<Point_t>>> &cloud_vec)
    {
        pcl::PointCloud<Point_t> cloud_pcl;
        for (auto cloud_pair : cloud_vec)
        {
            this->publish_cloud(publisher, cloud_pair.second, cloud_pair.first);
        }
        cloud_vec.clear();
    }

    void SteinICPOdometry::publish_cloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
                                         pcl::PointCloud<Point_t> cloud,
                                         rclcpp::Time)
    {
        auto cloud_msg = sensor_msgs::msg::PointCloud2();
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "odom_steinicp";
        publisher->publish(cloud_msg);
    }

    void SteinICPOdometry::publish_particle_info()
    {
        stein_particle_msgs::msg::SteinParticle particle_msg;
        auto particle_weight_to_pub = stein_particle_weight_buffer_.get_last_buffer();
        auto timestamp = stein_particle_weight_buffer_.get_last_time();
        auto particles = particle_weight_to_pub.first;
        auto weights = particle_weight_to_pub.second;
        particle_msg.x = std::vector<double>(particles.begin(), particles.begin()+particle_count_);
        particle_msg.y = std::vector<double>(particles.begin()+particle_count_, particles.begin()+2*particle_count_);
        particle_msg.z = std::vector<double>(particles.begin()+2*particle_count_, particles.begin()+3*particle_count_);
        particle_msg.roll = std::vector<double>(particles.begin()+3*particle_count_, particles.begin()+4*particle_count_);
        particle_msg.pitch = std::vector<double>(particles.begin()+4*particle_count_, particles.begin()+5*particle_count_);
        particle_msg.yaw = std::vector<double>(particles.begin()+5*particle_count_, particles.begin()+6*particle_count_);
        particle_msg.weights = weights;
        particle_msg.header.stamp = timestamp;
        stein_particle_publisher_->publish(particle_msg);

    }

    void SteinICPOdometry::publish_runtime()
    {
        stein_particle_msgs::msg::Runtime runtime_msg;
        runtime_msg.preprocessing_time = preprocessing_runtime_;
        runtime_msg.steinicp_time = steinicp_runtime_;
        runtime_publisher_->publish(runtime_msg);
    }

    void SteinICPOdometry::publish_thread_cb()
    {
        sensor_msgs::msg::PointCloud2 total_map_msg;
        auto original_msg = original_msg_;
        while(rclcpp::ok())
        {
//            std::this_thread::sleep_for(std::chrono::seconds(2));
            if(!odom_finished_)
            {
                continue;
            }

            odom_finished_ = false;
            this->publish_odometry();
            this->publish_cloud(source_publisher_, source_cloud_vec_);
            this->publish_cloud(neighbourmap_publisher_, target_cloud_vec_);

            *total_map_ = local_map_.GetMap();
            this->publish_cloud(localmap_publisher_, *total_map_, timestamp_odom_);

            original_msg = original_msg_;
            original_msg.header.frame_id = "odom_steinicp";
            original_publisher_->publish(original_msg);

            if (deskew_cloud_)
                this->publish_cloud(deskewed_publisher_, deskewed_cloud_);
            if (use_sc_)
            {
                this->publish_scancontext();
                this->publish_cloud(ds_publisher_, scds_cloud_);
            }
            if (save_particle_info_)
            {
                this->publish_particle_info();
            }

            this->publish_runtime();

        }
    }



}

