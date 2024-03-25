//
// Created by haoming on 26.05.23.
//

#ifndef STEIN_ICP_STEINICP_H
#define STEIN_ICP_STEINICP_H

#pragma once

#include <tuple>
#include <vector>
#include <chrono>

#include <pcl/filters/filter.h>
#include <torch/torch.h>
#include <Eigen/Eigen>

#include "knn.h"
#include "types.h"
#include "steinicp_utils.h"


namespace stein_icp
{

    /**
     *  parameter structure
     */
    struct SteinICPParam
    {
        SteinICPParam() = default;

        /**
         *
         * @param iter
         * @param batch_size
         * @param lr
         * @param max_dist          double input, initialize to tensor
         * @param optimizer_class
         * @param normalize
         */
        SteinICPParam(
                int                  iter,
                int                  batch_size,
                double               lr,
                double               max_dist,
                std::string         &optimizer_class,
                bool                 normalize,
                bool                 using_EarlyStop = false,
                int                  convergence_steps = 5,
                double               convergence_threshold = 1e-5
        );

        int iterations = 50;
        int batch_size = 50;
        double lr = 0.02;
        double max_dist = 1.0;
        bool normalize = true;
        std::string optimizer = "Adam";
        bool using_EarlyStop = false;
        int convergence_steps = 5;
        double convergence_threshold = 1e-5;

    };

    enum SteinICP_State{
        ALIGN_SUCCESS = 1,
        NO_OPTIMIZER = 2,
        LACK_CLOUD = 3
    };

    class SteinICP
    {

    protected: // vairables

        /**
         * \param x, y, z, roll, pitch, yaw:  with size particle_size*1*1, device type "cuda"
         * \param source_cloud,  target_cloud:  tensors point_count*3, device type "cuda"
         * \param max_dist: using to filter the correspondence
         */
        //from constructor or update function
        torch::Tensor source_cloud_, target_cloud_;
        torch::Tensor sourceKNN_idx_;
        int K_source_ = 100;
        torch::Tensor x_, y_, z_, roll_, pitch_, yaw_;
        int iter_;
        int batch_size_;
        torch::Tensor max_dist_;
        double lr_;
        std::string optimizer_class_;
        bool normalize_cloud_;

        //internal private
        bool optimizer_set_ = true;
        bool is_first_cloud_ = true;
        long particle_size_;
        torch::Tensor particle_size_tsr_;
        torch::Tensor batch_size_t_;
        std::unique_ptr<torch::optim::Optimizer> optimizer_ptr_;
        torch::Tensor cloud_last_;
        torch::Tensor max_abs_;
        torch::Tensor max_s_;
        torch::Tensor max_t_;
        torch::Tensor gradient_scaling_factor_;
        std::vector<torch::Tensor> pose_parameters_;

        torch::Tensor x_last_;
        torch::Tensor y_last_;
        torch::Tensor z_last_;
        torch::Tensor roll_last_;
        torch::Tensor pitch_last_;
        torch::Tensor yaw_last_;

        bool using_EarlyStop_ = false;
        int convergence_steps_ = 5;
        double convergence_threshold_ = 1e-5;
        torch::Tensor convergence_threshold_tsr_;

        torch::Tensor delta_t_tsr_ ;
        torch::Tensor delta_R_tsr_ ;
        torch::Tensor var_t_tr_ ;
        torch::Tensor var_R_tr_ ;
        torch::Tensor mean_log_tsr_ ;
        torch::Tensor var_log_tsr_ ;
        torch::Tensor obj_tsr_ ;

        //result output
        torch::Tensor particle_pose_;

        //Scan Context
        torch::Tensor scan_context_;
        torch::Tensor bin_pt_map_;
        bool use_scan_context_ = false;

    protected: // functions

        /**
         *  set optimizer
         */
        void set_optimizer();

        /**
         * \brief normalize the absolute value of global clouds(source_cloud, target cloud)    to [0,1]
         * \return return normalized source, target and the maximum distance max_abs
         */
        void normalize_clouds();

        /**
         *
         */
        void allocate_memory_for_log();

        /**
         *  \brief  generate mini-batch cloud from source cloud
         *  \return mini-batch from source, size : batchize*3
         */
        std::tuple<torch::Tensor, torch::Tensor> mini_batch_generator();

        std::tuple<torch::Tensor, torch::Tensor> mini_batch_generator_inLoop();

        /**
         *  \brief find K nearest poitns for each point in source cloud
         */
        void sourcecloud_KNN();

        /**
         *  \brief angle parameter to rotation matrix
         *  \return rotaion matrix particle_count*3*3
         */
        torch::Tensor Mat_rot();

        /**
         *  \brief translation parameter to matrix
         *  \return size particle_count *3
         */
        torch::Tensor Mat_trans();

        /**
         *  \brief filter the points
         */
        torch::Tensor point_filter(const torch::Tensor &cloud, const torch::Tensor &distance);



        /**
         * \brief calculation the partial derivative of rotation matrix
         * \return a vector containing 3 tensor. roll, pitch, yaw, particle_size*3*3
         */
        torch::Tensor partial_derivative(
                const torch::Tensor &roll,
                const torch::Tensor &pitch,
                const torch::Tensor &yaw         );


        /**
         * \brief calculate sgd-gradient
         * \param cloud_paired are in size of particle*batch_size*3
         * \return return sgn gradient. size: particle_size*6
         *                              value: mean_gradient * source point count
         */
        torch::Tensor sgd_grad(const torch::Tensor &source_paired,
                               const torch::Tensor &transformed_s_paired,
                               const torch::Tensor &target_paired,
                               const torch::Tensor &partial_derivatives,
                               const torch::Tensor &scaling_factor);

        /**
         *
         * @param pose_parameters   size: particle_count*6
         * @param sgd_grad
         * @return
         */
        torch::Tensor phi(torch::Tensor pose_parameters, const torch::Tensor &sgd_grad);

        /**
         *
         */
        torch::Tensor get_kernel(const torch::Tensor& x1, const torch::Tensor& x2);

        /**
         *@brief whether translation & rotation delta smaller than a threshold
         */
        bool is_trans_converged();

        bool is_rot_converged();

    public: // functions
        /**
         *
         * \brief initialize the Class SteinICP
         *
         * \param x, y, z, roll, pitch, yaw:  with size particle_size*1*1, device type "cuda"
         * \param source_cloud,  target_cloud:  tensors point_count*3, device type "cuda"
         * \param max_dist: using to filter the correspondence
         *
         */

        explicit SteinICP(SteinICPParam &parameters, const torch::Tensor &init_pose);

        ~SteinICP() = default;

        /**
         * @brief add new souce and target cloud to register
         * @param new_cloud
         * @param target
         * @param init_pose
         */
        void add_cloud(const torch::Tensor &new_cloud, const torch::Tensor &target, const torch::Tensor &init_pose);

        void add_cloud(const torch::Tensor &source, const torch::Tensor &init_pose);

        virtual /**
         * \brief stein-icp algorithm
         * \return pose particles, size: particle_count*6
         */
        SteinICP_State stein_align();

        /**
         * @brief get the raw particles of poses
         * @return
         */
        std::vector<double> get_particles();

        /**
         * @brief get the mean value of poses
         * @return
         */
        torch::Tensor get_transformation();

        /**
         * @brief return the distribution of particles
         */
        torch::Tensor get_distribution();

        /**
         * @brief return the convergence log
         */
        std::tuple<std::vector<double>, std::vector<double>> get_delta_log();

        std::tuple<std::vector<double>, std::vector<double>> get_var_log();

        std::vector<double> get_mean_log();

        std::vector<double> get_var6_log();

        std::vector<double> get_obj_log();

        void reset();


        /**
         * \brief ignore the dimension of particle_size.
         * \param source                    particle*batch_size * 3
         * \param transformed_source        particle*batch_size * 3
         * \param target                    target_point_count * 3
         *
         *  \return source_paired           particle_size *batch_size*3
         *          transform_paired        particle_sile *batch_size*3
         *          target_paired           particle_size *batch_size*3
         */
        std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> get_correspondence(
                                         const torch::Tensor &source,
                                         const torch::Tensor &transformed_source,
                                         const torch::Tensor &target);

        /**
         * @brief Adapt the max distance for point filter
         * @param max_dist
         */
        void set_max_dist(double max_dist);



    };




}

#endif //STEIN_ICP_STEINICP_H
