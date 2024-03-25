//
// Created by haoming on 22.08.23.
//
#pragma once
#ifndef STEINICP_BETASTEINICP_H
#define STEINICP_BETASTEINICP_H

#include "SteinICP.h"

namespace stein_icp
{
    struct BetaSteinICPOpt
    {
        BetaSteinICPOpt() = default;
        BetaSteinICPOpt(double r ,
                         double tau ,
                         int    md_iter,
                         double beta,
                         int weight_update_loop,
                         bool use_weight_mean
                         ):
                         r(r), tau(tau), md_iter(md_iter), beta(beta),
                         weight_update_loop(weight_update_loop), use_weight_mean(use_weight_mean)
                         {}
        double r = 0.02;
        double tau = 0.01;
        int md_iter = 20;
        double beta = 0;
        int weight_update_loop = 20;
        bool use_weight_mean = false;
    };

    class BetaSteinICP: public SteinICP
    {

    private:
        torch::Tensor w_p_;
        torch::Tensor real_weight_;
        double r_;
        double tau_;
        double md_iter_;
        double beta_;
        int weight_update_loop_;
        std::vector<double> stein_distance_;
        bool use_weight_mean_;


        torch::Tensor pose_parameters_tsr_;
        torch::Tensor sgd_gradient_;
    private:
        /**
         * @brief update the the weight for particles
         */
        void weight_update(const torch::Tensor &Kernel,
                           const torch::Tensor &bandwidth,
                           const torch::Tensor &pairwise_dist,
                           const torch::Tensor &sgd_grad);

        /**
         * @param pose_parameters   size: particle_count*6
         * @param sgd_grad
         * @return
         */
        torch::Tensor phi(const torch::Tensor& pose_parameters, const torch::Tensor &sgd_grad, const torch::Tensor& Kernel);

        /**
         * @brief
         */
        std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> get_kernel(const torch::Tensor& x1, const torch::Tensor& x2);


    public:
        BetaSteinICP() = default;
        explicit BetaSteinICP(SteinICPParam   &param,
                              const torch::Tensor   &init_pose,
                              const BetaSteinICPOpt &opt);

        /**
         * \brief stein-icp algorithm
         * \return pose particles, size: particle_count*6
         */
        SteinICP_State stein_align();

        torch::Tensor get_transformation();

        std::vector<double> get_stein_dist();

        std::vector<double> get_weight();

        std::vector<double> get_stein_weight();

    };
}

#endif //STEINICP_BETASTEINICP_H
