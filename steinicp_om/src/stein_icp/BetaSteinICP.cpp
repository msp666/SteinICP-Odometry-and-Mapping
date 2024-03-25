//
// Created by haoming on 22.08.23.
//

#include "BetaSteinICP.h"

namespace stein_icp
{
        BetaSteinICP::BetaSteinICP(SteinICPParam   &parameters,
                                   const torch::Tensor   &init_pose,
                                   const BetaSteinICPOpt &opt):
                                   SteinICP(parameters, init_pose.clone())
        {
            particle_size_ = init_pose.size(1);
            particle_size_tsr_ = torch::tensor({particle_size_}).to(torch::kCUDA);
            batch_size_t_ = torch::tensor({batch_size_}).to(torch::kCUDA);
            x_     = init_pose[0].view({particle_size_, 1, 1});
            y_     = init_pose[1].view({particle_size_, 1, 1});
            z_     = init_pose[2].view({particle_size_, 1, 1});
            roll_  = init_pose[3].view({particle_size_, 1, 1});
            pitch_ = init_pose[4].view({particle_size_, 1, 1});
            yaw_   = init_pose[5].view({particle_size_, 1, 1});
            pose_parameters_tsr_ = torch::stack({x_, y_, z_, roll_, pitch_, yaw_}).squeeze({2,3}).transpose(0,1);
            w_p_ = torch::ones({particle_size_, 1}).to(torch::kCUDA)/particle_size_;
            sgd_gradient_ = torch::zeros({particle_size_, 6}).to(torch::kCUDA);

            iter_ = parameters.iterations;
            normalize_cloud_ = parameters.normalize;
            batch_size_ = parameters.batch_size;
            max_dist_ = torch::tensor({parameters.max_dist}).to(torch::kCUDA);
            optimizer_class_ = parameters.optimizer;
            lr_ = parameters.lr;

            using_EarlyStop_ = parameters.using_EarlyStop;
            convergence_steps_ = parameters.convergence_steps;
            convergence_threshold_ = parameters.convergence_threshold;
            convergence_threshold_tsr_ = torch::tensor({convergence_threshold_}).to(torch::kCUDA);

            r_ = opt.r;
            tau_ = opt.tau;
            beta_ = opt.beta;
            md_iter_ = opt.md_iter;
            weight_update_loop_ = opt.weight_update_loop;
            use_weight_mean_ = opt.use_weight_mean;

            set_optimizer();
        }


        SteinICP_State BetaSteinICP::stein_align()
        {
            torch::Tensor particle_pose = torch::empty({6, particle_size_, 1}).to(torch::kCUDA);


            if(!optimizer_set_)
                return SteinICP_State::NO_OPTIMIZER;

            if(is_first_cloud_)
            {
                is_first_cloud_ = false;
                return SteinICP_State::LACK_CLOUD;
            }

            int convergence_counter = 0;
            stein_distance_.clear();

            real_weight_ = torch::ones({particle_size_, 1}).to(torch::kCUDA);
            w_p_ = torch::ones({particle_size_, 1}).to(torch::kCUDA)/particle_size_;

            double grad_time;
            /** SVGD iterations */
            this->allocate_memory_for_log();

            auto [mini_batch, target_batch] = mini_batch_generator();
//            this->sourcecloud_KNN();
            auto opt_start = std::chrono::steady_clock::now();
            for(int epoch = 0; epoch < iter_; epoch++)
            {

//                expand minibatch and target to match the particle size
//                auto [mini_batch, target_batch] = this->mini_batch_generator_inLoop();
                auto grad_start = std::chrono::steady_clock::now();
                torch::Tensor mini_batch2 = mini_batch[epoch].expand({particle_size_, batch_size_, 3}).to(torch::kCUDA);
                torch::Tensor targe_batch_it = target_batch[epoch];

                torch::Tensor mat_rot = Mat_rot();
                torch::Tensor mat_trans = Mat_trans();

                torch::Tensor source_transformed = torch::einsum("pmc, prc->pmr", {mini_batch2, mat_rot})
                                                   + mat_trans.view({particle_size_, 1, 3}).to(torch::kCUDA);

                const auto [source_paired, transformed_s_paired, target_paired] =
                        get_correspondence(mini_batch2, source_transformed, targe_batch_it);
                torch::Tensor partial_derivatives = partial_derivative(roll_.clone(), pitch_.clone(), yaw_.clone());

                sgd_gradient_ = sgd_grad(source_paired, transformed_s_paired, target_paired, partial_derivatives, gradient_scaling_factor_);
                pose_parameters_tsr_ = torch::stack({x_, y_, z_, roll_, pitch_, yaw_}).squeeze({2,3}).transpose(0,1);
                pose_parameters_tsr_ = pose_parameters_tsr_.detach().requires_grad_(true);

                torch::Tensor stein_grad = torch::zeros_like(sgd_gradient_) ;

                if(particle_size_ > 1)
                {

                    const auto[Kernel_SVGD, bandwidth, pairwise_dist] = get_kernel(pose_parameters_tsr_,
                                                                                   pose_parameters_tsr_.detach());
                    stein_grad = phi(pose_parameters_tsr_, -sgd_gradient_, Kernel_SVGD);

//                    if(//epoch%weight_update_loop_ == 0 ||
//                         epoch ==  iter_-1)
//                    {
//                        this->weight_update(Kernel_SVGD, bandwidth, pairwise_dist, sgd_gradient);
//                    }
//
//                    real_weight_ = torch::clamp(particle_size_*w_p_, tau_).pow(beta_);
//                    stein_grad = torch::mul(real_weight_, stein_grad);
                }
                else
                {
                    stein_grad = -sgd_gradient_.reshape({1,6});
                }

                x_.mutable_grad() = -stein_grad.index({Slice(),0}).reshape({particle_size_, 1, 1});
                y_.mutable_grad() = -stein_grad.index({Slice(), 1}).reshape({particle_size_, 1, 1});
                z_.mutable_grad() = -stein_grad.index({Slice(), 2}).reshape({particle_size_, 1, 1});
                roll_.mutable_grad() = -stein_grad.index({Slice(), 3}).reshape({particle_size_, 1, 1});
                pitch_.mutable_grad() = -stein_grad.index({Slice(), 4}).reshape({particle_size_, 1, 1});
                yaw_.mutable_grad() = -stein_grad.index({Slice(), 5}).reshape({particle_size_, 1, 1});

                x_last_ = x_.clone();
                y_last_ = y_.clone();
                z_last_ = z_.clone();
                roll_last_ = roll_.clone();
                pitch_last_ = pitch_.clone();
                yaw_last_ = yaw_.clone();

                optimizer_ptr_->step();
                optimizer_ptr_->zero_grad();
                optimizer_ptr_->state();

                if(using_EarlyStop_ )
                {
                    if(is_rot_converged() && is_trans_converged())
                        convergence_counter++;

                    if(convergence_counter >= convergence_steps_)
                    {
                        std::cout << "******** Align process converged ***********" <<std::endl;
                        break;
                    }
                }
                auto grad_end = std::chrono::steady_clock::now();
                std::chrono::duration<double> grad_duration = grad_end - grad_start;
                grad_time += grad_duration.count();

                particle_pose_ = torch::stack({x_*max_abs_, y_*max_abs_, z_*max_abs_, roll_, pitch_, yaw_}).squeeze({2,3});

                delta_t_tsr_[epoch] = torch::sum(torch::abs(x_-x_last_)
                                                +torch::abs(y_-y_last_)
                                                +torch::abs(z_-z_last_),0).reshape({1}) * max_abs_ / particle_size_tsr_;
                delta_R_tsr_[epoch] = torch::sum(torch::abs(roll_-roll_last_)
                                                +torch::abs(pitch_-pitch_last_)
                                                +torch::abs(yaw_-yaw_last_), 0).reshape({1}) / particle_size_tsr_;
                var_t_tr_[epoch] = torch::sum(torch::var(x_*max_abs_)+torch::var(y_*max_abs_)+torch::var(z_*max_abs_)).reshape({1});
                var_R_tr_[epoch] = torch::sum(torch::var(roll_)+torch::var(pitch_)+torch::var(yaw_)).reshape({1});
                obj_tsr_[epoch] = torch::sum(torch::sqrt(torch::square(transformed_s_paired-target_paired).sum(2)).sum(1)/batch_size_, 0).reshape({1});
                mean_log_tsr_[epoch] = torch::mul(w_p_, particle_pose_.transpose(0,1)).sum(0).reshape({6});
                var_log_tsr_[epoch] = torch::var(particle_pose_, 1, true).reshape({6});
            }

            if (use_weight_mean_)
            {
                const auto[Kernel_SVGD, bandwidth, pairwise_dist] = get_kernel(pose_parameters_tsr_,
                                                                               pose_parameters_tsr_.detach());
                this->weight_update(Kernel_SVGD, bandwidth, pairwise_dist, sgd_gradient_);
            }

            auto opt_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> opt_duration = opt_end - opt_start;
            std::cout << " finished in: "<< opt_duration.count() <<"s"<< std::endl;
//            std::cout << "**********SGD time************ " << grad_time << "s" << std::endl;

            //6*particle_size
            particle_pose_ = torch::stack({x_*max_abs_, y_*max_abs_, z_*max_abs_, roll_, pitch_, yaw_}).squeeze({2,3});
            return SteinICP_State::ALIGN_SUCCESS;
        }

        torch::Tensor BetaSteinICP::phi(const torch::Tensor& pose_parameters, const torch::Tensor &sgd_grad, const torch::Tensor& Kernel)
        {
//            pose_parameters = pose_parameters.detach().requires_grad_(true);
//        auto K_xx = get_kernel(pose_parameters, pose_parameters.detach());
            const auto& K_xx = Kernel;
            const std::vector<torch::Tensor> K_xx_v = {K_xx.sum()};
            const std::vector<torch::Tensor> pose_param_v = {pose_parameters};
            const auto grad_K = -torch::autograd::grad(K_xx_v, pose_param_v)[0];
            return (K_xx.detach().matmul(sgd_grad) + grad_K) / (pose_parameters.size(0));
        }

         std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> BetaSteinICP::get_kernel(const torch::Tensor& x1, const torch::Tensor& x2)
        {
            const auto x1_norm_squared= x1.pow(2).sum(-1, true);
            const auto x2_norm_squared = x2.pow(2).sum(-1, true);
            const auto pairwise_dist = torch::addmm(x2_norm_squared.transpose(-2,-1), x1, x2.transpose(-2,-1), 1,-2).add_(x1_norm_squared);

//        auto pairwise_dist = torch::cdist(x1, x2).pow(2);

            const auto h = torch::median(pairwise_dist) / log(x1.size(0) + 1);
//             const auto h = torch::median(pairwise_dist) / torch::log(particle_size_tsr_+torch::ones_like(particle_size_tsr_));
            return {torch::exp(-pairwise_dist / h), h, pairwise_dist};


        }

        void BetaSteinICP::weight_update(const torch::Tensor &Kernel,
                                         const torch::Tensor &bandwidth,
                                         const torch::Tensor &pairwise_dist,
                                         const torch::Tensor &sgd_grad)
        {
            const auto dd_kernel = torch::mul(Kernel, 2*6/bandwidth-4*pairwise_dist/bandwidth.pow(2)).to(torch::kCUDA);
            auto d_kernel = torch::zeros({particle_size_, particle_size_}).to(torch::kCUDA);

            const auto diff_T = (pose_parameters_tsr_.view({particle_size_, 1, 6}) - pose_parameters_tsr_).transpose(1,2);
            d_kernel = torch::einsum("pik, pkj->pij", {-sgd_grad.view({particle_size_, 1, 6}), diff_T}).squeeze(1);
            d_kernel = torch::mul((d_kernel + d_kernel.transpose(0,1))*2/bandwidth, Kernel);

            auto Kernel_pi = torch::mul(Kernel, torch::matmul(-sgd_grad, -sgd_grad.transpose(0,1).to(torch::kCUDA)))
                                                + d_kernel
                                                + dd_kernel;

            stein_distance_.push_back(Kernel_pi.sum(1).sum(0).item<float>()/(particle_size_*particle_size_));

            Kernel_pi = Kernel_pi/pow(particle_size_, 2);

            if(beta_ == 0)
              return;

            for(int i = 0; i<md_iter_; i++)
            {
                w_p_ = w_p_.mul(torch::exp(-r_ * (torch::matmul(Kernel_pi, w_p_))));
                w_p_ = 1/w_p_.sum(0)*w_p_;
            }
//            std::cout << "Omega 0 "<< w_p_[0] << std::endl;


        }

        std::vector<double> BetaSteinICP::get_stein_dist()
        {
            return stein_distance_;
        }

        std::vector<double> BetaSteinICP::get_weight()
        {
            auto weight_cpu = real_weight_.to(torch::kCPU);
            return std::vector<double>(weight_cpu.data_ptr<float>(), weight_cpu.data_ptr<float>()+particle_size_);
        }

    std::vector<double> BetaSteinICP::get_stein_weight()
    {
            auto weight_cpu = w_p_.to(torch::kCPU);
            return std::vector<double>(weight_cpu.data_ptr<float>(), weight_cpu.data_ptr<float>()+particle_size_);
    }

    torch::Tensor BetaSteinICP::get_transformation()
    {
//        return torch::mean(particle_pose_, 1);
            auto weighted_mean = torch::mul(particle_pose_, w_p_.transpose(0,1)).sum(1);
            return weighted_mean;
    }


}