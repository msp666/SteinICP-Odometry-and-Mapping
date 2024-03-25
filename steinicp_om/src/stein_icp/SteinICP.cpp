//
// Created by haoming on 26.05.23.
//



#include "SteinICP.h"

#include <memory>

namespace stein_icp
{

    SteinICP::SteinICP(SteinICPParam &parameters, const torch::Tensor &init_pose)
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

        set_optimizer();

    }

    void SteinICP::add_cloud(const torch::Tensor &source, const torch::Tensor &target,  const torch::Tensor &init_pose)
    {
             particle_size_ = init_pose.size(1);
             particle_size_tsr_ = torch::tensor({particle_size_}).to(torch::kCUDA);
             x_     = init_pose[0].view({particle_size_, 1, 1});
             y_     = init_pose[1].view({particle_size_, 1, 1});
             z_     = init_pose[2].view({particle_size_, 1, 1});
             roll_  = init_pose[3].view({particle_size_, 1, 1});
             pitch_ = init_pose[4].view({particle_size_, 1, 1});
             yaw_   = init_pose[5].view({particle_size_, 1, 1});

             target_cloud_ = target.clone();
             source_cloud_ = source.clone();
             is_first_cloud_ = false;

             max_abs_ = torch::tensor({1}).to(torch::kCUDA);
             gradient_scaling_factor_ = torch::tensor({source_cloud_.size(0)}).to(torch::kCUDA);
             if(normalize_cloud_)
                 normalize_clouds();
             pose_parameters_ = {x_, y_, z_, roll_, pitch_, yaw_};
             set_optimizer();

    }


    void SteinICP::add_cloud(const torch::Tensor &source, const torch::Tensor &init_pose)
    {
        particle_size_ = init_pose.size(1);
        particle_size_tsr_ = torch::tensor({particle_size_}).to(torch::kCUDA);
        x_     = init_pose[0].view({particle_size_, 1, 1});
        y_     = init_pose[1].view({particle_size_, 1, 1});
        z_     = init_pose[2].view({particle_size_, 1, 1});
        roll_  = init_pose[3].view({particle_size_, 1, 1});
        pitch_ = init_pose[4].view({particle_size_, 1, 1});
        yaw_   = init_pose[5].view({particle_size_, 1, 1});

        if(is_first_cloud_)
        {
            cloud_last_ = source.clone();
        }
        else
        {
            target_cloud_ = cloud_last_.clone();
            cloud_last_ = source.clone();
            source_cloud_ = source.clone();
            max_abs_ = torch::tensor({1}).to(torch::kCUDA);
            gradient_scaling_factor_ = torch::tensor({source_cloud_.size(0)}).to(torch::kCUDA);
            if(normalize_cloud_)
                normalize_clouds();
            pose_parameters_ = {x_, y_, z_, roll_, pitch_, yaw_};
            set_optimizer();

        }


    }

    /*Stein ICP process*/
    SteinICP_State SteinICP::stein_align()
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
        this->allocate_memory_for_log();
        auto [mini_batch, target_batch] = mini_batch_generator();
        /** SVGD iterations */
        auto opt_start = std::chrono::steady_clock::now();
        for(int epoch = 0; epoch < iter_; epoch++)
        {
            //expand minibatch and target to match the particle size
            torch::Tensor mini_batch2 = mini_batch[epoch].expand({particle_size_, batch_size_, 3}).to(torch::kCUDA);
            torch::Tensor targe_batch_it = target_batch[epoch].view({batch_size_, K_source_, 3});

            torch::Tensor mat_rot = Mat_rot();
            torch::Tensor mat_trans = Mat_trans();
            torch::Tensor source_transformed = torch::einsum("pmc, prc->pmr", {mini_batch2, mat_rot})
                                               + mat_trans.view({particle_size_, 1, 3}).to(torch::kCUDA);

            auto [source_paired, transformed_s_paired, target_paired] = get_correspondence(mini_batch2, source_transformed, targe_batch_it);

            auto partial_derivatives = partial_derivative(roll_.clone(), pitch_.clone(), yaw_.clone());
            auto sgd_gradient = sgd_grad(source_paired, transformed_s_paired, target_paired, partial_derivatives, gradient_scaling_factor_);
            auto pose_parameters_tsr = torch::stack({x_, y_, z_, roll_, pitch_, yaw_}).squeeze({2,3}).transpose(0,1);
            auto stein_grad = phi(pose_parameters_tsr, -sgd_gradient);

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

            if(using_EarlyStop_)
            {
                if(is_rot_converged() & is_trans_converged())
                    convergence_counter++;

                if(convergence_counter >= convergence_steps_)
                {
                    std::cout << "******** Align process converged ***********" <<std::endl;
                    break;
                }
            }


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
            mean_log_tsr_[epoch] = particle_pose_.transpose(0,1).sum(0).reshape({6})/particle_size_;
            var_log_tsr_[epoch] = torch::var(particle_pose_, 1, true).reshape({6});
        }

        auto opt_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> opt_duration = opt_end - opt_start;
        std::cout << " finished in: "<< opt_duration.count() <<"s"<< std::endl;

        //6*particle_size
        particle_pose_ = torch::stack({x_*max_abs_, y_*max_abs_, z_*max_abs_, roll_, pitch_, yaw_}).squeeze({2,3});
        return SteinICP_State::ALIGN_SUCCESS;
    }

    void SteinICP::set_optimizer()
    {
        pose_parameters_ = {x_, y_, z_, roll_, pitch_, yaw_};
        if(optimizer_class_ == "Adam")
        {
            std::cout << "Optimization using: " << optimizer_class_;
            torch::optim::AdamOptions adamopt(lr_);
            adamopt.betas(std::make_tuple(0.9, 0.999));
            optimizer_ptr_ = std::make_unique<torch::optim::Adam>(pose_parameters_, adamopt);
            optimizer_set_ = true;
        }
        else if(optimizer_class_ == "RMSprop")
        {
            std::cout << "Optimization using: " << optimizer_class_;
            torch::optim::RMSpropOptions rmspropopt(lr_);
            rmspropopt.weight_decay(1e-8);
            rmspropopt.momentum(0.9);

            optimizer_ptr_ = std::make_unique<torch::optim::RMSprop>(pose_parameters_, rmspropopt);
            optimizer_set_ = true;
        }
        else if(optimizer_class_ == "SGD")
        {
            std::cout << "Optimization using: " << optimizer_class_;
            optimizer_ptr_ = std::make_unique<torch::optim::SGD>(pose_parameters_, torch::optim::SGDOptions(lr_));
            optimizer_set_ = true;
        }
        else if(optimizer_class_ == "Adagrad")
        {
            std::cout << "Optimization using: " << optimizer_class_;
            optimizer_ptr_ = std::make_unique<torch::optim::Adagrad>(pose_parameters_, torch::optim::AdagradOptions(lr_));
            optimizer_set_ = true;
        }
        else
        {
            std::cout<<"No optimizer chosen"<<std::endl;
            optimizer_set_ = false;
        }
    }

    void SteinICP::normalize_clouds()
    {

        max_s_ = torch::max(torch::abs(source_cloud_));
        max_t_ = torch::max(torch::abs(target_cloud_));
        max_abs_ = torch::max(max_s_, max_t_);
        source_cloud_ = source_cloud_ / max_abs_;
        target_cloud_ = target_cloud_ / max_abs_;

        x_ = x_/max_abs_;
        y_ = y_/max_abs_;
        z_ = z_/max_abs_;

        gradient_scaling_factor_ = max_abs_ * source_cloud_.size(0);
        //std::vector<torch::Tensor> cloud_n = {source_n, target_n, max_abs};
    }

    void SteinICP::allocate_memory_for_log()
    {
        delta_t_tsr_ = torch::zeros({iter_,1}).to(torch::kCUDA);
        delta_R_tsr_ = torch::zeros({iter_,1}).to(torch::kCUDA);
        var_t_tr_ = torch::zeros({iter_+1,1}).to(torch::kCUDA);
        var_R_tr_ = torch::zeros({iter_+1,1}).to(torch::kCUDA);
        var_t_tr_[0] = torch::sum(torch::var(x_*max_abs_)+torch::var(y_*max_abs_)+torch::var(z_*max_abs_));
        var_R_tr_[0] = torch::sum(torch::var(roll_)+torch::var(pitch_)+torch::var(yaw_));
        mean_log_tsr_ = torch::zeros({iter_, 6}).to(torch::kCUDA);
        var_log_tsr_ = torch::zeros({iter_, 6}).to(torch::kCUDA);
        obj_tsr_ = torch::zeros({iter_,1}).to(torch::kCUDA);
    }

    std::tuple<torch::Tensor, torch::Tensor> SteinICP::mini_batch_generator()
    {
        auto idx = torch::randint(source_cloud_.size(0), batch_size_*iter_).to(torch::kCUDA);
        torch::Tensor mini_batch = torch::empty({batch_size_*iter_, 3}).to(torch::kCUDA);
        mini_batch = torch::index_select(source_cloud_, 0, idx);
        this->sourcecloud_KNN();
        auto sourceKNN_batch_idx = torch::index_select(sourceKNN_idx_, 0, idx).view({batch_size_*iter_*K_source_}).to(torch::kCUDA).to(torch::kLong);
        auto target_batch = torch::index_select(target_cloud_, 0, sourceKNN_batch_idx);
        //std::cout<<mini_batch<<std::endl;
        return {mini_batch.reshape({iter_, batch_size_, 3}), target_batch.reshape({iter_, batch_size_, K_source_, 3})};
    }

    std::tuple<torch::Tensor, torch::Tensor> SteinICP::mini_batch_generator_inLoop()
    {
        auto idx = torch::randperm(source_cloud_.size(0)).index({Slice(0,  batch_size_)}).to(torch::kCUDA);
        torch::Tensor mini_batch = torch::empty({batch_size_, 3}).to(torch::kCUDA);
        mini_batch = torch::index_select(source_cloud_, 0, idx);
        auto sourceKNN_batch_idx = torch::index_select(sourceKNN_idx_, 0, idx).view({batch_size_*K_source_}).to(torch::kCUDA).to(torch::kLong);
        auto target_batch = torch::index_select(target_cloud_, 0, sourceKNN_batch_idx);
        //std::cout<<mini_batch<<std::endl;
        return {mini_batch.reshape({batch_size_, 3}), target_batch.reshape({batch_size_, K_source_, 3})};
    }

    void SteinICP::sourcecloud_KNN()
    {
        const int N_s = source_cloud_.size(0);
        const int N_t = target_cloud_.size(0);
        auto knn = KNearestNeighborIdx(
                                        source_cloud_.view({1, N_s, 3}),
                                        target_cloud_.view({1, N_t, 3}),
                                        torch::tensor({N_s}).to(torch::kCUDA),
                                        torch::tensor({N_t}).to(torch::kCUDA),
                                        2,
                                        K_source_,
                                        -1
                                        );
        sourceKNN_idx_ = std::get<0>(knn).to(torch::kCUDA).reshape({N_s, K_source_});
    }

    torch::Tensor SteinICP::Mat_rot()
    {
        const auto Cosyaw = torch::cos(yaw_);
        const auto Sinyaw = torch::sin(yaw_);
        const auto Cospitch = torch::cos(pitch_);
        const auto Sinpitch = torch::sin(pitch_);
        const auto Cosroll = torch::cos(roll_);
        const auto Sinroll = torch::sin(roll_);
        torch::Tensor R = torch::stack(
                {torch::stack(
                                { Cospitch*Cosyaw,
                                  Sinroll*Sinpitch*Cosyaw - Cosroll*Sinyaw,
                                  Sinroll*Sinyaw + Cosroll*Sinpitch*Cosyaw
                                }, 3
                             ).squeeze(1),
                 torch::stack(
                                { Cospitch*Sinyaw,
                                  Cosroll*Cosyaw + Sinroll*Sinpitch*Sinyaw,
                                  Cosroll*Sinpitch*Sinyaw - Sinroll*Cosyaw
                                }, 3
                             ).squeeze(1),
                 torch::stack(
                                { -Sinpitch,
                                  Sinroll*Cospitch,
                                  Cosroll*Cospitch
                                }, 3
                             ).squeeze(1)
                }, 2
                ).squeeze(1);
        return R;
    }

    torch::Tensor SteinICP::Mat_trans()
    {
        return torch::cat({x_, y_, z_}, 1);
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> SteinICP::get_correspondence(
                                               const torch::Tensor &source,                /// particle*batch_size_ * 3
                                               const torch::Tensor &transformed_source,    /// particle*batch_size_ * 3
                                               const torch::Tensor &target                 /// target_point_count * 3
                                               )
    {
//        std::vector<torch::Tensor> cloud_paired;
//        auto knn = KNearestNeighborIdx( transformed_source,
//                                       target.expand({particle_size_, target.size(0), 3}).to(torch::kCUDA),
//                                       batch_size_t_*torch::ones_like(x_).reshape({particle_size_,1}).to(torch::kLong),
//                                       target.size(0)*torch::ones_like(x_).reshape({particle_size_,1}).to(torch::kLong),
//                                       2,
//                                       1,
//                                       -1);
        auto knn = KNearestNeighborIdx( transformed_source.transpose(0,1),
                                        target,
                                        particle_size_*torch::ones({batch_size_, 1}).to(torch::kLong).to(torch::kCUDA),
                                        K_source_*torch::ones({batch_size_, 1}).to(torch::kLong).to(torch::kCUDA),
                                        2,
                                        1,
                                        -1);
        auto dist_cr = std::get<1>(knn).transpose(0,1).to(torch::kCUDA);
        auto target_index = std::get<0>(knn).transpose(0,1).to(torch::kCUDA).reshape({particle_size_, batch_size_});

        return {
                point_filter(source, dist_cr),
                point_filter(transformed_source, dist_cr),
                point_filter((target.transpose(0,1)).index({target_index,
                                                            torch::arange(batch_size_).view({batch_size_}).to(torch::kCUDA)}),
                                                            dist_cr)
                };
    }

    torch::Tensor SteinICP::point_filter(const torch::Tensor &cloud, const torch::Tensor &distance_cr)
    {
        return torch::lt(distance_cr, max_dist_/max_abs_).to(torch::kCUDA)*cloud;
    }

    torch::Tensor SteinICP::partial_derivative(const torch::Tensor &roll,
                                                            const torch::Tensor &pitch,
                                                            const torch::Tensor &yaw)
    {

        const auto Cosyaw = torch::cos(yaw);  // A
        const auto Sinyaw = torch::sin(yaw);  // B
        const auto Cospitch = torch::cos(pitch); // C
        const auto Sinpitch = torch::sin(pitch); // D
        const auto Cosroll = torch::cos(roll); // E
        const auto Sinroll = torch::sin(roll); // F

        const auto Sinpitch_Cosroll = Sinpitch * Cosroll;  // DE
        const auto Sinpitch_Sinroll = Sinpitch * Sinroll;  // DF
        const auto Cosyaw_Cospitch = Cosyaw * Cospitch;  // AC
        const auto Cosyaw_Sinroll = Cosyaw * Sinroll;  // AF
        const auto Cosyaw_Cosroll = Cosyaw * Cosroll;  // AE

        const auto Cosyaw_Sinpitch_Cosroll = Cosyaw * Sinpitch_Cosroll;  // ADE
        const auto Cosyaw_Sinpitch_Sinroll = Cosyaw * Sinpitch_Sinroll;  // ADF
        const auto Sinyaw_Cospitch = Sinyaw * Cospitch;  // BC
        const auto Sinyaw_Cosroll = Sinyaw * Cosroll;  // BE
        const auto Sinyaw_Sinroll = Sinyaw * Sinroll;  // BF
        const auto Sinyaw_Sinpitch_Cosroll = Sinyaw * Sinpitch_Cosroll; // BDE


        const auto torch_0 = torch::zeros_like(roll).to(torch::kCUDA);
//        auto particle_size = roll.size(0);
//        const auto torch_0 = torch::zeros({particle_size, 1, 1}).to(torch::kCUDA);

        torch::Tensor partial_roll = torch::stack(
                                        {torch::stack({torch_0,   Cosyaw_Sinpitch_Cosroll + Sinyaw_Sinroll,   Sinyaw_Cosroll-Cosyaw_Sinpitch_Sinroll},      3).squeeze(1),
                                         torch::stack({torch_0,   -Cosyaw_Sinroll + Sinyaw_Sinpitch_Cosroll,   Sinyaw*(-Sinpitch_Sinroll)-Cosyaw_Cosroll},  3).squeeze(1),
                                         torch::stack({torch_0,   Cospitch*Cosroll ,   Cospitch*(-Sinroll)},      3).squeeze(1)
                                        }, 2
                                         ).squeeze(1);

        torch::Tensor partial_pitch = torch::stack(
                                         {
                                             torch::stack({Cosyaw * -Sinpitch,   Cosyaw_Cospitch * Sinroll,   Cosyaw_Cospitch * Cosroll}, 3).squeeze(1),
                                             torch::stack({Sinyaw * -Sinpitch,   Sinyaw_Cospitch *Sinroll,    Sinyaw_Cospitch * Cosroll}, 3).squeeze(1),
                                             torch::stack({-Cospitch ,    -Sinpitch_Sinroll,   -Sinpitch_Cosroll}, 3).squeeze(1)
                                         },2
                                         ).squeeze(1);
        torch::Tensor partial_yaw = torch::stack(
                                       {
                                           torch::stack({-Sinyaw_Cospitch,   -Sinyaw * Sinpitch_Sinroll - Cosyaw_Cosroll,   Cosyaw_Sinroll - Sinyaw_Sinpitch_Cosroll}, 3).squeeze(1),
                                           torch::stack({ Cosyaw_Cospitch,   -Sinyaw_Cosroll + Cosyaw_Sinpitch_Sinroll,   Cosyaw_Sinpitch_Cosroll + Sinyaw_Sinroll}, 3).squeeze(1),
                                           torch::stack({torch_0, torch_0,   torch_0}, 3).squeeze(1),
                                       }, 2
                                       ).squeeze(1);

        return torch::stack({partial_roll, partial_pitch, partial_yaw});
    }

    torch::Tensor SteinICP::sgd_grad(const torch::Tensor &source_paired,
                                     const torch::Tensor &transformed_s_paired,
                                     const torch::Tensor &target_paired,
                                     const torch::Tensor &partial_derivatives,
                                     const torch::Tensor &scaling_factor)
    {
        //TODO: Why here is so slow?

//        auto nonzero_count = torch::count_nonzero(torch::linalg::vector_norm(source_paired, 2, 1, false, torch::kFloat32)).to(torch::kCUDA);
//        auto zero_count = batch_size_t - nonzero_count;
        c10::optional<int64_t> dim = 1;

        const auto nonzero_count = (torch::count_nonzero(transformed_s_paired.sum(2), dim).to(torch::kCUDA));  ///particle*1

//        const auto zero_count = batch_size_t_ - nonzero_count;
//        auto zero_count_tensor = torch::zeros_like(zero_count).to(torch::kCUDA);


//        if(torch::equal(zero_count, batch_size_t_))
//        {
//            zero_count_tensor = torch::ones_like(zero_count).to(torch::kCUDA);
//        }

//        std::cout <<"[SteinICP]: " << torch::mean(nonzero_count.to(torch::kFloat32), 0).item<float>() << "corresponding points are founded in average" << std::endl;



        auto error = transformed_s_paired - target_paired;   ///particle*batch_size_*3
        torch::Tensor sgd_gradient = torch::zeros({particle_size_, 6}).to(torch::kCUDA);
//
        const auto error_squared = torch::norm(error, 2, 2, true);
        const auto weighted_error = torch::square(max_dist_ / (max_dist_ + 3 * error_squared)) * error;
        error = weighted_error.clone();
        sgd_gradient.index_put_({Slice(), Slice(0,3)}, error.sum(1)/(nonzero_count+torch::ones_like(nonzero_count).to(torch::kCUDA)).reshape({particle_size_,1}));
        sgd_gradient.index_put_({Slice(),3}, torch::einsum("pbc, pbc->pb",
                                                      {
                                                               error,
                                                               torch::einsum("prc, pbc->pbr", {partial_derivatives[0], source_paired})
                                                             }).sum(1)/(nonzero_count+torch::ones_like(nonzero_count).to(torch::kCUDA)))/max_abs_;
        sgd_gradient.index_put_({Slice(), 4}, torch::einsum("pbc, pbc->pb",
                                                       {
                                                               error,
                                                               torch::einsum("prc, pbc->pbr", {partial_derivatives[1], source_paired})
                                                       }).sum(1)/(nonzero_count+torch::ones_like(nonzero_count).to(torch::kCUDA)))/max_abs_;
        sgd_gradient.index_put_({Slice(), 5}, torch::einsum("pbc, pbc->pb",
                                                       {
                                                               error,
                                                               torch::einsum("prc, pbc->pbr", {partial_derivatives[2], source_paired})
                                                       }).sum(1)/(nonzero_count+torch::ones_like(nonzero_count).to(torch::kCUDA)))/max_abs_;


        return sgd_gradient * scaling_factor;
    }


    torch::Tensor SteinICP::phi(torch::Tensor pose_parameters, const torch::Tensor &sgd_grad)
    {
        pose_parameters = pose_parameters.detach().requires_grad_(true);
//        auto K_xx = get_kernel(pose_parameters, pose_parameters.detach());
        auto K_xx = get_kernel(pose_parameters, pose_parameters.detach());
        std::vector<torch::Tensor> K_xx_v = {K_xx.sum()};
        std::vector<torch::Tensor> pose_param_v = {pose_parameters};
        auto grad_K = -torch::autograd::grad(K_xx_v, pose_param_v)[0];
        return (K_xx.detach().matmul(sgd_grad)+grad_K)/pose_parameters.size(0);
    }

    torch::Tensor SteinICP::get_kernel(const torch::Tensor &x1, const torch::Tensor &x2)
    {
        auto x1_square = x1.pow(2).sum(-1, true);
        auto x2_square = x2.pow(2).sum(-1, true);
        auto pairwise_dist = torch::addmm(x2_square.transpose(-2,-1), x1, x2.transpose(-2,-1), 1,-2).add_(x1_square);

//        auto pairwise_dist = torch::cdist(x1, x2).pow(2);
        auto h = 0.5 * torch::median(pairwise_dist)/ log(x1.size(0));
        return torch::exp(-0.5*pairwise_dist/h);
    }

    bool SteinICP::is_trans_converged()
    {
//        auto delta = torch::sum(torch::sqrt(torch::square(x_*max_abs_-x_last_*max_abs_)
//                                +torch::square(y_*max_abs_-y_last_*max_abs_)
//                                +torch::square(z_*max_abs_-z_last_*max_abs_)),0).item<double>()/particle_size_;
//        delta_t_.push_back(delta);
//        return delta < convergence_threshold_;

//        auto delta_tsr = torch::sum(torch::sqrt(torch::square(x_*max_abs_-x_last_*max_abs_)
//                                   +torch::square(y_*max_abs_-y_last_*max_abs_)
//                                   +torch::square(z_*max_abs_-z_last_*max_abs_)),0) / particle_size_tsr_;

        auto delta_tsr = torch::sum(torch::abs(x_ - x_last_)
                                    +torch::abs(y_ - y_last_)
                                    +torch::abs(z_ -z_last_),0) * max_abs_ / particle_size_tsr_;

        return torch::is_nonzero(torch::lt(delta_tsr.reshape({1}), convergence_threshold_tsr_));

    }

    bool SteinICP::is_rot_converged()
    {
        auto delta_tsr = torch::sum(torch::abs(roll_-roll_last_)
                                    +torch::abs(pitch_-pitch_last_)
                                    +torch::abs(yaw_-yaw_last_), 0) / particle_size_tsr_;

        return torch::is_nonzero(torch::lt(delta_tsr.reshape({1}), convergence_threshold_tsr_));

    }


    void SteinICP::set_max_dist(double max_dist)
    {
        max_dist_ = torch::tensor({max_dist}).to(torch::kCUDA);
    }

    torch::Tensor SteinICP::get_transformation()
    {
        return torch::mean(particle_pose_, 1);
    }

    torch::Tensor SteinICP::get_distribution()
    {
        return torch::var(particle_pose_, 1);
    }

    std::vector<double> SteinICP::get_particles()
    {
        auto particle_pose_cpu = particle_pose_.to(torch::kCPU);
        auto Result_particle = std::vector<double>(particle_pose_cpu.data_ptr<float>(), particle_pose_cpu.data_ptr<float>()+6*particle_size_);
        return Result_particle;
    }

    std::tuple<std::vector<double>, std::vector<double>> SteinICP::get_delta_log()
    {
        auto delta_t_cpu = delta_t_tsr_.to(torch::kCPU);
        auto delta_t = std::vector<double>(delta_t_cpu.data_ptr<float>(), delta_t_cpu.data_ptr<float>()+iter_);
        auto delta_R_cpu = delta_R_tsr_.to(torch::kCPU);
        auto delta_R = std::vector<double>(delta_R_cpu.data_ptr<float>(), delta_R_cpu.data_ptr<float>()+iter_);
        return {delta_t, delta_R};
    }

    std::tuple<std::vector<double>, std::vector<double>> SteinICP::get_var_log()
    {
        auto var_t_cpu = var_t_tr_.to(torch::kCPU);
        auto var_t = std::vector<double>(var_t_cpu.data_ptr<float>(), var_t_cpu.data_ptr<float>()+iter_+1);
        auto var_R_cpu = var_R_tr_.to(torch::kCPU);
        auto var_R = std::vector<double>(var_R_cpu.data_ptr<float>(), var_R_cpu.data_ptr<float>()+iter_+1);
        return {var_t, var_R};
    }

    std::vector<double> SteinICP::get_var6_log()
    {
        auto var_log_cpu = var_log_tsr_.to(torch::kCPU);
        auto var_log = std::vector<double>(var_log_cpu.data_ptr<float>(), var_log_cpu.data_ptr<float>()+iter_*6);
        return var_log;
    }


    std::vector<double> SteinICP::get_mean_log()
    {
        auto mean_log_cpu = mean_log_tsr_.to(torch::kCPU);
        auto mean_log = std::vector<double>(mean_log_cpu.data_ptr<float>(), mean_log_cpu.data_ptr<float>()+iter_*6);
        return mean_log;
    }

    std::vector<double> SteinICP::get_obj_log()
    {
        auto obj_cpu = obj_tsr_.to(torch::kCPU);
        auto obj_log = std::vector<double>(obj_cpu.data_ptr<float>(), obj_cpu.data_ptr<float>()+iter_);
        return obj_log;
    }

    void SteinICP::reset() {
        is_first_cloud_ = true;
//        source_cloud_ = torch::empty({0});
//        target_cloud_ = torch::empty({0});
    }

}
