//
// Created by haoming on 01.06.23.
//

#ifndef STEIN_ICP_UTILS_H
#define STEIN_ICP_UTILS_H

#include <string>
#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <torch/torch.h>
#include <gtsam/geometry/Pose3.h>

#include "types.h"


namespace stein_icp
{
/**
 *
 * \brief visualizetion point cloud
 */
    void pcl_visualizer(const Cloud_t::Ptr &cloud, const std::string &title = "3D viewer");

/**
 *
 * \brief transfer Cloud_t.points to a 1 dimension std::vector
 */
    std::vector<float> pcl2vector(const Cloud_t::Ptr &cloud);  //pcl to std::vector

/**
 *
 * \brief transfer std::vector to tensor
 */
    torch::Tensor vector2tensor(std::vector<float>, Device_type device);    //std::vector to torch::Tensor

/**
 * \brief
 * @return
 */
    std::vector<std::vector<float>> tensor2vector(const torch::Tensor &t);

/**
 * \brief    initialize pose particles with particle_count, in uniform distribution
 *
 * \return   return pose in size: 6*particle_count*1
 */
    torch::Tensor
    initialize_particles(int particle_count, Device_type device, const torch::Tensor &ub, const torch::Tensor &lb);


    /**
     *  \brief transform Tensor to Pose3
     */
    gtsam::Pose3 tensor2gtsamPose3(const torch::Tensor &this_point);

    std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Vector3d> tensor2gtsamtransform(const torch::Tensor &this_point);

    Eigen::Matrix<double, 4, 4> tensor2Matrix(const torch::Tensor &this_point);

}




#endif //STEIN_ICP_UTILS_H
