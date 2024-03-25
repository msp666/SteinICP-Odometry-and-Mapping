//
// Created by haoming on 01.06.23.
//

#include <string>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>

#include "types.h"
#include "steinicp_utils.h"


namespace stein_icp {
    void pcl_visualizer(const Cloud_t::Ptr &cloud, const std::string &title) {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<Point_t>(cloud, title);
        viewer->addCoordinateSystem(1.0);
        viewer->spin();
    }

    std::vector<float> pcl2vector(const Cloud_t::Ptr &cloud) {
        auto p2v_start = std::chrono::steady_clock::now();
        std::vector<float> cloud_vec;
        for (auto pt: cloud->points) {
            cloud_vec.push_back(pt.x);
            cloud_vec.push_back(pt.y);
            cloud_vec.push_back(pt.z);
        }
        auto p2v_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> p2v_duration = p2v_end - p2v_start;
        std::cout << "pcl to vector time: " << p2v_duration.count() << "s" << std::endl;
        return cloud_vec;
    }

    torch::Tensor vector2tensor(std::vector<float> v, Device_type device) {
        auto v2t_start = std::chrono::steady_clock::now();
        long size_v = v.size() / 3;
        torch::Tensor cloud_tensor = torch::from_blob(v.data(), {size_v, 3}
                //,torch::TensorOptions().dtype(torch::kFloat32)
        ).to(device);
        auto v2t_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> v2t_duration = v2t_end - v2t_start;
        std::cout << "vector to tensor time: " << v2t_duration.count() << "s" << std::endl;
        return cloud_tensor;
    }

    std::vector<std::vector<float>> tensor2vector(const torch::Tensor &t) {

        auto t2v_start = std::chrono::steady_clock::now();
        torch::Tensor t_CPU = t.to(torch::kCPU);
        std::vector<float> v1(t_CPU.data_ptr<float>(), t_CPU.data_ptr<float>() + 99);
        std::vector<float> v2(t_CPU.data_ptr<float>() + 100, t_CPU.data_ptr<float>() + 199);
        std::vector<float> v3(t_CPU.data_ptr<float>() + 200, t_CPU.data_ptr<float>() + 299);
        std::vector<float> v4(t_CPU.data_ptr<float>() + 300, t_CPU.data_ptr<float>() + 399);
        std::vector<float> v5(t_CPU.data_ptr<float>() + 400, t_CPU.data_ptr<float>() + 499);
        std::vector<float> v6(t_CPU.data_ptr<float>() + 500, t_CPU.data_ptr<float>() + 599);

        auto t2v_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> t2v_duration = t2v_end - t2v_start;
        std::cout << "tensor to vector: " << t2v_duration.count() << std::endl;
        return {v1, v2, v3, v4, v5, v6};


    }

    torch::Tensor
    initialize_particles(int particle_count, Device_type device, const torch::Tensor &ub, const torch::Tensor &lb) {
        torch::Tensor init_pose = torch::empty({6, particle_count, 1}).to(device);
        init_pose[0] = (ub[0] - lb[0]) * torch::rand({particle_count, 1}).to(device) + lb[0];
        init_pose[1] = (ub[1] - lb[1]) * torch::rand({particle_count, 1}).to(device) + lb[1];
        init_pose[2] = (ub[2] - lb[2]) * torch::rand({particle_count, 1}).to(device) + lb[2];
        init_pose[3] = (ub[3] - lb[3]) * torch::rand({particle_count, 1}).to(device) + lb[3];
        init_pose[4] = (ub[4] - lb[4]) * torch::rand({particle_count, 1}).to(device) + lb[4];
        init_pose[5] = (ub[5] - lb[5]) * torch::rand({particle_count, 1}).to(device) + lb[5];
        return init_pose;

    }


    gtsam::Pose3 tensor2gtsamPose3(const torch::Tensor &this_point)
    {
        auto pose_tr_cpu = this_point.to(torch::kCPU);
        std::vector<double> pose_vector = std::vector<double>(pose_tr_cpu.data_ptr<float>(), pose_tr_cpu.data_ptr<float>()+6);
        auto x = pose_vector[0];
        auto y = pose_vector[1];
        auto z = pose_vector[2];
        auto roll = pose_vector[3];
        auto pitch = pose_vector[4];
        auto yaw = pose_vector[5];

        return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    }

    std::tuple<Eigen::Matrix<double, 3, 3>, Eigen::Vector3d> tensor2gtsamtransform(const torch::Tensor &this_point)
    {
        auto x = this_point[0].item<double>();
        auto y = this_point[1].item<double>();
        auto z = this_point[2].item<double>();
        auto roll = this_point[3].item<double>();
        auto pitch = this_point[4].item<double>();
        auto yaw = this_point[5].item<double>();


        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

//        auto rotation_matrix = gtsam::Rot3::RzRyRx(double(roll), double(pitch), yaw).matrix();
        Eigen::Matrix<double, 3, 3> rotation_matrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
        auto translation_vector = Eigen::Vector3d(x, y, z);

        return std::tuple(rotation_matrix, translation_vector);
    }

    Eigen::Matrix<double, 4, 4> tensor2Matrix(const torch::Tensor &this_point)
    {
        auto [rot, tran] = tensor2gtsamtransform(this_point);
//        auto rot_mat = rot.matrix();
        gtsam::Matrix4 T;
        T.block(0,0,3,3) = rot;
        T.block(0,3,3,1) = tran;
        T.block(3,0,1,4) << 0,0,0,1;
        return T;
    }

}

