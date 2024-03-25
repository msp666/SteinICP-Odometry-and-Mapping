//
// Created by haoming on 14.08.23.
//

#ifndef STEINICP_SCANCONTEXT_H
#define STEINICP_SCANCONTEXT_H

#pragma once

#include <Eigen/Eigen>


#include "types.h"

namespace ScanContext
{

    struct ScanContextParam
    {
        ScanContextParam() = default;
        ScanContextParam(const int num_ring, const int num_sector, const double max_radius)
        : PC_NUM_RING_(num_ring), PC_NUM_SECTOR_(num_sector), PC_MAX_RADIUS_(max_radius)
        {}

        const double LIDAR_HEIGHT_ = 2.0; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

        int    PC_NUM_RING_ = 20; // 20 in the original paper (IROS 18)
        int    PC_NUM_SECTOR_ = 60; // 60 in the original paper (IROS 18)
        double PC_MAX_RADIUS_ = 80.0; // 80 meter max in the original paper (IROS 18)


    };

    class ScanContext
    {

    private:
        const double LIDAR_HEIGHT_ = 2.0; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

        int    PC_NUM_RING_ = 20; // 20 in the original paper (IROS 18)
        int    PC_NUM_SECTOR_ = 60; // 60 in the original paper (IROS 18)
        double PC_MAX_RADIUS_ = 80.0; // 80 meter max in the original paper (IROS 18)
        const double PC_UNIT_SECTORANGLE_ = 360.0 / double(PC_NUM_SECTOR_);
        const double PC_UNIT_RINGGAP_ = PC_MAX_RADIUS_ / double(PC_NUM_RING_);

        std::vector<std::pair<int, int>> index_2D_;
        pcl::PointCloud<Point_t> pointCloud_;
        Eigen::MatrixXf ScanContext_ = -1000 * Eigen::MatrixXf::Ones(PC_NUM_RING_, PC_NUM_SECTOR_);

    private:

    public:
        ScanContext() = default;

        ScanContext(const ScanContextParam &sc_param);

        void set_param(const ScanContextParam &sc_param);

        std::tuple<Eigen::MatrixXf, std::vector<float>> makeScancontext( Cloud_t &_scan_down );

        pcl::PointCloud<Point_t> DownsampleCloud();

    };
}

#endif //STEINICP_SCANCONTEXT_H
