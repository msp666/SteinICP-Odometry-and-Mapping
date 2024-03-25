//
// Created by haoming on 14.08.23.
//

#include "ScanContext.h"

namespace ScanContext
{


    float xy2theta( const float & _x, const float & _y )
    {
        if ( (_x >= 0) & (_y >= 0))
            return (180/M_PI) * atan(_y / _x);

        if ( (_x < 0) & (_y >= 0))
            return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

        if ( (_x < 0) & (_y < 0))
            return 180 + ( (180/M_PI) * atan(_y / _x) );

        if ( (_x >= 0) & (_y < 0))
            return 360 - ( (180/M_PI) * atan((-_y) / _x) );
    } // xy2theta



    ScanContext::ScanContext(const ScanContextParam &sc_param)
    {
        this->set_param(sc_param);
    }

    void ScanContext::set_param(const ScanContextParam &sc_param)
    {
        PC_NUM_RING_     = sc_param.PC_NUM_RING_;
        PC_NUM_SECTOR_   = sc_param.PC_NUM_SECTOR_;
        PC_MAX_RADIUS_   = sc_param.PC_MAX_RADIUS_;
    }

    std::tuple<Eigen::MatrixXf, std::vector<float>> ScanContext::makeScancontext(Cloud_t &_scan_down)
    {
        pointCloud_ = _scan_down;
        index_2D_.clear();

        int num_pts_scan_down = _scan_down.points.size();

        // main
        const int NO_POINT = -1000;
        Eigen::MatrixXf desc = NO_POINT * Eigen::MatrixXf::Ones(PC_NUM_RING_, PC_NUM_SECTOR_);

        std::vector<float> index_1D;

        Point_t pt;
        float azim_angle, azim_range; // wihtin 2d plane
        int ring_idx, sctor_idx;
        for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
        {
            pt.x = _scan_down.points[pt_idx].x;
            pt.y = _scan_down.points[pt_idx].y;
            pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT_; // naive adding is ok (all points should be > 0).

            // xyz to ring, sector
            azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
            azim_angle = xy2theta(pt.x, pt.y);

            // if range is out of roi, pass
            if( azim_range > PC_MAX_RADIUS_ )
            {
                index_1D.push_back(-1);
                continue;
            }


            ring_idx = std::max( std::min( PC_NUM_RING_, int(ceil( (azim_range / PC_MAX_RADIUS_) * PC_NUM_RING_ )) ), 1 );
            sctor_idx = std::max( std::min( PC_NUM_SECTOR_, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR_ )) ), 1 );

            index_1D.push_back((ring_idx-1)*PC_NUM_SECTOR_+sctor_idx-1);
            index_2D_.emplace_back(ring_idx-1, sctor_idx-1);

            // taking maximum z
            if ( desc(ring_idx-1, sctor_idx-1) < pt.z ) // -1 means cpp starts from 0
                desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
        }

        // reset no points to zero (for cosine dist later)
        for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
            for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
                if( desc(row_idx, col_idx) == NO_POINT )
                    desc(row_idx, col_idx) = 0;

        ScanContext_ = desc;

        return {desc, index_1D};
    }

    pcl::PointCloud<Point_t> ScanContext::DownsampleCloud()
    {
        pcl::PointCloud<Point_t> cloud_ds;
        auto max_sc = ScanContext_.maxCoeff();
        for(size_t i = 0; i < index_2D_.size(); ++i)
        {
            if (ScanContext_(index_2D_[i].first, index_2D_[i].second) > 0.2 * max_sc)
            {
                cloud_ds.push_back(pointCloud_.points[i]);
            }

        }
        return cloud_ds;
    }



}