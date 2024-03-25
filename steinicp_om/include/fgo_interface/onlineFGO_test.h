//
// Created by haoming on 12.07.23.
//

#ifndef STEINICP_ONLINEFGO_H
#define STEINICP_ONLINEFGO_H

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "steinicp_ros2.h"

///Test node, only used for an instance of steinICPROS2

namespace online_fgo
{
    class onlineFGO_test : public rclcpp::Node
    {
        private:
        std::shared_ptr<stein_icp::SteinICPOdometry> steinicp_odom_;

        public:
        onlineFGO_test();

        ~onlineFGO_test() override = default;


    };
}


#endif //STEINICP_ONLINEFGO_H
