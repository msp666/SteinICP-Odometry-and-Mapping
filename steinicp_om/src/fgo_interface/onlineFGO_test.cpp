//
// Created by haoming on 12.07.23.
//

#include "onlineFGO_test.h"

namespace online_fgo
{
      onlineFGO_test::onlineFGO_test() : rclcpp::Node("onlineFGO_test")
      {
          RCLCPP_INFO(this->get_logger(), "[FGO test]: Node initializing");
          auto cb_group_odom = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          auto cb_group_vs = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          steinicp_odom_ = std::make_shared<stein_icp::SteinICPOdometry>(*this,
                                                                     cb_group_odom,
                                                                     cb_group_vs,
                                                                     "SteinICP_integrator");

      }
}