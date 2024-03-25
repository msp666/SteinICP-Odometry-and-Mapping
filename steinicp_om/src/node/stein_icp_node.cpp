//
// Created by haoming on 27.06.23.
//


#include <rclcpp/rclcpp.hpp>
#include "onlineFGO_test.h"

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<online_fgo::onlineFGO_test>();
    static const size_t THREAD_NUM = 7;
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), THREAD_NUM);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}