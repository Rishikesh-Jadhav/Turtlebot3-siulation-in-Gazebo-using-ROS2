#include <rclcpp/rclcpp.hpp>
#include "../include/odom_updater/odom_updater.h"

/**
 * @brief This main function transforms frame.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomUpdater>());
    rclcpp::shutdown();
    return 0;
}