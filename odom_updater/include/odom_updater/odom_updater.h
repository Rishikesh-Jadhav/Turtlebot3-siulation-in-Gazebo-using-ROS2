/**
 * @file odom_updater.h
 * @author Kiran S Patil, Nishant Panday, Rishikesh Jadhav
 * @brief  Header file for class odom_updater
 * @version 0.1
 * @date 2022-12-16
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>

/**
 * @brief class OdomUpdater contains members used to broadcast 
 *        frame /robot1/base_footprint as a child of frame /robot1/odom.
 * 
 * 
 */
class OdomUpdater : public rclcpp::Node
{
  public:
    OdomUpdater()
    : Node("odom_updater")
    {
      m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

      m_tf_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/robot1/odom", 10, std::bind(&OdomUpdater::odom_callback, this, std::placeholders::_1));
    }

  private:
      //atributes
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_tf_subscriber;
      std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

      /**
       * @brief callback function to transform "/robot1/odom" frame to "/robot1/base_footprint" frame
       * 
       * @param msg 
       */
      void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
};