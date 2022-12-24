/**
 * @file target_reacher.h
 * @author Kiran S Patil, Nishant Panday, Rishikesh Jadhav
 * @brief  Header file for class target reacher
 * @version 0.8
 * @date 2022-12-16
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * @brief class TargetReacher derived publically from rclcpp::node class contains three callback functions.
 *        parameters, publisher and subscribers are defined in its ctor.
 * @class parameters and other attributes needed by the callback functions are declared privately. 
 */
class TargetReacher : public rclcpp::Node
{

    public:

        /**
         * @brief ctor of class TargetReacher.
         * @param[in] bot_controller a shared pointer with object of class BotController.
         *            defines the Node "Target_reacher"
         */
        TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
        {

            m_bot_controller = bot_controller;
            aruco_target_x = this->declare_parameter<double>("aruco_target.x");
            aruco_target_y = this->declare_parameter<double>("aruco_target.y");
            aruco_0_x = this->declare_parameter<double>("final_destination.aruco_0.x");
            aruco_0_y = this->declare_parameter<double>("final_destination.aruco_0.y");
            aruco_1_x = this->declare_parameter<double>("final_destination.aruco_1.x");
            aruco_1_y = this->declare_parameter<double>("final_destination.aruco_1.y");
            aruco_2_x = this->declare_parameter<double>("final_destination.aruco_2.x");
            aruco_2_y = this->declare_parameter<double>("final_destination.aruco_2.y");
            aruco_3_x = this->declare_parameter<double>("final_destination.aruco_3.x");
            aruco_3_y = this->declare_parameter<double>("final_destination.aruco_3.y");
            frame_id  = this->declare_parameter<std::string>("final_destination.frame_id");

            m_bot_controller->set_goal(aruco_target_x,aruco_target_y);
            goal_x = aruco_target_x;
            goal_y = aruco_target_y;

            /**
             * @brief When running a node in a multi-threaded Executor, 
             *        ROS2 offers callback groups for controlling the execution of different callbacks.
             * 
             */
            m_callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            m_callback_group_3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            auto subscription_option1 = rclcpp::SubscriptionOptions();
            subscription_option1.callback_group = m_callback_group_1;
            auto subscription_option2 = rclcpp::SubscriptionOptions();
            subscription_option2.callback_group = m_callback_group_2;
            auto subscription_option3 = rclcpp::SubscriptionOptions();
            subscription_option3.callback_group = m_callback_group_3;
            

            publisher_2 = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
            m_msg = geometry_msgs::msg::Twist();
            subscription_1 = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10,  std::bind(&TargetReacher::msg_callback, this, std::placeholders::_1), subscription_option1);
            subscription_2 = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10,  std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1), subscription_option2);

            m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

            subscription_3 = this->create_subscription<nav_msgs::msg::Odometry>("robot1/odom", 10,  std::bind(&TargetReacher::odom_callback, this, std::placeholders::_1),subscription_option3);
            
            m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

           
            m_timer_listener = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / 1)),  std::bind(&TargetReacher::listener_callback, this));
        }

        /**
         * @brief  aruco callback funtion is used to determine final goal coordiantes depending on the marker id 
         * @param[in] aruco_msg a shared pointer of type ros2_aruco_interfaces::msg::ArucoMarkers
         *            a subscriper to this topics helps to get the marker id  
         */
        void aruco_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> aruco_msg);

        /**
         * @brief function used for broadcasting a frame final_destination in the frame origin.
         * 
         */
        void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
    
    private:

        std::shared_ptr<BotController> m_bot_controller;
        bool m_goal;
        double aruco_target_x;
        double aruco_target_y;
        double aruco_0_x;
        double aruco_0_y;
        double aruco_1_x;
        double aruco_1_y;
        double aruco_2_x;
        double aruco_2_y;
        double aruco_3_x;
        double aruco_3_y;
        double goal_x;
        double goal_y;
        int count{0};
        std::string frame_id;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_1;
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_2;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3;

        rclcpp::TimerBase::SharedPtr m_timer_broadcaster{nullptr};      
        std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
        rclcpp::TimerBase::SharedPtr m_timer_listener{nullptr};
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

        geometry_msgs::msg::Twist m_msg;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_2;

        rclcpp::CallbackGroup::SharedPtr m_callback_group_1;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_2;
        rclcpp::CallbackGroup::SharedPtr m_callback_group_3;

        /**
         * @brief   callback function used by subscription_1 
         *          when goal is reached a message is published to the Topic goal_reached need a subscriber to this Topic.
         * @param[in] msg2 of type Bool, returns true when goal is reached. 
         */
        void msg_callback(const std::shared_ptr<std_msgs::msg::Bool> msg2);

        /**
         * @brief  listener callback to get the transform between final_destination and /robot1/odom . 
         *         The result gives the pose of the ﬁnal_destination in the Frame /robot1/odom
         * 
         */
        void listener_callback();

 
};

