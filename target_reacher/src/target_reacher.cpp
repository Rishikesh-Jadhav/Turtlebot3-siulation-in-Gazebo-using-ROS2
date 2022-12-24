#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"


void TargetReacher::msg_callback(const std::shared_ptr<std_msgs::msg::Bool> msg2)
{
    RCLCPP_INFO(this->get_logger(), "Data: '%d'", msg2->data);
    if(msg2->data == 1)
    {
        RCLCPP_INFO(this->get_logger(), "turn now");
        m_msg.angular.z = 0.2;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", m_msg.angular.z);
        publisher_2->publish(m_msg);
        
    }
} 

void TargetReacher::aruco_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> aruco_msg)
{  
    if((aruco_msg->marker_ids.at(0) == 0 || aruco_msg->marker_ids.at(0) == 1 || aruco_msg->marker_ids.at(0) == 2 || aruco_msg->marker_ids.at(0) == 3) && count ==0)
    {
        RCLCPP_INFO(this->get_logger(), "data: '%ld'", aruco_msg->marker_ids.at(0));
        RCLCPP_INFO(this->get_logger(), "aruco detected : %ld",aruco_msg->marker_ids.at(0));
        m_msg.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%lf'", m_msg.angular.z);
        publisher_2->publish(m_msg);
        count++;

        switch(aruco_msg->marker_ids.at(0))
        {
            case 0 : RCLCPP_INFO(this->get_logger(), "final_destination.aruco_0.x '%lf'", aruco_0_x);
                        RCLCPP_INFO(this->get_logger(), "final_destination.aruco_0.y '%lf'", aruco_0_y);
                        goal_x = aruco_0_x;
                        goal_y = aruco_0_y;
                break;
            case 1 : RCLCPP_INFO(this->get_logger(), "final_destination.aruco_1.x '%lf'", aruco_1_x);
                        RCLCPP_INFO(this->get_logger(), "final_destination.aruco_1.y '%lf'", aruco_1_y);
                        goal_x = aruco_1_x;
                        goal_y = aruco_1_y;
                break;
            case 2 : RCLCPP_INFO(this->get_logger(), "final_destination.aruco_2.x '%lf'", aruco_2_x);
                        RCLCPP_INFO(this->get_logger(), "final_destination.aruco_2.y '%lf'", aruco_2_y);
                        goal_x = aruco_2_x;
                        goal_y = aruco_2_y;
                break;
            case 3 : RCLCPP_INFO(this->get_logger(), "final_destination.aruco_3.x '%lf'", aruco_3_x);
                        RCLCPP_INFO(this->get_logger(), "final_destination.aruco_3.y '%lf'", aruco_3_y);
                        goal_x = aruco_3_x;
                        goal_y = aruco_3_y;
                break;
        }
        m_goal = true;
    }
}

void TargetReacher::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = frame_id;
    t.child_frame_id = "/final_destination";

    t.transform.translation.x = goal_x;
    t.transform.translation.y = goal_y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    m_tf_broadcaster->sendTransform(t);

}

void TargetReacher::listener_callback()
{
    if(m_goal)
    {
        geometry_msgs::msg::TransformStamped t;

        try
        {
            t = m_tf_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Transform %s to %s: %s","robot1/odom", "final_destination", ex.what());
            return;
        }
        m_bot_controller->set_goal(t.transform.translation.x, t.transform.translation.y);
    RCLCPP_INFO(this->get_logger(), "Location of goal in world is: [%f, %f, %f]", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    }
}

