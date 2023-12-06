/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Action client "navigate_to_pose"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2022.05.12)
 *********************************************************/
#pragma once

#include <iras_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosAction.h>
#include <iras_behaviortree_ros2/tools/convert.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>

// TODO: rename to BaseMovementNav2

using NavigateToPoseROS2Nav = nav2_msgs::action::NavigateToPose;

class BaseMovementNav2 : public RosAction<NavigateToPoseROS2Nav>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("frame_id"),
                                                   BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y"),
                                                   BT::InputPort<float>("quaternion_x"),
                                                   BT::InputPort<float>("quaternion_y"),
                                                   BT::InputPort<float>("quaternion_z"),
                                                   BT::InputPort<float>("quaternion_w")}; }

    BaseMovementNav2(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros_name() { return "navigate_to_pose"; }

    void on_send(NavigateToPoseROS2Nav::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const NavigateToPoseROS2Nav::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<NavigateToPoseROS2Nav>::WrappedResult &result, const NavigateToPoseROS2Nav::Goal &goal) override;
};