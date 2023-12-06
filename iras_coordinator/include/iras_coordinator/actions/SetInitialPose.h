/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Sends a ROS2-Topic message "/initialpose"
 *
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <iras_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>
#include <iras_behaviortree_ros2/tools/convert.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class SetInitialPose : public RosNode
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y"),
                                                   BT::InputPort<float>("z"),
                                                   BT::InputPort<float>("quaternion_x"),
                                                   BT::InputPort<float>("quaternion_y"),
                                                   BT::InputPort<float>("quaternion_z"),
                                                   BT::InputPort<float>("quaternion_w")}; }

    SetInitialPose(const std::string &name, const BT::NodeConfiguration &config);

    std::string ros_name() { return "initialpose"; }

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override { return BT::NodeStatus::RUNNING; }
    void on_halted() override {}

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
};