/*********************************************************
 * Neobotix - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Action client "MoveArmToPose"
 *
 * @author Philipp Kirsch
 * @since 1.0.0 (2022.06.28)
 *********************************************************/
#pragma once

#include <iras_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosAction.h>
#include <iras_behaviortree_ros2/tools/convert.h>
#include <iras_interfaces/action/move_arm_move_it.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using MoveArmMoveIt = iras_interfaces::action::MoveArmMoveIt;
// TODO:
// Custom Action interface for arm control

class ArmMovementMoveIt : public RosAction<MoveArmMoveIt>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y"),
                                                   BT::InputPort<float>("z"),
                                                   BT::InputPort<float>("rotation_x"),
                                                   BT::InputPort<float>("rotation_y"),
                                                   BT::InputPort<float>("rotation_z"),
                                                   BT::InputPort<bool>("cart"), // TODO: rename cartesian
                                                   BT::InputPort<float>("speed")}; }

    ArmMovementMoveIt(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros_name() { return "move_to_pose"; }

    void on_send(MoveArmMoveIt::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const MoveArmMoveIt::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<MoveArmMoveIt>::WrappedResult &result, const MoveArmMoveIt::Goal &goal) override;
};