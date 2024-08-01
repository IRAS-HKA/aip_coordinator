/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "MoveArmToExitGraspPoseSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <aip_interfaces/srv/move_to_pose.hpp>

using MoveArmToExitGraspPoseSrv = aip_interfaces::srv::MoveToPose;

class MoveArmToExitGraspPose : public RosService<MoveArmToExitGraspPoseSrv>
{
public:
    static BT::PortsList providedPorts();

    MoveArmToExitGraspPose(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<MoveArmToExitGraspPoseSrv::Request> request) override;
    bool on_result(std::shared_ptr<MoveArmToExitGraspPoseSrv::Response> response, std::shared_ptr<MoveArmToExitGraspPoseSrv::Request> request) override;
};