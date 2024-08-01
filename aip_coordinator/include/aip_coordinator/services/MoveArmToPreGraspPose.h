/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "MoveArmToPreGraspPoseSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <aip_interfaces/srv/move_to_pose.hpp>

using MoveArmToPreGraspPoseSrv = aip_interfaces::srv::MoveToPose;

class MoveArmToPreGraspPose : public RosService<MoveArmToPreGraspPoseSrv>
{
public:
    static BT::PortsList providedPorts();

    MoveArmToPreGraspPose(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<MoveArmToPreGraspPoseSrv::Request> request) override;
    bool on_result(std::shared_ptr<MoveArmToPreGraspPoseSrv::Response> response, std::shared_ptr<MoveArmToPreGraspPoseSrv::Request> request) override;
};