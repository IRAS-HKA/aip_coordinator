/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "CloseGripperSrv"
 *
 * @author Maurice Droll, Andreas Schmitt, Leo Schäfer 
 * @since 1.0.0 (2024.03.05)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <aip_interfaces/srv/viz_open_gripper.hpp> 
#include <aip_interfaces/msg/move_cylinders.hpp>
#include <aip_grasp_planning_interfaces/msg/cylinder_combination.hpp>

using VizOpenGripperSrv = aip_interfaces::srv::VizOpenGripper;

class VizOpenGripper : public RosService<VizOpenGripperSrv>
{
public:
    static BT::PortsList providedPorts();

    VizOpenGripper(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<VizOpenGripperSrv::Request> request) override;
    bool on_result(std::shared_ptr<VizOpenGripperSrv::Response> response, std::shared_ptr<VizOpenGripperSrv::Request> request) override;
};