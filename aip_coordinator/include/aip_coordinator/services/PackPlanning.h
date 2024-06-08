/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "PackPlanningSrv"
 *
 * @author 
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <aip_packing_planning_interfaces/srv/pack_sequence.hpp>

using PackPlanningSrv = aip_packing_planning_interfaces::srv::PackSequence;

class PackPlanning : public RosService<PackPlanningSrv>
{
public:
    static BT::PortsList providedPorts();

    PackPlanning(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<PackPlanningSrv::Request> request) override;
    bool on_result(std::shared_ptr<PackPlanningSrv::Response> response, std::shared_ptr<PackPlanningSrv::Request> request) override;
};