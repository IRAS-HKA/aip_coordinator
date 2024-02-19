/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "DetectObjectSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <object_detector_tensorflow_interfaces/srv/detect_object_position.hpp>

using DetectObjectSrv = object_detector_tensorflow_interfaces::srv::DetectObjectPosition;

class DetectObject : public RosService<DetectObjectSrv>
{
public:
    static BT::PortsList providedPorts();

    DetectObject(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<DetectObjectSrv::Request> request) override;
    bool on_result(std::shared_ptr<DetectObjectSrv::Response> response, std::shared_ptr<DetectObjectSrv::Request> request) override;
};