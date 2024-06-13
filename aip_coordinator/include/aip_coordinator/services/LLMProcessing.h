/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "LLMProcessingSrv"
 *
 * @author 
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <llm_interfaces/srv/scene_interpretation.hpp>

using LLMProcessingSrv = llm_interfaces::srv::SceneInterpretation;

class LLMProcessing : public RosService<LLMProcessingSrv>
{
public:
    static BT::PortsList providedPorts();

    LLMProcessing(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<LLMProcessingSrv::Request> request) override;
    bool on_result(std::shared_ptr<LLMProcessingSrv::Response> response, std::shared_ptr<LLMProcessingSrv::Request> request) override;
};