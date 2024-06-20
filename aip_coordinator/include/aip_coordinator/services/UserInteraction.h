#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>


#include <llm_interfaces/srv/user_interaction.hpp>

using UserInteractionSrv = llm_interfaces::srv::UserInteraction;

class UserInteraction : public RosService<UserInteractionSrv>
{
public:
    static BT::PortsList providedPorts();

    UserInteraction(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<UserInteractionSrv::Request> request) override;
    bool on_result(std::shared_ptr<UserInteractionSrv::Response> response, std::shared_ptr<UserInteractionSrv::Request> request) override;
};
