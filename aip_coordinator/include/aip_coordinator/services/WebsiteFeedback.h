/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "WebsiteFeedbackSrv"
 *
 * @author
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <llm_interfaces/srv/website_feedback.hpp>
#include <aip_packing_planning_interfaces/msg/package_sequence.hpp>
#include <aip_packing_planning_interfaces/msg/solution_feedback.hpp>
#include <aip_grasp_planning_interfaces/msg/cylinder_combination.hpp>



// using PackPlanningSrv = aip_packing_planning_interfaces::srv::PackSequence;
using WebsiteFeedbackSrv = llm_interfaces::srv::WebsiteFeedback;

class WebsiteFeedback : public RosService<WebsiteFeedbackSrv>
{
public:
    static BT::PortsList providedPorts();

    WebsiteFeedback(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<WebsiteFeedbackSrv::Request> request) override;
    bool on_result(std::shared_ptr<WebsiteFeedbackSrv::Response> response, std::shared_ptr<WebsiteFeedbackSrv::Request> request) override;
};