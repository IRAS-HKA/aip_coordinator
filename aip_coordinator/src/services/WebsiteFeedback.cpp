#include <aip_coordinator/services/WebsiteFeedback.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string WebsiteFeedback::ros2_service_name()
{
    return "/website_feedback";
}

/**
 * @brief Set the list of ports provided by the BT node.
 *
 * New port:
 *      direction = [BT::InputPort, BT::OutputPort, BT::BidirectionalPort]
 *      data_type = <[float, int, std::string]>
 *      name = ("name")
 *
 * @return List of provided ports.
 */
BT::PortsList WebsiteFeedback::providedPorts()
{
    return {
            BT::InputPort<aip_packing_planning_interfaces::msg::PackageSequence>("package_sequence"),
            BT::InputPort<aip_packing_planning_interfaces::msg::SolutionFeedback>("feedback"),
            BT::InputPort<std::vector<aip_grasp_planning_interfaces::msg::CylinderCombination>>("cylinder_ids"),            
            BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("grasp_poses"),
            BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("place_poses")           
            }; // ::Response
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void WebsiteFeedback::on_send(std::shared_ptr<WebsiteFeedbackSrv::Request> request)
{

    request->package = ports.get_value<aip_packing_planning_interfaces::msg::PackageSequence>("package_sequence");
    request->feedback = ports.get_value<aip_packing_planning_interfaces::msg::SolutionFeedback>("feedback");

    request->cylinder_ids = ports.get_value<std::vector<aip_grasp_planning_interfaces::msg::CylinderCombination>>("cylinder_ids");
    request->grasp_poses = ports.get_value<std::vector<geometry_msgs::msg::Pose>>("grasp_poses");
    request->place_poses = ports.get_value<std::vector<geometry_msgs::msg::Pose>>("place_poses");

    log("Sending Website Feedback Request to Website Feedback Service");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool WebsiteFeedback::on_result(std::shared_ptr<WebsiteFeedbackSrv::Response> response, std::shared_ptr<WebsiteFeedbackSrv::Request>)
{

    return true;
}