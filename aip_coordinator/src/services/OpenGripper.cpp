#include <aip_coordinator/services/OpenGripper.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string OpenGripper::ros2_service_name()
{
    return "/open_gripper";
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
BT::PortsList OpenGripper::providedPorts()
{
    return {BT::InputPort<std::vector<aip_grasp_planning_interfaces::msg::CylinderCombination>>("cylinder_ids")
    };      
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void OpenGripper::on_send(std::shared_ptr<OpenGripperSrv::Request> request)
{
    // Get FIRST cylinder_id combination from Vector of input port cylinder_ids

    aip_grasp_planning_interfaces::msg::CylinderCombination cylinder_combination1;

    cylinder_combination1 = ports.get_value<std::vector<aip_grasp_planning_interfaces::msg::CylinderCombination>>("cylinder_ids")[1];

    request->cylinder_ids = cylinder_combination1;

   // log the content of the request message
    log("Request for Close Gripper: Length of call " + std::to_string(request->cylinder_ids.cylinder_ids.size()));

    std::string cylinder_ids_str;
    for (std::vector<int>::size_type i = 0; i < request->cylinder_ids.cylinder_ids.size(); ++i) {
        cylinder_ids_str += std::to_string(request->cylinder_ids.cylinder_ids[i]) + " ";
    }
    log("Request for Close Gripper of cylinder_id combinations: " + cylinder_ids_str);
 
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool OpenGripper::on_result(std::shared_ptr<OpenGripperSrv::Response>, std::shared_ptr<OpenGripperSrv::Request>)
{

    log("Opened Gripper successfully.");

    return true;
}