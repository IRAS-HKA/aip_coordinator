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

    request->cylinder_ids = ports.get_value<std::vector<std::vector<int>>>("cylinder_ids")[0];

    log("Request for Open Gripper: Length of call " + std::to_string(request->cylinder_ids.size()));

    log("Request for Open Gripper of first cylinder_id combination" + std::to_string(request->cylinder_ids.at(0)));
    //log("Request for Open Gripper " + std::to_string(request->cylinder_ids.at(1)));

}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool OpenGripper::on_result(std::shared_ptr<OpenGripperSrv::Response>, std::shared_ptr<OpenGripperSrv::Request>)
{

    log("Opened Gripper successfully.");

    return true;
}