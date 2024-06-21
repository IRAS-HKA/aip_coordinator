#include <aip_coordinator/services/CloseGripper.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string CloseGripper::ros2_service_name()
{
    return "/close_gripper";
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
BT::PortsList CloseGripper::providedPorts()
{
    return {
        BT::InputPort<std::vector<aip_grasp_planning_interfaces::msg::CylinderCombination>>("cylinder_ids"),
        BT::InputPort<int>("object_no")
    };      
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void CloseGripper::on_send(std::shared_ptr<CloseGripperSrv::Request> request)
{
    // Get FIRST cylinder_id combination from Vector of input port cylinder_ids

    int object_no;
    object_no = ports.get_value<int>("object_no");
    
    aip_grasp_planning_interfaces::msg::CylinderCombination cylinder_combination;

    cylinder_combination = ports.get_value<std::vector<aip_grasp_planning_interfaces::msg::CylinderCombination>>("cylinder_ids")[object_no];

    request->cylinder_ids = cylinder_combination;

    // log the content of the request message
    log("Requesting Close Gripper for object_no" + std::to_string(object_no) + "with Length of call " + std::to_string(request->cylinder_ids.cylinder_ids.size()));
    // log("Request for Close Gripper: Length of call " + std::to_string(request->cylinder_ids.cylinder_ids.size()));

    std::string cylinder_ids_str;
    for (std::vector<int>::size_type i = 0; i < request->cylinder_ids.cylinder_ids.size(); ++i) {
        cylinder_ids_str += std::to_string(request->cylinder_ids.cylinder_ids[i]) + " ";
    }
    log("Request for Close Gripper of cylinder_id combinations: " + cylinder_ids_str);
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool CloseGripper::on_result(std::shared_ptr<CloseGripperSrv::Response>, std::shared_ptr<CloseGripperSrv::Request>)
{

    log("Closed Gripper successfully.");

    return true;
}