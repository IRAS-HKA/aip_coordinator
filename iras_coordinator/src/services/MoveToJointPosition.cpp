#include <iras_coordinator/services/MoveToJointPosition.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string MoveToJointPosition::ros2_service_name()
{
    return "/move_to_joint_position";
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
BT::PortsList MoveToJointPosition::providedPorts()
{
    return {};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void MoveToJointPosition::on_send(std::shared_ptr<MoveToJointPositionSrv::Request> request)
{
    request->joint_position.push_back(0.0);
    request->joint_position.push_back(0.0);
    request->joint_position.push_back(0.0);
    request->joint_position.push_back(0.0);
    request->joint_position.push_back(0.0);
    request->joint_position.push_back(0.0);
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool MoveToJointPosition::on_result(std::shared_future<std::shared_ptr<MoveToJointPositionSrv::Response>>, std::shared_ptr<MoveToJointPositionSrv::Request>)
{
    log("MoveToJointPosition completed");
    return true;
}