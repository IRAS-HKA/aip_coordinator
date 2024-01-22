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
    return {BT::InputPort<float>("joint1"),
            BT::InputPort<float>("joint2"),
            BT::InputPort<float>("joint3"),
            BT::InputPort<float>("joint4"),
            BT::InputPort<float>("joint5"),
            BT::InputPort<float>("joint6")};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void MoveToJointPosition::on_send(std::shared_ptr<MoveToJointPositionSrv::Request> request)
{
    request->joint_position.push_back(ports.get_value<float>("joint1"));
    request->joint_position.push_back(ports.get_value<float>("joint2"));
    request->joint_position.push_back(ports.get_value<float>("joint3"));
    request->joint_position.push_back(ports.get_value<float>("joint4"));
    request->joint_position.push_back(ports.get_value<float>("joint5"));
    request->joint_position.push_back(ports.get_value<float>("joint6"));
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool MoveToJointPosition::on_result(std::shared_future<std::shared_ptr<MoveToJointPositionSrv::Response>>, std::shared_ptr<MoveToJointPositionSrv::Request>)
{
    log("MoveToJointPosition completed");
    return true;
}