#include <aip_coordinator/services/UserInteraction.h>
/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string UserInteraction::ros2_service_name()
{
    return "/userinteraction";
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
BT::PortsList UserInteraction::providedPorts()
{
    return {
        BT::InputPort<object_detector_tensorflow_interfaces::msg::Detections>("detections"),
        BT::OutputPort<std::string>("user_input"),
    };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void UserInteraction::on_send(std::shared_ptr<UserInteractionSrv::Request> request)
{
    // request->detections = ports.get_value<std::vector<object_detector_tensorflow_interfaces::msg::Detection>>("detections");
    request->detections = ports.get_value<object_detector_tensorflow_interfaces::msg::Detections>("detections");

    log("Detection are being sent to website container");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool UserInteraction::on_result(std::shared_ptr<UserInteractionSrv::Response> response, std::shared_ptr<UserInteractionSrv::Request>)
{
    ports.set_value<std::string>("user_input", response.get()->user_input);
    // ports.set_value<std::string>("class_name", response.get()->class_name);

    log("UserInput: " + response.get()->user_input);
    return true;
}