#include <aip_coordinator/services/DetectObjects.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string DetectObjects::ros2_service_name()
{
    return "detection_node/detect_objects";   // ROS2 service server name -> may be changed to detect_objects
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
BT::PortsList DetectObjects::providedPorts()
{
    return {
            // BT::InputPort<bool>("start_detection"),   // check interface update 
            BT::OutputPort<object_detector_tensorflow_interfaces::msg::Detections>("detections"),
            BT::OutputPort<sensor_msgs::msg::Image>("result_image"), // RGB image with BB, Center Point etc.
            BT::OutputPort<sensor_msgs::msg::Image>("reference_image") // Depth image
            };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */

void DetectObjects::on_send(std::shared_ptr<DetectObjectsSrv::Request> )
{
    // request->start_detection = ports.get_value<bool>("detect_objects");
    // log("Requested ODTF to detect objects in szene: (" + Converter::ftos(request->start_detection));
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool DetectObjects::on_result(std::shared_ptr<DetectObjectsSrv::Response> response, std::shared_ptr<DetectObjectsSrv::Request>)
{
    ports.set_value<object_detector_tensorflow_interfaces::msg::Detections>("detections", response->detections);
    ports.set_value<sensor_msgs::msg::Image>("result_image", response->result_image);
    ports.set_value<sensor_msgs::msg::Image>("reference_image", response->reference_image);
    log("Received " + std::to_string(response->detections.detections.size()) + " detections");

    return true;

}