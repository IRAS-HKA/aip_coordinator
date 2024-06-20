#include <aip_coordinator/services/LLMProcessing.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string LLMProcessing::ros2_service_name()
{
    return "/sene_interpretation";   
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
BT::PortsList LLMProcessing::providedPorts()
{
    return {
            BT::InputPort<std::string>("user_input"),
            BT::InputPort<object_detector_tensorflow_interfaces::msg::Detections>("detections"),        
            BT::OutputPort<std::vector<std::string>>("objects_to_pick")
            };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void LLMProcessing::on_send(std::shared_ptr<LLMProcessingSrv::Request> request)
{
    request->user_input = ports.get_value<std::string>("user_input");

    auto detections = ports.get_value<object_detector_tensorflow_interfaces::msg::Detections>("detections");
    request->detections = detections.detections;

    // request->detections = ports.get_value<object_detector_tensorflow_interfaces::msg::Detections>("detections");

    log("Sending Request for User Input: " + request->user_input + " to LLM Processing Service");

    std::stringstream ss;
    ss << "Detections:" << std::endl;

    // for (const auto& detection : request->detections.detections) {
    //     ss << "  Detection:" << std::endl;
    //     ss << "    Class ID: " << detection.class_id << std::endl;
    //     ss << "    Class Name: " << detection.class_name << std::endl;
    //     ss << "    Probability: " << detection.probability << std::endl;
    //     ss << "    Center: (" << detection.center.x << ", " << detection.center.y << ", " << detection.center.z << ")" << std::endl;
    //     ss << "    Bounding Box: [x: " << detection.bounding_box.x_offset << ", y: " << detection.bounding_box.y_offset 
    //        << ", width: " << detection.bounding_box.width << ", height: " << detection.bounding_box.height << "]" << std::endl;
    // }

    log(ss.str());

}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool LLMProcessing::on_result(std::shared_ptr<LLMProcessingSrv::Response> response, std::shared_ptr<LLMProcessingSrv::Request>)
{

    // Extract objects_to_pick sequence from response
    std::vector<std::string> objects_to_pick_sequence(response->objects_to_pick.begin(), response->objects_to_pick.end());

    // Set the extracted values to the output port
    ports.set_value("objects_to_pick", objects_to_pick_sequence);

    // Log the received objects_to_pick
    std::stringstream ss;
    ss << "Received objects to pick from LLM: ";
    for (const auto& object : objects_to_pick_sequence) {
        ss << object << " ";
    }
    log(ss.str());
    
    return true;
}