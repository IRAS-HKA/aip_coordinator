#include <aip_coordinator/services/DetectObject.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string DetectObject::ros2_service_name()
{
    return "/detect_and_transform_node/detect_object_and_transform";
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
BT::PortsList DetectObject::providedPorts()
{
    return {BT::InputPort<std::string>("class_name"),
            BT::InputPort<std::string>("base_frame"),
            BT::InputPort<std::string>("camera_type"),
            BT::OutputPort<float>("x"),
            BT::OutputPort<float>("y"),
            BT::OutputPort<float>("z"),
            BT::OutputPort<float>("probability"),
            BT::OutputPort<std::string>("class_name")};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void DetectObject::on_send(std::shared_ptr<DetectObjectSrv::Request> request)
{
    request->class_name = ports.get_value<std::string>("class_name");
    request->base_frame = ports.get_value<std::string>("base_frame");
    request->camera_type = ports.get_value<std::string>("camera_type");

    log("Detect object of class " + request->class_name + " in frame " + request->base_frame);
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool DetectObject::on_result(std::shared_ptr<DetectObjectSrv::Response> response, std::shared_ptr<DetectObjectSrv::Request>)
{
    ports.set_value<float>("x", response.get()->point.x);
    ports.set_value<float>("y", response.get()->point.y);
    ports.set_value<float>("z", response.get()->point.z);
    ports.set_value<float>("probability", response.get()->probability);
    ports.set_value<std::string>("class_name", response.get()->class_name);

    if (response.get()->probability == 0.0)
    {
        log("No object of class " + response.get()->class_name + " detected");
        return false;
    }

    log("Detected object of class " + response.get()->class_name + " with probability " + Converter::ftos(response.get()->probability) + " at position (" + Converter::ftos(response.get()->point.x) + ", " + Converter::ftos(response.get()->point.y) + ", " + Converter::ftos(response.get()->point.z) + ")");
    return true;
}