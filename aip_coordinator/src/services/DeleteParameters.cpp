// #include <yaml-cpp/yaml.h>
#include <aip_coordinator/services/DeleteParameters.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string DeleteParameters::ros2_service_name()
{
    return "/LLM/delete_parameters";
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
BT::PortsList DeleteParameters::providedPorts()
{
    return {}; // ::Response
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void DeleteParameters::on_send(std::shared_ptr<DeleteParametersSrv::Request> request)
{

}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool DeleteParameters::on_result(std::shared_ptr<DeleteParametersSrv::Response> response, std::shared_ptr<DeleteParametersSrv::Request>)
{
    return true;
}