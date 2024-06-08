#include <aip_coordinator/services/PackPlanning.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string PackPlanning::ros2_service_name()
{
    return "/pack_planning";   // ToDo: TBC
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
BT::PortsList PackPlanning::providedPorts()
{
    return {BT::InputPort<std::vector<std::string>>("objects_to_pick"),
            BT::OutputPort<aip_packing_planning_interfaces::srv::PackSequence>("PackSequence"),}; // ::Response
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void PackPlanning::on_send(std::shared_ptr<PackPlanningSrv::Request> request)
{
  
    request->objects_to_pick = ports.get_value<std::vector<std::string>>("objects_to_pick");

     std::string objects_to_pick_str = std::accumulate(request->objects_to_pick.begin(), request->objects_to_pick.end(), std::string(),
        [](const std::string& a, const std::string& b) -> std::string {
            return a + (a.length() > 0 ? ", " : "") + b;
        });

    log("Sending Pack Planning Request for Objects: " + objects_to_pick_str + " to Pack Planning Service");
    // log("Sending Pack Planning Request for Objects: " + request->objects_to_pick + " to Pack Planning Service");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool PackPlanning::on_result(std::shared_ptr<PackPlanningSrv::Response> response, std::shared_ptr<PackPlanningSrv::Request>)
{
    ports.set_value<aip_packing_planning_interfaces::msg::PackageSequence>("PackageSequence", response->package);

    log("Received a PackingPlan containing " + std::to_string(response->package.packages.size()) + " packages");
    
    return true;
}