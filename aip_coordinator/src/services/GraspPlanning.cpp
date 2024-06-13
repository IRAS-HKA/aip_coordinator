#include <aip_coordinator/services/GraspPlanning.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string GraspPlanning::ros2_service_name()
{
    return "/grasp_planning";   // ToDo: TBC
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
BT::PortsList GraspPlanning::providedPorts()
{
    return {BT::InputPort<std::vector<std::string>>("objects_to_pick"),
            BT::InputPort<sensor_msgs::msg::Image>("depth_image"),
            BT::InputPort<std::vector<sensor_msgs::msg::Image>>("masks"),
            BT::OutputPort<std::vector<geometry_msgs::msg::Pose>>("grasp_pose"),
            BT::OutputPort<std::vector<std_msgs::msg::Int32>>("cylinder_ids"),
            BT::OutputPort<std::vector<geometry_msgs::msg::Pose>>("place_pose")}; // ::Response
            
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void GraspPlanning::on_send(std::shared_ptr<GraspPlanningSrv::Request> request)
{
  
    request->objects_to_pick = ports.get_value<std::vector<std::string>>("objects_to_pick");
    request->depth_image = ports.get_value<sensor_msgs::msg::Image>("depth_image");
    request->masks = ports.get_value<std::vector<sensor_msgs::msg::Image>>("masks");

     std::string objects_to_pick_str = std::accumulate(request->objects_to_pick.begin(), request->objects_to_pick.end(), std::string(),
        [](const std::string& a, const std::string& b) -> std::string {
            return a + (a.length() > 0 ? ", " : "") + b;
        });

    log("Sending Grasp Planning Request for Objects: " + objects_to_pick_str + " to Grasp Planning Service");
    // log("Sending Pack Planning Request for Objects: " + request->objects_to_pick + " to Pack Planning Service");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool GraspPlanning::on_result(std::shared_ptr<GraspPlanningSrv::Response> response, std::shared_ptr<GraspPlanningSrv::Request> request)
{
    ports.set_value<std::vector<geometry_msgs::msg::Pose>>("grasp_pose", response->grasp_pose);
    ports.set_value<std::vector<std_msgs::msg::Int32>>("cylinder_ids", response->cylinder_ids);
    ports.set_value<std::vector<geometry_msgs::msg::Pose>>("place_pose", response->place_pose);

    log("Received a Grasping planning containing " + std::to_string(request->objects_to_pick.size()) + " packages");
    
    return true;
}