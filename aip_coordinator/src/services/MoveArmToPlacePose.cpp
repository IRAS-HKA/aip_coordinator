#include <aip_coordinator/services/MoveArmToPlacePose.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string MoveArmToPlacePose::ros2_service_name()
{
    return "/move_to_pose";
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
BT::PortsList MoveArmToPlacePose::providedPorts()
{
    return {BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("place_poses"),
            BT::InputPort<int>("object_no"),
            BT::InputPort<bool>("cartesian")};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void MoveArmToPlacePose::on_send(std::shared_ptr<MoveArmToPlacePoseSrv::Request> request)
{
    // Get First Pose from Vector of input port grasp_pose
    int object_no;
    object_no = ports.get_value<int>("object_no");
    
    geometry_msgs::msg::Pose place_pose; 
    
    place_pose = ports.get_value<std::vector<geometry_msgs::msg::Pose>>("place_poses")[object_no];

    request->pose.position.x = place_pose.position.x;
    request->pose.position.y = place_pose.position.y;
    request->pose.position.z = place_pose.position.z;
    request->pose.orientation.x = place_pose.orientation.x;
    request->pose.orientation.y = place_pose.orientation.y;
    request->pose.orientation.z = place_pose.orientation.z;
    request->pose.orientation.w = place_pose.orientation.w;

    log("Requesting MoveArmToPlacePose for object_no" + std::to_string(object_no));  
    log("Move arm to place pose (" + Converter::ftos(request->pose.position.x) + ", " + Converter::ftos(request->pose.position.y) + ", " + Converter::ftos(request->pose.position.z) + ")");
    log("Orientation (" + Converter::ftos(request->pose.orientation.x) + ", " + Converter::ftos(request->pose.orientation.y) + ", " + Converter::ftos(request->pose.orientation.z) + ", " + Converter::ftos(request->pose.orientation.w) + ")");

    request->cart = ports.get_value<bool>("cartesian");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool MoveArmToPlacePose::on_result(std::shared_ptr<MoveArmToPlacePoseSrv::Response>, std::shared_ptr<MoveArmToPlacePoseSrv::Request>)
{
    log("MoveArmToPlacePoseSrv completed");
    return true;
}