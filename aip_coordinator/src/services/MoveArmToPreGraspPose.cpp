#include <aip_coordinator/services/MoveArmToPreGraspPose.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string MoveArmToPreGraspPose::ros2_service_name()
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
BT::PortsList MoveArmToPreGraspPose::providedPorts()
{
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("grasp_poses"),
        BT::InputPort<int>("object_no"),
        BT::InputPort<bool>("cartesian")
        };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void MoveArmToPreGraspPose::on_send(std::shared_ptr<MoveArmToPreGraspPoseSrv::Request> request)
{
    // Get First Pose from Vector of input port grasp_pose
    int object_no;
    object_no = ports.get_value<int>("object_no");
    
    geometry_msgs::msg::Pose grasp_pose; 
    
    grasp_pose = ports.get_value<std::vector<geometry_msgs::msg::Pose>>("grasp_poses")[object_no];

    request->pose.position.x = grasp_pose.position.x;
    request->pose.position.y = grasp_pose.position.y;
    request->pose.position.z = grasp_pose.position.z;
    request->pose.orientation.x = 0;
    request->pose.orientation.y = 0;
    request->pose.orientation.z = 0;
    request->pose.orientation.w = 1;
    
    log("Requesting MoveArmToPreGraspPose for object_no" + std::to_string(object_no));
    log("Move arm to grasp pose (" + Converter::ftos(request->pose.position.x) + ", " + Converter::ftos(request->pose.position.y) + ", " + Converter::ftos(request->pose.position.z) + ")");
    log("Orientation (" + Converter::ftos(request->pose.orientation.x) + ", " + Converter::ftos(request->pose.orientation.y) + ", " + Converter::ftos(request->pose.orientation.z) + ", " + Converter::ftos(request->pose.orientation.w) + ")");

    request->cart = ports.get_value<bool>("cartesian");

    log("Cartesian: " + std::to_string(request->cart));

}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool MoveArmToPreGraspPose::on_result(std::shared_ptr<MoveArmToPreGraspPoseSrv::Response>, std::shared_ptr<MoveArmToPreGraspPoseSrv::Request>)
{
    log("MoveArmToPreGraspPoseSrv completed");
    return true;
}