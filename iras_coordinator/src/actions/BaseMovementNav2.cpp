#include <iras_coordinator/actions/BaseMovementNav2.h>

std::string BaseMovementNav2::ros2_action_name()
{
    return "navigate_to_pose";
}

BT::PortsList BaseMovementNav2::providedPorts()
{
    // Arguments: <data_type>(name, direction[input, output, bidirectional])
    add_port<std::string>("frame_id", "input");
    add_port<float>("x", "input");
    add_port<float>("y", "input");
    add_port<float>("quaternion_x", "input");
    add_port<float>("quaternion_y", "input");
    add_port<float>("quaternion_z", "input");
    add_port<float>("quaternion_w", "input");
    return bt_port_list_;
}

void BaseMovementNav2::on_send(NavigateToPoseROS2Nav::Goal &goal)
{
    if (ports.port_has_value<float>("quaternion_x") && ports.port_has_value<float>("quaternion_y") && ports.port_has_value<float>("quaternion_z") && ports.port_has_value<float>("quaternion_w"))
    {
        goal.pose.pose.orientation.x = ports.get_port_value<float>("quaternion_x");
        goal.pose.pose.orientation.y = ports.get_port_value<float>("quaternion_y");
        goal.pose.pose.orientation.z = ports.get_port_value<float>("quaternion_z");
        goal.pose.pose.orientation.w = ports.get_port_value<float>("quaternion_w");
        log("Note: Quaternion values are set.");
    }
    else
    {
        goal.pose.pose.orientation.x = 0;
        goal.pose.pose.orientation.y = 0;
        goal.pose.pose.orientation.z = 0;
        goal.pose.pose.orientation.w = 1;
        log("Note: No quaternion values set. Using default values.");
    }

    if (ports.port_has_value<std::string>("frame_id"))
    {
        goal.pose.header.frame_id = ports.get_port_value<std::string>("frame_id");
    }
    else
    {
        goal.pose.header.frame_id = "map";
    }

    goal.pose.pose.position.x = ports.get_port_value<float>("x");
    goal.pose.pose.position.y = ports.get_port_value<float>("y");

    goal.pose.pose.position.z = 0; // z-value not neccessary
    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
}

void BaseMovementNav2::on_feedback(const std::shared_ptr<const NavigateToPoseROS2Nav::Feedback>)
{
    // log("Current position: (x=" + Converter::ftos((float)feedback->current_pose.pose.position.x) +
    //     ", y=" + Converter::ftos((float)feedback->current_pose.pose.position.y) +
    //     "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
}

void BaseMovementNav2::on_result(const rclcpp_action::ClientGoalHandle<NavigateToPoseROS2Nav>::WrappedResult &, const NavigateToPoseROS2Nav::Goal &goal)
{
    log("Goal reached! (x=" + Converter::ftos(goal.pose.pose.position.x) +
        ", y=" + Converter::ftos(goal.pose.pose.position.y) +
        "), Total time: " + std::to_string((int)get_node_handle()->now().seconds() - goal.pose.header.stamp.sec) + "s");
}