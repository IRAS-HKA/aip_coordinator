#include <iras_coordinator/actions/SetInitialPose.h>

SetInitialPose::SetInitialPose(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    pose_publisher_ = get_node_handle()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
}

BT::NodeStatus SetInitialPose::on_start()
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<float> x_input = getInput<float>("x");
    BT::Optional<float> y_input = getInput<float>("y");
    BT::Optional<float> z_input = getInput<float>("z");
    BT::Optional<float> qx_input = getInput<float>("quaternion_x");
    BT::Optional<float> qy_input = getInput<float>("quaternion_y");
    BT::Optional<float> qz_input = getInput<float>("quaternion_z");
    BT::Optional<float> qw_input = getInput<float>("quaternion_w");

    if (!x_input.has_value())
    {
        throw BT::RuntimeError("missing required input [x]: ", x_input.error());
    }
    if (!y_input.has_value())
    {
        throw BT::RuntimeError("missing required input [y]: ", y_input.error());
    }
    if (!z_input.has_value())
    {
        throw BT::RuntimeError("missing required input [z]: ", z_input.error());
    }
    if (!qx_input.has_value())
    {
        throw BT::RuntimeError("missing required input [qx_input]: ", qx_input.error());
    }
    if (!qy_input.has_value())
    {
        throw BT::RuntimeError("missing required input [qy_input]: ", qy_input.error());
    }
    if (!qz_input.has_value())
    {
        throw BT::RuntimeError("missing required input [qz_input]: ", qz_input.error());
    }
    if (!qw_input.has_value())
    {
        throw BT::RuntimeError("missing required input [qw_input]: ", qw_input.error());
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.pose.pose.position.x = x_input.value();
    msg.pose.pose.position.y = y_input.value();
    msg.pose.pose.position.z = z_input.value();
    msg.pose.pose.orientation.x = qx_input.value();
    msg.pose.pose.orientation.y = qy_input.value();
    msg.pose.pose.orientation.z = qz_input.value();
    msg.pose.pose.orientation.w = qw_input.value();

    pose_publisher_->publish(msg);

    log("Goal: (x=" + convert::ftos(msg.pose.pose.position.x) + ", y=" + convert::ftos(msg.pose.pose.position.y) + ", z=" + convert::ftos(msg.pose.pose.position.z) + ")");

    return BT::NodeStatus::SUCCESS;
}