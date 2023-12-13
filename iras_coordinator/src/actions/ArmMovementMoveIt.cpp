#include <iras_coordinator/actions/ArmMovementMoveIt.h>

#define DEG2RAD(x) ((x) * 3.1415 / 180.0f)

std::string ArmMovementMoveIt::ros2_action_name()
{
    return "move_to_pose";
}

BT::PortsList ArmMovementMoveIt::providedPorts()
{
    // Arguments: <data_type>(name, direction[input, output, bidirectional])
    add_port<float>("x", "input");
    add_port<float>("y", "input");
    add_port<float>("z", "input");
    add_port<float>("rotation_x", "input");
    add_port<float>("rotation_y", "input");
    add_port<float>("rotation_z", "input");
    add_port<bool>("cart", "input"); // TODO: rename cartesian
    add_port<float>("speed", "input");
    return bt_port_list_;
}

void ArmMovementMoveIt::on_send(MoveArmMoveIt::Goal &goal)
{
    // TODO done
    // Get inputs
    BT::Optional<float> x_input = getInput<float>("x");
    BT::Optional<float> y_input = getInput<float>("y");
    BT::Optional<float> z_input = getInput<float>("z");
    BT::Optional<float> rx_input = getInput<float>("rotation_x");
    BT::Optional<float> ry_input = getInput<float>("rotation_y");
    BT::Optional<float> rz_input = getInput<float>("rotation_z");
    BT::Optional<bool> cart_input = getInput<bool>("cart");
    BT::Optional<float> speed_input = getInput<float>("speed");

    // TODO
    // Werte abfragen

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
    if (!rx_input.has_value())
    {
        throw BT::RuntimeError("missing required input [rx_input]: ", rx_input.error());
    }
    if (!ry_input.has_value())
    {
        throw BT::RuntimeError("missing required input [ry_input]: ", ry_input.error());
    }
    if (!rz_input.has_value())
    {
        throw BT::RuntimeError("missing required input [rz_input]: ", rz_input.error());
    }
    if (!cart_input.has_value())
    {
        throw BT::RuntimeError("missing required input [cart_input]: ", cart_input.error());
    }
    if (!speed_input.has_value())
    {
        throw BT::RuntimeError("missing required input [speed_input]: ", speed_input.error());
    }

    // TODO
    // werte belegen

    goal.pose.pose.position.x = x_input.value();
    goal.pose.pose.position.y = y_input.value();
    goal.pose.pose.position.z = z_input.value();

    tf2::Quaternion q;
    q.setRPY(DEG2RAD(rx_input.value()), DEG2RAD(ry_input.value()), DEG2RAD(rz_input.value()));
    q.normalize();
    goal.pose.pose.orientation = tf2::toMsg(q);

    goal.cart = cart_input.value();
    goal.speed = speed_input.value();

    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ", z=" + Converter::ftos(goal.pose.pose.position.z) + ")\n(rx=" + Converter::ftos(rx_input.value()) + ", ry=" + Converter::ftos(ry_input.value()) + ", rz=" + Converter::ftos(rz_input.value()) + ")");
}

void ArmMovementMoveIt::on_feedback(const std::shared_ptr<const MoveArmMoveIt::Feedback> feedback)
{
    log("Status: " + feedback->status_code);
}

void ArmMovementMoveIt::on_result(const rclcpp_action::ClientGoalHandle<MoveArmMoveIt>::WrappedResult &result, const MoveArmMoveIt::Goal &goal)
{
    log("Goal reached! (x=" + Converter::ftos(goal.pose.pose.position.x) +
        ", y=" + Converter::ftos(goal.pose.pose.position.y) +
        ", z=" + Converter::ftos(goal.pose.pose.position.z) +
        "), Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - -goal.pose.header.stamp.sec)) + "s" +
        ", code: " + std::to_string(result.result->err_code) + " " + result.result->msg);
}