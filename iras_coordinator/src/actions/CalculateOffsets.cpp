#include <iras_coordinator/actions/CalculateOffsets.h>

CalculateOffsets::CalculateOffsets(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node_handle()->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus CalculateOffsets::on_start()
{
    BT::Optional<float> x_input = getInput<float>("x");
    BT::Optional<float> y_input = getInput<float>("y");
    BT::Optional<float> z_input = getInput<float>("z");
    BT::Optional<float> offset_x = getInput<float>("offset_x");
    BT::Optional<float> offset_y = getInput<float>("offset_y");
    BT::Optional<float> offset_z = getInput<float>("offset_z");
    BT::Optional<std::string> marker_frame = getInput<std::string>("marker_frame");
    BT::Optional<std::string> base_frame = getInput<std::string>("base_frame");
    BT::Optional<int> max_seconds = getInput<int>("max_seconds");
    base_frame_ = base_frame.value();
    duration_ = max_seconds.value();
    start_time_ = time(NULL);
    last_checked_ = time(NULL);

    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node_handle());
    rclcpp::Time now = get_node_handle()->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = marker_frame.value();
    t.child_frame_id = "offset_frame";

    t.transform.translation.x = offset_x.value();
    t.transform.translation.y = offset_y.value();
    t.transform.translation.z = offset_z.value();
    tf2::Quaternion q;
    q.setRPY(
        0.0,
        0.0,
        0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_publisher_->sendTransform(t);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CalculateOffsets::on_running()
{
    if ((time(NULL) - start_time_) < duration_)
    {
        if ((time(NULL) - last_checked_) > 1)
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            last_checked_ = time(NULL);

            try
            {
                transformStamped = tf_buffer_->lookupTransform(
                    base_frame_, "offset_frame",
                    tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    get_node_handle()->get_logger(), "Could not transform: %s", ex.what());

                return BT::NodeStatus::RUNNING;
            }
            setOutput<float>("out_x", transformStamped.transform.translation.x);
            setOutput<float>("out_y", -transformStamped.transform.translation.y);
            setOutput<float>("out_z", transformStamped.transform.translation.z);
            log("Offset Position in base_frame: (x=" + Converter::ftos(transformStamped.transform.translation.x) + ", y=" + Converter::ftos(transformStamped.transform.translation.y) + ", z=" + Converter::ftos(transformStamped.transform.translation.z) + ")");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }
    return BT::NodeStatus::FAILURE;
}
