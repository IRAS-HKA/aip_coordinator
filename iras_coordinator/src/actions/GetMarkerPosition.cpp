#include <iras_coordinator/actions/GetMarkerPosition.h>

#define RAD2DEG(x) ((x) * 180.0f / 3.1415)
#define DEG2RAD(x) ((x) * 3.1415 / 180.0f)
#define OFFSET_Y -0.035

using namespace std::chrono_literals;

GetMarkerPosition::GetMarkerPosition(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node_handle()->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus GetMarkerPosition::on_start()
{
    BT::Optional<int> marker_id = getInput<int>("marker_id");
    BT::Optional<int> max_seconds = getInput<int>("max_seconds");

    if (!marker_id.has_value())
    {
        throw BT::RuntimeError("missing required input [marker_id]: ", marker_id.error());
    }
    if (!max_seconds.has_value())
    {
        throw BT::RuntimeError("missing required input [max_seconds]: ", max_seconds.error());
    }

    duration_ = max_seconds.value();
    start_time_ = time(NULL);
    last_checked_ = time(NULL);
    marker_frame_ = "marker_" + std::to_string(marker_id.value());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetMarkerPosition::on_running()
{
    if ((time(NULL) - start_time_) < duration_)
    {
        if ((time(NULL) - last_checked_) > 1)
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            last_checked_ = time(NULL);

            try
            {
                // tf_buffer_->canTransform("base_link", marker_frame_);
                transformStamped = tf_buffer_->lookupTransform(
                    "base_link", marker_frame_, // get_node_handle()->get_clock()->now(), 500ms);
                    tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    get_node_handle()->get_logger(), "Could not transform: %s", ex.what());

                // log("Could not transform!" + ex.what());
                return BT::NodeStatus::RUNNING;
            }

            double x = 0, y = 0, z = 0, roll = 0, mean_roll = 0, pitch = 0, mean_pitch = 0, yaw = 0, mean_yaw = 0;

            if (marker_mean_.size() != 2)
            {
                marker_mean_.push_back(transformStamped);

                tf2::Quaternion q(transformStamped.transform.rotation.x,
                                  transformStamped.transform.rotation.y,
                                  transformStamped.transform.rotation.z,
                                  transformStamped.transform.rotation.w);

                tf2::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                roll = RAD2DEG(roll);
                pitch = RAD2DEG(pitch);
                yaw = RAD2DEG(yaw);

                marker_mean_.back().transform.rotation.x = roll;
                marker_mean_.back().transform.rotation.y = pitch;
                marker_mean_.back().transform.rotation.z = yaw;
                log(std::to_string(transformStamped.transform.translation.x));

                log("Marker Pose " + std::to_string(marker_mean_.size()));
                log("MarkerPosition: (x=" + Converter::ftos(transformStamped.transform.translation.x) + ", y=" + Converter::ftos(transformStamped.transform.translation.y) + ", z=" + Converter::ftos(transformStamped.transform.translation.z) + ")");
                log("MarkerPosition: (roll=" + Converter::ftos(roll) + ", pitch=" + Converter::ftos(pitch) + ", yaw=" + Converter::ftos(yaw) + ")");

                return BT::NodeStatus::RUNNING;
            }

            for (auto &&marker : marker_mean_)
            {
                x += marker.transform.translation.x;
                y += marker.transform.translation.y;
                z += marker.transform.translation.z;

                mean_roll += marker.transform.rotation.x;
                mean_pitch += marker.transform.rotation.y;
                mean_yaw += marker.transform.rotation.z;
                // log(std::to_string(mean_roll));
            }
            // log(std::to_string(marker_mean_.size()));

            log(std::to_string(x));
            log(std::to_string(marker_mean_.size()));

            x = x / marker_mean_.size();
            y = y / marker_mean_.size();
            z = z / marker_mean_.size();
            mean_roll = mean_roll / marker_mean_.size();
            mean_pitch = (mean_pitch / marker_mean_.size()); //- 90.0;
            mean_yaw = (mean_yaw / marker_mean_.size());     //+ 180.0;

            setOutput<float>("x", x);
            setOutput<float>("y", y);
            setOutput<float>("z", z);
            setOutput<float>("rotation_x", mean_roll);
            setOutput<float>("rotation_y", mean_pitch);
            setOutput<float>("rotation_z", mean_yaw);

            log("Mean Marker Pose");
            log("MarkerPosition: (x=" + Converter::ftos(x) + ", y=" + Converter::ftos(y) + ", z=" + Converter::ftos(z) + ")");
            log("MarkerPosition: (roll=" + Converter::ftos(mean_roll) + ", pitch=" + Converter::ftos(mean_pitch) + ", yaw=" + Converter::ftos(mean_yaw) + ")");

            tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node_handle());
            rclcpp::Time now = get_node_handle()->get_clock()->now();
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = now;
            t.header.frame_id = "base_link";
            t.child_frame_id = "mean_marker";

            t.transform.translation.x = x;
            t.transform.translation.y = y + OFFSET_Y;
            t.transform.translation.z = z;
            tf2::Quaternion q;
            q.setRPY(
                DEG2RAD(mean_roll),
                DEG2RAD(mean_pitch),
                DEG2RAD(mean_yaw));
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_publisher_->sendTransform(t);

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    return BT::NodeStatus::FAILURE;
}