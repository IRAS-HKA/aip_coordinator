/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Calculates MarkerPosition from tf transformations to apriltag no. 4
 *
 * @author Andreas Zachariae
 * @author Frederik Plahl
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <iras_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>
#include <iras_behaviortree_ros2/tools/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <time.h>
#include <thread>

// TODO: rename to GetFramePose
// TODO: generalize marker to frame

class GetMarkerPosition : public RosNode
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<int>("marker_id"),
                                                   BT::InputPort<int>("max_seconds"),
                                                   BT::OutputPort<float>("x"),
                                                   BT::OutputPort<float>("y"),
                                                   BT::OutputPort<float>("z"),
                                                   BT::OutputPort<float>("rotation_x"),
                                                   BT::OutputPort<float>("rotation_y"),
                                                   BT::OutputPort<float>("rotation_z")}; }

    GetMarkerPosition(const std::string &name, const BT::NodeConfiguration &config);

    std::string ros_name() { return "GetMarkerPosition"; }

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override {}

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    time_t start_time_;
    time_t duration_;
    time_t last_checked_;
    std::string marker_frame_;
    std::vector<geometry_msgs::msg::TransformStamped> marker_mean_;
    int num_markers_ = 0;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};