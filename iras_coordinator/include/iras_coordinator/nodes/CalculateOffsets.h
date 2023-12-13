/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : CalculateOffsets in seconds
 *
 * @author Andreas Zachariae
 * @author Frederik Plahl
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <iras_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <memory>
#include <time.h>

class CalculateOffsets : public RosNode
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("offset_x"),
                                                   BT::InputPort<float>("offset_y"),
                                                   BT::InputPort<float>("offset_z"),
                                                   BT::OutputPort<float>("out_x"),
                                                   BT::OutputPort<float>("out_y"),
                                                   BT::OutputPort<float>("out_z"),
                                                   BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y"),
                                                   BT::InputPort<float>("z"),
                                                   BT::InputPort<std::string>("base_frame"),
                                                   BT::InputPort<std::string>("marker_frame"),
                                                   BT::InputPort<int>("max_seconds")}; }

    CalculateOffsets(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override {}

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
    time_t start_time_;
    time_t duration_;
    time_t last_checked_;
    std::string base_frame_;
};