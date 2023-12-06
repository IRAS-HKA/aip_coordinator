/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Wait in seconds
 *
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <iras_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>
#include <iras_behaviortree_ros2/tools/convert.h>

#include <time.h>

class Wait : public RosNode
{
public:
    static BT::PortsList providedPorts() { return {
        BT::InputPort<int>("seconds"),
    }; }

    Wait(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config) {}

    std::string ros_name() { return "Wait"; }

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override;

private:
    time_t start_time_;
    time_t duration_;
};