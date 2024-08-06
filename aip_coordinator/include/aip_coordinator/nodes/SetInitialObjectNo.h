/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Set Initial Object No to initialize and reset object no in flexible BT for Automated Item Picking
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>


class SetInitialObjectNo : public RosNode
{
public:
    static BT::PortsList providedPorts();

    SetInitialObjectNo(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config) {}

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override;

private:

};