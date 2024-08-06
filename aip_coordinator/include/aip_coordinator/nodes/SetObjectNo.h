/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Set ObjectNo to use in flexible BT for Automated Item Picking
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <aip_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>


class SetObjectNo : public RosNode
{
public:
    static BT::PortsList providedPorts();

    SetObjectNo(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config) {}

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override;

private:

};