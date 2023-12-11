/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Wrapper for a node in the Behavior Tree framework
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.04.14)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <cmath>

#include <behaviortree_cpp_v3/action_node.h>

#include <iras_behaviortree_ros2/tools/Progress.h>
#include <iras_behaviortree_ros2/tools/Converter.h>
#include <iras_behaviortree_ros2/components/RosInterface.h>
// #include <iras_behaviortree_ros2/tools/PortHandler.h>

class RosNode : public RosInterface, public BT::StatefulActionNode
{
public:
    RosNode(const std::string &name, const BT::NodeConfiguration &config) : RosInterface(name), BT::StatefulActionNode(name, config) {}

protected:
    static BT::PortsList bt_port_list_;
    // static PortHandler port_handler_;
    Progress progress_;

    virtual BT::NodeStatus on_start() = 0;
    virtual BT::NodeStatus on_running() = 0;
    virtual void on_halted() {}

    template <typename T = void>
    static void add_port(std::string name, std::string direction)
    {
        if (direction == "input")
        {
            bt_port_list_.insert(BT::InputPort<T>(name));
        }
        else if (direction == "output")
        {
            bt_port_list_.insert(BT::OutputPort<T>(name));
        }
        else if (direction == "bidirectional")
        {
            bt_port_list_.insert(BT::BidirectionalPort<T>(name));
        }
        else
        {
            std::cout << "ERROR: Invalid port direction: " << direction << std::endl;
        }

        // xml_port_list_.insert(std::make_pair(name, (Port(name, direction))));
        std::cout << "Added port: " << name << std::endl;
    }

    template <typename T>
    auto get_port_value(std::string name) const
    {
        BT::Optional<T> input = getInput<T>(name);
        if (!input.has_value())
        {
            throw BT::RuntimeError("missing required input [" + name + "]: ", input.error());
        }
        return input.value();
    }

    template <typename T>
    bool port_has_value(std::string name)
    {
        BT::Optional<T> input = getInput<T>(name);
        return input.has_value();
    }

private:
    virtual BT::NodeStatus onStart() override
    {
        progress_.current_step = 0;

        check_required_nodes();

        return on_start();
    }

    virtual BT::NodeStatus onRunning() override { return on_running(); }

    virtual void onHalted() override { on_halted(); }
};
