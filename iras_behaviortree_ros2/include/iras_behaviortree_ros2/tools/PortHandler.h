/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Abstraction for handling ports and their values
 *
 * @author Andreas Zachariae
 * @since 2.0.0 (2023.12.13)
 *********************************************************/
#pragma once

#include <cpp_core/default.h>

#include <behaviortree_cpp_v3/tree_node.h>
#include <iras_behaviortree_ros2/tools/XmlGenerator.h>

class StaticPortHandler
{
public:
    static BT::PortsList bt_port_list_;
    static std::unordered_map<std::string, XmlPort> xml_port_list_;

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

        xml_port_list_.insert(std::make_pair(name, (XmlPort(name, direction))));
        std::cout << "Added port: " << name << std::endl;
    }

    static void print_port_list();

    static void print_number_of_ports();

    static void create_ports();
};

class PortHandler
{
public:
    PortHandler(BT::TreeNode *node) : bt_node_handle_(node) {}

    // XmlGenerator xml_generator_;

    // print all keys of the blackboard
    void print_keys() const
    {
        const BT::NodeConfiguration &config = bt_node_handle_->config();
        std::vector<BT::StringView> keys = config.blackboard->getKeys();
        std::cout << "Keys: " << keys.size() << std::endl;
        for (auto key : keys)
        {
            std::cout << BT::convertFromString<std::string>(key) << std::endl;
        }
    }

    template <typename T>
    auto get_port_value(std::string name) const
    {
        BT::Optional<T> input = bt_node_handle_->getInput<T>(name);
        if (!input.has_value())
        {
            throw BT::RuntimeError("missing required input [" + name + "]: ", input.error());
        }
        return input.value();
    }

    template <typename T>
    bool port_has_value(std::string name)
    {
        BT::Optional<T> input = bt_node_handle_->getInput<T>(name);
        return input.has_value();
    }

    template <typename T>
    void set_port_value(std::string name, T value)
    {
        bt_node_handle_->setOutput<T>(name, value);
    }

private:
    BT::TreeNode *bt_node_handle_;
};
