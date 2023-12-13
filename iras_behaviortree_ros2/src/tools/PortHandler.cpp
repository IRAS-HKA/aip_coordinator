#include <iras_behaviortree_ros2/tools/PortHandler.h>

BT::PortsList StaticPortHandler::bt_port_list_ = {};
std::unordered_map<std::string, XmlPort> StaticPortHandler::xml_port_list_ = {};

void StaticPortHandler::print_port_list()
{
    std::cout << "Ports: " << std::endl;
    for (const auto &port : xml_port_list_)
    {
        std::cout << "  " << port.first << " (" << port.second.direction << ")" << std::endl;
    }
}

void StaticPortHandler::print_number_of_ports()
{
    std::cout << "Number of ports: " << xml_port_list_.size() << " / " << bt_port_list_.size() << std::endl;
}

void StaticPortHandler::create_ports()
{
    //     BT::PortsList ArmMovementMoveIt::providedPorts()
    // {
    //     // Arguments: <data_type>(name, direction[input, output, bidirectional])
    //     add_port<float>("x", "input");
    //     add_port<float>("y", "input");
    //     add_port<float>("z", "input");
    //     add_port<float>("rotation_x", "input");
    //     add_port<float>("rotation_y", "input");
    //     add_port<float>("rotation_z", "input");
    //     add_port<bool>("cart", "input"); // TODO: rename cartesian
    //     add_port<float>("speed", "input");
    //     return bt_port_list_;
    // }
}