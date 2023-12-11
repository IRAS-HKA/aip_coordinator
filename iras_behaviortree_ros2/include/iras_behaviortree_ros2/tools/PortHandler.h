#include <tinyxml2.h>
#include <tuple>

#include <cpp_core/default.h>
#include <behaviortree_cpp_v3/tree_node.h>

class Port
{
public:
    Port(std::string name, std::string direction) : name(name), direction(direction) {}

    std::string getName() const { return name; }
    std::string getDirection() const { return direction; }

private:
    std::string name;
    std::string direction;
};

class PortHandler
{
public:
    static std::unordered_map<std::string, Port> xml_port_list_;
    std::string node_xml_id;
    std::string node_type;

    template <typename T>
    void addPortList(const std::string &nodeID, const std::vector<Port> &ports)
    {
        for (const auto &port : ports)
        {
            this->ports.push_back({nodeID, port});
        }
    }

    void print_port_list()
    {
        std::cout << "Node: " << node_xml_id << std::endl;
        std::cout << "Type: " << node_type << std::endl;
        std::cout << "Ports: " << std::endl;
        for (const auto &port : xml_port_list_)
        {
            std::cout << "  " << port.first << " (" << port.second.direction << ")" << std::endl;
        }
    }

    void generateGrootPalette(const std::string &filename)
    {
        tinyxml2::XMLDocument doc;

        // Create an Action element for each action
        for (const auto &portPair : ports)
        {
            tinyxml2::XMLElement *actionElement = doc.NewElement("Action");
            actionElement->SetAttribute("ID", portPair.first.c_str());

            // Add the ports to the Action element
            tinyxml2::XMLElement *portElement = doc.NewElement((portPair.second.getDirection() + "_port").c_str());
            portElement->SetAttribute("name", portPair.second.getName().c_str());
            portElement->SetAttribute("type", getType());
            actionElement->InsertEndChild(portElement);

            doc.InsertEndChild(actionElement);
        }

        // Save the XML to a file
        doc.SaveFile(filename.c_str());
    }

private:
    std::vector<std::pair<std::string, Port>> ports;
};

int main()
{
    PortHandler generator;

    // Define the list of ports for each action
    std::vector<Port<float>> calculateOffsetsPorts = {
        Port<float>("offset_x", "input", 0.0f),
        Port<float>("out_x", "output", 0.0f),
    };

    std::vector<Port<int>> armMovementMoveItPorts = {
        Port<int>("x", "input", 0),
        Port<int>("speed", "output", 0),
    };

    // Add the port lists to the generator
    generator.addPortList("CalculateOffsets", calculateOffsetsPorts);
    generator.addPortList("ArmMovementMoveIt", armMovementMoveItPorts);

    // Generate the Groot palette
    generator.generateGrootPalette("output.xml");

    return 0;
}