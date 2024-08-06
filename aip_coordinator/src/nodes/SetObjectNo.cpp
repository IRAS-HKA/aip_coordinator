#include <aip_coordinator/nodes/SetObjectNo.h>

/**
 * @brief Set the list of ports provided by the BT node.
 *
 * New port:
 *      direction = [BT::InputPort, BT::OutputPort, BT::BidirectionalPort]
 *      data_type = <[float, int, std::string]>
 *      name = ("name")
 *
 * @return List of provided ports.
 */
BT::PortsList SetObjectNo::providedPorts()
{
    return {BT::BidirectionalPort<int>("object_no")};
}

/**
 * @brief Define what happens when this node is ticked for the first time.
 * @return BT::NodeStatus RUNNING (Has to return RUNNING to allow on_running to be called)
 */
BT::NodeStatus SetObjectNo::on_start()
{
    int object_no = ports.get_value<int>("object_no");

    object_no++;

    ports.set_value<int>("object_no", object_no);    

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is ticked in RUNNING mode.
 * @return BT::NodeStatus SUCCESS or FAILURE or RUNNING
 */
BT::NodeStatus SetObjectNo::on_running()
{
    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is halted.
 */
void SetObjectNo::on_halted()
{
}