#include <aip_coordinator/nodes/SetInitialObjectNo.h>

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
BT::PortsList SetInitialObjectNo::providedPorts()
{
    return {BT::OutputPort<int>("object_no")};
}

/**
 * @brief Define what happens when this node is ticked for the first time.
 * @return BT::NodeStatus RUNNING (Has to return RUNNING to allow on_running to be called)
 */
BT::NodeStatus SetInitialObjectNo::on_start()
{


    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is ticked in RUNNING mode.
 * @return BT::NodeStatus SUCCESS or FAILURE or RUNNING
 */
BT::NodeStatus SetInitialObjectNo::on_running()
{

    int object_no = 0;

    ports.set_value<int>("object_no", object_no);    
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Define what happens when this node is halted.
 */
void SetInitialObjectNo::on_halted()
{
}