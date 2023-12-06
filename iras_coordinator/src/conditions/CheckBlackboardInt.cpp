#include <iras_coordinator/conditions/CheckBlackboardInt.h>

BT::NodeStatus CheckBlackboardInt::on_check()
{
    BT::Optional<int> input = getInput<int>("input");
    BT::Optional<int> compare_input = getInput<int>("compare_to");

    if (!input.has_value())
    {
        throw BT::RuntimeError("missing required input [input]: ", input.error());
    }
    if (!compare_input.has_value())
    {
        throw BT::RuntimeError("missing required input [compare_to]: ", compare_input.error());
    }

    if (getInput<int>("input").value() == getInput<int>("compare_to").value())
    {
        log("values matched");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}