#include <iras_coordinator/actions/Wait.h>

BT::NodeStatus Wait::on_start()
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<int> seconds = getInput<int>("seconds");

    if (!seconds.has_value())
    {
        throw BT::RuntimeError("missing required input [seconds]: ", seconds.error());
    }

    duration_ = seconds.value();
    start_time_ = time(NULL);

    log("Wait for " + std::to_string(seconds.value()) + " s...");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::on_running()
{
    if ((time(NULL) - start_time_) > duration_)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void Wait::on_halted()
{
}