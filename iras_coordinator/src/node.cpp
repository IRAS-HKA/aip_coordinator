/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Coordinator"
 * Purpose : main-function and start of the behavior tree
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#include <behaviortree_cpp_v3/bt_factory.h>
#include <iras_behaviortree_ros2/components/RosInterface.h>

// Conditions
#include <iras_coordinator/conditions/CheckStop.h>
#include <iras_coordinator/conditions/CheckBattery.h>
#include <iras_coordinator/conditions/CheckDiagnosticStatus.h>
#include <iras_coordinator/conditions/CheckBlackboard.h>
#include <iras_coordinator/conditions/CheckBlackboardInt.h>

// User Input
#include <iras_coordinator/actions/ParameterRequest.h>

// Arm
#include <iras_coordinator/actions/ArmMovementMoveIt.h>

// Navigation
#include <iras_coordinator/actions/BaseMovementNav2.h>
#include <iras_coordinator/actions/ClearGlobalCostmap.h>
#include <iras_coordinator/actions/ClearLocalCostmap.h>
#include <iras_coordinator/actions/SetInitialPose.h>

// Camera
#include <iras_coordinator/actions/CalculateOffsets.h>
#include <iras_coordinator/actions/GetMarkerPosition.h>

// Misc
#include <iras_coordinator/actions/Wait.h>

BT::Tree create_tree(const std::string &path)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFailureNode");

    // Conditions
    factory.registerNodeType<CheckStop>("CheckStop");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<CheckDiagnosticStatus>("CheckDiagnosticStatus");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");
    factory.registerNodeType<CheckBlackboardInt>("CheckBlackboardInt");

    // User Input
    factory.registerNodeType<ParameterRequest<bool>>("BoolParameterRequest");
    factory.registerNodeType<ParameterRequest<int>>("IntParameterRequest");
    factory.registerNodeType<ParameterRequest<float>>("FloatParameterRequest");
    factory.registerNodeType<ParameterRequest<std::string>>("StringParameterRequest");

    // Arm
    factory.registerNodeType<ArmMovementMoveIt>("ArmMovementMoveIt");

    // Navigation
    factory.registerNodeType<BaseMovementNav2>("BaseMovementNav2");
    factory.registerNodeType<ClearGlobalCostmap>("ClearGlobalCostmap");
    factory.registerNodeType<ClearLocalCostmap>("ClearLocalCostmap");
    factory.registerNodeType<SetInitialPose>("SetInitialPose");

    // Camera
    factory.registerNodeType<CalculateOffsets>("CalculateOffsets");
    factory.registerNodeType<GetMarkerPosition>("GetMarkerPosition");

    // Misc
    factory.registerNodeType<Wait>("Wait");

    return factory.createTreeFromFile(path);
}

BT::NodeStatus run_tree(BT::Tree &tree)
{
    BT::NodeStatus status;
    BT::NodeStatus last_status;

    do
    {
        RosInterface::spin_some();

        status = tree.tickRoot();

        if (status != last_status)
        {
            switch (status)
            {
            case BT::NodeStatus::IDLE:
                std::cout << "[MainTree] Status: IDLE" << std::endl;
                break;
            case BT::NodeStatus::RUNNING:
                std::cout << "[MainTree] Status: RUNNING" << std::endl;
                break;
            case BT::NodeStatus::SUCCESS:
                std::cout << "[MainTree] Status: SUCCESS" << std::endl;
                break;
            case BT::NodeStatus::FAILURE:
                std::cout << "[MainTree] Status: FAILURE" << std::endl;
                break;
            }

            last_status = status;
        }

    } while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING);

    return status;
}

int main(int argc, char **argv)
{
    RosInterface::init("Coordinator", argc, argv);

    RosInterface::get_node_handle()->declare_parameter("main_tree_path", rclcpp::PARAMETER_STRING);

    std::string main_tree_path;

    try
    {
        main_tree_path = RosInterface::get_node_handle()->get_parameter("main_tree_path").as_string();
        std::cout << main_tree_path << std::endl;
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException &e)
    {
        std::cerr << e.what() << " not declared\n";
    }

    BT::Tree tree = create_tree(main_tree_path);

    run_tree(tree);

    RosInterface::shutdown();

    return 0;
}
