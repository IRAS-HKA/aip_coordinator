# IRAS Coordinator
This package uses the BehaviorTree.IRAS package which is a wrapper around the [BehaviorTree.cpp v3.8](https://www.behaviortree.dev/) package.

The IRAS Coordinator defines the behavior trees with single BT actions.

The set of actions can be arranged freely with the graphical user interface [Groot](https://github.com/BehaviorTree/Groot).

![groot](https://github.com/BehaviorTree/Groot/raw/master/groot-screenshot.png)

## How to start

Build and start the docker container

    source build_docker.sh
    source start_docker.sh

Inside the container launch the Coordinator node with parameters

    ros2 launch iras_coordinator test.launch.py
    
To view or modify the behavior trees, attach a new shell and start Groot

    docker exec -it coordinator bash
    ros2 run groot Groot

## How to start with C++ debugging

Install VSCode extension:
- Remote Development
- ROS
- C/C++

Mount settings folder .vscode to target directory for development
```bash
# Add parameter to docker run command
-v $PWD/.vscode:/home/docker/ros2_ws/src/.vscode
```

```bash
source start_docker.sh
```

1. Attach to running docker container with VSCode remote extension
2. Open remote folder where .vscode is mounted to
3. Install `ROS` and `C/C++` extension in container
4. Use command palette (strg + shift + p) and `Tasks: Run Task` and `Build`
5. Use VSCode debugger and stop points for debugging

## How to design a new Behavior Tree

## How to create a new custom node

There are currently 4 different types of nodes supported:
- **actions** Use for ROS2 action client
- **services** Use for ROS2 service client
- **conditions** Use for classic BT condition with ROS2 node handle access
- **nodes** Use for classic BT action with ROS2 node handle access

This instructions gives an example for a ROS2 action client

1. Add new header file in the corresponding folder at `iras_coordinator/include/iras_coordinator/actions`. For this, copy an existing file from that folder and rename. Use this structure as template. Copy `MoveBase.h` and rename to `MyCustomActionNode.h`.
2. Do the same for the soruce file in `iras_coordinator/src/actions`. Copy `MoveBase.cpp` and rename to `MyCustomActionNode.cpp`.
3. In your header file `MyCustomActionNode.h` include the header files of your ROS2 interface you want to use. Change
```cpp
~~#include <iras_interfaces/action/move_arm_move_it.hpp>~~
```
5. Change the class name to the same name as the file
```cpp
class MoveArm : public RosAction<MoveArmMoveIt>
```


## TODO
- [x] Groot Palette for new navigation actions clear costmap initialpose
- [ ] (Integrate LifecycleManager to Coordinater)
- [ ] (Integrate cpp_core)
- [x] Better name ros_name() Make function for each type action, service, etc
- [x] Add Groot publisher integration
- [ ] Add RosPublisher and RosSubscriber
- [x] finish PortHandler and rename XMLDefinition
- [x] Add general doc with purpose of this repo
- [ ] Add instructions how to add new nodes
- [x] Finish refactoring of lagacy nodes with new style
- [x] Add cpp debugger for ros nodes and launch files (ROS extension has to be installed)

## Port from FOXY to HUMBLE:
- declare_parameter needs a type argument `this->declare_parameter("my_str", rclcpp::PARAMETER_STRING);`
