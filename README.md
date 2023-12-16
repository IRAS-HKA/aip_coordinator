# IRAS Coordinator
This package uses the IRAS/Common/BehaviorTree.ROS package which is a wrapper around the [BehaviorTree.cpp](https://www.behaviortree.dev/) package.

The IRAS Coordinator defines the behavior trees with single BT actions.

The set of actions can be arranged freely with the graphical user interface [Groot](https://github.com/BehaviorTree/Groot).

![example_tree](https://d33wubrfki0l68.cloudfront.net/f8b2bac65168251a46ec25232f20db7961327ffc/88ad1/images/readthedocs.png)

![groot](https://github.com/BehaviorTree/Groot/raw/master/groot-screenshot.png)

## How to start

Use the docker container

    . start_docker.sh

Inside the container launch the Coordinator node with parameters

    ros2 launch iras_coordinator test.launch.py
    
To view or modify the behavior trees, attach a new shell

    docker exec -it coordinator bash
    ros2 run groot Groot


## TODO
- [x] Groot Palette for new navigation actions clear costmap initialpose
- [ ] Integrate LifecycleManager to Coordinater
- [ ] Integrate cpp_core
- [x] Better name ros_name() Make function for each type action, service, etc
- [x] Add Groot publisher integration
- [ ] Add RosPublisher and RosSubscriber
- [x] finish PortHandler and rename XMLDefinition
- [ ] Add general doc with purpose of this repo
- [ ] Add instructions how to add new nodes
- [ ] Finish refactoring of lagacy nodes with new style
- [x] Add cpp debugger for ros nodes and launch files (ROS extension has to be installed)

## Port from FOXY to HUMBLE:
- declare_parameter needs a type argument `this->declare_parameter("my_str", rclcpp::PARAMETER_STRING);`
