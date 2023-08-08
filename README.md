# Bumpybot Base Controller

This repository contains our implementation of a 
[ros-controller](https://github.com/ros-controls/ros_controllers) 
for the omniwheel robot Bumpybot. It takes as inputs a _twist_ in the form of the 
conventional `cmd_vel` and computes the torques at each wheel to achieve the
desired motion. Currently, this approach
uses only kinematics of the robot and a simple proportional controller.

This also includes an RViz plugin to do simple online tuning of the controller. 

# How to use

If running as a stand-alone package, clone into your catkin workspace source directory, e.g.,

```
~ /catkin_ws/src $ git clone https://github.com/carlosiglezb/trikey_base_controller.git
```

and change the `config/base_controller.yaml` according to your robot's parameters and link names. 


### Note
`CMake/FindEigen3` hints at the eigen3 location already but may not work on your installation. If it doesn't work check the readme of Trikey Base Controller for instructions.
