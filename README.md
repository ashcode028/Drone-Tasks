# ROS
This repo was made while doing a project in a hackathon e-yantra .
## Requirements:
- Ubuntu 18.04
- ROS Melodic

## Task 1 : To move the turtle inside the turtlesim window in a circle and stop at its initial location.
### Procedure:
1. First, create a package named pkg_task0, within your catkin workspace. Once done, compile and source the packages.
```
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
2. 

### Expected Output:
![](https://github.com/ashcode028/ROS/blob/eba454c354ebef48c1b4cf8dfc1c59f0f75cb10d/Move_turtle/images/task1_output.gif)
To know whether the nodes are talking to each other as expected one can use the command rqt_graph. Below, you can find an expected graph for this task.
![](https://github.com/ashcode028/ROS/blob/eba454c354ebef48c1b4cf8dfc1c59f0f75cb10d/Move_turtle/images/task0_rqt_graph.png)
 ## Task 2A: Design a Attitude Controller
