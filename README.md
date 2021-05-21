# ROS
This repo was made while doing a project in a hackathon e-yantra .
## Requirements:
- Ubuntu 18.04
- ROS Melodic
- Gazebo world

## Task 1 : To move the turtle inside the turtlesim window in a circle and stop at its initial location.
### Procedure:
1. First, create a package named pkg_task0, within your catkin workspace. Once done, compile and source the packages.
```
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
2. Within this package, you should have a scripts folder inside which you'll create a python script, named node_turtle_revolve.py`
3. After completing the python script. Make it executable, if it isn't already. To do that, enter the following code.
```
chmod +x ~/catkin_ws/src/pkg_task0/scripts/node_turtle_revolve.py
```
4. Before executing make sure that roscore is running along with turtlesim_node. You can either run them in separate terminals or simply create a task0.launch file inside the ~/catkin_ws/src/pkg_task0/launch/ folder. Launch file can run multiple nodes unlike a python/cpp script. Run the launch file, enter,
```
roslaunch pkg_task0 task0.launch 
```
This runs three processes in parallel
1. roscore
2. turtlesim_node
3. node_turtle_revolve.py

## Observations :
1. The turtle needs to move in a circular motion with a certain radius. This radius should be sufficient to fit within the turtlesim window. But making it rotate in a circular manner, with only velocities to control is something to think about.
2. Use linear velocity as well as angular velocity with some combination to get this done.
3. Keep tracking the distance travelled so as to know when to stop.
### Expected Output:
![](https://github.com/ashcode028/ROS/blob/eba454c354ebef48c1b4cf8dfc1c59f0f75cb10d/Move_turtle/images/task1_output.gif)
To know whether the nodes are talking to each other as expected one can use the command rqt_graph. Below, you can find an expected graph for this task.
![](https://github.com/ashcode028/ROS/blob/eba454c354ebef48c1b4cf8dfc1c59f0f75cb10d/Move_turtle/images/task0_rqt_graph.png)
 ## Task 2A: Design a Altitude Controller
- The objective of the task is to design an attitude controller for the eDrone. In other words, attitude of a drone means the orientation of the drone in terms of Euler angles ie. roll, pitch and yaw.
- To achieve control over the attitude of eDrone, you need to design a PID controller. The controller is a closed loop controller with the present orientation of eDrone being fed back by the IMU sensor. A general diagram representing the control system for the attitude controller is shown in Figure 1.
![](https://github.com/ashcode028/ROS/blob/ea38bbea958b29fec1833910fdd72553fdb8f564/vitarana_drone/images/attitude_control.png)
- To implement this controller, you will be writing an algorithm for the same in a python script(rosnode) named attitude_controller.py in the scripts folder of the vitarana_drone ROS package.
- A boiler plate script for the attitude controller is provided to you. Read the script line by line carefully and try to understand the structure explained in it.
- After completing the script and tuning its PID gains, the attitude controller should be able to provide an interface where it accepts the orientation set-points in terms of roll, pitch and yaw set-point for the eDrone in the standard servo message format ie. 1000 - 2000 and the eDrone should be able to stabilise itself to the given set-point.
```
For eg. if the input given to the attitude controller is

rcRoll = 1500, rcPitch = 1500, rcYaw = 1500

```
- Then the eDrone must stabilise itself at roll=0° pitch=0° yaw=0°

NOTE: The input to the attitude controller should only range from 1000 - 2000 where 1000 is the lower limit and 2000 is the upper limit. The limits can be set according to you. For eg. if you want to set the limit for angles from -10° to 10°, then when input 1000 is given, eDrone should stabilise to -10°, when input 1500 is given, eDrone should stabilise at 0° and when input 2000 is given eDrone should stabilise at 10°.
- The publisher-subscriber structure of the attitude controller rosnode is shown below
```
PUBLICATIONS				SUBSCRIPTIONS
/edrone/pwm				/edrone/drone_command
                        /edrone/imu/data
```
- The inputs (set-points) to the attitude controller are given on the rostopic /edrone/drone_command, which uses a custom rosmsg named edrone_cmd.
The message structure of rosmsg edrone_cmd can be seen by typing the command.
```
rosmsg show vitarana_drone/edrone_cmd
```
![](https://github.com/ashcode028/ROS/blob/ea38bbea958b29fec1833910fdd72553fdb8f564/vitarana_drone/images/rosmsg_show_edrone_cmd.png)
- The output of the attitude controller are the speed of 4 propellers in Pulse Width Modulated (PWM) values ranging from 0 - 1023. The output should be published on the rostopic /edrone/pwm which uses a custom rosmsg named prop_speed
```
rosmsg show vitarana_drone/prop_speed
```
![](https://github.com/ashcode028/ROS/blob/ea38bbea958b29fec1833910fdd72553fdb8f564/vitarana_drone/images/rosmsg_show_prop_speed.png)
 ## Task 2B: Design a Position Controller
- The objective of the task is to design a position controller for the eDrone. The position will be described in terms of GPS co-ordinates ie. latitude, longitude & altitude.
- To achieve control over the position of eDrone, you need to design another PID controller which will be in cascade with the attitude controller designed in Task 2A.
- The overall cascaded control system representation is shown in figure.
![](https://github.com/ashcode028/ROS/blob/36883db3911a706890729a1f72291db56537aec6/vitarana_drone/images/cascade_control_system.png)
- As you can see from the block diagram, the position of the eDrone is being fed back using the GPS sensor. The input to the position controller is the position of eDrone in terms of latitude, longitude and altitude. The output of the position controller is the input to the attitude controller which is the orientation at which the eDrone should maintain.
```
For eg. if the input given to the position controller is

latitude = 19.0001, longitude = 72.0000, altitude = 1m

and the current position of the eDrone is

latitude = 19.0000, longitude = 72.0000, altitude = 1m

```
then the position controller should decide that the eDrone should roll so that it reaches its set-point and after reaching the set-point, the eDrone should maintain its position at the given set-point.
- To implement this controller, you will be writing the algorithm for the same in a python script(rosnode) named position_controller.py in the scripts folder of the vitarana_drone ROS package.
- After completing the script tuning PID gains. the position controller should be able to provide an interface where it accepts the position set-point in terms of latitude, longitude and altitude and the eDrone should go and stabilise at the given co-ordinates until next set-point is given.
- The publisher-subscriber structure of the position controller rosnode is shown below
```
PUBLICATIONS				            SUBSCRIPTIONS
/edrone/drone_command			    /edrone/gps
```
- After you complete the position controller, the final task is to use the controller to land on a landing marker.
- The Gazebo world for the task is prepared. To launch it type
```
roslaunch vitarana_drone task_1.launch
```
- Problem statement:
The eDrone starts from the origin ie. latitude = 19.0, longitude = 72.0, altitude = 0.31 The eDrone has to go exactly above the start position to the height when the GPS altitude is 3 meters. Next, the eDrone should go the set-point: latitude =19.0000451704 , longitude = 72.0, altitude = 3m. Next, the eDrone should come down at the same latitude-longitude and land on the marker that is placed below it.
![](https://github.com/ashcode028/ROS/blob/36883db3911a706890729a1f72291db56537aec6/vitarana_drone/images/task.gif)
- There is no specific time window for which you need to stay at each set-point, but a set-point will be considered valid only if the eDrone comes in the threshold box of that particular set-point even for one instance. The threshold box can be calculated by using the tolerance of ±0.000004517 in latitude, ±0.0000047487 in longitude and ±0.2m in altitude.
- Use your position controller script to dynamically change the set-points to complete the task.



