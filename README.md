# ros_aco_path_planner

ACO path planning plugin for Global Planner based on AS algorithm.

**Instructions:**
* Clone this repository inside catkin_ws/src using `git clone https://github.com/fabio-cabeccia/ros_aco_path_planner.git`.
* Build the package either individually or along with the rest of the workspace.
* Add the following line to your move_base launch file: 
  `<param name="base_global_planner" value="aco_ros/aco_ros_planner"/>` or use the file `move_base_mod.launch` inside `/launch`.

**Currently known problems:**
* If using slam and non static map, all the cell inside the costmap will not have 
  a value other than 0 (free space). This leads to a situation where the ant, while exploring, finds itself on the border of the map and it can not    continue to move.

**Setting up the robot for the experiments**
* Run the master in a bash window using `roscore`.
* Connect to turtlebot3 using `ssh pi@robot_ip` in another window.
* Once connected to the robot, launch the bringup in the same window with `roslaunch turtlebot3_bringup turtlebot3_robot.launch`.

Now, if using a map, launch this command in a new bash window: `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=pathofthemap`, which will bringup AMCL, move_base and RViz with all robot's sensors
If not, use the following instead `roslaunch turtlebot3_slam turtlebot3_slam.launch`, which will only bringup RViz and the sensors. Move_base must be started separately in a new terminal with `roslaunch turtlebot3_navigation move_base_mod.launch`.

To send a goal use this button on the top of RViz GUI ![tasto](https://user-images.githubusercontent.com/91678128/135478687-a4db3467-fe66-4ea3-b344-008cdc651df6.png).

**Possible improvements**
* In the next cell choosing procedure, one could also back up the cost of the cell with a new information based on the distance between that cell and the goal. In this case, closest cell will have a higher value than farther ones. 


