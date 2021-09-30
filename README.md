# ros_aco_path_planner

ACO path planning plugin for Global Planner based on AS algorithm.

**Instructions:**
* Clone this repository inside catkin_ws/src using `git clone https://github.com/fabio-cabeccia/ros_aco_path_planner.git`.
* Build the package either individually or along with the rest of the workspace.
* Add the following line to your move_base launch file: 
  `<param name="base_global_planner" value="aco_ros/aco_ros_planner"/>`

**Currently known problems:**
* If using slam and non static map, all the cell inside the costmap will not have 
  a value other than 0 (free space). 
