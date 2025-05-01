# RRT Node for Navigation in Turtlebot 3 

In this README, we describe how to install, setup, and run our custom RRT-based navigation node in the Turtlebot 3 simulated environment on ROS 2 Humble for Ubuntu 22.04.

## Installing Dependencies (ROS 2 Humble, Turtlebot 3, and ROS Gazebo)

Before running anything, ensure that you have followed the installation instructions described [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

The next steps for ensuring that our RRT node will work in the Turtlebot 3 environment is to have the necessary launch files ready for starting the environment. This entails following the instructions in section 6 linked [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) for installing the Gazebo simulation and testing it to make sure it works. BEFORE RUNNING THIS, make sure that the ROS Gazebo package is installed for ROS 2. If you do not install this IT WILL NOT WORK. In our case, we are using ROS2 Humble on Ubuntu 22.04, so it is recommended that we use Gazebo Fortress. For more information on this, see
- https://gazebosim.org/docs/fortress/getstarted/
- https://gazebosim.org/docs/fortress/install_ubuntu/

This is added as a precursor because, if you find that the next steps are not working properly, then follow the instructions in the links above to install the correct Gazebo version. This is likely going to be the issue. 

ALSO, make sure to source the `~/.bashrc` file at every chance you get. If any new terminals are opened or any changes made to the `~/.bashrc` file, then it is necessary to re-source it.

## Setup

Once the installation steps are complete, you can proceed to setting up the node defined in this repository. By this point you should have a directory called `turtlebot3_ws` in your root directory, i.e., `~/turtlebot3_ws`. Starting from your root directory, navigate inside `turtlebot3_ws` with by running the command `cd turtlebot3_ws`. Now we're ready to go.

1. Change directories into `~/turtlebot3_ws/src` by running `cd src`.
2. Clone this repository with 
	`git clone https://github.com/sammsaski/cs5891-finalproject.git`.
3. Rename this repository to be called `custom_rrt_planner` by running 
	- `mv cs5891-finalproject/ custom_rrt_planner/`.
4. Change directories to go inside the `custom_rrt_planner` directory by running `cd custom_rrt_planner`.
5. Move the scripts for starting up Gazebo, RViz, and the RRT node into the `~/turtlebot3_ws` directory by running
	- `mv gazebo.sh ~/turtlebot3_ws/gazebo.sh`,
	- `mv rviz.sh ~/turtlebot3_ws/rviz.sh`,
 	- `mv rrt.sh ~/turtlebot3_ws/rrt.sh`.
6. Move the map information into your root directory by running `mv map.yaml ~` and `mv map.pgm ~`

At this point the file structure is exactly how we want it. The scripts are inside the `turtlebot3_ws` directory, there exists a directory called `custom_rrt_planner` inside of `~/turtlebot3_ws/src` containing all of the code for our custom RRT node, and the map information is located in the root directory, i.e., `~/`. 

Before we move on to running the RRT node, make sure to first change directories into the `~/turtlebot3_ws` directory and compile everything together with a `colcon build`. Once that completes, then be sure to run `source install/setup.bash`. At some point while installing ROS 2 Humble and Turtlebot 3, you may have added the following lines of code to your `~/.bashrc` file:

```
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=30 #TURTLEBOT3
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
```

in which case you can just run `source ~/.bashrc` instead.

Finally, double-check to make sure that all of the `gazebo.sh`, `rviz.sh`, and `rrt.sh` files are executable. If they are not, run `chmod +x <script_name>.sh` and replace `<script_name>` with the respective script you want to make an executable. Additionally, change directories into `~/turtlebot3_ws/src/custom_rrt_planner/custom_rrt_planner` and make sure that the `rrt.py` file is also executable. Once that is all done return to the `~/turtlebot3_ws` directory to move onto the next section where we explain how to run our RRT navigation node.

## Running RRT

Again, before going into the steps for starting the environment and running the navigation node, make sure to run `colcon build` in the root workspace directory, e.g. `~/turtlebot3_ws` and to source the `~/turtlebot3_ws/install/setup.bash` file as we have regularly done throughout this course while working with ROS.

As a reference, the execution of this scripts and the following commands to start the navigation will go as follows:

1. First, make sure you are in the `~/turtlebot3_ws` directory by running `cd ~/turtlebot3_ws`.
2. Run `./gazebo.sh` - to start the Gazebo simulator
3. Run `./rviz.sh` - to start RViz
4. Run `./rrt.sh` - to start our custom RRT node
5. In the RViz window, select the `2D Pose Estimate` button and click on the map where the robot is and drag until the green arrow is pointing in the same direction as the robot. This publishes an estimate of the initial pose.
6. In a separate terminal window (don't forget to `source ~/turtlebot3_ws/install/setup.bash` again), publish the goal pose with the following command

	```
	ros2 topic pub /rrt_goal geometry_msgs/msg/PoseStamped "header:
  	  frame_id: 'map'
  	  stamp: {sec: 0, nanosec: 0}
	pose:
  	  position: {x: <>, y: <>, z: <>}
  	  orientation: {x: <>, y: <>, z: <>, w: <>}" --once
	```
   but don't forget to replace all of the `<>` values in the `position` and `orientation` options.

   As an example, here is a command we use to publish the goal information with the position and orientation values already filled:

	```
	ros2 topic pub /rrt_goal geometry_msgs/msg/PoseStamped "header:
  	  frame_id: 'map'
  	  stamp: {sec: 0, nanosec: 0}
	pose:
  	  position: {x: 2.0, y: 0.0, z: 0.0}
  	  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" --once
	```

7. Sit and watch the robot move! (Note, that the algorithm will not search for a path continuously so if it fails on the first try, then you will need to publish the goal again to get it to work.)

## Example Goal Poses

For your convenience, we provide some example goal poses for you to use. All of them will be in the following form:

	```
	ros2 topic pub /rrt_goal geometry_msgs/msg/PoseStamped "header:
  	  frame_id: 'map'
  	  stamp: {sec: 0, nanosec: 0}
	pose:
  	  position: {x: <>, y: <>, z: 0.0}
  	  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" --once
	```

except you need to fill the `x` and `y` values of the `pose.position` field with the following values:

- x: 1.5, y: 1
- x: 1.5, y: 2
- x: 1.5, y: -1
- x: 3.65, y: 0.55

