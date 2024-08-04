# Hand-Detection-Controller
This repo contains a zip file which is develop to be used as a hand detection controller for Gazebo simulation, utilizing hand gestures to control robotic movements. The teleop_cv node detects gestures and publishes linear and angular velocity commands to the /cmd_vel topic. The turtlebot3_diff_drive node subscribes to this topic to drive the TurtleBot3 accordingly.
## Deployment

To deploy this project in your ROS enviroment.You have to build your workspace after transfering the package controller.

```bash
colon build
```

After building the package you have to source the setup files.

```bash
source /opt/ros/humble/setup.bash
source ~/your_workspace/install/setup.bash
```
Once that is done launch your own TurtleBot3 models which is an ROS package you can install. 

```bash
sudo apt update 
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
```

This will create a bot that can is launched in a Gazebo simulation. Here is an example of one of launching on of the bots in the Gazebo simulation.

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo gazebo_world.launch.py
```

Once the bot is launched open a new terminal to run the node present in the package.
```bash
ros2 run controller teleop_cv
```
You can also load in your own custom worlds with your own custom Bots loaded but they should all have a topic called /cmd_vel.
