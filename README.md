
## MUBot:  Control and Monitoring Using ROS Navigation Stack and ROS-Mobile App
This repository is about autonomous navigation and SLAM with MUBot in our laboratory virtual environment as well as its real-time control and monitoring from ROS-Mobile App leveraging on the framework developed by [Rottmann Nils et al](https://github.com/ROS-Mobile/ROS-Mobile-Android). 
The robot runs on ROS Noetic  and ROS Melodic with laptop or Nvidia Jetson Nano used as the host computer. The hardware implementation and documentation has already been described in [this repository](https://github.com/ai-lab-science/MUBot-The-design-documentation-Remote-Control-and-ROS-Simulation). It is necessary to go through the repository to set up the environment and install other ROS packages not mentioned here.

### SLAM with MUBot in our laboratory virtual world:

The laboratory world file is generated from Gazebo using the approximate measurement of our laboratory plan. The map of the laboratory environment is generated from a laser scanner (360-degree Lidar sensor) with the help of gmapping, Adaptive Monte Carlo Localisation (AMCL), and move_base algorithms.

- **Dependencies to install:**

Before you begin to launch the robot in the virtual laboratory world, the following ROS packages need to be installed. I am using ROS Noetic for this simulation, in case you are using a different ROS version, ensure that you replace wherever I mentioned noetic with your ROS distribution.
```
sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-robot-joint-state-publisher
sudo apt-get install ros-noetic-robot-joint-state-publisher-gui
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-slam-gmapping
sudo apt-get install ros-noetic-rqt-robot-steering
sudo apt-get install gazebo11 libgazebo11-*
```
- **The laboratory virtual world:**

Run the following command:
```
roslaunch mubot_navigation gazebo_house.launch
```
This launches the laboratory layout in the gazebo and the current location of the robot in the laboratory as shown in the figure below. 

![laboratory_world](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/laboratory_world.png

On a new terminal, run the following command to view the robot's urdf and the lidar data in Rviz. Ensure that `roscore` is running.
```
roslaunch mubot_navigation rviz_display.launch
```
![laboratory_world](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/rviz_view.png)

The red dotted lines are the lidar scan showing the location of obstacles (objects) within the robot environment. Use the `rqt_robot_steering` tool to move the robot around the laboratory, and try to introduce obstacles on the way to see the robot's response to it. Once done, press Ctrl+C in all the open terminal to close them.

![obstacles](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/obst_intr.gif)

- **Map generation:**

The map of the laboratory is generated using a laser scanner (RPlidar) which subscribes to lidar scan (sensor_msgs/LaserScan) topic to create the map from, and the tf (tf/tfMesage) to relate frames from lidar, robot_base and odom. To generate a new map, run the following command:

```
roslaunch mubot_navigation gazebo_house.launch
```
On a new terminal, run:
```
roslaunch mubot_navigation rviz_display.launch
```
Ensure that gmapping algorithm is setup and running in a separate terminal. 
```
rosrun gmapping slam_gmapping scan:=scan
```
Use rqt_robot_steering to move the robot around the laboratory to map the environment. Just run `rqt_robot_steering` or `rosrun rqt_robot_steering rqt_robot_steering` on a new terminal.
Once you are done mapping, save the map in your maps directory using the following ROS command:
```
rosrun map_server map_saver -f laboratory_map
```
Ensure that the map server is installed and running properly. Else, run `sudo apt-get install ros-noetic-map-server`. You can view the saved map by running `rosrun map_server map_server laboratory_map.yaml` from a terminal.
If everything went well, you would be able to obtain something similar to the following pictures.

![mapping](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/lab_map.png)

![mapped area](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/mapped_lab.png)

- **Autonomous navigation:**

For autonomous navigation, we used the ros navigation stack to move the robot from its current location to the goal location. Launch the autonomous navigation by running the following command:
```
roslaunch mubot_navigation gazebo_house.launch
```
On a new terminal window, 
```
roslaunch mubot_navigation mubot_navigation.launch
```
Set a goal location for the robot by clicking on the "2D Nav Goal" button on the Rviz display window and then click on any place in the map that you would like the robot to go. Although not compulsory, drag while you click on the goal location to position the direction you would like the robot to face when it reaches the goal location. You would obtain something similar to the video below.
![send to goal](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/auto-nav.gif)

- **Send Mubot to a goal location:**

We can send mubot to a goal location within the laboratory without need to specify it from the Rviz GUI as above. All you need to do is to run the node below in a new terminal. Mubot will ask you where do you want to go. Select one location from the 14 locations in the laboratory by enterying a number (1 to 14) describing your choice place as in the video below.

```
rosrun mubot_navigation send_goals
```
![send to goal](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/send_to_goal.gif)


### Real-time control and monitoring with ROS-Mobile device:

- **Setup (Arduino ROS integration):**

First, you need to be sure that Arduino has been set up to communicate with ROS. [This tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) could be helpful. I have included a ros_lib which I have modified to make your setup much more easier. Make sure that the rosserial node (rosserial_python serial_node.py) is running properly. [This tutorial](http://wiki.ros.org/rosserial_python#serial_node.py) could be helpful for the setup.
The whole communication structure is described in the following figure.

![communication](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/communication.jpg)

- **Control and monitoring from ROS-Mobile device:**

Install the ROS-Mobile App on your Android device. The instruction for the setup can be [found here](https://github.com/ROS-Mobile/ROS-Mobile-Android). If everything went fine, then proceed to teleoperate the robot from the Android device. First, upload `odrive_ros.ino` sketch to the Arduino. Ensure that the robot is switched ON, the battery connected and the wheels are free to spin. Ensure that the Odrive controller is calibrated prior to launching the following files. I have prepared a python scripts to make the calibration much more easier. The scripts `odrive_parameter_configuration.py` and `odrive_calibration.py` can be [found in this repository](https://github.com/ai-lab-science/MUBot-The-design-documentation-Remote-Control-and-ROS-Simulation#Motors-configuration-and-calibration).
Once satisfied that everything is ok, then run
```
roslaunch mubot_navigation gazebo_house.launch
```
to load the laboratory world in gazebo and 
```
roslaunch mubot_navigation mubot_bringup.launch
```ttps://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/media/ros-mobile-control.gif
to launch the rosserial node. 
Now the robot can be controlled using the ROS-Mobile App which enables ROS to control the robot's joint velocities. Linear (forward and backward movement) and angular (rotation around the z-axis).

![watch the testing video here](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/ros_mobile_app.png)

![watch the testing video here](https://github.com/ai-lab-science/MUBot-Control-and-Monitoring-Using-ROS-Navigation-Stack-and-ROS-Mobile-App/blob/main/mubot_navigation/media/ros-mobile-control.gif)

### Real-time autonomous navigation:
In progress . . .
