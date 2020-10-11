# Turtlebot follow a square using RGB-D camera and QR code

This project use MATLAB and ROS environment to control a turtlebot 3 waffle. By using a RGB-D camera, the robot is capable of tracking a QR code and navigating perpendicular to the QR code  

<b> <i>Required Software: </i></b>
- Ubuntu 18.04
- MATLAB
- ROS melodic

<b> <i>Packages: </i></b>
  - turtlebot3_simulation -  https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
  - turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3.git 
  - turtlebot3_msgs - https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git 

<i>The model used in this project is TURTLEBOT3 WAFFLE, this model have a RGB-D camera built in can can be set by: </i>
<code> export TURTLEBOT3_MODEL=waffle </code>

<h2><b>Setup and Launching the simulation</b></h2>
Copy the files in the turtlebot3_gazebos.zip into the path: catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo
Before launch,the files have to be sourced:
<code>source devel/setup.bash</code>

The simulation can be launched by:
<code>roslaunch turtlebot3_gazebo turtlebot3_warehouse_withQR.launch</code>
<h2><b> Operation </b></h2> 
The robot will use the RGB-D camera to scan for the QR code, if the QR code is not in the frame, the robot will turn counter clockwise until it sees the QR code.

When the QR code is observed, the robot will calculate the normal of the plane of the QR code and the intersection between the plane normal and the line perpendicular to the plane normal that intersect with the current location of the robot.
The robot then process to go to the intersection point, after it arrives at the intersection point, the robot then turn toward the QR code and perform the orientation adjustment so that the QR code is aligned in front of the robot, lastly the robot will goes in a straight line toward the QR code.

All the navigation of the robot is controlled with PID controller, this makes the movement efficient and reduce unwanted errors due to sudden acceleration or deceleration.

## Code Structure
This code contain 3 main part:
1. PID control: PID control is used to control the angular and linear velocity of the robot, this will prevent the errors from sudden acceleration or deceleration, Turtlebot is lightweight with a differential drive system, this make the robot prone to skidding,turting and sliding errors. All the Kp Ki and Kd is tuned to create the critically damped effect, this help avoid overshoot and creating inefficient, moreover, the steady-state error is also minimized by tuning all the parameters.
2. Image processing: This project uses SURF feature detection to balance between precision and computing cost. Harris feature is not used due to the non-scalling property,this makes the feature not effective. ORB feature is not as good as SURF feature at longer distance, this reduce the range of the robot and affect the overal performance of the project. 
3. Robot control: Robot is controlled via ROS message, the linear speed and angular speed is set and send from matlab, all the speed is controlled by the PID control.

The program will run endlessly in a while loop, you can move the QR code and the robot will automatically navigate.
### Contribution
Vu Duy Nguyen: *40%*

Gia Thinh Nguyen: *35%*

Anup Kharel: *25%*
