### Terminal 1 - Git Checkout & Simulation:
##### source /opt/ros/noetic/setup.bash <br>
##### cd ~/simulation_ws/src/tortoisebot_waypoints <br>
##### git checkout task1 <br>
##### source ~/simulation_ws/devel/setup.bash <br>
##### roslaunch tortoisebot_gazebo tortoisebot_playground.launch <br>
##### Abort and Re-Launch Gazebo if there are any issues. Use kill -9 <gazebo_pid>. <br>

### Terminal 2 - ROS1 Test:
##### source /opt/ros/noetic/setup.bash <br>
##### cd ~/simulation_ws && catkin_make && source devel/setup.bash <br>
##### rostest tortoisebot_waypoints waypoints_test.test --reuse-master <br>

### For SUCCESS: <br>
##### rostest tortoisebot_waypoints waypoints_test.test --reuse-master x:=0.5 y:=0.5 tolerance:=0.2 <br>
### For FAILURE: <br>
##### rostest tortoisebot_waypoints waypoints_test.test --reuse-master x:=1.0 y:=2.0 tolerance:=0.2 <br>
