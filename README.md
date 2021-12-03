# Wall distance

Offboard control to ensure wall following. Once offboard mode activated the quad will always face the nearest wall and stay at the same distance.
While wall following is enabled, user can still steer the drone up/down (z-axis) and left/right (y-axis).
Stack and tested in Gazebo SITL

## Launch
To launch SITL gazebo with test world :`roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_rplidar world:=/home/mathieu/catkin_ws/src/wall_distance/worlds/wall_follow.world`

To launch this node + joystick node : `roslaunch wall_distance wall_distance.launch`

---

! You must ensure the file /opt/ros/$ROS_DISTRO/share/mavros/launch/px4_config.yaml is set with the following lines :
setpoint_velocity:
 mav_frame: BODY_NED
Otherwise the drone will steer in a North/East frame and not a frame linked to the drone's body
