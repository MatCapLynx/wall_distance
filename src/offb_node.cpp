/**
 * @file offb_node.cpp
 * @brief Offboard control to ensure wall following. Once offboard mode activated the quad will always face the nearest wall and stay at the same distance.
 While wall following is enabled, user can still steer the drone up/down (z-axis) and left/right (y-axis).
 * Stack and tested in Gazebo SITL
 To launch SITL gazebo with test world : roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_rplidar world:=/home/mathieu/catkin_ws/src/wall_distance/worlds/wall_follow.world
 To launch this node + joystick node : roslaunch wall_distance wall_distance.launch

 ! You must ensure the file /opt/ros/$ROS_DISTRO/share/mavros/launch/px4_config.yaml is set with the following lines :
 setpoint_velocity:
  mav_frame: BODY_NED
Otherwise the drone will steer in a North/East frame and not a frame linked to the drone's body
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

#define HOUGH_TRESHOLD 20

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::Joy joy_state;
void joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_state = *msg;
}

//Global variables computed in this callback function
sensor_msgs::LaserScan current_lidar_scan;
float wall_distance;
float prev_wall_distance;
float wall_angle;
int acc_value;
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    current_lidar_scan = *msg;
    float angle_increment = current_lidar_scan.angle_increment;
    int num_readings = ceil( (current_lidar_scan.angle_max - current_lidar_scan.angle_min) / angle_increment );

    int num_points = 0; //number of points with a non inf value
    float scan_map [360] [2]; //xy position of every point scanned
    for (int i =0; i < num_readings; i++){
      if (current_lidar_scan.ranges[i] <= current_lidar_scan.range_max){ //obstacle detected
        scan_map[num_points][0] = cos(current_lidar_scan.angle_min + angle_increment*i)*current_lidar_scan.ranges[i]; //x value
        scan_map[num_points][1] = sin(current_lidar_scan.angle_min + angle_increment*i)*current_lidar_scan.ranges[i]; //y value
        num_points++;
      }
    }
    num_points++;

    // ROS_INFO("Number of laser points: %d (%d directions scanned)",num_points,num_readings);

    //Hough line detection algorithm
    float min_detection_angle = -M_PI;
    float max_detection_angle = M_PI;
    int num_angle_values = 200;
    float hough_angle_increment = (max_detection_angle - min_detection_angle)/num_angle_values;
    float angles [num_angle_values];
    for (int i = 0; i < num_angle_values; i++){
      angles[i] = min_detection_angle + hough_angle_increment*i;
    }

    float min_detection_radius = current_lidar_scan.range_min;
    float max_detection_radius = current_lidar_scan.range_max;
    int num_radius_values = 50;
    float hough_radius_increment = (max_detection_radius - min_detection_radius)/num_radius_values;

    //Fill accumulator with values
    int accumulator_matrix [num_angle_values][num_radius_values] = {};
    for (int i = 0; i < num_points; i++){
      for (int j = 0; j < num_angle_values; j++){
        float radius = scan_map[i][0]*cos(angles[j]) + scan_map[i][1]*sin(angles[j]);
        if (radius >= 0 && radius <= max_detection_radius){ //line is detected only in front of the drone
          int radius_index = round((radius - min_detection_radius)/hough_radius_increment);
          accumulator_matrix[j][radius_index]++;
        }
        // std::cout << radius << "," << radius_index << "  ";
      }
    }

    //Find line with the smallest distance (radius)
    acc_value = 0;
    float detected_angle;
    float detected_radius;
    float min_radius = INT_MAX;
    for (int i = 0; i < num_angle_values; i++){
      for (int j = 0; j < num_radius_values; j++){
        detected_radius = min_detection_radius + j*hough_radius_increment;
        // std::cout << accumulator_matrix[i][j] << "  ";
        if (accumulator_matrix[i][j] > num_points*0.3 && detected_radius < min_radius){
          acc_value = accumulator_matrix[i][j];
          detected_angle = min_detection_angle + i*hough_angle_increment;
          min_radius = detected_radius;
        }
      }
    }
    prev_wall_distance = wall_distance;
    wall_distance = min_radius;
    wall_angle = detected_angle;

    if (wall_distance < min_detection_radius * 1.05){
      acc_value = 0; //No confidence
    }
    // ROS_INFO("Max value: %d, Detected angle: %f, Detected radius: %f",acc_value,detected_angle,min_radius);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>
            ("laser/scan", 100, lidar_cb);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
            ("/joy", 100, joy_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Command point
    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;

    // Command velocity
    geometry_msgs::TwistStamped velocity;
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
    velocity.twist.angular.z = 0; //yaw

    //Raw Command
    // mavros_msgs::PositionTarget target;
    // target.coordinate_frame = 8; //MAV_FRAME_BODY_FRD*
    // // target.type_mask = 0b0000001111111000;
    // target.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |  //Bitmap to indicate which dimensions should be ignored by the vehicle
    //                    mavros_msgs::PositionTarget::IGNORE_AFY |
    //                    mavros_msgs::PositionTarget::IGNORE_AFZ |
    //                    // mavros_msgs::PositionTarget::IGNORE_VX  |
    //                    // mavros_msgs::PositionTarget::IGNORE_VY  |
    //                    // mavros_msgs::PositionTarget::IGNORE_VZ  |
    //                    mavros_msgs::PositionTarget::IGNORE_YAW |
    //                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    // target.position.x = 0;
    // target.position.y = 0;
    // target.position.z = 1.5;
    // target.velocity.x = 0;
    // target.velocity.y = 0;
    // target.velocity.z = 0;
    // // target.yaw = 0;
    // // target.yaw_rate = 0; //rad/s


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        cmd_vel_pub.publish(velocity);
        // local_raw_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    //
    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool distance_set = false;
    float target_distance;

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        // local_pos_pub.publish(pose);
        cmd_vel_pub.publish(velocity);
        // local_raw_pub.publish(target);

        ROS_INFO("Publishing velocity x:%f, y:%f, z:%f, y:%f",
          // position x:%f, y:%f, z:%f | pose.pose.position.x = 0,  pose.pose.position.y = 0, pose.pose.position.z = 2,
          velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z,velocity.twist.angular.z);
        // ROS_INFO("Publishing target x:%f, y:%f, z:%f, vx:%f, vy:%f, vz:%f, y:%f",
        //   target.position.x, target.position.y, target.position.z, target.velocity.x, target.velocity.y, target.velocity.z, target.yaw);

        // target.yaw = -wall_angle;
        if (acc_value > HOUGH_TRESHOLD){ //High confidence in the detected wall
          if( current_state.mode == "OFFBOARD" && !distance_set){ //set target distance when OFFBOARD mode is activated
            target_distance = wall_distance;
            distance_set = true;
          }

          if (distance_set) {
            velocity.twist.angular.z = wall_angle; //face the wall
            velocity.twist.linear.x = (wall_distance - target_distance)*0.8;
          }
        }

        else {
          velocity.twist.linear.x = 0;
          velocity.twist.linear.y = 0;
          velocity.twist.linear.z = 0;
          velocity.twist.angular.z = 0;
        }

        //Control x and z axis with joystick input
        if (!joy_state.axes.empty()){
          velocity.twist.linear.y = joy_state.axes[6];
          velocity.twist.linear.z = joy_state.axes[7];
        }

        if( current_state.mode != "OFFBOARD" ){ //unset target distance when OFFBOARD mode is stopped
          distance_set = false;
        }



        ROS_INFO("Wall detected at distance %fm with angle %frad with confidence %d, target distance is %f",wall_distance,wall_angle,acc_value,target_distance);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
