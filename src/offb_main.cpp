/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <std_msgs/Bool.h>
#include <termios.h>
#include <fcntl.h>


mavros_msgs::State current_state;
bool change_to_vel_cmd;
double current_height;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void changeToVelCallback(const std_msgs::Bool::ConstPtr & msg){
    change_to_vel_cmd = msg->data;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg){
    current_height = msg->pose.position.z;
}

/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */


/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_main");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                                   ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                   ("mavros/local_position/pose", 10, poseCallback);

    ros::Subscriber change_to_vel_sub = nh.subscribe<std_msgs::Bool>
                                   ("/chang_to_vel", 10, changeToVelCallback);
                                   
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(20.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request(0);
    if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
    }
    last_request = ros::Time::now();

    if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    last_request = ros::Time::now();

    change_to_vel_cmd = false;

    double des_height = 0.0;
    
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.twist.linear.x = 0.3;

    while (ros::ok()) {
        if(!change_to_vel_cmd){
            ROS_INFO("setpoint: %.1f, %.1f, %.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            local_pos_pub.publish(pose);
            des_height = current_height;
        }else{
            cmd_vel.twist.linear.z = -(current_height - des_height);
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("cmd_vel: %.3f, %.3f, %.3f", cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
