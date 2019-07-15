/*
 * Copyright [2012] <Bonn-Rhein-Sieg University>
 *
 * teleop_joypad.h
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 *      Maintainer: Oscar Lima
 */

#ifndef MBOT_TELEOP_TELEOP_JOYPAD_H_
#define MBOT_TELEOP_TELEOP_JOYPAD_H_

#include <string>
#include <vector>
#include <algorithm>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8MultiArray.h>

#define MAX_JOYPAD 1.0

class TeleOpJoypad
{
public:
    explicit TeleOpJoypad(ros::NodeHandle &nh);

private:
    bool getJoypadConfigParameter();
    void getBaseParameter();

    void cbJointStates(const sensor_msgs::JointState::ConstPtr& joint_states);

    void cbJoypad(const sensor_msgs::Joy::ConstPtr& command);

    ros::NodeHandle* nh_;

    double speed_factor_;

    bool button_deadman_pressed_prev_;
    bool button_manual_annotation_prev_;

    geometry_msgs::Twist base_cart_vel_;
    geometry_msgs::Twist base_cart_zero_vel_;
    geometry_msgs::Twist base_cart_factor_;

    double head_camera_tilt_value_;
    double head_yaw_value_;
    std_msgs::UInt8MultiArray head_yaw_control_msg_;

    // Subscriber
    ros::Subscriber sub_joypad_;
    ros::Subscriber sub_joint_states_;
    
    // Publisher
    ros::Publisher pub_base_cart_vel_;
    ros::Publisher pub_manual_annotation_;
    ros::Publisher pub_head_yaw_;
    ros::Publisher pub_head_camera_tilt_;

    int button_index_deadman_;
    int button_index_run_;
    int button_index_manual_annotation_;
    int button_index_reset_;

    int axes_index_base_linear_x_;
    int axes_index_base_linear_y_;
    int axes_index_base_angular_z_;

    int axes_index_head_rotation_;
    int axes_index_head_camera_tilt_;
};

#endif  // MIR_TELEOP_TELEOP_JOYPAD_H_
