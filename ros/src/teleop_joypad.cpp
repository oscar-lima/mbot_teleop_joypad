/*
 * Copyright [2011] <Bonn-Rhein-Sieg University>
 *
 * teleop_joypad.cpp
 *
 *  Created on: May 27, 2012
 *      Author: Frederik Hegger
 *      Maintainer: Oscar Lima
 */

#include <mbot_teleop/teleop_joypad.h>
#include <string>
#include <angles/angles.h>

TeleOpJoypad::TeleOpJoypad(ros::NodeHandle &nh)
{
    nh_ = &nh;

    button_deadman_pressed_prev_ = false;

    if (!this->getJoypadConfigParameter())
    {
        ROS_ERROR("could not get joypad parameters.");
        exit(0);
    }

    this->getBaseParameter();

    // prepare head control msg
    head_yaw_control_msg_.data.resize(2);
    head_yaw_control_msg_.data[1] = 15;

    sub_joint_states_ = nh_->subscribe < sensor_msgs::JointState > ("/joint_states", 1, &TeleOpJoypad::cbJointStates, this);
    pub_head_yaw_ = nh_->advertise < std_msgs::UInt8MultiArray > ("cmd_yaw", 1);
    pub_head_camera_tilt_ = nh_->advertise < std_msgs::Float64 > ("cmd_tilt", 1);

    sub_joypad_ = nh_->subscribe < sensor_msgs::Joy > ("joy", 1, &TeleOpJoypad::cbJoypad, this);
    pub_base_cart_vel_ = nh_->advertise < geometry_msgs::Twist > ("cmd_vel", 1);
    pub_manual_annotation_ = nh_->advertise < std_msgs::Empty > ("/manual_annotation", 1);
}

bool TeleOpJoypad::getJoypadConfigParameter()
{
    XmlRpc::XmlRpcValue name_list, index_list;
    std::string types[] =
    { "buttons", "axes" };

    for (int i = 0; i < 2; ++i)  // TBD
    {
        if (!nh_->getParam("joypad/" + types[i] + "/name", name_list) || !nh_->getParam("joypad/" + types[i] +
            "/index", index_list))
        {
            return false;
        }

        ROS_ASSERT(name_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(index_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(name_list.size() == index_list.size());

        for (int j = 0; j < name_list.size(); ++j)
        {
            if (i == 0)
            {
                if (name_list[j] == "deadman")
                    button_index_deadman_ = index_list[j];
                else if (name_list[j] == "run")
                    button_index_run_ = index_list[j];
                else if (name_list[j] == "manual_annotation")
                    button_index_manual_annotation_ = index_list[j];
                else if (name_list[j] == "reset")
                    button_index_reset_ = index_list[j];
                else
                    ROS_WARN_STREAM("button name <<" << name_list[j] << ">> in yaml but not used in node");
            }

            else if (i == 1)
            {
                if (name_list[j] == "base_linear_x")
                    axes_index_base_linear_x_ = index_list[j];
                else if (name_list[j] == "base_linear_y")
                    axes_index_base_linear_y_ = index_list[j];
                else if (name_list[j] == "base_angular_z")
                    axes_index_base_angular_z_ = index_list[j];
                else if (name_list[j] == "head_rotation")
                    axes_index_head_rotation_ = index_list[j];
                else if (name_list[j] == "head_camera_tilt")
                    axes_index_head_camera_tilt_ = index_list[j];
                else
                    ROS_WARN_STREAM("axes name <<" << name_list[j] << ">> in yaml but not used in node");
            }
        }
    }

    return true;
}

void TeleOpJoypad::getBaseParameter()
{
    double param = 0;
    ros::param::param<double>("~base_max_linear_x_vel", param, 0.3);
    base_cart_factor_.linear.x = param / MAX_JOYPAD;
    ros::param::param<double>("~base_max_linear_y_vel", param, 0.3);
    base_cart_factor_.linear.y = param / MAX_JOYPAD;
    ros::param::param<double>("~base_max_angular_vel", param, 0.5);
    base_cart_factor_.angular.z = param / MAX_JOYPAD;
}

void TeleOpJoypad::cbJointStates(const sensor_msgs::JointState::ConstPtr& joint_states)
{
    const static auto yaw_it = std::find(joint_states->name.begin(), joint_states->name.end(), "base_link_to_head_link_joint");
    const static auto tilt_it = std::find(joint_states->name.begin(), joint_states->name.end(), "head_link_to_head_camera_link");

//     ROS_ASSERT(yaw_it != joint_states->name.end() && tilt_it != joint_states->name.end());
//     ROS_ASSERT(tilt_it != joint_states->name.end() && tilt_it != joint_states->name.end());

    const static size_t tilt_idx = std::distance(joint_states->name.begin(), tilt_it);
    const static size_t yaw_idx = std::distance(joint_states->name.begin(), yaw_it);

    head_camera_tilt_value_ = joint_states->position[tilt_idx];
    head_yaw_value_ = joint_states->position[yaw_idx];
}

void TeleOpJoypad::cbJoypad(const sensor_msgs::Joy::ConstPtr& command)

{
    if (static_cast<bool>(command->buttons[button_index_reset_]))
    {
        // Reset head yaw and tilt
        head_yaw_control_msg_.data[0] = 90;
        pub_head_yaw_.publish(head_yaw_control_msg_);
        
        std_msgs::Float64 msg;
        msg.data = 1.572;
        pub_head_camera_tilt_.publish(msg); 

        // Dont allow to do anything else while this is presset
        return ;
    }

    if (static_cast<bool>(command->buttons[button_index_deadman_]))
    {
        // deadman button is pressed, is safe to move the base

        speed_factor_ = !static_cast<bool>(command->buttons[button_index_run_]) ? 0.5 : 1.0;
        
        base_cart_vel_.linear.x = command->axes[axes_index_base_linear_x_] * base_cart_factor_.linear.x * speed_factor_;
        base_cart_vel_.linear.y = command->axes[axes_index_base_linear_y_] * base_cart_factor_.linear.y * speed_factor_;
        base_cart_vel_.angular.z = command->axes[axes_index_base_angular_z_] * base_cart_factor_.angular.z * speed_factor_;

        if (fabs(base_cart_vel_.linear.x) < 0.01)
            base_cart_vel_.linear.x = 0.0;
        if (fabs(base_cart_vel_.linear.y) < 0.01)
            base_cart_vel_.linear.y = 0.0;
        if (fabs(base_cart_vel_.angular.z) < 0.01)
            base_cart_vel_.angular.z = 0.0;

        pub_base_cart_vel_.publish(base_cart_vel_);
    }
    else
    {
        if (button_deadman_pressed_prev_)
        {
            pub_base_cart_vel_.publish(base_cart_zero_vel_);
        }
    }

    // Head rotation and tilt head camera
    const int head_rotation = command->axes[axes_index_head_rotation_];
    const int head_camera_tilt = command->axes[axes_index_head_camera_tilt_];

    if(head_rotation != 0)
    {
        // Put in the control msg
        const double angles_raw = angles::to_degrees(head_yaw_value_) + 90 + 10.0*(-head_rotation); // head_rotation is 1 or -1
        // Constrain to min 5 max 175 degrees
        head_yaw_control_msg_.data[0] = std::max(std::min(angles_raw,175.0), 5.0);

        pub_head_yaw_.publish(head_yaw_control_msg_);
    }
    if(head_camera_tilt != 0)
    {
        std_msgs::Float64 msg;
        msg.data = head_camera_tilt_value_ + angles::from_degrees(5.0)*(head_camera_tilt);
        pub_head_camera_tilt_.publish(msg);
    }
    
    // Manual annotation button.
    // When the button is pressed down, an annotation is published.
    if (static_cast<bool>(command->buttons[button_index_manual_annotation_]))
    {
        if(!button_manual_annotation_prev_)
        {
            std_msgs::Empty annotation_msg;
            pub_manual_annotation_.publish(annotation_msg);
        }
    }

    // remember buttons states
    button_deadman_pressed_prev_ = static_cast<bool>(command->buttons[button_index_deadman_]);
    button_manual_annotation_prev_ = static_cast<bool>(command->buttons[button_index_manual_annotation_]);
}
