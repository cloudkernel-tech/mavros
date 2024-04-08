/**
 * @brief vcu command velocity plugin
 * @file vcu_command_velocity.cpp
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2024, Cloudkernel Technologies, https://cloudkernel.cn
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <geometry_msgs/Twist.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief vcu command velocity plugin
 *
 * @see odom_cb()	transforming and sending odometry to fcu
 * @see handle_vcu_cmd_vel_input()	receiving nav cmd vel from other task modules and sending it to the pursuit autopilot
 */
class VcuCommandVelocityPlugin : public plugin::PluginBase {

public:

	VcuCommandVelocityPlugin() : PluginBase(),
		vcu_cmd_vel_nh("~vcu_command_velocity")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// publishers
		vcu_cmd_vel_pub = vcu_cmd_vel_nh.advertise<geometry_msgs::Twist>("output", 10);

		// subscribers
		nav_cmd_vel_sub = vcu_cmd_vel_nh.subscribe("from_nav", 1, &VcuCommandVelocityPlugin::nav_cmd_vel_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&VcuCommandVelocityPlugin::handle_cmd_vel_from_autopilot)
		};
	}

private:
	ros::NodeHandle vcu_cmd_vel_nh;			//!< node handler
	ros::Publisher vcu_cmd_vel_pub;			
	ros::Subscriber nav_cmd_vel_sub;	

	void handle_cmd_vel_from_autopilot(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VCU_COMMAND_VELOCITY &cmd_vel_msg)
	{
		auto cmd_vel_output_msg = boost::make_shared<geometry_msgs::Twist>();

        //convert cmd vel in NED to ENU frame 
        cmd_vel_output_msg->linear.x = cmd_vel_msg.linear_vel[1];
        cmd_vel_output_msg->linear.y = cmd_vel_msg.linear_vel[0];
        cmd_vel_output_msg->linear.z = -cmd_vel_msg.linear_vel[2];

        cmd_vel_output_msg->angular.x = cmd_vel_msg.angular_vel[1];
        cmd_vel_output_msg->angular.y = cmd_vel_msg.angular_vel[0];
        cmd_vel_output_msg->angular.z = -cmd_vel_msg.angular_vel[2];

        vcu_cmd_vel_pub.publish(cmd_vel_output_msg);

	}


	void nav_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &nav_cmd_vel)
	{
		mavlink::common::msg::VCU_COMMAND_VELOCITY msg{};

        //convert data in ENU to NED for autopilot convention
        msg.linear_vel[0] = nav_cmd_vel->linear.y; 
        msg.linear_vel[1] = nav_cmd_vel->linear.x; 
        msg.linear_vel[2] = -nav_cmd_vel->linear.z;

        msg.angular_vel[0] = nav_cmd_vel->angular.y;
        msg.angular_vel[1] = nav_cmd_vel->angular.x; 
        msg.angular_vel[2] = -nav_cmd_vel->angular.z;

		// send ODOMETRY msg
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VcuCommandVelocityPlugin, mavros::plugin::PluginBase)
