/**
 * @brief vcu base status plugin
 * @file vcu_base_status.cpp
 * @author Cloudkernel Technologies 
 *
 * @addtogroup plugin
 * @{
 */
/*
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/utils.h>

#include <pursuit_msgs/VcuBaseStatus.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief vcu base status plugin
 *
 * Receive vcu base status from the autopilot
 *
 *
 */
class VcuBaseStatusPlugin : public plugin::PluginBase {
public:

	VcuBaseStatusPlugin() : PluginBase(),
		vcu_base_status_nh("~vcu_base_status")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

        vcu_base_status_pub = vcu_base_status_nh.advertise<pursuit_msgs::VcuBaseStatus>("output", 10);
	}

	Subscriptions get_subscriptions()
	{
		return { 
            make_handler(&VcuBaseStatusPlugin::handle_vcu_base_status_from_autopilot)
        };
	}

private:
	ros::NodeHandle vcu_base_status_nh;			//!< node handler

    ros::Publisher vcu_base_status_pub;		

	void handle_vcu_base_status_from_autopilot(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VCU_BASE_STATUS &vcu_base_status_msg)
	{
        //Convert FRD to FLU convention here
		auto output_msg = boost::make_shared<pursuit_msgs::VcuBaseStatus>();

        output_msg->vcu_base_type = vcu_base_status_msg.vcu_base_status;
        output_msg->gear_position = vcu_base_status_msg.gear_position;
        output_msg->speed = vcu_base_status_msg.speed;
        output_msg->steering_angle_valid = vcu_base_status_msg.steering_angle_valid;
        output_msg->steering_angle = -vcu_base_status_msg.steering_angle;
        output_msg->twist_valid = vcu_base_status_msg.twist_valid;

        output_msg->vel[0] = vcu_base_status_msg.vel[0];
        output_msg->vel[1] = -vcu_base_status_msg.vel[1];
        output_msg->vel[2] = -vcu_base_status_msg.vel[2];

        vcu_base_status_pub.publish(output_msg);
    }

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VcuBaseStatusPlugin, mavros::plugin::PluginBase)
