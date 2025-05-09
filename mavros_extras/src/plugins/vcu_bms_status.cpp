/**
 * @brief vcu bms status plugin
 * @file vcu_bms_status.cpp
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

#include <pursuit_msgs/VcuBmsStatus.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief vcu bms status plugin
 *
 * Receive vcu bms status from the autopilot
 *
 *
 */
class VcuBmsStatusPlugin : public plugin::PluginBase {
public:

	VcuBmsStatusPlugin() : PluginBase(),
		vcu_bms_status_nh("~vcu_bms_status")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

        vcu_bms_status_pub = vcu_bms_status_nh.advertise<pursuit_msgs::VcuBmsStatus>("output", 10);
	}

	Subscriptions get_subscriptions()
	{
		return { 
            make_handler(&VcuBmsStatusPlugin::handle_vcu_bms_status_from_autopilot)
        };
	}

private:
	ros::NodeHandle vcu_bms_status_nh;			//!< node handler

    ros::Publisher vcu_bms_status_pub;		

	void handle_vcu_bms_status_from_autopilot(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VCU_BMS_STATUS &vcu_bms_status_msg)
	{
        //Convert FRD to FLU convention here
		auto output_msg = boost::make_shared<pursuit_msgs::VcuBmsStatus>();

		output_msg->header.stamp = ros::Time::now();

        output_msg->voltage = vcu_bms_status_msg.voltage;
        output_msg->current = vcu_bms_status_msg.current;
        output_msg->remained_capacity = vcu_bms_status_msg.remained_capacity;
		output_msg->remained_percentage = vcu_bms_status_msg.remained_percentage;
        output_msg->max_temperature = vcu_bms_status_msg.max_temperature;
        output_msg->min_temperature = vcu_bms_status_msg.min_temperature;

        vcu_bms_status_pub.publish(output_msg);
    }

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VcuBmsStatusPlugin, mavros::plugin::PluginBase)
