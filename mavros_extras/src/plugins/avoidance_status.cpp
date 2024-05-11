/**
 * @brief Avoidance Status plugin
 * @file avoidance_status.cpp
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

#include <pursuit_msgs/AvoidanceStatus.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief Avoidance status plugin
 *
 * Sends avoidance status to the autopilot
 *
 *
 * @see avoidance_status_cb()	transforming and sending odometry to fcu
 */
class AvoidanceStatusPlugin : public plugin::PluginBase {
public:

	AvoidanceStatusPlugin() : PluginBase(),
		avoidance_status_nh("~avoidance_status")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// subscribers
		avoidance_status_sub = avoidance_status_nh.subscribe("send", 1, &AvoidanceStatusPlugin::avoidance_status_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle avoidance_status_nh;			//!< node handler
	ros::Subscriber avoidance_status_sub;			//!< avoidance status subscriber

	/**
	 * @brief Sends avoidance status data msgs to the FCU.
	 * @param req	received Avoidance status msg
	 */
	void avoidance_status_cb(const pursuit_msgs::AvoidanceStatus::ConstPtr &msg_input)
	{
		mavlink::common::msg::AVOIDANCE_STATUS msg{};

        msg.flag_obstacle_in_far_front = msg_input->flag_obstacle_in_far_front;
        msg.flag_obstacle_far_nearby = msg_input->flag_obstacle_far_nearby;
        msg.flag_obstacle_in_front = msg_input->flag_obstacle_in_front;
        msg.flag_obstacle_in_rear = msg_input->flag_obstacle_in_rear;
        msg.flag_obstacle_nearby = msg_input->flag_obstacle_nearby;
		msg.flag_nav_task_active = msg_input->flag_nav_task_active;
        msg.flag_nav_local_plan_valid = msg_input->flag_nav_local_plan_valid;
        msg.flag_laser_scan_data_valid = msg_input->flag_laser_scan_data_valid;

		// send avoidance status msg
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
		
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::AvoidanceStatusPlugin, mavros::plugin::PluginBase)
