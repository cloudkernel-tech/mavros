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

#include <pursuit_msgs/VcuDirectCanCtrl.h>

namespace mavros {
namespace extra_plugins {

/**
 * @brief vcu bms status plugin
 *
 * Receive vcu bms status from the autopilot
 *
 *
 */
class VcuDirectCanCtrlPlugin : public plugin::PluginBase {
public:

	VcuDirectCanCtrlPlugin() : PluginBase(),
		vcu_direct_can_ctrl_nh("~vcu_bms_status")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// subscribers
		vcu_direct_ctrl_msg_sub = vcu_direct_can_ctrl_nh.subscribe("send", 1, &VcuDirectCanCtrlPlugin::vcu_direct_can_ctrl_cb, this);

	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle vcu_direct_can_ctrl_nh;			//!< node handler

    ros::Subscriber vcu_direct_ctrl_msg_sub;	


    void vcu_direct_can_ctrl_cb(const pursuit_msgs::VcuDirectCanCtrl::ConstPtr &msg_input)
	{
		mavlink::common::msg::VCU_DIRECT_CAN_CTRL msg{};

		msg.msg_id = msg_input->msg_id;

		std::copy(msg_input->msg_body.begin(), msg_input->msg_body.end(), msg.msg_body.begin());	// std::array = boost::array

		// send avoidance status msg
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
		
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VcuDirectCanCtrlPlugin, mavros::plugin::PluginBase)
