/**
 * @brief GPS status plugin
 * @file gps_status.cpp
 * @author Amilcar Lucas <amilcar.lucas@iav.de>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/GPSRAW.h>
#include <mavros_msgs/GPSRTK.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nmea_msgs/Gpgga.h>
#include <eigen_conversions/eigen_msg.h>



namespace mavros {
namespace extra_plugins {

static constexpr double RAD_TO_DEG = 180.0 / M_PI;

using mavlink::common::RTK_BASELINE_COORDINATE_SYSTEM;

/**
 * @brief Mavlink GPS status plugin.
 *
 * This plugin publishes GPS sensor data from a Mavlink compatible FCU to ROS.
 * It also subscribes to onboard GPS RTK and forwards the GPS msg & heading to FCU
 */
class GpsStatusPlugin : public plugin::PluginBase {
public:
	GpsStatusPlugin() : PluginBase(),
		gpsstatus_nh("~gpsstatus")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

        /*added for Pursuit autopilot when gps is installed with the companion computer, 
			applicable to QF rtk only here, refer to QF rtk ros driver: https://gitee.com/qonfon/qfrtk_t3
		*/
        //subscription to onboard rtk data
        gps_rtk_onboard_sub = gpsstatus_nh.subscribe("rtk/navsatfix_output", 5, &GpsStatusPlugin::gps_rtk_onboard_cb, this);
        //subscription to onboard rtk odom data (rtk heading)
        gps_rtk_odom_sub = gpsstatus_nh.subscribe("rtk/odom_output",5, &GpsStatusPlugin::gps_rtk_odom_cb, this);
		//subscription to onboard rtk GPGGA msg
		gps_rtk_gpgga_sub = gpsstatus_nh.subscribe("rtk/gpgga_output",5, &GpsStatusPlugin::gps_rtk_gpgga_cb, this);

		gps1_raw_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRAW>("gps1/raw", 10);
		gps2_raw_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRAW>("gps2/raw", 10);
		gps1_rtk_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRTK>("gps1/rtk", 10);
		gps2_rtk_pub = gpsstatus_nh.advertise<mavros_msgs::GPSRTK>("gps2/rtk", 10);

        _last_rtk_odom_quaternion_data.w = 1.0f;
        _last_rtk_odom_quaternion_data.x = _last_rtk_odom_quaternion_data.y = _last_rtk_odom_quaternion_data.z = 0.0f;
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&GpsStatusPlugin::handle_gps_raw_int),
			       make_handler(&GpsStatusPlugin::handle_gps2_raw),
			       make_handler(&GpsStatusPlugin::handle_gps_rtk),
			       make_handler(&GpsStatusPlugin::handle_gps2_rtk)
		};
	}

private:
	ros::NodeHandle gpsstatus_nh;

    ros::Subscriber gps_rtk_onboard_sub;
    ros::Subscriber gps_rtk_odom_sub;
    ros::Subscriber gps_rtk_gpgga_sub;

    geometry_msgs::Quaternion 	_last_rtk_odom_quaternion_data;
	geometry_msgs::Twist 		_last_rtk_odom_twist_data;
	nmea_msgs::Gpgga _last_rtk_gpgga_data;
    bool    _flag_rtk_quaternion_valid{false};//we have to set heading as NAN when there is no new update (px4 firmware convention)
	bool 	_flag_rtk_odom_valid{false};//odom valid flag
	bool 	_flag_rtk_gpgga_valid{false};

	ros::Publisher gps1_raw_pub;
	ros::Publisher gps2_raw_pub;
	ros::Publisher gps1_rtk_pub;
	ros::Publisher gps2_rtk_pub;

	/* -*- callbacks -*- */
    void gps_rtk_onboard_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
		//send to FCU with mavlink GPS_INPUT msg id for simplicity
		mavlink::common::msg::GPS_INPUT onboard_gps{};

		onboard_gps.time_usec = msg->header.stamp.toNSec() / 1000; //unit: us
		onboard_gps.gps_id = 0;
		onboard_gps.ignore_flags = 0;

		//QF rtk status input: 0初始化， 1单点定位， 2码差分， 3无效PPS， 4固定解， 5浮点解， 6正在估算
		//GPS_INPUT msg definition: 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D fix with RTK. 6: 3D float with RTK
		if (msg->status.status == 0)
			onboard_gps.fix_type = 0;
		else if (msg->status.status == 4)
			onboard_gps.fix_type = 5; //3D fix with RTK
		else if (msg->status.status == 5)
			onboard_gps.fix_type = 6; //3D float with RTK
		else 
			onboard_gps.fix_type = 1; //set no fix for others as the accuracy doesn't reach cm level

		onboard_gps.lat = (int32_t)(msg->latitude*1.0e7);  //int32_t, unit: degE7
		onboard_gps.lon = (int32_t)(msg->longitude*1.0e7);  //int32_t, unit: degE7
		onboard_gps.alt = msg->altitude; //float, unit: m

		onboard_gps.eph = (msg->position_covariance[0] > msg->position_covariance[4]) ? msg->position_covariance[0] : msg->position_covariance[4];
		onboard_gps.epv = msg->position_covariance[8];

		//hdop, vdop
		if (_flag_rtk_gpgga_valid){
			onboard_gps.hdop = _last_rtk_gpgga_data.hdop;
			onboard_gps.vdop = 1.0; //todo, NMEA GPGSA msg
		} else {
			onboard_gps.hdop = onboard_gps.vdop = NAN;
		}

		//velocity
		if (_flag_rtk_odom_valid){
			//ENU to NED
			onboard_gps.ve = _last_rtk_odom_twist_data.linear.x; //unit: m/s
			onboard_gps.vn = _last_rtk_odom_twist_data.linear.y;
			onboard_gps.vd = -_last_rtk_odom_twist_data.linear.z; 
		} else {
			onboard_gps.vn = onboard_gps.ve = onboard_gps.vd = 0;
		}

		//satellite
		if (_flag_rtk_gpgga_valid)
			onboard_gps.satellites_visible = _last_rtk_gpgga_data.num_sats;
		else
			onboard_gps.satellites_visible = 0;
		
		//heading unit: cdeg (Centesimal angle measurement, 0.01deg)
		if (_flag_rtk_quaternion_valid){

			//convert to NED
			auto rpy = ftf::quaternion_to_rpy(
					ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(_last_rtk_odom_quaternion_data.w, _last_rtk_odom_quaternion_data.x, 
					_last_rtk_odom_quaternion_data.y, _last_rtk_odom_quaternion_data.z))));

			float yaw = wrapAngleTo2PiOpenZero(rpy.z()); //(0, 2PI]
			onboard_gps.yaw = uint16_t(yaw*RAD_TO_DEG*100); //cdeg unit

			//data protection
			if (onboard_gps.yaw == 0)
				onboard_gps.yaw = 36000;

			_flag_rtk_quaternion_valid = false; //set to false until new data arrives from rtk odom

		} else {
			onboard_gps.yaw = 0; //set as zero when there is no measurement, see mavlink msg definition
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(onboard_gps);

    }

    void gps_rtk_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {
		_last_rtk_odom_quaternion_data = msg->pose.pose.orientation;
		_last_rtk_odom_twist_data = msg->twist.twist;

		_flag_rtk_odom_valid = true;
		_flag_rtk_quaternion_valid = true;
    }

	void gps_rtk_gpgga_cb(const nmea_msgs::Gpgga::ConstPtr &msg)
    {
		_last_rtk_gpgga_data = *msg;
		_flag_rtk_gpgga_valid = true;
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RAW_INT">mavlink GPS_RAW_INT message</a> into the gps1/raw topic.
	 */
	void handle_gps_raw_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RAW_INT &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRAW>();
		ros_msg->header            = m_uas->synchronized_header("/wgs84", mav_msg.time_usec);
		ros_msg->fix_type          = mav_msg.fix_type;
		ros_msg->lat               = mav_msg.lat;
		ros_msg->lon               = mav_msg.lon;
		ros_msg->alt               = mav_msg.alt;
		ros_msg->eph               = mav_msg.eph;
		ros_msg->epv               = mav_msg.epv;
		ros_msg->vel               = mav_msg.vel;
		ros_msg->cog               = mav_msg.cog;
		ros_msg->satellites_visible = mav_msg.satellites_visible;
		ros_msg->alt_ellipsoid     = mav_msg.alt_ellipsoid;
		ros_msg->h_acc             = mav_msg.h_acc;
		ros_msg->v_acc             = mav_msg.v_acc;
		ros_msg->vel_acc           = mav_msg.vel_acc;
		ros_msg->hdg_acc           = mav_msg.hdg_acc;
		ros_msg->dgps_numch        = UINT8_MAX;	// information not available in GPS_RAW_INT mavlink message
		ros_msg->dgps_age          = UINT32_MAX;// information not available in GPS_RAW_INT mavlink message
        ros_msg->yaw               = mav_msg.yaw;

		gps1_raw_pub.publish(ros_msg);
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS2_RAW">mavlink GPS2_RAW message</a> into the gps2/raw topic.
	 */
	void handle_gps2_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS2_RAW &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRAW>();
		ros_msg->header            = m_uas->synchronized_header("/wgs84", mav_msg.time_usec);
		ros_msg->fix_type          = mav_msg.fix_type;
		ros_msg->lat               = mav_msg.lat;
		ros_msg->lon               = mav_msg.lon;
		ros_msg->alt               = mav_msg.alt;
		ros_msg->eph               = mav_msg.eph;
		ros_msg->epv               = mav_msg.epv;
		ros_msg->vel               = mav_msg.vel;
		ros_msg->cog               = mav_msg.cog;
		ros_msg->satellites_visible = mav_msg.satellites_visible;
		ros_msg->alt_ellipsoid     = INT32_MAX;	// information not available in GPS2_RAW mavlink message
		ros_msg->h_acc             = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->v_acc             = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->vel_acc           = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->hdg_acc           = UINT32_MAX;// information not available in GPS2_RAW mavlink message
		ros_msg->dgps_numch        = mav_msg.dgps_numch;
		ros_msg->dgps_age          = mav_msg.dgps_age;

		gps2_raw_pub.publish(ros_msg);
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RTK">mavlink GPS_RTK message</a> into the gps1/rtk topic.
	 */
	void handle_gps_rtk(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS_RTK &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRTK>();
		switch (static_cast<RTK_BASELINE_COORDINATE_SYSTEM>(mav_msg.baseline_coords_type))
		{
		case RTK_BASELINE_COORDINATE_SYSTEM::ECEF:
			ros_msg->header.frame_id = "earth";
			break;
		case RTK_BASELINE_COORDINATE_SYSTEM::NED:
			ros_msg->header.frame_id = "map";
			break;
		default:
			ROS_ERROR_NAMED("gps_status", "GPS_RTK.baseline_coords_type MAVLink field has unknown \"%d\" value", mav_msg.baseline_coords_type);
		}
		ros_msg->header              = m_uas->synchronized_header(ros_msg->header.frame_id, mav_msg.time_last_baseline_ms * 1000);
		ros_msg->rtk_receiver_id     = mav_msg.rtk_receiver_id;
		ros_msg->wn                  = mav_msg.wn;
		ros_msg->tow                 = mav_msg.tow;
		ros_msg->rtk_health          = mav_msg.rtk_health;
		ros_msg->rtk_rate            = mav_msg.rtk_rate;
		ros_msg->nsats               = mav_msg.nsats;
		ros_msg->baseline_a          = mav_msg.baseline_a_mm;
		ros_msg->baseline_b          = mav_msg.baseline_b_mm;
		ros_msg->baseline_c          = mav_msg.baseline_c_mm;
		ros_msg->accuracy            = mav_msg.accuracy;
		ros_msg->iar_num_hypotheses  = mav_msg.iar_num_hypotheses;

		gps1_rtk_pub.publish(ros_msg);
	}

	/**
	 * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS2_RTK">mavlink GPS2_RTK message</a> into the gps2/rtk topic.
	 */
	void handle_gps2_rtk(const mavlink::mavlink_message_t *msg, mavlink::common::msg::GPS2_RTK &mav_msg) {
		auto ros_msg = boost::make_shared<mavros_msgs::GPSRTK>();
		switch (static_cast<RTK_BASELINE_COORDINATE_SYSTEM>(mav_msg.baseline_coords_type))
		{
		case RTK_BASELINE_COORDINATE_SYSTEM::ECEF:
			ros_msg->header.frame_id = "earth";
			break;
		case RTK_BASELINE_COORDINATE_SYSTEM::NED:
			ros_msg->header.frame_id = "map";
			break;
		default:
			ROS_ERROR_NAMED("gps_status", "GPS_RTK2.baseline_coords_type MAVLink field has unknown \"%d\" value", mav_msg.baseline_coords_type);
		}
		ros_msg->header              = m_uas->synchronized_header(ros_msg->header.frame_id, mav_msg.time_last_baseline_ms * 1000);
		ros_msg->rtk_receiver_id     = mav_msg.rtk_receiver_id;
		ros_msg->wn                  = mav_msg.wn;
		ros_msg->tow                 = mav_msg.tow;
		ros_msg->rtk_health          = mav_msg.rtk_health;
		ros_msg->rtk_rate            = mav_msg.rtk_rate;
		ros_msg->nsats               = mav_msg.nsats;
		ros_msg->baseline_a          = mav_msg.baseline_a_mm;
		ros_msg->baseline_b          = mav_msg.baseline_b_mm;
		ros_msg->baseline_c          = mav_msg.baseline_c_mm;
		ros_msg->accuracy            = mav_msg.accuracy;
		ros_msg->iar_num_hypotheses  = mav_msg.iar_num_hypotheses;

		gps2_rtk_pub.publish(ros_msg);
	}

	//wrap an angle to (0, 2PI]
	double wrapAngleTo2PiOpenZero(double angle) {
		angle = std::fmod(angle, 2.0f*M_PI);
		if (angle <= 0)
			angle += 2.0f*M_PI;
		return angle;
	}

};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::GpsStatusPlugin, mavros::plugin::PluginBase)
