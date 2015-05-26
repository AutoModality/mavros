/*
 * hil_gps.cpp
 *
 *  Created on: May 26, 2015
 *      Author: dan
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <brain_box_msgs/BBPose.h>
#include <brain_box_msgs/BBGlobalPosition.h>

namespace mavplugin {

/**
 * @brief Setpoint attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class HILGPSPlugin : public MavRosPlugin {
public:
	HILGPSPlugin() :
		hg_nh("~hil_gps"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		ROS_DEBUG_NAMED("hil_gps", "hil_gps");
		gps_sub = hg_nh.subscribe("/sensor/gps/global_position", 10, &HILGPSPlugin::global_position_cb, this);
	}

	const std::string get_name() const {
		return "HILGPS";
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	UAS *uas;

	ros::NodeHandle hg_nh;
	ros::Subscriber gps_sub;

	void hil_gps(uint64_t time_usec,
			uint32_t lat, uint32_t lon, uint32_t alt, uint16_t eph, uint16_t epv, uint16_t vel,
			uint16_t vn, uint16_t ve, uint16_t vd, uint16_t cog, uint8_t fix_type, uint8_t satellites_visible)
	{

		mavlink_message_t msg;
		mavlink_msg_hil_gps_pack_chan(UAS_PACK_CHAN(uas), &msg, time_usec, fix_type,
				lat, lon, alt, eph, epv, vel,
				vn, ve, vd, cog, satellites_visible);

		UAS_FCU(uas)->send_message(&msg);
	}

	void global_position_cb(const brain_box_msgs::BBGlobalPosition::ConstPtr &req) {

		hil_gps(req->time_usec, req->lon, req->lat, req->alt,
				req->eph, req->epv, req->vel,
				req->vn, req->ve, req->vd, req->cog,
				req->fix_type, req->satellites_visible);
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::HILGPSPlugin, mavplugin::MavRosPlugin)




