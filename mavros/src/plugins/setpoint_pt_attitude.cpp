#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <mavros_msgs/PoseThrottle.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

namespace mavplugin {
/**
 * @brief Setpoint attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointPTAttitudePlugin : public MavRosPlugin {
public:
	SetpointPTAttitudePlugin() :
		sp_nh("~setpoint_attitude"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		pose_sub = sp_nh.subscribe("pt_attitude", 10, &SetpointPTAttitudePlugin::pose_cb, this);
	
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber pose_sub;

	/* -*- low-level send -*- */

	void set_attitude_target(uint32_t time_boot_ms,
			uint8_t type_mask,
			float q[4],
			float roll_rate, float pitch_rate, float yaw_rate,
			float thrust) {
		mavlink_message_t msg;
		mavlink_msg_set_attitude_target_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_boot_ms,
				UAS_PACK_TGT(uas),
				type_mask,
				q,
				roll_rate, pitch_rate, yaw_rate,
				thrust);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send attitude setpoint to FCU attitude controller
	 *
	 * @note ENU frame.
	 */
	void send_attitude_target(const ros::Time &stamp, const Eigen::Affine3d &tr, float throttle) {
		/* Thrust + RPY, also bits numbering started from 1 in docs
		 */
		const uint8_t no_ignore = 0x07;
		float q[4];

		UAS::quaternion_to_mavlink(
				UAS::transform_frame_enu_ned(Eigen::Quaterniond(tr.rotation())),
				q);

		set_attitude_target(stamp.toNSec() / 1000000,
				no_ignore,
				q,
				0.0, 0.0, 0.0,
				throttle);
	}


	/* -*- callbacks -*- */

	void pose_cb(const mavros_msgs::PoseThrottle::ConstPtr &req) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);

		send_attitude_target(req->header.stamp, tr, req->throttle.data);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointPTAttitudePlugin, mavplugin::MavRosPlugin)
