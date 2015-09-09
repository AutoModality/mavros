#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <brain_box_msgs/BBPose.h>

namespace mavplugin {

/**
 * @brief Setpoint attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointBBAttitudePlugin : public MavRosPlugin,
	private TFListenerMixin<SetpointBBAttitudePlugin> {
public:
	SetpointBBAttitudePlugin() :
		sp_nh("~setpoint_bb_attitude"),
		uas(nullptr),
		tf_rate(10.0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// may be used to mimic attitude of an object, a gesture, etc.
		sp_nh.param<std::string>("attitude/frame_id", frame_id, "local_origin");
		sp_nh.param<std::string>("attitude/child_frame_id", child_frame_id, "attitude");
		sp_nh.param("attitude/tf_rate_limit", tf_rate, 10.0);

		ROS_DEBUG_NAMED("attitude", "Setpoint attitude topic type: BBPose");
		att_sub = sp_nh.subscribe("/vstate/pose/setpoint", 10, &SetpointBBAttitudePlugin::pose_cb, this);

		latency_total_pub = sp_nh.advertise<std_msgs::Float64>("/diag/target_track/latency/total", 100);
		latency_pub = sp_nh.advertise<brain_box_msgs::BBLatency>("/diag/target_track/latency", 100);
	}

	const std::string get_name() const {
		return "SetpointBBAttitude";
	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class TFListenerMixin;
	UAS *uas;

	ros::NodeHandle sp_nh;
	ros::Subscriber att_sub;
	ros::Publisher latency_pub;
	ros::Publisher latency_total_pub;

	std::string frame_id;
	std::string child_frame_id;

	double tf_rate;

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
		ROS_INFO_STREAM_NAMED("attitude", "Set Attitude, Q0 =" << q[0] << ", Q1=" << q[1] << ", Q2=" << q[2] << ", Q3=" << q[3]);
		ROS_INFO_NAMED("attitude", "T=%02x", type_mask);

		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- mid-level helpers -*- */

	/**
	 * Send attitude setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_transform(const tf::Transform &transform, const ros::Time &stamp, float throttle) {
		// Thrust + RPY, also bits noumbering started from 1 in docs
		// const uint8_t ignore_all_except_q = (1<<6)|(7<<0);
		const uint8_t no_ignore = 0x07;
		float q[4];

		// ENU->NED, description in #49.
		tf::Quaternion tf_q = transform.getRotation();
		q[0] = tf_q.w();
		q[1] = tf_q.y();
		q[2] = tf_q.x();
		q[3] = -tf_q.z();

		set_attitude_target(stamp.toNSec() / 1000000,
				no_ignore,
				q,
				0.0, 0.0, 0.0,
				throttle);
	}

	/**
	 * Send angular velocity setpoint to FCU attitude controller
	 *
	 * ENU frame.
	 */
	void send_attitude_ang_velocity(const ros::Time &stamp, const float vx, const float vy, const float vz) {
		// Q + Thrust, also bits noumbering started from 1 in docs
		const uint8_t ignore_all_except_rpy = (1<<7)|(1<<6);
		float q[4] = { 1.0, 0.0, 0.0, 0.0 };

		set_attitude_target(stamp.toNSec() / 1000000,
				ignore_all_except_rpy,
				q,
				vy, vx, -vz,
				0.0);
	}

	/* -*- callbacks -*- */

	void pose_cb(const brain_box_msgs::BBPose::ConstPtr &req) {
		tf::Transform transform;
		poseMsgToTF(req->pose_throttle.pose, transform);
		send_attitude_transform(transform, req->header.stamp, req->pose_throttle.throttle.data);

		// send out latency data
	    ros::Time time = ros::Time::now();
		brain_box_msgs::BBLatency latency = req->latency;
	    latency.mavros_stamp = time;
        latency_pub.publish(latency);
		std_msgs::Float64 latency_total;
		latency_total.data = latency.mavros_stamp.toSec() - req->header.stamp.toSec();
        latency_total_pub.publish(latency_total);

	}

	inline bool is_normalized(float throttle, const float min, const float max) {
		if (throttle < min) {
			ROS_WARN_NAMED("attitude", "Not normalized throttle! Thd(%f) < Min(%f)", throttle, min);
			return false;
		}
		else if (throttle > max) {
			ROS_WARN_NAMED("attitude", "Not normalized throttle! Thd(%f) > Max(%f)", throttle, max);
			return false;
		}

		return true;
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointBBAttitudePlugin, mavplugin::MavRosPlugin)

