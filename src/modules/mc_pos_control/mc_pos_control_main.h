/*
 * mc_pos_control_main.h
 *
 *  Created on: Apr 9, 2018
 *      Author: lidq
 */

#ifndef SRC_MODULES_MC_POS_CONTROL_MC_POS_CONTROL_MAIN_H_
#define SRC_MODULES_MC_POS_CONTROL_MC_POS_CONTROL_MAIN_H_

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/extctl_sp.h>

#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define SIGMA_SINGLE_OP			0.000001f
#define SIGMA_NORM				0.001f
#define TIMEOUT_EXTCTL			(1000 * 1000)
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl: public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int start();

	bool cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r, const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res);

private:
	
	/** Time in us that direction change condition has to be true for direction change state */
	static constexpr uint64_t DIRECTION_CHANGE_TRIGGER_TIME_US = 100000;

	bool _task_should_exit = false; /**<true if task should exit */
	bool _gear_state_initialized = false; /**<true if the gear state has been initialized */
	bool _reset_pos_sp = true; /**<true if position setpoint needs a reset */
	bool _reset_alt_sp = true; /**<true if altitude setpoint needs a reset */
	bool _do_reset_alt_pos_flag = true; /**< TODO: check if we need this */
	bool _mode_auto = false; /**<true if in auot mode */
	bool _pos_hold_engaged = false; /**<true if hold positon in xy desired */
	bool _alt_hold_engaged = false; /**<true if hold in z desired */
	bool _run_pos_control = true; /**< true if position controller should be used */
	bool _run_alt_control = true; /**<true if altitude controller should be used */
	bool _reset_int_z = true; /**<true if reset integral in z */
	bool _reset_int_xy = true; /**<true if reset integral in xy */
	bool _reset_yaw_sp = true; /**<true if reset yaw setpoint */
	bool _hold_offboard_xy = false; /**<TODO : check if we need this extra hold_offboard flag */
	bool _hold_offboard_z = false;
	bool _in_smooth_takeoff = false; /**<true if takeoff ramp is applied */
	bool _in_landing = false; /**<true if landing descent (only used in auto) */
	bool _lnd_reached_ground = false; /**<true if controller assumes the vehicle has reached the ground after landing */
	bool _triplet_lat_lon_finite = true; /**<true if triplets current is non-finite */
	
	int _control_task; /**< task handle for task */
	orb_advert_t _mavlink_log_pub; /**< mavlink log advert */
	
	int _vehicle_status_sub; /**< vehicle status subscription */
	int _vehicle_land_detected_sub; /**< vehicle land detected subscription */
	int _vehicle_attitude_sub; /**< control state subscription */
	int _control_mode_sub; /**< vehicle control mode subscription */
	int _params_sub; /**< notification of parameter updates */
	int _manual_sub; /**< notification of manual control updates */
	int _local_pos_sub; /**< vehicle local position */
	int _pos_sp_triplet_sub; /**< position setpoint triplet */
	int _home_pos_sub; /**< home position */
	int _extctl_sp_sub; /**< home position */
	
	orb_advert_t _att_sp_pub; /**< attitude setpoint publication */
	orb_advert_t _local_pos_sp_pub; /**< vehicle local position setpoint publication */
	
	orb_id_t _attitude_setpoint_id;

	struct vehicle_status_s _vehicle_status; /**< vehicle status */
	struct vehicle_land_detected_s _vehicle_land_detected; /**< vehicle land detected */
	struct vehicle_attitude_s _att; /**< vehicle attitude */
	struct vehicle_attitude_setpoint_s _att_sp; /**< vehicle attitude setpoint */
	struct manual_control_setpoint_s _manual; /**< r/c channel data */
	struct vehicle_control_mode_s _control_mode; /**< vehicle control mode */
	struct vehicle_local_position_s _local_pos; /**< vehicle local position */
	struct position_setpoint_triplet_s _pos_sp_triplet; /**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s _local_pos_sp; /**< vehicle local position setpoint */
	struct home_position_s _home_pos; /**< home position */
	struct extctl_sp_s _extctl_sp;

	control::BlockParamFloat _manual_thr_min; /**< minimal throttle output when flying in manual mode */
	control::BlockParamFloat _manual_thr_max; /**< maximal throttle output when flying in manual mode */
	control::BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	control::BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	control::BlockParamFloat _acceleration_hor_max; /**<maximum velocity setpoint slewrate for auto & fast manual brake */
	control::BlockParamFloat _acceleration_hor; /**<acceleration for auto and maximum for manual in velocity control mode*/
	control::BlockParamFloat _deceleration_hor_slow; /**< slow velocity setpoint slewrate for manual deceleration*/
	control::BlockParamFloat _acceleration_z_max_up; /** max acceleration up */
	control::BlockParamFloat _acceleration_z_max_down; /** max acceleration down */
	control::BlockParamFloat _cruise_speed_90; /**<speed when angle is 90 degrees between prev-current/current-next*/
	control::BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed*/
	control::BlockParamFloat _nav_rad; /**< radius that is used by navigator that defines when to update triplets */
	control::BlockParamFloat _takeoff_ramp_time; /**< time contant for smooth takeoff ramp */
	control::BlockParamFloat _jerk_hor_max; /**< maximum jerk in manual controlled mode when braking to zero */
	control::BlockParamFloat _jerk_hor_min; /**< minimum jerk in manual controlled mode when braking to zero */
	control::BlockParamFloat _mis_yaw_error; /**< yaw error threshold that is used in mission as update criteria */
	control::BlockParamFloat _acc_auto_xy;
	control::BlockParamFloat _acc_auto_z;
	control::BlockParamFloat _acc_manu_xy;
	control::BlockParamFloat _acc_manu_z;
	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	systemlib::Hysteresis _manual_direction_change_hysteresis;

	math::LowPassFilter2p _filter_manual_pitch;
	math::LowPassFilter2p _filter_manual_roll;

	enum manual_stick_input
	{
		brake,
		direction_change,
		acceleration,
		deceleration
	};

	manual_stick_input _user_intention_xy; /**< defines what the user intends to do derived from the stick input */
	manual_stick_input _user_intention_z; /**< defines what the user intends to do derived from the stick input in z direciton */
	
	struct
	{
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t slow_land_alt1;
		param_t slow_land_alt2;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_tilt_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t alt_mode;
		param_t opt_recover;
		param_t rc_flt_smp_rate;
		param_t rc_flt_cutoff;
	} _params_handles; /**< handles for interesting parameters */
	
	struct
	{
		float thr_min;
		float thr_max;
		float thr_hover;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_tilt_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_max_xy;
		float hold_max_z;
		float vel_max_xy;
		float vel_cruise_xy;
		float vel_max_up;
		float vel_max_down;
		float slow_land_alt1;
		float slow_land_alt2;
		int32_t alt_mode;

		bool opt_recover;

		float rc_flt_smp_rate;
		float rc_flt_cutoff;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
	} _params { };

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	bool _ref_alt_is_global; /** true when the reference altitude is defined in a global reference frame */
	hrt_abstime _ref_timestamp;
	hrt_abstime _last_warn;
	hrt_abstime _last_extctl;

	math::Vector<3> _thrust_int;
	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev; /**< velocity on previous step */
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d; /**< derivative of current velocity */
	math::Vector<3> _curr_pos_sp; /**< current setpoint of the triplets */
	math::Vector<3> _prev_pos_sp; /**< previous setpoint of the triples */
	matrix::Vector2f _stick_input_xy_prev; /**< for manual controlled mode to detect direction change */
	
	math::Matrix<3, 3> _R; /**< rotation matrix from attitude quaternions */
	float _yaw; /**< yaw angle (euler) */
	float _yaw_takeoff; /**< home yaw angle present when vehicle was taking off (euler) */
	float _man_yaw_offset; /**< current yaw offset in manual mode */
	
	float _vel_max_xy; /**< equal to vel_max except in auto mode when close to target */
	float _acceleration_state_dependent_xy; /**< acceleration limit applied in manual mode */
	float _acceleration_state_dependent_z; /**< acceleration limit applied in manual mode in z */
	float _manual_jerk_limit_xy; /**< jerk limit in manual mode dependent on stick input */
	float _manual_jerk_limit_z; /**< jerk limit in manual mode in z */
	float _z_derivative; /**< velocity in z that agrees with position rate */
	
	float _takeoff_vel_limit; /**< velocity limit value which gets ramped up */
	
	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _heading_reset_counter;

	matrix::Dcmf _R_setpoint;

	/**
	 * Update our local parameter cache.
	 */
	int parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void poll_subscriptions();

	float throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 */
	void update_ref();

	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void reset_alt_sp();

	/**
	 * Set position setpoint using manual control
	 */
	void control_manual(float dt);

	void control_non_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void control_auto(float dt);
	void control_auto_vel_sp(float dt);
	void control_auto_stop(float dt);
	void control_auto_extctl(float dt);
	void control_auto_idle(float dt, bool &next_setpoint_valid, math::Vector<3> &next_sp);
	void control_auto_idle_pos_loiter_follow(float dt, bool &next_setpoint_valid, math::Vector<3> &next_sp);
	void control_auto_idle_vel_max(float dt, math::Vector<3> &pos_sp, float total_dist_z, float dist_to_prev_z, float dist_to_current_z);
	void control_auto_idle_min_dis(float dt, math::Vector<3> &pos_sp, math::Vector<3> &next_sp, float yaw_diff, float dist_to_current_z, bool next_setpoint_valid, matrix::Vector2f vec_prev_to_current, matrix::Vector2f vec_pos_to_current, matrix::Vector2f pos_sp_diff);
	void control_auto_idle_velocity(float dt);

	void control_auto_reset_sp();
	/***
	 * check current previous next
	 */
	void control_auto_check();
	void control_auto_check_valid_current(bool &current_setpoint_valid, bool &triplet_updated);
	void control_auto_check_valid_previous(bool &previous_setpoint_valid, bool &triplet_updated, math::Vector<3> &prev_sp);
	void control_auto_check_valid_next(bool &next_setpoint_valid, bool &triplet_updated, math::Vector<3> next_sp);

	void control_position(float dt);
	void calculate_velocity_setpoint(float dt);
	void calculate_thrust_setpoint(float dt);

	void vel_sp_slewrate(float dt);

	void update_velocity_derivative();

	void do_control(float dt);

	void generate_attitude_setpoint(float dt);

	float get_cruising_speed_xy();

	bool in_auto_takeoff();

	float get_vel_close(const matrix::Vector2f &unit_prev_to_current, const matrix::Vector2f &unit_current_to_next);

	void set_manual_acceleration_xy(matrix::Vector2f &stick_input_xy_NED, const float dt);

	void set_manual_acceleration_z(float &max_acc_z, const float stick_input_z_NED, const float dt);

	/**
	 * limit altitude based on several conditions
	 */
	void limit_altitude();

	void warn_rate_limited(const char *str);

	bool manual_wants_takeoff();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void task_main();
};

#endif /* SRC_MODULES_MC_POS_CONTROL_MC_POS_CONTROL_MAIN_H_ */
