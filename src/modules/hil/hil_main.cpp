/**
 * @file hil_main.cpp
 * HIL Sim
 *
 * *
 * @author X. QI <qx0148@woozoom.net>
 *
 * 
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <controllib/block/BlockParam.hpp>
#include <controllib/blocks.hpp>
#include <lib/geo/geo.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/hil_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_attitude.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int hil_main(int argc, char *argv[]);

#define PLANE_TYPE 1

class HilSim: public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	HilSim();

	/**
	 * Destructor, also kills the main task
	 */
	~HilSim();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int start();

private:

	bool _task_should_exit; /**< if true, task_main() should exit */
	int _control_task; /**< task handle */

	int _sensors_sub;

	int _params_sub; /**< parameter updates subscription */
	int _actuatorsSub;
	int _actuatorsSub_1;
	int _manulSub;
	int _armed_sub; /**< arming status subscription */

	orb_advert_t _att_pub; /**< vehicle attitude */
	orb_advert_t _ctrl_state_pub; /**< control state */
	orb_advert_t _global_pos_pub; /**< global position */
	orb_advert_t _local_pos_pub; /**< position in local frame */
	orb_advert_t _gps_pub;
	orb_advert_t _airspeed_pub;
	orb_advert_t _hil_status_pub;
	orb_advert_t _mavlink_log_pub;

	struct actuator_controls_s _actuators; /**< actuator controls */
	struct actuator_controls_s _actuators_1; /**< actuator controls */
	struct manual_control_setpoint_s _manul;
	struct hil_status_s _hil_status;
	struct sensor_combined_s _sensors;
	struct actuator_armed_s _armed;

	perf_counter_t _loop_perf; /**< loop performance counter */

	struct
	{
		param_t ref_lat;
		param_t ref_lon;
		param_t ref_alt;
		param_t hil_enabled;
		param_t hil_en_flag;
	} _params_handles; /**< handles for interesting parameters */

	struct
	{
		int ref_lat; /**< yaw control feed-forward */
		int ref_lon;
		int ref_alt;
		int hil_enabled;
		int hil_en_flag;
	} _params;

	struct
	{
		double Long = 0;/*!< ¾­¶ÈÖµ*/
		double Lati = 0;/*!< Î³¶ÈÖµ*/
		double Alti = 0;/*!< ¸ß¶ÈÖµ*/

		/*ËÙ¶ÈÏà¹Ø*/
		double XSpeed = 0;/*!< X-ÖáËÙ¶ÈÖµ*/
		double YSpeed = 0;/*!< Y-ÖáËÙ¶ÈÖµ*/
		double ZSpeed = 0;/*!< Z-ÖáËÙ¶ÈÖµ*/

		/*¼ÓËÙ¶ÈÏà¹Ø*/
		double XAcc = 0;/*!< X-Öá¼ÓËÙ¶ÈÖµ*/
		double YAcc = 0;/*!< Y-Öá¼ÓËÙ¶ÈÖµ*/
		double ZAcc = 0;/*!< Z-Öá¼ÓËÙ¶ÈÖµ*/

		/*½ÇËÙ¶ÈÏà¹Ø*/
		double XAng = 0;
		double YAng = 0;
		double ZAng = 0;

		/*º½×ËÏà¹Ø*/
		double Roll = 0;
		double Pitch = 0;
		double Yaw = 0;

		double Nav_Alarm_Flag = 0;/*< µ¼º½±¨¾¯±êÖ¾*/
		double Gps_Sate_Num = 0; /*!< GPSËÑÐÇÊý*/
		double GpsFixState = 0; /*!< GPSËÑÐÇ¶¨Î»Ä£Ê½*/
		double SenUptFlag = 0; /*¸÷´«¸ÐÆ÷¸üÐÂ±êÖ¾Î»*/

	} gNaviData;
	float ILSPrePosiX = 0.0f;
	float ILSPrePosiY = 0.0f;
	float ILSPrePosiZ = 0.0f;

	struct quat
	{
		float w;
		float x;
		float y;
		float z;
	};

	//¿ØÖÆÁ¿¼Ä´æ
	float RollCtrlInArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float RollGyroOutArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float PitchCtrlInArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float PitchGyroOutArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float YawCtrlInArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float YawGyroOutArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float AltiCtrlInArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float AltiDwOutArray[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	float RollCtrlInArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float RollGyroOutArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float PitchCtrlInArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float PitchGyroOutArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float YawCtrlInArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float YawGyroOutArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float AltiCtrlInArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float AltiDwOutArray_F[10] =
	{
	    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	double ref_lat;
	double ref_lon;
	double ref_alt;

	float _delta_q_reset[4] =
	{
	    0 };

	//float testvar = 0;

	/**
	 * Update our local parameter cache.
	 */
	int parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void parameter_update_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void task_main();

	/**
	 * Dynamics for HIL
	 */
	void HIL_Dynamics_Cal();

	void Angle2Q(float xyz[3], struct quat &q);

	void HIL_TransformationFromInertialToBody(double Rx, double Ry, double Rz, double Ix, double Iy, double Iz, double *Bx, double *By, double *Bz);

	float InnerLoopSimulationModelCal(float t_In, float *Para_In, float *Pre_In, int InOrder, float *Para_Out, float *Pre_Out, int OutOrder);

	void InnerLoopSimulationDataUpdate(float t_In, float *Pre_In, int InOrder);

	double AngleMaxPI_Adj(double t_In);

};

namespace hil
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

HilSim *g_control;
}

HilSim::HilSim() :
		    SuperBlock(NULL, "MC"),
		    _task_should_exit(false),
		    _control_task(-1),

		    /* subscriptions */
		    //_ctrl_state_sub(-1),
		    _sensors_sub(-1),
		    _params_sub(-1),
		    _actuatorsSub(-1),
		    _actuatorsSub_1(-1),
		    _manulSub(-1),
		    _armed_sub(-1),

		    /* publications */
		    _att_pub(nullptr),
		    _ctrl_state_pub(nullptr),
		    _global_pos_pub(nullptr),
		    _local_pos_pub(nullptr),
		    _gps_pub(nullptr),
		    _airspeed_pub(nullptr),
		    _hil_status_pub(nullptr),
		    _mavlink_log_pub(nullptr),

		    /* performance counters */
		    _loop_perf(perf_alloc(PC_ELAPSED, "hil"))

{
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_actuators_1, 0, sizeof(_actuators_1));
	memset(&_manul, 0, sizeof(_manul));
	memset(&_hil_status, 0, sizeof(_hil_status));
	_hil_status.flag_hil_enabled = false;
	memset(&_armed, 0, sizeof(_armed));

	ref_lat = 0.0;
	ref_lon = 0.0;
	ref_alt = 0.0;

	_params.ref_lat = 0;
	_params.ref_lon = 0;
	_params.ref_alt = 0;
	_params.hil_enabled = 0;
	//_params.hil_en_flag = 0;

	_params_handles.ref_lat = param_find("HIL_REF_LAT");
	_params_handles.ref_lon = param_find("HIL_REF_LON");
	_params_handles.ref_alt = param_find("HIL_REF_ALT");
	_params_handles.hil_enabled = param_find("HIL_ENABLED");
	_params_handles.hil_en_flag = param_find("HIL_EN_FLAG");

	parameters_update();
}

HilSim::~HilSim()
{
	if (_control_task != -1)
	{
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do
		{
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50)
			{
				px4_task_delete(_control_task);
				break;
			}
		}
		while (_control_task != -1);
	}

	hil::g_control = nullptr;
}

int HilSim::parameters_update()
{
	updateParams();

	param_get(_params_handles.ref_lat, &_params.ref_lat);
	param_get(_params_handles.ref_lon, &_params.ref_lon);
	param_get(_params_handles.ref_alt, &_params.ref_alt);
	param_get(_params_handles.hil_enabled, &_params.hil_enabled);
	param_get(_params_handles.hil_en_flag, &_params.hil_en_flag);

	_params.hil_enabled = 1;
	_params.hil_en_flag = 1;

	return OK;
}

void HilSim::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated)
	{
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void HilSim::task_main_trampoline(int argc, char *argv[])
{
	hil::g_control->task_main();
}

void HilSim::task_main()
{
	/*
	 * do subscriptions
	 */
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_actuatorsSub = orb_subscribe(ORB_ID(actuator_controls_0));
	_actuatorsSub_1 = orb_subscribe(ORB_ID(actuator_controls_1));
	_manulSub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* initialize parameters cache */
	parameters_update();

	// reset
	int para_reset = 0;
	param_set(param_find("CBRK_USB_CHK"), &para_reset);

	//int hil_disabled_flag = 0;
	//param_set(param_find("HIL_EN_FLAG"), &hil_disabled_flag);

	//mavlink_and_console_log_info(_mavlink_fd,"[The-One] HIL thread started");
	warnx("HIL ok");

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	//fds[0].fd = _ctrl_state_sub;
	fds[0].fd = _sensors_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit)
	{
		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
		{
			continue;
		}
		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0)
		{
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);
		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN)
		{
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f)
			{
				dt = 0.002f;

			}
			else if (dt > 0.02f)
			{
				dt = 0.02f;
			}
			setDt(dt);

			orb_copy(ORB_ID(sensor_combined), _sensors_sub, &_sensors); //T_T
			parameters_update();

			if (_hil_status.flag_hil_enabled)
			{
				/* check for updates in other topics */
				parameter_update_poll();

				bool updated;
				orb_check(_armed_sub, &updated);

				if (updated)
				{
					orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
				}
				//if (_armed.armed) {
				orb_copy(ORB_ID(actuator_controls_0), _actuatorsSub, &_actuators);
				orb_copy(ORB_ID(actuator_controls_1), _actuatorsSub_1, &_actuators_1); // DO NOT use actuator_control_1 uorb topic?
				orb_copy(ORB_ID(manual_control_setpoint), _manulSub, &_manul);
				HIL_Dynamics_Cal(); //\BC\C6\CB㲢\B7\A2\B2\BC\B6\AF\C1\A6ѧ\BF\D8\D6ƽ\E1\B9\FB
				//}
			}
			if (_params.hil_enabled)
			{
				_hil_status.timestamp = hrt_absolute_time();
				_hil_status.flag_hil_enabled = true;

				int hil_disabled = 0;
				param_set(param_find("HIL_ENABLED"), &hil_disabled); //modified by GCS

				int usb_break_for_arm = 197848;
				param_set(param_find("CBRK_USB_CHK"), &usb_break_for_arm);

				int hil_enabled_flag = 1;
				param_set(param_find("HIL_EN_FLAG"), &hil_enabled_flag); // used to control other threads

				param_get(param_find("HIL_EN_FLAG"), &_params.hil_en_flag);

				_hil_status.hil_flag_sd = _hil_status.flag_hil_enabled + _params.hil_en_flag * 2;

				ref_lat = _params.ref_lat * 1e-7 * 3.14159265 / 180.0;
				ref_lon = _params.ref_lon * 1e-7 * 3.14159265 / 180.0;
				ref_alt = _params.ref_alt * 1e-3;

				if (_hil_status_pub != nullptr)
				{
					/* HIL status */
					orb_publish(ORB_ID(hil_status), _hil_status_pub, &_hil_status);

				}
				else
				{
					/* advertise and publish */
					_hil_status_pub = orb_advertise(ORB_ID(hil_status), &_hil_status);
				}
				//mavlink_and_console_log_info(&_mavlink_log_pub, "[The-One] HIL started");
			}

		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
	return;
}

int HilSim::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("hil", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 3500, (px4_main_t) &HilSim::task_main_trampoline, nullptr);

	if (_control_task < 0)
	{
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int hil_main(int argc, char *argv[])
{
	if (argc < 2)
	{
		warnx("usage: hil {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start"))
	{

		if (hil::g_control != nullptr)
		{
			warnx("already running");
			return 1;
		}

		hil::g_control = new HilSim;

		if (hil::g_control == nullptr)
		{
			warnx("alloc failed");
			return 1;
		}

		if (OK != hil::g_control->start())
		{
			delete hil::g_control;
			hil::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop"))
	{
		if (hil::g_control == nullptr)
		{
			warnx("not running");
			return 1;
		}

		delete hil::g_control;
		hil::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status"))
	{
		if (hil::g_control)
		{
			warnx("running");
			return 0;

		}
		else
		{
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}

void HilSim::HIL_Dynamics_Cal(void)
{

	static hrt_abstime last_time = 0;
	double dt = (hrt_absolute_time() - last_time) / 1000000.0f;
	if (dt < 0.02)
	{
		return;
	}
	else if (dt >= 0.02)
	{
		dt = 0.02;
		last_time = hrt_absolute_time();
	}

	//double LatiRef = 0.728157412;  /*\CB\F9\C4\CF\C7\F8\B2ο\BC\B5\E3*/
	//double LongRef = 2.15488069;
	double LatiRef = ref_lat; //0.72689248238;
	double LongRef = ref_lon; //2.1545112196;
	double AltRef = ref_alt; //100.0;

#if PLANE_TYPE == 1

#define roll_p 0.7
#define pitch_p 0.7
#define yaw_p 0.25
#define alt_p 1.0

	//\B6\A8\D2\E5ֱ\C9\FD\BB\FA\B6\AF\C1\A6ѧ\B2ο\BCģ\D0\CD
	float RollModelNum[10] =
	{
	    0, 0.5 * 6 * roll_p, 0, 0, 0, 0, 0, 0, 0, 0 };
	float RollModelDen[10] =
	{
	    -0.5 * 1.8 * roll_p, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float PitchModelNum[10] =
	{
	    0, 0.5 * 6 * pitch_p, 0, 0, 0, 0, 0, 0, 0, 0 };
	float PitchModelDen[10] =
	{
	    -0.5 * 1.8* pitch_p, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float YawModelNum[10] =
	{
	    0, 0.1 * 6 * yaw_p, 0, 0, 0, 0, 0, 0, 0, 0 };
	float YawModelDen[10] =
	{
	    -0.881911378298176 * 1.1 * yaw_p, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float AltiModelNum[10] =
	{
	    0, 0.1813 * 8 * alt_p, 0, 0, 0, 0, 0, 0, 0, 0 };
	float AltiModelDen[10] =
	{
	    -0.8187 * 0.7* alt_p, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif
#if PLANE_TYPE == 2

	//\B6\A8\D2\E5\B9̶\A8\D2\ED\B6\AF\C1\A6ѧ\B2ο\BCģ\D0\CD
	float RollModelNum[10] =
	{	0,0.03,0,0,0,0,0,0,0,0};
	float RollModelDen[10] =
	{	-0.8,0,0,0,0,0,0,0,0,0};
	float PitchModelNum[10] =
	{	0,0.03,0,0,0,0,0,0,0,0};
	float PitchModelDen[10] =
	{	-0.9,0,0,0,0,0,0,0,0,0};
	float YawModelNum[10] =
	{	0,0.05,0,0,0,0,0,0,0,0};
	float YawModelDen[10] =
	{	-0.95,0,0,0,0,0,0,0,0,0};
	float AltiModelNum[10] =
	{	0,0.15,0,0,0,0,0,0,0,0};
	float AltiModelDen[10] =
	{	-0.9,0,0,0,0,0,0,0,0,0};

#endif

#if PLANE_TYPE == 3
	//\C7\E3ת\D0\FD\D2\ED\B6\AF\C1\A6ѧ\B9\B9\BD\A8\A3\AC\B7\D6Ϊֱ\C9\FD\BB\FA\BA͹̶\A8\D2\ED\C1\BD\B2\BF\B7ֶ\E6\C3\E6\BF\D8\D6\C6ģ\D0Ͳ\CE\CA\FD
	//\B6\A8\D2\E5ֱ\C9\FD\BB\FA\B6\AF\C1\A6ѧ\B2ο\BCģ\D0Ͳ\CE\CA\FD
	float RollModelNum_H[10] =
	{	0,0.007690850687394,0.007499875959545,0,0,0,0,0,0,0};
	float RollModelDen_H[10] =
	{	-1.912183383802675,0.927374110449614,0,0,0,0,0,0,0,0};
	float PitchModelNum_H[10] =
	{	0,0.007690850687394,0.007499875959545,0,0,0,0,0,0,0};
	float PitchModelDen_H[10] =
	{	-1.912183383802675,0.927374110449614,0,0,0,0,0,0,0,0};
	float YawModelNum_H[10] =
	{	0,0.1138088621701824,0,0,0,0,0,0,0,0};
	float YawModelDen_H[10] =
	{	-0.881911378298176,0,0,0,0,0,0,0,0,0};

	//\B6\A8\D2\E5\B9̶\A8\D2\ED\B6\AF\C1\A6ѧ\B2ο\BCģ\D0Ͳ\CE\CA\FD
	float RollModelNum_F[10] =
	{	0,0.03,0,0,0,0,0,0,0,0};
	float RollModelDen_F[10] =
	{	-0.8,0,0,0,0,0,0,0,0,0};
	float PitchModelNum_F[10] =
	{	0,0.03,0,0,0,0,0,0,0,0};
	float PitchModelDen_F[10] =
	{	-0.9,0,0,0,0,0,0,0,0,0};
	float YawModelNum_F[10] =
	{	0,0.05,0,0,0,0,0,0,0,0};
	float YawModelDen_F[10] =
	{	-0.95,0,0,0,0,0,0,0,0,0};

	//\B6\AF\C1\A6\CA\E4\B3\F6\C2\DD\D0\FD\BD\B0\B2ο\BCģ\D0Ͳ\CE\CA\FD
	float AltiModelNum_F[10] =
	{	0,0.15,0,0,0,0,0,0,0,0};
	float AltiModelDen_F[10] =
	{	-0.9,0,0,0,0,0,0,0,0,0};

#endif

	//static float ThrottleCtrlInArray[10] = {0,0,0,0,0,0,0,0,0,0};
	//static float ThrottleRpmOutArray[10] = {0,0,0,0,0,0,0,0,0,0};

	double Lati, Long;
	double p = 0, q = 0, r = 0, phi = 0, theta = 0, pusi = 0, agx = 0, agy = 0, agz = 0, vgx = 0, vgy = 0, vgz = 0, x = 0, y = 0, z = 0;
	double phi0 = 0, theta0 = 0, pusi0 = 0, vgx0 = 0, vgy0 = 0, vgz0 = 0, x0 = 0, y0 = 0, z0 = 0;
	//double tCtrlRoll=0;
	//double tCtrlPitch=0;
	double vbx = 0, vby = 0, vbz = 0;

#if PLANE_TYPE ==1 || PLANE_TYPE ==2
	float ulat, ulon, uped, ucol;
	if (_armed.armed)
	{
		ulat = _actuators.control[0];
		ulon = _actuators.control[1];
		uped = _actuators.control[2];
		ucol = _actuators.control[3];
	}
	else
	{
		ulat = 0.0f;
		ulon = 0.0f;
		uped = 0.0f;
		ucol = 0.0f;
	}

	warnx("%+.4f %+.4f %+.4f %+.4f", (double)ulat, (double)ulon, (double)uped, (double)ucol);
	// ulat = _manul.y;
	// ulon = -_manul.x;
	// uped = _manul.r;
	// ucol = _manul.z;

#endif

#if PLANE_TYPE ==3
	//\B9̶\A8\D2\ED\BF\D8\D6\C6\C1\BF
	float ulat_F, ulon_F, uped_F;
	ulat_F = _actuators_1.control[0];
	ulon_F = _actuators_1.control[1];
	uped_F = _actuators_1.control[2];

	//ֱ\C9\FD\BB\FA\BF\D8\D6\C6\C1\BF
	float ulat_H, ulon_H, uped_H;
	ulat_H = _actuators.control[0];
	ulon_H = _actuators.control[1];
	uped_H = _actuators.control[2];

	//\C7\E3ת\BF\D8\D6\C6\C1\BF
	float u_tilt = _actuators_1.control[4];

	//\D3\CD\C3Ż\F2\C2ݾ\E0\BF\D8\D6\C6\C1\BF
	float ucol = _actuators.control[3];
#endif

	phi0 = gNaviData.Roll;
	theta0 = gNaviData.Pitch;
	pusi0 = gNaviData.Yaw;
	vgx0 = gNaviData.XSpeed;
	vgy0 = gNaviData.YSpeed;
	vgz0 = gNaviData.ZSpeed;
	HIL_TransformationFromInertialToBody(phi0, theta0, pusi0, vgx0, vgy0, vgz0, &vbx, &vby, &vbz);
	x0 = ILSPrePosiX;
	y0 = ILSPrePosiY;
	z0 = ILSPrePosiZ;
#if PLANE_TYPE == 1

	double dw;
	//\BC\C6\CB\E3\BD\C7\CB\D9\C2\CA
	p = -0.00 * vby * fabs(vby) * fabs(phi0) + (double) InnerLoopSimulationModelCal(ulat, RollModelNum, RollCtrlInArray, 2, RollModelDen, RollGyroOutArray, 1);
	q = 0.00 * vbx * fabs(vbx) * fabs(theta0) + (double) InnerLoopSimulationModelCal(ulon, PitchModelNum, PitchCtrlInArray, 2, PitchModelDen, PitchGyroOutArray, 1);
	r = -0.0000 * vby * fabs(vby) - 0.0000 * ((double) ucol) + (double) InnerLoopSimulationModelCal(2.0f * uped, YawModelNum, YawCtrlInArray, 2, YawModelDen, YawGyroOutArray, 1);
	dw = (double) InnerLoopSimulationModelCal(-6.0f * (ucol - 0.5f), AltiModelNum, AltiCtrlInArray, 2, AltiModelDen, AltiDwOutArray, 1);

	phi = phi0 + ((q * sin(phi0) + r * cos(phi0)) * tan(theta0) + p) * dt;
	theta = theta0 + (q * cos(phi0) - r * sin(phi0)) * dt;
	pusi = pusi0 + (q * sin(phi0) + r * cos(phi0)) / cos(theta0) * dt;
	phi = AngleMaxPI_Adj(phi);
	theta = AngleMaxPI_Adj(theta);
	pusi = AngleMaxPI_Adj(pusi);

	agx = (sin(phi) * sin(pusi) + cos(phi) * sin(theta) * cos(pusi)) * (dw - 9.8) - 0.01 * vgx0 * fabs(vgx0);
	agy = (-sin(phi) * cos(pusi) + cos(phi) * sin(theta) * sin(pusi)) * (dw - 9.8) - 0.01 * vgy0 * fabs(vgy0);
	agz = ((dw) * cos(phi) * cos(theta)) - 0.02 * sqrt(vbx * vbx + vby * vby) - 0.07 * vgz0 * fabs(vgz0);
	/*0.5*sqrt(vbx*vbx+vby*vby)*/
	/*
	 testvar++;
	 if (testvar > 100.0f){
	 testvar = 0;
	 //if (z<0) {
	 mavlink_and_console_log_info(_mavlink_fd,"xyz: %3.2f, %3.2f, %3.2f, %3.2f", vbx, vby, vbz, dw);
	 //}
	 }
	 */

#endif
#if PLANE_TYPE == 2

	float Tp, Tq, Tr, Fen;
	float kp = 0.002f, kq = 0.002f, kr = 0.002f; //\B6\E6Ч\BDǼ\D3\CBٶ\C8\D4\F6\D2\E6ϵ\CA\FD,\CF\E0\B6\D4\D3\DAvb
	float kws = 1.0f + 0.3f * _actuators.control[4];//\BB\FA\D2\ED\C9\FD\C1\A6\BC\D3\CBٶ\C8ϵ\CA\FD\A3\AC\CF\E0\B6\D4\D3\DAvb_z\A3\AC\B2\BB\BF\BC\C2ǻ\FA\D2\ED\C1\BD\B8\F6\B2\E0\C3\E6\D7\E8\C1\A6, ͬʱ\BF\BC\C2ǽ\F3\D2\EDʡ\C1\A6\D7\F7\D3\C3
	float kfx = 0.003f + 0.003f * _actuators.control[6], kfy = 0.01f, kfz = 0.01f;//\BB\FA\C9\ED\BC\D3\CBٶ\C8\D7\E8\C4\E1ϵ\CA\FD\A3\AC\CF\E0\B6\D4\D3\DAvb\A3\ACͬʱ\BF\BC\C2\C7ɲ\B3\B5\B0\E5ǰ\CF\F2\D7\E8\C4\E1\D7\F7\D3\C3
	double vbx_l, vby_l, vbz_l;//\B1\BE\CC\E5ˮƽ\CBٶ\C8
	float abz_w;//\BB\FA\D2\ED\B1\BE\CC\E5z\D6\E1\BC\D3\CBٶ\C8
	float afbx, afby, afbz;//\BB\FA\CC屾\CC\E5\BC\D3\CBٶ\C8

	HIL_TransformationFromInertialToBody(0.0,0.0,pusi0,vgx0,vgy0,vgz0,&vbx_l,&vby_l,&vbz_l);

	Tp = (double)kp* fabs(vbx) * vbx * (double)ulat;//\BC\C6\CB\E3\CBٶȴ\F8\C0\B4\B5Ķ\E6Ч\D4\F6\D2\E6
	Tq = (double)kq* fabs(vbx) * vbx * (double)ulon;
	Tr = (double)kr* fabs(vbx) * vbx * (double)uped;

	p = (double)InnerLoopSimulationModelCal(Tp,RollModelNum,RollCtrlInArray,2,RollModelDen,RollGyroOutArray,1);
	q = (double)InnerLoopSimulationModelCal(Tq,PitchModelNum,PitchCtrlInArray,2,PitchModelDen,PitchGyroOutArray,1);
	r = (double)InnerLoopSimulationModelCal(Tr,YawModelNum,YawCtrlInArray,2,YawModelDen,YawGyroOutArray,1);

	Fen = (double)InnerLoopSimulationModelCal(10.0f * ucol,AltiModelNum,AltiCtrlInArray,2,AltiModelDen,AltiDwOutArray,1);

	if (fabs(theta0 - 3.1415926 / 2.0) < 0.00001)
	{
		theta0 = 3.1415926 / 2.0 + theta0 / fabs(theta0) * 0.00001;
	}

	phi = phi0 + ((q * sin(phi0) + r * cos(phi0)) * tan(theta0) + p) * dt;
	theta = theta0 + (q * cos(phi0) - r*sin(phi0)) *dt;
	pusi = pusi0 + (q * sin(phi0)+r * cos(phi0))/cos(theta0) * dt;

	phi = AngleMaxPI_Adj(phi);
	theta = AngleMaxPI_Adj(theta);
	pusi = AngleMaxPI_Adj(pusi);

	abz_w = -(double)kws * fabs(vbz) * vbz;
	afbx = -(double)kfx * fabs(vbx) * vbx;
	afby = -(double)kfy * fabs(vby) * vby;
	afbz = -(double)kfz * fabs(vbz) * vbz;

	math::Matrix<3, 3> R_btog;
	R_btog.from_euler(phi, theta, pusi);
	math::Vector<3> ag_p;
	math::Vector<3> ab_p(afbx + Fen, afby, abz_w + afbz);
	ag_p = R_btog * ab_p;

	agx = (double)ag_p(0);
	agy = (double)ag_p(1);
	agz = (double)ag_p(2) + 9.8;

#endif

#if PLANE_TYPE == 3

	float Tp, Tq, Tr, Fen, Tilt_angle;
	float Tp_F, Tq_F, Tr_F; //\B9̶\A8\D2\ED\C8\FD\D6\E1Ť\BE\D8
	float Tp_H, Tq_H, Tr_H;//ֱ\C9\FD\BB\FA\C8\FD\D6\E1Ť\BE\D8
	float tilt_L = 1.0;//ǰ\BA\F3\C2\DD\D0\FD\BD\B0\BE\E0\C0\EB
	float kp = 0.002f, kq = 0.002f, kr = 0.002f;//\B6\E6Ч\BDǼ\D3\CBٶ\C8\D4\F6\D2\E6ϵ\CA\FD,\CF\E0\B6\D4\D3\DAvb
	float kws = 0.8f + 0.2f * _actuators_1.control[4];//\BB\FA\D2\ED\C9\FD\C1\A6\BC\D3\CBٶ\C8ϵ\CA\FD\A3\AC\CF\E0\B6\D4\D3\DAvb_z\A3\AC\B2\BB\BF\BC\C2ǻ\FA\D2\ED\C1\BD\B8\F6\B2\E0\C3\E6\D7\E8\C1\A6, ͬʱ\BF\BC\C2ǽ\F3\D2\EDʡ\C1\A6\D7\F7\D3\C3
	float kfx = 0.003f + 0.01f * _actuators_1.control[6], kfy = 0.01f, kfz = 0.01f;//\BB\FA\C9\ED\BC\D3\CBٶ\C8\D7\E8\C4\E1ϵ\CA\FD\A3\AC\CF\E0\B6\D4\D3\DAvb\A3\ACͬʱ\BF\BC\C2\C7ɲ\B3\B5\B0\E5ǰ\CF\F2\D7\E8\C4\E1\D7\F7\D3\C3
	double vbx_l, vby_l, vbz_l;//\B1\BE\CC\E5ˮƽ\CBٶ\C8
	float abz_w;//\BB\FA\D2\ED\B1\BE\CC\E5z\D6\E1\BC\D3\CBٶ\C8
	float afbx, afby, afbz;//\BB\FA\CC屾\CC\E5\BC\D3\CBٶ\C8

	HIL_TransformationFromInertialToBody(0.0,0.0,pusi0,vgx0,vgy0,vgz0,&vbx_l,&vby_l,&vbz_l);

//\BC\C6\CB㱾\CC\E5\D7\F8\B1\EAϵ\CF¹̶\A8\D2\ED\B6\E6\C3\E6\B2\D9\D7\DDŤ\BEر\EA׼ֵ
	Tp_F = (double)kp* fabs(vbx) * vbx * (double)ulat_F;//\BC\C6\CB\E3\CBٶȴ\F8\C0\B4\B5Ķ\E6Ч\D4\F6\D2\E6
	Tq_F = (double)kq* fabs(vbx) * vbx * (double)ulon_F;
	Tr_F = (double)kr* fabs(vbx) * vbx * (double)uped_F;

//\C8\CFΪת\B2\D5\D0\FDת\B6\E6\BB\FA\CBٶȿ죬\BA\F6\C2\D4ת\B2\D5ת\B6\AF\B6\AF\C1\A6ѧ
	Tilt_angle = u_tilt * 3.1415926f / 2.0f;

//\BC\C6\CB\E3ת\B2\D5\D7\F8\B1\EAϵ\CF\C2ֱ\C9\FD\BB\FA\C8\FD\D6\E1\B2\D9\D7\DDŤ\BEر\EA׼ֵ
	Tp_H = (double)InnerLoopSimulationModelCal(ulat_H,RollModelNum_H,RollCtrlInArray,3,RollModelDen_H,RollGyroOutArray,2);
	Tq_H = (double)InnerLoopSimulationModelCal((double)ulon_H * (double)tilt_L * cos((double)Tilt_angle),PitchModelNum_H,PitchCtrlInArray,3,PitchModelDen_H,PitchGyroOutArray,2);
	Tr_H = (double)InnerLoopSimulationModelCal(2.0f * uped_H,YawModelNum_H,YawCtrlInArray,2,YawModelDen_H,YawGyroOutArray,1);

//\BC\C6\CB㱾\CC\E5\D7\F8\B1\EAϵ\CF\C2\C2\DD\D0\FD\BD\B0Ť\BE\D8
	math::Matrix<3, 3> R_tilt_to_body;
	R_tilt_to_body.from_euler(0.0f, -Tilt_angle, 0.0f);
	math::Vector<3> Tg;
	math::Vector<3> Tb(Tp_H, Tq_H, Tr_H);
	Tg = R_tilt_to_body * Tb;

	Tp = Tp_F;
	Tq = Tq_F;
	Tr = Tr_F;

//\BC\C6\CB\E3Ť\BEز\FA\C9\FA\B5ĽǼ\D3\CBٶȵ\FC\B4\FA\B5õ\BD\BD\C7\CB\D9\C2\CA
	p = (double)Tg(0) + (double)InnerLoopSimulationModelCal(Tp,RollModelNum_F,RollCtrlInArray_F,2,RollModelDen_F,RollGyroOutArray_F,1);
	q = (double)Tg(1) + (double)InnerLoopSimulationModelCal(Tq,PitchModelNum_F,PitchCtrlInArray_F,2,PitchModelDen_F,PitchGyroOutArray_F,1);
	r = (double)Tg(2) + (double)InnerLoopSimulationModelCal(Tr,YawModelNum_F,YawCtrlInArray_F,2,YawModelDen_F,YawGyroOutArray_F,1);

//\BC\C6\CB\E3\C2\DD\D0\FD\BD\B0\C1\A6
	Fen = (double)InnerLoopSimulationModelCal(15.0f * ucol,AltiModelNum_F,AltiCtrlInArray,2,AltiModelDen_F,AltiDwOutArray,1);

	if (fabs(theta0 - 3.1415926 / 2.0) < 0.00001)
	{
		theta0 = 3.1415926 / 2.0 + theta0 / fabs(theta0) * 0.00001;
	}

	phi = phi0 + ((q * sin(phi0) + r * cos(phi0)) * tan(theta0) + p) * dt;
	theta = theta0 + (q * cos(phi0) - r*sin(phi0)) *dt;
	pusi = pusi0 + (q * sin(phi0)+r * cos(phi0))/cos(theta0) * dt;

	phi = AngleMaxPI_Adj(phi);
	theta = AngleMaxPI_Adj(theta);
	pusi = AngleMaxPI_Adj(pusi);

	abz_w = -(double)kws * fabs(vbz) * vbz;
	afbx = -(double)kfx * fabs(vbx) * vbx;
	afby = -(double)kfy * fabs(vby) * vby;
	afbz = -(double)kfz * fabs(vbz) * vbz;

	math::Matrix<3, 3> R_btog;
	R_btog.from_euler(phi, theta, pusi);
	math::Vector<3> ag_p;
	math::Vector<3> ab_p((double)afbx + (double)Fen * sin((double)Tilt_angle), afby, (double)abz_w + (double)afbz - (double)Fen * cos((double)Tilt_angle));
	ag_p = R_btog * ab_p;

	agx = (double)ag_p(0);
	agy = (double)ag_p(1);
	agz = (double)ag_p(2) + 9.8;

	//   mavlink_log_info(_mavlink_fd, "acc_z = %d, %d",(int)(ag_p(2)*100.0f), (int)(agz * 100) );

#endif

	vgx = vgx0 + agx * dt;
	vgy = vgy0 + agy * dt;
	vgz = vgz0 + agz * dt;

	x = (x0 + vgx * dt * 1);
	y = (y0 + vgy * dt * 1);
	z = (z0 + vgz * dt * 1);

	//int sim_hil_en = 0;
	//param_get(param_find("PE_SIM_HIL"), &sim_hil_en);
	int sim_hil_en = 100;

#if PLANE_TYPE ==1
	if (z > 0)
	{
		p = 0;
		q = 0;
		r = 0;
		phi = 0;
		theta = 0;
		pusi = 0;
		agx = 0;
		agy = 0;
		agz = 0;
		vgx = 0;
		vgy = 0;
		vgz = 0;
		x = 0;
		y = 0;
		z = 0;
		ILSPrePosiX = 0.0f;
		ILSPrePosiY = 0.0f;
		ILSPrePosiZ = 0.0f;

		for (int i = 0; i < 10; i++)
		{
			RollCtrlInArray[i] = 0.0f;
			RollGyroOutArray[i] = 0.0f;
			PitchCtrlInArray[i] = 0.0f;
			PitchGyroOutArray[i] = 0.0f;
			YawCtrlInArray[i] = 0.0f;
			YawGyroOutArray[i] = 0.0f;
			AltiCtrlInArray[i] = 0.0f;
			AltiDwOutArray[i] = 0.0f;
		}
	}

#endif
#if PLANE_TYPE==2
	if(z > 0)
	{
		p = 0;
		if (q <0)
		{
			q = 0;
		}
		phi = 0;
		if (theta < 0)
		{
			theta = 0;
		}

		if (agz > 0) agz = 0;
		if (vgz > 0) vgz = 0;
		z = 0;

		for (int i = 0; i < 10; i++)
		{
			RollCtrlInArray[i] = 0.0f;
			RollGyroOutArray[i] = 0.0f;
		}
	}
#endif
#if PLANE_TYPE ==3
	if(z > 0)
	{
		p = 0;
		q = 0;
		r = 0;
		phi = 0;
		theta = 0;
		pusi = 0;
		agx = 0;
		agy = 0;
		agz = 0;
		vgx = 0;
		vgy = 0;
		vgz = 0;
		x = 0;
		y = 0;
		z = 0;
		ILSPrePosiX = 0.0f;
		ILSPrePosiY = 0.0f;
		ILSPrePosiZ = 0.0f;

		for (int i = 0; i < 10; i++)
		{
			RollCtrlInArray[i] = 0.0f;
			RollGyroOutArray[i] = 0.0f;
			PitchCtrlInArray[i] = 0.0f;
			PitchGyroOutArray[i] = 0.0f;
			YawCtrlInArray[i] = 0.0f;
			YawGyroOutArray[i] = 0.0f;
			RollCtrlInArray_F[i] = 0.0f;
			RollGyroOutArray_F[i] = 0.0f;
			PitchCtrlInArray_F[i] = 0.0f;
			PitchGyroOutArray_F[i] = 0.0f;
			YawCtrlInArray_F[i] = 0.0f;
			YawGyroOutArray_F[i] = 0.0f;
		}

	}
#endif

	if (sim_hil_en < 1)
	{
		p = 0;
		q = 0;
		r = 0;
		phi = 0;
		theta = 0;
		pusi = 0;
		agx = 0;
		agy = 0;
		agz = 0;
		vgx = 0;
		vgy = 0;
		vgz = 0;
		x = 0;
		y = 0;
		z = 0;
		ILSPrePosiX = 0.0f;
		ILSPrePosiY = 0.0f;
		ILSPrePosiZ = 0.0f;
		for (int i = 0; i < 10; i++)
		{
			RollCtrlInArray[i] = 0.0f;
			RollGyroOutArray[i] = 0.0f;
			PitchCtrlInArray[i] = 0.0f;
			PitchGyroOutArray[i] = 0.0f;
			YawCtrlInArray[i] = 0.0f;
			YawGyroOutArray[i] = 0.0f;
			RollCtrlInArray_F[i] = 0.0f;
			RollGyroOutArray_F[i] = 0.0f;
			PitchCtrlInArray_F[i] = 0.0f;
			PitchGyroOutArray_F[i] = 0.0f;
			YawCtrlInArray_F[i] = 0.0f;
			YawGyroOutArray_F[i] = 0.0f;
			AltiCtrlInArray[i] = 0.0f;
			AltiDwOutArray[i] = 0.0f;
		}
	}

	Lati = LatiRef + x / 6378137;
	Long = LongRef + y / (6378137 * cos(LatiRef));
	gNaviData.XAng = (float) p;
	gNaviData.YAng = (float) q;
	gNaviData.ZAng = (float) r;
	gNaviData.Roll = (float) phi;
	gNaviData.Pitch = (float) theta;
	gNaviData.Yaw = (float) pusi;
	gNaviData.XAcc = (float) agx;
	gNaviData.YAcc = (float) agy;
	gNaviData.ZAcc = (float) agz;
	gNaviData.XSpeed = (float) vgx;
	gNaviData.YSpeed = (float) vgy;
	gNaviData.ZSpeed = (float) vgz;
	gNaviData.Lati = Lati;
	gNaviData.Long = Long;
	gNaviData.Alti = (float) AltRef - (float) z;
	gNaviData.SenUptFlag = 0xff;
	ILSPrePosiX = (float) x;
	ILSPrePosiY = (float) y;
	ILSPrePosiZ = (float) z;

	//\B6\D4\D3\DA\D7\CB̬\B7\A2\B2\BC\D0\C5Ϣ\B8\B3ֵ
	float xyz[3] =
	{
	    0 };
	xyz[0] = gNaviData.Roll;
	xyz[1] = gNaviData.Pitch;
	xyz[2] = gNaviData.Yaw;
	struct quat _qvalue =
	{
	    0 };
	Angle2Q(xyz, _qvalue);

	vehicle_attitude_s _att_hil;
	_att_hil.timestamp = hrt_absolute_time();
	_att_hil.rollspeed = gNaviData.XAng;
	_att_hil.pitchspeed = gNaviData.YAng;
	_att_hil.yawspeed = gNaviData.ZAng;

	_att_hil.q[0] = _qvalue.w;
	_att_hil.q[1] = _qvalue.x;
	_att_hil.q[2] = _qvalue.y;
	_att_hil.q[3] = _qvalue.z;

	_att_hil.delta_q_reset[0] = _delta_q_reset[0];
	_att_hil.delta_q_reset[1] = _delta_q_reset[1];
	_att_hil.delta_q_reset[2] = _delta_q_reset[2];
	_att_hil.delta_q_reset[3] = _delta_q_reset[3];

	if (_att_pub != nullptr)
	{
		/* publish the attitude */
		orb_publish(ORB_ID(vehicle_attitude), _att_pub, &_att_hil);
		_delta_q_reset[0] = _qvalue.w;
		_delta_q_reset[1] = _qvalue.x;
		_delta_q_reset[2] = _qvalue.y;
		_delta_q_reset[3] = _qvalue.z;
	}
	else
	{
		/* advertise and publish */
		_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &_att_hil);
	}

	//\B6\D4local_posi\BD\F8\D0и\B3ֵ
	vehicle_local_position_s _local_pos_hil
	{ };
	_local_pos_hil.timestamp = hrt_absolute_time();
	_local_pos_hil.xy_valid = true;
	_local_pos_hil.z_valid = true;
	_local_pos_hil.v_xy_valid = true;
	_local_pos_hil.v_z_valid = true;
	_local_pos_hil.x = x;
	_local_pos_hil.y = y;
	_local_pos_hil.z = z;
	_local_pos_hil.vx = vgx;
	_local_pos_hil.vy = vgy;
	_local_pos_hil.vz = vgz;
	_local_pos_hil.yaw = gNaviData.Yaw;
	_local_pos_hil.xy_global = true;
	_local_pos_hil.z_global = true;
	_local_pos_hil.ref_timestamp = hrt_absolute_time();
	_local_pos_hil.ref_lat = LatiRef * 180.0 / 3.1415926;
	_local_pos_hil.ref_lon = LongRef * 180.0 / 3.1415926;
	_local_pos_hil.ref_alt = AltRef;
	_local_pos_hil.dist_bottom = 0.0f;
	_local_pos_hil.dist_bottom_rate = 0.0f;
	_local_pos_hil.dist_bottom_valid = false;
	_local_pos_hil.eph = 0.01;
	_local_pos_hil.epv = 0.01;

	if (_local_pos_pub != nullptr)
	{
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &_local_pos_hil);

	}
	else
	{
		/* advertise and publish */
		_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &_local_pos_hil);
	}

	//\B6\D4global\BD\F8\D0и\B3ֵ
	vehicle_global_position_s _global_pos_hil
	{ };
	_global_pos_hil.timestamp = hrt_absolute_time();
	_global_pos_hil.lat = gNaviData.Lati * 180.0 / 3.1415926;
	_global_pos_hil.lon = gNaviData.Long * 180.0 / 3.1415926;
	_global_pos_hil.alt = gNaviData.Alti;
	_global_pos_hil.vel_n = vgx;
	_global_pos_hil.vel_e = vgy;
	_global_pos_hil.vel_d = vgz;
	_global_pos_hil.yaw = gNaviData.Yaw;
	_global_pos_hil.eph = _local_pos_hil.eph;
	_global_pos_hil.epv = _local_pos_hil.epv;
	_global_pos_hil.terrain_alt = -z;
	_global_pos_hil.terrain_alt_valid = false;
	_global_pos_hil.dead_reckoning = false;

	if (_global_pos_pub != nullptr)
	{
		/* publish the global position */
		orb_publish(ORB_ID(vehicle_global_position), _global_pos_pub, &_global_pos_hil);
	}
	else
	{
		/* advertise and publish */
		_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &_global_pos_hil);
	}

	//\B6\D4\D3\DAGPS\D0\C5Ϣ\B8\B4\D6Ʋ\A2\B7\A2\B2\BC
	struct vehicle_gps_position_s _gps_hil;
	_gps_hil.timestamp = hrt_absolute_time();
	_gps_hil.lat = (int32_t) (gNaviData.Lati * 180.0 / 3.1415926 * 10000000);
	_gps_hil.lon = (int32_t) (gNaviData.Long * 180.0 / 3.1415926 * 10000000);
	_gps_hil.alt = (int32_t) (gNaviData.Alti * 1000);
	_gps_hil.satellites_used = 23;
	_gps_hil.time_utc_usec = hrt_absolute_time();
	_gps_hil.s_variance_m_s = 0.01f;
	_gps_hil.c_variance_rad = 0.01f;
	_gps_hil.fix_type = 3;
	_gps_hil.eph = 0.01f;
	_gps_hil.epv = 0.01f;
	_gps_hil.vel_n_m_s = vgx;
	_gps_hil.vel_e_m_s = vgy;
	_gps_hil.vel_d_m_s = vgz;
	_gps_hil.vel_m_s = sqrtf(_gps_hil.vel_n_m_s * _gps_hil.vel_n_m_s + _gps_hil.vel_e_m_s * _gps_hil.vel_e_m_s + _gps_hil.vel_d_m_s * _gps_hil.vel_d_m_s);
	_gps_hil.cog_rad = gNaviData.Yaw;
	_gps_hil.vel_ned_valid = true;

//	mavlink_log_info(&_mavlink_log_pub, "GPS : %.7f, %.7f ", _gps_hil.lat, _gps_hil.lon);

	if (_gps_pub != nullptr)
	{
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &_gps_hil);

	}
	else
	{
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_gps_hil);
	}
}

void HilSim::Angle2Q(float xyz[3], struct quat &q)
{
	float x = xyz[0] / 2;
	float y = xyz[1] / 2;
	float z = xyz[2] / 2;
	q.w = cosf(x) * cosf(y) * cosf(z) + sinf(x) * sinf(y) * sinf(z);
	q.x = sinf(x) * cosf(y) * cosf(z) - cosf(x) * sinf(y) * sinf(z);
	q.y = cosf(x) * sinf(y) * cosf(z) + sinf(x) * cosf(y) * sinf(z);
	q.z = cosf(x) * cosf(y) * sinf(z) - sinf(x) * sinf(y) * cosf(z);
}

void HilSim::HIL_TransformationFromInertialToBody(double Rx, double Ry, double Rz, double Ix, double Iy, double Iz, double *Bx, double *By, double *Bz)
{
	double R11;
	double R12;
	double R13;
	double R21;
	double R22;
	double R23;
	double R31;
	double R32;
	double R33;
	/*\B5\BC\BA\BD\B5\BD\B1\BE\CC\E5*/
	R11 = cos(Rz) * cos(Ry);
	R12 = sin(Rz) * cos(Ry);
	R13 = -sin(Ry);

	R21 = -sin(Rz) * cos(Rx) + cos(Rz) * sin(Ry) * sin(Rx);
	R22 = cos(Rz) * cos(Rx) + sin(Rz) * sin(Ry) * sin(Rx);
	R23 = cos(Ry) * sin(Rx);

	R31 = sin(Rz) * sin(Rx) + cos(Rz) * sin(Ry) * cos(Rx);
	R32 = -cos(Rz) * sin(Rx) + sin(Rz) * sin(Ry) * cos(Rx);
	R33 = cos(Ry) * cos(Rx);

	/*\B5\BC\BA\BD\B5\BD\B1\BE\CC\E5*/
	*Bx = R11 * Ix + R12 * Iy + R13 * Iz;
	*By = R21 * Ix + R22 * Iy + R23 * Iz;
	*Bz = R31 * Ix + R32 * Iy + R33 * Iz;
}

float HilSim::InnerLoopSimulationModelCal(float t_In, float *Para_In, float *Pre_In, int InOrder, float *Para_Out, float *Pre_Out, int OutOrder)
{
	int i = 0;
	float tOut = 0;
	InnerLoopSimulationDataUpdate(t_In, Pre_In, InOrder);
	for (i = 0; i < InOrder; i++)
	{
		tOut += (*(Para_In + i)) * (*(Pre_In + i));
	}
	for (i = 0; i < OutOrder; i++)
	{
		tOut -= (*(Para_Out + i)) * (*(Pre_Out + i));
	}
	InnerLoopSimulationDataUpdate(tOut, Pre_Out, OutOrder);
	return tOut;

}

void HilSim::InnerLoopSimulationDataUpdate(float t_In, float *Pre_In, int InOrder)
{
	int i = 0;
	for (i = InOrder - 1; i > 0; i--)
	{
		*(Pre_In + i) = *(Pre_In + i - 1);
	}
	*Pre_In = t_In;
}

double HilSim::AngleMaxPI_Adj(double t_In)
{
	double t_pi = 3.1415926f;
	double tOut = 0;
	if (t_In > t_pi)
	{
		tOut = t_In - 2 * t_pi;
	}
	else if (t_In < -t_pi)
	{
		tOut = t_In + 2 * t_pi;
	}
	else
	{
		tOut = t_In;
	}
	return tOut;
}
