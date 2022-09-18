/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RCInput.hpp"

/*Telemetry defines*/
#include <unistd.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vehicle_attitude.h>
#include <math.h>
#include <matrix/math.hpp>
using namespace matrix;

#define frac(f) (f - (int)f)

#define NAVIGATION_STATE_MANUAL 0		// Manual mode
#define NAVIGATION_STATE_ALTCTL 1		// Altitude control mode
#define NAVIGATION_STATE_POSCTL 2		// Position control mode
#define NAVIGATION_STATE_AUTO_MISSION 3		// Auto mission mode
#define NAVIGATION_STATE_AUTO_LOITER 4		// Auto loiter mode
#define NAVIGATION_STATE_AUTO_RTL 5		// Auto return to launch mode
#define NAVIGATION_STATE_AUTO_RCRECOVER 6	// RC recover mode
#define NAVIGATION_STATE_AUTO_RTGS 7		// Auto return to groundstation on data link loss
#define NAVIGATION_STATE_AUTO_LANDENGFAIL 8 	// Auto land on engine failure
#define NAVIGATION_STATE_AUTO_LANDGPSFAIL 9	// Auto land on gps failure (e.g. open loop loiter down)
#define NAVIGATION_STATE_ACRO 10		// Acro mode
#define NAVIGATION_STATE_UNUSED 11		// Free slot
#define NAVIGATION_STATE_DESCEND 12		// Descend mode (no position control)
#define NAVIGATION_STATE_TERMINATION 13		// Termination mode
#define NAVIGATION_STATE_OFFBOARD 14
#define NAVIGATION_STATE_STAB 15		// Stabilized mode
#define NAVIGATION_STATE_RATTITUDE 16		// Rattitude (aka "flip") mode
#define NAVIGATION_STATE_AUTO_TAKEOFF 17	// Takeoff
#define NAVIGATION_STATE_AUTO_LAND 18		// Land
#define NAVIGATION_STATE_AUTO_FOLLOW_TARGET 19	// Auto Follow
#define NAVIGATION_STATE_AUTO_PRECLAND 20	// Precision land with landing target
#define NAVIGATION_STATE_ORBIT 21       // Orbit in a circle
#define NAVIGATION_STATE_MAX 22

#define LED_GREEN 0
#define LED_GREEN_BLINK 1
#define LED_BLUE 2
#define LED_BLUE_BLINK 3
#define LED_PURPLE 4
#define LED_PURPLE_BLINK 5
#define LED_WHITE 6
#define LED_WHITE_BLINK 7
#define LED_YELLOW 8
#define LED_YELLOW_BLINK 9
#define LED_RED 10
#define LED_RED_BLINK 11
#define LED_LIGHTBLUE 12
#define LED_LIGHTBLUE_BLINK 13
#define LED_OFF 14

struct uorb_subscription_data_s {

	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	uORB::SubscriptionData<sensor_combined_s> sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::SubscriptionData<vehicle_air_data_s> vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::SubscriptionData<vehicle_global_position_s> vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_gps_position_s> vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_status_flags_s> vehicle_status_flags_sub{ORB_ID(vehicle_status_flags)};
	uORB::SubscriptionData<vehicle_attitude_s> vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
};
static struct uorb_subscription_data_s *subscription_data = nullptr;

/* Initializes and updates the uORB subscriptions. */
bool RCInput::uorb_init()
{
	subscription_data = new uorb_subscription_data_s();

	if (!subscription_data) {
		return false;
	}

	return true;
}

void RCInput::uorb_deinit()
{
	if (subscription_data) {
		delete subscription_data;
		subscription_data = nullptr;
	}
}

void RCInput::uorb_update_topics()
{
	subscription_data->battery_status_sub.update();
	subscription_data->sensor_combined_sub.update();
	subscription_data->vehicle_air_data_sub.update();
	subscription_data->vehicle_global_position_sub.update();
	subscription_data->vehicle_gps_position_sub.update();
	subscription_data->vehicle_status_sub.update();
	subscription_data->vehicle_status_flags_sub.update();
	subscription_data->vehicle_attitude_sub.update();
}

/* End of UORB functions*/

/* Send telemetry packet */
int RCInput::send_packet(int uart_fd)
{
	RCInput::telemPayload telemetryPayload;
	RCInput::telemData telemetryData;

	{
		RCInput::uorb_update_topics();
		if (counter >= 65535) counter = 0;
		counter ++;

		const battery_status_s &bat = subscription_data->battery_status_sub.get();
		const vehicle_gps_position_s &gps = subscription_data->vehicle_gps_position_sub.get();
		const vehicle_global_position_s &gpos = subscription_data->vehicle_global_position_sub.get();
		const vehicle_status_flags_s &vsflags = subscription_data->vehicle_status_flags_sub.get();
		const vehicle_status_s &vs = subscription_data->vehicle_status_sub.get();
		const vehicle_attitude_s &att = subscription_data->vehicle_attitude_sub.get();
		//const vehicle_air_data_s &air_data = subscription_data->vehicle_air_data_sub.get();

		//Disarmed
		if (vsflags.condition_global_position_valid == 1 && vsflags.condition_home_position_valid == 1 && vsflags.condition_local_position_valid == 1 && vsflags.condition_local_velocity_valid == 1)
		{
			gps_status = 0b10000000; //GPS status READY
			telemetryPayload.flightmode = 0x10;
		}
		else
		{
			gps_status = 0b00000000; //GPS status ACQUIRING
			telemetryPayload.flightmode = 0x08;
		}

		//Armed
		if (vs.arming_state == 2)
		{
			if(vs.nav_state == NAVIGATION_STATE_STAB) telemetryPayload.flightmode = 0x00;
			if(vs.nav_state == NAVIGATION_STATE_ALTCTL) telemetryPayload.flightmode = 0x01;
			if(vs.nav_state == NAVIGATION_STATE_POSCTL) telemetryPayload.flightmode = 0x03;
			if(vs.nav_state == NAVIGATION_STATE_AUTO_MISSION || vs.nav_state ==  NAVIGATION_STATE_AUTO_LOITER) telemetryPayload.flightmode = 0x21;
			if(vs.nav_state ==  NAVIGATION_STATE_AUTO_RTL) telemetryPayload.flightmode = 0x0d;
			if(vs.nav_state == NAVIGATION_STATE_ACRO) telemetryPayload.flightmode = 0x14;
			if(vs.nav_state == NAVIGATION_STATE_RATTITUDE) telemetryPayload.flightmode = 0x14;
			if(vs.failsafe == true || vs.nav_state == NAVIGATION_STATE_AUTO_RCRECOVER || vs.nav_state == NAVIGATION_STATE_AUTO_RTGS || vs.nav_state == NAVIGATION_STATE_AUTO_LANDENGFAIL || vs.nav_state == NAVIGATION_STATE_AUTO_LANDGPSFAIL) telemetryPayload.flightmode = 0x0c;
		}

		//Emergency
		if (vs.arming_state == 3) telemetryPayload.flightmode = 0x0c;

		//Get pitch/roll/yaw values from the vehicle_attitude topic's quaternion
		Quatf q(att.q);
		Eulerf euler(q);

		telemetryPayload.length = 0x26;
		telemetryPayload.type = 0x02;
		telemetryPayload.t = RCInput::counter;
		telemetryPayload.lat = gps.lat;
		telemetryPayload.lon = gps.lon;
		telemetryPayload.alt = roundf(frac(air_data.baro_alt_meter) * 100.0f);
		//telemetryPayload.alt = gps.alt * 0.1f;
		telemetryPayload.vx = (int16_t)gpos.vel_n * 100.0f;
		telemetryPayload.vy = (int16_t)gpos.vel_e * 100.0f;
		telemetryPayload.vz = (int16_t)gpos.vel_d * 100.0f;
		telemetryPayload.nsat = gps_status + (uint8_t)(gps.satellites_used);
		telemetryPayload.voltage = (uint8_t)roundf((bat.voltage_v-5) * 10.0f);//    0x63; //hex
		telemetryPayload.current = 0;
		telemetryPayload.roll = (int16_t)euler.phi() * 100.0f;
		telemetryPayload.pitch = (int16_t)euler.theta() * 100.0f;
		telemetryPayload.yaw = (int16_t)euler.psi() * 100.0f;
		telemetryPayload.motorStatus = 0xFF; //OK
		telemetryPayload.gpsStatus = 0x61;
		telemetryPayload.obsStatus = 0x55;
		telemetryPayload.optionbytes = 0x05;
		telemetryPayload.alarmbytes = 0x00;
		telemetryPayload.gps_acc = 0x00;       // from gps.eph to u_int8_t [0..200]
		telemetryData.header1 =  0x55; //Header 1
		telemetryData.header2 =  0x55; //Header 2
		telemetryData.payload = telemetryPayload;
		telemetryData.crc8 = st24_common_crc8((uint8_t*)&telemetryPayload, sizeof(telemetryPayload));

		int packet_len = sizeof(telemetryData);

		::write(uart_fd, (uint8_t*)&telemetryData, packet_len);

	}
	return 1;
}




using namespace time_literals;


work_s RCInput::_work = {};

RCInput::RCInput(bool run_as_task, char *device) :
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	// rc input, published to ORB
	_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;

	// initialize it as RC lost
	_rc_in.rc_lost = true;

	// initialize raw_rc values and count
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_raw_rc_values[i] = UINT16_MAX;
	}

#ifdef RC_SERIAL_PORT

	if (device) {
		strncpy(_device, device, sizeof(_device));
		_device[sizeof(_device) - 1] = '\0';
	}

#endif
}

RCInput::~RCInput()
{
#ifdef RC_SERIAL_PORT
	dsm_deinit();
#endif

	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int
RCInput::init()
{

	// dsm_init sets some file static variables and returns a file descriptor
	_rcs_fd = dsm_init(_device);

	if (_rcs_fd < 0) {
		return -errno;
	}

	//Wait before uorb initialization
	px4_usleep(1000000);
	RCInput::uorb_init();

	return 0;
}

int
RCInput::task_spawn(int argc, char *argv[])
{
	bool run_as_task = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device = RC_SERIAL_PORT;

	while ((ch = px4_getopt(argc, argv, "td:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't':
			run_as_task = true;
			break;

		case 'd':
			device = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return -1;
	}


	if (!run_as_task) {

		/* schedule a cycle to start things */
		int ret = work_queue(HPWORK, &_work, (worker_t)&RCInput::cycle_trampoline_init, (void *)device, 0);

		if (ret < 0) {
			return ret;
		}

		// we need to wait, otherwise 'device' could go out of scope while still being accessed
		wait_until_running();

		_task_id = task_id_is_work_queue;

	} else {

		/* start the IO interface task */

		const char *const args[] = { device, nullptr };
		_task_id = px4_task_spawn_cmd("rc_input",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_SLOW_DRIVER,
					      1000,
					      (px4_main_t)&run_trampoline,
					      (char *const *)args);

		if (_task_id < 0) {
			_task_id = -1;
			return -errno;
		}
	}

	return PX4_OK;
}

void
RCInput::cycle_trampoline_init(void *arg)
{
	RCInput *dev = new RCInput(false, (char *)arg);

	if (!dev) {
		PX4_ERR("alloc failed");
		return;
	}

	int ret = dev->init();

	if (ret != 0) {
		PX4_ERR("init failed (%i)", ret);
		delete dev;
		return;
	}

	_object.store(dev);

	dev->cycle();
}
void
RCInput::cycle_trampoline(void *arg)
{
	RCInput *dev = reinterpret_cast<RCInput *>(arg);
	dev->cycle();
}

void
RCInput::fill_rc_in(uint16_t raw_rc_count_local,
		    uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		    hrt_abstime now, bool frame_drop, bool failsafe,
		    unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */

	{
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

#ifdef RC_SERIAL_PORT
void RCInput::rc_io_invert(bool invert)
{
	// First check if the board provides a board-specific inversion method (e.g. via GPIO),
	// and if not use an IOCTL
	if (!board_rc_invert_input(_device, invert)) {
		ioctl(_rcs_fd, TIOCSINVERT, invert ? (SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX) : 0);
	}
}
#endif

void
RCInput::run()
{
	int ret = init();

	if (ret != 0) {
		PX4_ERR("init failed (%i)", ret);
		exit_and_cleanup();
		return;
	}
	cycle();
}

void
RCInput::cycle()
{
	while (true) {

		perf_begin(_cycle_perf);

		const hrt_abstime cycle_timestamp = hrt_absolute_time();


		bool rc_updated = false;

		// This block scans for a supported serial RC input and locks onto the first one found
		// Scan for 300 msec, then switch protocol
		constexpr hrt_abstime rc_scan_max = 300_ms;

		unsigned frame_drops;
		frame_drops = 0;

		if (_report_lock && _rc_scan_locked) {
			_report_lock = false;
			//PX4_WARN("RCscan: %s RC input locked", RC_SCAN_STRING[_rc_scan_state]);
		}

		int newBytes = 0;

		if (_run_as_task) {
			// TODO: needs work (poll _rcs_fd)
			// int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
			// then update priority to SCHED_PRIORITY_FAST_DRIVER
			// read all available data from the serial RC input UART
			newBytes = ::read(_rcs_fd, &_rcs_buf[0], SBUS_BUFFER_SIZE);

		} else {
			// read all available data from the serial RC input UART
			newBytes = ::read(_rcs_fd, &_rcs_buf[0], SBUS_BUFFER_SIZE);
		}



		{

			if (_rc_scan_begin == 0) {
				_rc_scan_begin = cycle_timestamp;
				// Configure serial port for DSM
				dsm_config(_rcs_fd);
				rc_io_invert(false);

			} else if (_rc_scan_locked
				   || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

				if (newBytes > 0) {
					// parse new data
					uint8_t st24_rssi, lost_count;

					rc_updated = false;

					for (unsigned i = 0; i < (unsigned)newBytes; i++) {
						/* set updated flag if one complete packet was parsed */
						st24_rssi = RC_INPUT_RSSI_MAX;
						rc_updated = (OK == st24_decode(_rcs_buf[i], &st24_rssi, &lost_count,
										&_raw_rc_count, _raw_rc_values, input_rc_s::RC_INPUT_MAX_CHANNELS));
					}

					// The st24 will keep outputting RC channels and RSSI even if RC has been lost.
					// The only way to detect RC loss is therefore to look at the lost_count.

					if (rc_updated) {
						if (lost_count == 0) {
							// we have a new ST24 frame. Publish it.
							_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_ST24;
							fill_rc_in(_raw_rc_count, _raw_rc_values, cycle_timestamp,
								   false, false, frame_drops, st24_rssi);
							_rc_scan_locked = true;

						} else {
							// if the lost count > 0 means that there is an RC loss
							_rc_in.rc_lost = true;
						}
					}
				}

			} else {

				st24_reset();

				/*Used to restore RC link*/
				_rc_scan_begin = 0;

			}

			//Send telemetry packet
			send_packet(_rcs_fd);

		}


		perf_end(_cycle_perf);

		if (rc_updated) {
			perf_count(_publish_interval_perf);

			_to_input_rc.publish(_rc_in);

		} else if (!rc_updated && ((hrt_absolute_time() - _rc_in.timestamp_last_signal) > 1_s)) {
			_rc_scan_locked = false;
		}

		if (_run_as_task) {
			if (should_exit()) {
				break;
			}

		} else {
			if (should_exit()) {

				//Is this needed //TR
				RCInput::uorb_deinit();

				exit_and_cleanup();

			} else {
				/* schedule next cycle */
				work_queue(HPWORK, &_work, (worker_t)&RCInput::cycle_trampoline, this, USEC2TICK(_current_update_interval));
			}

			break;
		}
	}
}


RCInput *RCInput::instantiate(int argc, char *argv[])
{
	// No arguments to parse. We also know that we should run as task
	return new RCInput(true, argv[0]);
}

int RCInput::custom_command(int argc, char *argv[])
{

	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RCInput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RCInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Typhoon H RC input driver with telemetry support, based on PX4's rc_input driver.

### Implementation
By default the module runs on the work queue, to reduce RAM usage. It can also be run in its own thread,
specified via start flag -t, to reduce latency.
When running on the work queue, it schedules at a fixed frequency.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_input", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task (without any mode set, use any of the mode_* cmds)");
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Run as separate task instead of the work queue", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "RC device", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int RCInput::print_status()
{
	PX4_INFO("Running %s", (_run_as_task ? "as task" : "on work queue"));

	if (!_run_as_task) {
		PX4_INFO("Max update rate: %i Hz", 1000000 / _current_update_interval);
	}
	if (_device[0] != '\0') {
		PX4_INFO("Serial device: %s", _device);
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	if (hrt_elapsed_time(&_rc_in.timestamp) < 1_s) {
		print_message(_rc_in);
	}

	return 0;
}

extern "C" __EXPORT int rc_input_main(int argc, char *argv[]);

int
rc_input_main(int argc, char *argv[])
{
	return RCInput::main(argc, argv);
}
