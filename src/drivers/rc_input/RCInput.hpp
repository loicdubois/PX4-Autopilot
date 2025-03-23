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

#pragma once

#include <float.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <lib/perf/perf_counter.h>
#include <lib/rc/crsf.h>
#include <lib/rc/dsm.h>
#include <lib/rc/sbus.h>
#include <lib/rc/st24.h>
#include <lib/rc/sumd.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_module.h>
#include <px4_workqueue.h>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_command.h>

#include "crsf_telemetry.h"

#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

class RCInput : public ModuleBase<RCInput>
{
public:

	RCInput(bool run_as_task, char *device);
	virtual ~RCInput();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static RCInput *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int	init();

	/**
	 *  Sends a packet to UART
	 *  @param uart_fd file-descriptor of UART device
	 */
	int send_packet(int uart_fd);

	void uorb_deinit();
	bool uorb_init();
	void uorb_update_topics();

	int port_number;
	int counter;
	uint8_t gps_status;


	//Alignment/padding
	#pragma pack(push, 1)

	typedef struct {
		uint8_t length;       ///< length includes type, data, and crc = sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
		uint8_t type;       ///< from enum ST24_PACKET_TYPE
		uint16_t t;     ///< packet counter or clock
		int32_t lat;      ///< latitude (degrees)  +/- 90 deg
		int32_t lon;      ///< longitude (degrees)  +/- 180 deg
		int32_t alt;      ///< 0.01m resolution, altitude (meters)
		int16_t vx, vy, vz;     ///< velocity 0.01m res, +/-320.00 North-East- Down

		uint8_t nsat;     ///< Sum of GPS status and number of satellites.
		//GPS status is bit 8 of the byte. Number of satellites is defined using the first 5 bits.
		//0b10011111 = GPS Ready, 31 satellites.
  		//0b00011111 = GPS Acguiring, 31 satellites.

		uint8_t voltage;    ///< 25.4V  voltage = 5 + 255*0.1 = 30.5V, min=5V
		uint8_t current;    ///< 0.5A resolution
		int16_t roll, pitch, yaw; ///< 0.01 degree resolution
		uint8_t motorStatus;    ///< 1 bit per motor for status 1=good, 0= fail

		uint8_t gpsStatus;    ///< gps and obs status   IMU_status
		/* Example: 0x61
		* 0x[X]Y
		* 0001 yyyy = gps disabled, obs rdy
		* 0010 yyyy = gps acquiring, obs disabled
		* 0100 yyyy = gps disabled, obs disabled
		* 1000 yyyy = gps disabled, obs disabled
		*
		* 0xX[Y]
		* xxxx 0001 = none
		* xxxx 0010 = none
		* xxxx 0100 = none
		* xxxx 1000 = none
		*/

		uint8_t obsStatus; ///< obs_avoidance | unknown   pressure_compass_status
		/* Example: 0x55
		* 0x[X]Y
		* 0001 yyyy = obs avoidance fail
		* 0010 yyyy = obs avoidance fail
		* 0100 yyyy = obs avoidance disabled
		* 1000 yyyy = obs avoidance fail
		*
		* 0xX[Y]
		* xxxx 0001 = none
		* xxxx 0010 = none
		* xxxx 0100 = none
		* xxxx 1000 = none
		*/

		uint8_t flightmode;
		uint8_t optionbytes; ///< drone model | flight mode
		/* Example: 0x0510
		*
		* 0x00XY (Bytes 0-1)
		*
		* 0x[X]Y
		* 0001 yyyy = READY
		* 0010 yyyy = IPS
		* 0100 yyyy = "N/A"
		* 1000 yyyy = THR
		*
		* 0x1[Y]
		* 0001 1000 = WATCH
		* 0001 0100 = RATE
		* 0001 0010 = MAG CALI
		* 0001 0001 = NO RC
		*
		*0xXY00 (Bytes 2-3)
		*
		* 0x[X]Y
		* 0001 yyyy = nothing
		* 0010 yyyy = nothing
		* 0100 yyyy = nothing
		* 1000 yyyy = nothing
		*
		* 0xX[Y]
		* //xxxx 0001 = BATT RED, drone model: hex
		* //xxxx 0010 = BATT FULL, drone model: quad
		* //xxxx 0100 = BATT RED, drone model: hex
		* //xxxx 1000 = BATT RED, drone model: hex
		* 0x03xx = drone model: quad, batt red
		* 0x05xx = drone model: hex, batt ok <---- Typhoon H (captured)
		* 0x06xx = drone model: hex, batt red
		* 0x07xx = drone model: hex, batt ok <---- ?
		*/


		/* Provided by h-elsner
		f_mode (decimal)	Meaning	​				Display​
		0			Stability mode (Blue solid)		THR​
		1			Blue flashing - GPS off​		THR​
		2​			Blue - GPS lost				THR​
		3​			Angle mode (Purple solid)​		Angle​
		4​			Purple flashing - GPS off​		Angle​
		5​			Angle mode (Purple solid)-GPS lost​	Angle​
		6​			Smart mode​				Smart​
		7​			Smart mode - GPS lost​			Angle​
		8​			Motor starting​				Start​
		9​			Temperature calibration​		Temp​
		10​			Pressure calibration​			Pre Cali​
		11​			Accelerator calibration​		Acc Cali​
		12​			Emergency/Killed​			EMER​
		13​			RTH coming​				Home​
		14​			RTH landing​				Land​
		15​			Binding​				Bind​
		16​			Initializing, Ready to start​		Ready​
		17​			Waiting for RC​				No RC​
		18​			Magnetometer calibration​		Mag Cali​
		19​			Unknown mode​​
		20​			Agility mode (Rate)​			Rate​
		21​			Smart mode - Follow me​			Follow​
		22​			Smart mode - Follow me - GPS lost​	THR​
		23​			Smart mode - Camera tracking​		Watch​
		24​			Camera tracking - GPS lost​		THR​
		26​			Task Curve Cable Cam​			CCC​
		27​			Task Journey​				JOUR​
		28​			Task Point of Interest​			POI​
		29​			Task Orbit​				ORBIT​
		32​			Indoor Positioning System​		IPS​
		33​			Waypoints​				WAYPOINT​
		*/

		uint8_t alarmbytes;     ///< Error flags
		/* Example: 0x21   Voltage warning 1 - voltage low + compass warning
		*
		* 0xXY
		*
		* 0x[X]Y
		* 0001 yyyy = imu temperature warning
		* 0010 yyyy = compass warning (captured, uncalibrated compass attached)
		* 0100 yyyy = Fly-away checker warning (never used)
		* 1000 yyyy = no fly zone warning
		*
		* 0xX[Y]
		* xxxx 0001 = battery voltage low
		* xxxx 0010 = battery voltage critically low
		* xxxx 0100 = motor failsafe mode  (five rotor mode)
		* xxxx 1000 = complete motor ESC failure
		*/

		uint8_t gps_acc;    ///<  GPSacc_H   GPS Horizontal accracy * 20
		/*
		* maximum 200 for no GPS fix
		*/
	} telemPayload;

	typedef struct {
		uint8_t header1;      ///< 0x55 for a valid packet
		uint8_t header2;      ///< 0x55 for a valid packet
		telemPayload payload;
		uint8_t crc8;       ///< crc8 checksum, calculated by st24_common_crc8 and including fields length, type and st24_data
	} telemData;

	#pragma pack(pop)

private:
	enum RC_SCAN {
		RC_SCAN_PPM = 0,
		RC_SCAN_SBUS,
		RC_SCAN_DSM,
		RC_SCAN_SUMD,
		RC_SCAN_ST24,
		RC_SCAN_CRSF
	} _rc_scan_state{RC_SCAN_SBUS};

	static constexpr char const *RC_SCAN_STRING[6] {
		"PPM",
		"SBUS",
		"DSM",
		"SUMD",
		"ST24",
		"CRSF"
	};

	hrt_abstime _rc_scan_begin{0};

	bool _rc_scan_locked{false};
	bool _report_lock{true};

	unsigned	_current_update_interval{4000};

	bool 		_run_as_task{false};

	static struct work_s	_work;

	uORB::Subscription	_vehicle_cmd_sub{ORB_ID(vehicle_command)};
	uORB::Subscription	_adc_sub{ORB_ID(adc_report)};

	input_rc_s	_rc_in{};

	float		_analog_rc_rssi_volt{-1.0f};
	bool		_analog_rc_rssi_stable{false};

	uORB::PublicationMulti<input_rc_s>	_to_input_rc{ORB_ID(input_rc)};

	int		_rcs_fd{-1};
	char		_device[20] {};					///< device / serial port path

	uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};

	uint16_t _raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS] {};
	uint16_t _raw_rc_count{};

	CRSFTelemetry *_crsf_telemetry{nullptr};

	perf_counter_t      _cycle_perf;
	perf_counter_t      _publish_interval_perf;

	static void	cycle_trampoline(void *arg);
	static void	cycle_trampoline_init(void *arg);
	int 		start();

	void fill_rc_in(uint16_t raw_rc_count_local,
			uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
			hrt_abstime now, bool frame_drop, bool failsafe,
			unsigned frame_drops, int rssi);

	void set_rc_scan_state(RC_SCAN _rc_scan_state);

	void rc_io_invert(bool invert);

};
