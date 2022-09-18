/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "typhoon_led.h"

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

/**
 * Initializes and updates the uORB subscriptions.
 */
bool TYPHOON_led::uorb_init()
{
	subscription_data = new uorb_subscription_data_s();

	if (!subscription_data) {
		return false;
	}

	return true;
}

void TYPHOON_led::uorb_deinit()
{
	if (subscription_data) {
		delete subscription_data;
		subscription_data = nullptr;
	}
}

void TYPHOON_led::uorb_update_topics()
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


void TYPHOON_led::set_led(int led_mode)
{
	//Solid LED, the user is in control, no automatic functionality running
	//Blinking LED, the drone is doing something automatic behind users back

	//Green, blinking: the drone is initializating
	//Green, solid: ready for flight

	//Non-GNSS-assisted flight modes
	//Blue, solid: stabilize
	//Blue, blinking: altitude hold

	//GNSS-assisted flight modes
	//Purple, solid: Position
	//Purple, blinking: Mission running

	//Aerobatic modes
	//White, solid: Acro/Manual/Rate
	//White, blinking: RAttitude

	//Error
	//Red, solid: Malfunction, arming prohibited
	//Red, blinking: Malfunction in flight, RTH or autoland in progress

	switch (led_mode)
	{
		case(LED_OFF):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;

		case(LED_GREEN):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, true);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;

		case(LED_GREEN_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, led_blink);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;

		case(LED_BLUE):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, true);
		break;

		case(LED_BLUE_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, led_blink);
		break;

		case(LED_LIGHTBLUE):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, true);
			stm32_gpiowrite(GPIO_LED_BLUE, true);
		break;

		case(LED_LIGHTBLUE_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, false);
			stm32_gpiowrite(GPIO_LED_GREEN, led_blink);
			stm32_gpiowrite(GPIO_LED_BLUE, led_blink);
		break;

		case(LED_PURPLE):
			stm32_gpiowrite(GPIO_LED_RED, true);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, true);
		break;

		case(LED_PURPLE_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, led_blink);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, led_blink);
		break;

		case(LED_WHITE):
			stm32_gpiowrite(GPIO_LED_RED, true);
			stm32_gpiowrite(GPIO_LED_GREEN, true);
			stm32_gpiowrite(GPIO_LED_BLUE, true);
		break;

		case(LED_WHITE_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, led_blink);
			stm32_gpiowrite(GPIO_LED_GREEN, led_blink);
			stm32_gpiowrite(GPIO_LED_BLUE, led_blink);
		break;

		case(LED_YELLOW):
			stm32_gpiowrite(GPIO_LED_RED, true);
			stm32_gpiowrite(GPIO_LED_GREEN, true);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;

		case(LED_YELLOW_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, led_blink);
			stm32_gpiowrite(GPIO_LED_GREEN, led_blink);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;

		case(LED_RED):
			stm32_gpiowrite(GPIO_LED_RED, true);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;

		case(LED_RED_BLINK):
			stm32_gpiowrite(GPIO_LED_RED, led_blink);
			stm32_gpiowrite(GPIO_LED_GREEN, false);
			stm32_gpiowrite(GPIO_LED_BLUE, false);
		break;
	}
}

void TYPHOON_led::led_control()
{
	while (true)
	{
		TYPHOON_led::uorb_update_topics();

		//const battery_status_s &bat = subscription_data->battery_status_sub.get();
		//const vehicle_gps_position_s &gps = subscription_data->vehicle_gps_position_sub.get();
		//const vehicle_global_position_s &gpos = subscription_data->vehicle_global_position_sub.get();
		const vehicle_status_flags_s &vsflags = subscription_data->vehicle_status_flags_sub.get();
		const vehicle_status_s &vs = subscription_data->vehicle_status_sub.get();
		//const vehicle_attitude_s &att = subscription_data->vehicle_attitude_sub.get();
		//const vehicle_air_data_s &air_data = subscription_data->vehicle_air_data_sub.get();

		//Disarmed. Show green or blinking green color, depending on the GNSS status
		if (vsflags.condition_global_position_valid == 1 && vsflags.condition_home_position_valid == 1 && vsflags.condition_local_position_valid == 1 && vsflags.condition_local_velocity_valid == 1)
		{
			led_status = LED_GREEN;
		}
		else
		{
			led_status = LED_GREEN_BLINK;
		}

		//Armed
		if (vs.arming_state == 2)
		{
			//Show light blue color if selected flight mode does not match to any of the cases below
			led_status = LED_LIGHTBLUE;
			if(vs.nav_state == NAVIGATION_STATE_STAB) led_status = LED_BLUE;
			if(vs.nav_state == NAVIGATION_STATE_ALTCTL) led_status = LED_BLUE_BLINK;
			if(vs.nav_state == NAVIGATION_STATE_POSCTL) led_status = LED_PURPLE;
			if(vs.nav_state == NAVIGATION_STATE_AUTO_MISSION || vs.nav_state ==  NAVIGATION_STATE_AUTO_LOITER) led_status = LED_PURPLE_BLINK;
			if(vs.nav_state ==  NAVIGATION_STATE_AUTO_RTL) led_status = LED_PURPLE_BLINK;
			if(vs.nav_state == NAVIGATION_STATE_ACRO) led_status = LED_WHITE;
			if(vs.nav_state == NAVIGATION_STATE_RATTITUDE) led_status = LED_WHITE_BLINK;
			if(vs.failsafe == true || vs.nav_state == NAVIGATION_STATE_AUTO_RCRECOVER || vs.nav_state == NAVIGATION_STATE_AUTO_RTGS || vs.nav_state == NAVIGATION_STATE_AUTO_LANDENGFAIL || vs.nav_state == NAVIGATION_STATE_AUTO_LANDGPSFAIL) led_status = LED_RED_BLINK;
		}

		//Emergency
		if (vs.arming_state == 3) led_status = LED_RED;


		if(led_blink_counter < 10)
		{
			led_blink_counter++;
		}
		else
		{
			led_blink_counter = 0;
			led_blink = !led_blink;
		}
		TYPHOON_led::set_led(led_status);
		px4_usleep(100000);

	}
}

int TYPHOON_led::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int TYPHOON_led::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TYPHOON_led::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}


	return 0;
}

TYPHOON_led *TYPHOON_led::instantiate(int argc, char *argv[])
{

	TYPHOON_led *instance = new TYPHOON_led(false, false);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

TYPHOON_led::TYPHOON_led(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void TYPHOON_led::run()
{
	led_blink_counter = 0;
	led_blink = 1;
	TYPHOON_led::uorb_init();

	TYPHOON_led::led_control();

	TYPHOON_led::uorb_deinit();
}

int TYPHOON_led::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module controls Typhoon's RGB LED.

### Implementation
The module receives telemetry data from uORB topics and controls the RGB LED via GPIO.

### Examples

$ typhoon_led
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("typhoon_led", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	return 0;
}

int typhoon_led_main(int argc, char *argv[])
{
	return TYPHOON_led::main(argc, argv);
}
