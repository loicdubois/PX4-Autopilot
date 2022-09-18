/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "typhoon_ldg.h"
//#include "../platforms/px4_middleware.h"
//#include "../platforms/"

//#include <px4_platform_common/getopt.h>
//#include <px4_platform_common/log.h>
#include <px4_posix.h>
#include <unistd.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/landing_gear.h>


int Typhoon_ldg::print_status()
{
	PX4_INFO("Running. Do not stop or restart.");
	return 0;
}

int Typhoon_ldg::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int Typhoon_ldg::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("Typhoon_ldg",
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

Typhoon_ldg *Typhoon_ldg::instantiate(int argc, char *argv[])
{
	Typhoon_ldg *instance = new Typhoon_ldg();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Typhoon_ldg::Typhoon_ldg()
	: ModuleParams(nullptr)
{
}

void Typhoon_ldg::BitBang(void *arg)
{
	Typhoon_ldg *dev = reinterpret_cast<Typhoon_ldg *>(arg);
	dev->Bang();
}

void Typhoon_ldg::Bang()
{
	if(_task_running == 1)
	{
		/*Send inverted pulse for older MCU boards*/
		if( _call_times == 0)
		{
			px4_arch_gpiowrite(LDG_PIN, false);
			hrt_call_after(&_call, _pwm_pulse, (hrt_callout)&Typhoon_ldg::BitBang, this);
		}

		if( _call_times == 1)
		{
			px4_arch_gpiowrite(LDG_PIN, true);
			hrt_call_after(&_call, (20000-_pwm_pulse), (hrt_callout)&Typhoon_ldg::BitBang, this);
		}

		/*Send a low "reset" pulse*/
		if( _call_times == 2)
		{
			px4_arch_gpiowrite(LDG_PIN, false);
			hrt_call_after(&_call, (20000), (hrt_callout)&Typhoon_ldg::BitBang, this);
		}

		/*Send normal PWM pulse for later MCU boards*/
		if( _call_times == 3)
		{
			px4_arch_gpiowrite(LDG_PIN, true);
			hrt_call_after(&_call, _pwm_pulse, (hrt_callout)&Typhoon_ldg::BitBang, this);
		}

		if( _call_times == 4)
		{
			px4_arch_gpiowrite(LDG_PIN, false);
			hrt_call_after(&_call, (20000-_pwm_pulse), (hrt_callout)&Typhoon_ldg::BitBang, this);
		}

		/*Send a high "reset" pulse*/
		if( _call_times == 5)
		{
			px4_arch_gpiowrite(LDG_PIN, true);
			hrt_call_after(&_call, (20000), (hrt_callout)&Typhoon_ldg::BitBang, this);
		}

		if (_call_times < 5)
		{
			_call_times ++;
		}
		else
		{
			_call_times = 0;
		}
	}

}

void Typhoon_ldg::run()
{

	_call_times = 0;
	_task_running = 1;
	hrt_call_after(&_call, 1000, (hrt_callout)&Typhoon_ldg::BitBang, this);

	// Example: run the loop synchronized to the uorb_landing_gear topic publication
	int uorb_landing_gear_sub = orb_subscribe(ORB_ID(landing_gear));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = uorb_landing_gear_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct landing_gear_s uorb_landing_gear;
			orb_copy(ORB_ID(landing_gear), uorb_landing_gear_sub, &uorb_landing_gear);

			if(uorb_landing_gear.landing_gear == -1)
			{
				_pwm_pulse = 1000;
			}
			if(uorb_landing_gear.landing_gear == 1)
			{
				_pwm_pulse = 2000;
			}
		}
		parameters_update();
	}
	_task_running = 0;
	px4_usleep(100000);
	orb_unsubscribe(uorb_landing_gear_sub);
}

void Typhoon_ldg::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Typhoon_ldg::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This driver actuates Typhoon H480's landing gears according to data in landing_gear uORB topic.
### Implementation
This approach was chosen instead of PX4's native PWM control, as there are at least two differently wired Typhoon H
mainboards available. The other one outputs regular PWM to the landing gear controller, while another one has its PWM pin inverted.
The driver first sends an inverted PWM pulse, and then a regular pulse, separated by high and low "idle" signals.
### Examples
CLI usage example:
$ Typhoon_ldg start
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Typhoon_ldg", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int typhoon_ldg_main(int argc, char *argv[])
{
	return Typhoon_ldg::main(argc, argv);
}
