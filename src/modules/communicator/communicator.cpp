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

#include "communicator.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#include <drivers/drv_pwm_output.h>

#include <meas/class_meas.hpp>

extern MEASClass classMeas;

const unsigned mode_flag_armed = 128; // following MAVLink spec
const unsigned mode_flag_custom = 1;

int Communicator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
### Description
This module is used with the Simulation Framework.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ communicator start -a <ip> -p <port>

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("communicator", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('a', "127.0.0.1", "x.x.x.x",
			"Destination IP Address", false);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Destination Port", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Communicator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Communicator::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
	get_instance()->do_something();
	return 0;
	}

	return print_usage("unknown command");
}


int Communicator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("communicator",
	SCHED_DEFAULT,
	SCHED_PRIORITY_DEFAULT,
	2048,
	(px4_main_t)&run_trampoline,
	(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}
#include <v2.0/mavlink_types.h>
#include <v2.0/common/mavlink.h>
	return 0;
}

Communicator *Communicator::instantiate(int argc, char *argv[])
{
	char ip[16];
	unsigned int port = 4002;
	int ch;

	for (ch = 0; ch < argc; ch++)
	{
		if (strcmp(argv[ch], "-a") == 0)
			strcpy(ip, argv[ch + 1]);

		if (strcmp(argv[ch], "-p") == 0)
			port = (int)strtol(argv[ch + 1], nullptr, 10);
	}
	Communicator *instance = new Communicator(ip, port);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Communicator::Communicator(const char* ip_addr, uint32_t w_port)
	: ModuleParams(nullptr),
	_system_type(MAV_TYPE_QUADROTOR)
{
	InitializePort(ip_addr, w_port);
}

void Communicator::run()
{
	int i;

	px4_pollfd_struct_t fds[1];

	// Example: run the loop synchronized to the actuator_output topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int actuator_outputs_sub[ORB_MULTI_MAX_INSTANCES];
	int vehicle_status_sub;
	int parameter_update_sub;


	// ===================================================================
	// subscribe to topics

	// Params
	parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	// Outputs
	for (i = 0; i < ORB_MULTI_MAX_INSTANCES; i++)
	{
		actuator_outputs_sub[i] = orb_subscribe_multi(ORB_ID(actuator_outputs), i);
	}
	// Status
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	parameters_update(parameter_update_sub, true);

	fds[0].fd = actuator_outputs_sub[0];
	fds[0].events = POLLIN;

	while (!should_exit()) {
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(50000);
			continue;

		} else
			if (fds[0].revents & POLLIN)
			{
				poll_topics(actuator_outputs_sub, vehicle_status_sub);
				send_controls(actuator_outputs_sub);
			}
		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(parameter_update_sub);
	for (i = 0; i < ORB_MULTI_MAX_INSTANCES; i++)
	{
		orb_unsubscribe(actuator_outputs_sub[i]);
	}

	while (!should_exit()) {
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(50000);
			continue;

		} else
			if (fds[0].revents & POLLIN)
			{
				poll_topics(actuator_outputs_sub, vehicle_status_sub);
				send_controls(actuator_outputs_sub);
			}
		parameters_update(parameter_update_sub);
	}
	orb_unsubscribe(parameter_update_sub);
	for (i = 0; i < ORB_MULTI_MAX_INSTANCES; i++)
	{
		orb_unsubscribe(actuator_outputs_sub[i]);
	}
}

void Communicator::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}

void Communicator::poll_topics(int _actuator_outputs_sub[ORB_MULTI_MAX_INSTANCES], int _vehicle_status_sub)
{
	int i;

	// copy new actuator data if available
	bool updated;

	for (i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

		orb_check(_actuator_outputs_sub[i], &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub[i], &_actuators[i]);
		}
	}

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}


void Communicator::pack_actuator_message(mavlink_hil_actuator_controls_t &msg, unsigned index)
{
	msg.time_usec = hrt_absolute_time();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

	/* scale outputs depending on system type */
	if (_system_type == MAV_TYPE_QUADROTOR ||
	    _system_type == MAV_TYPE_HEXAROTOR ||
	    _system_type == MAV_TYPE_OCTOROTOR ||
	    _system_type == MAV_TYPE_VTOL_DUOROTOR ||
	    _system_type == MAV_TYPE_VTOL_QUADROTOR ||
	    _system_type == MAV_TYPE_VTOL_TILTROTOR ||
	    _system_type == MAV_TYPE_VTOL_RESERVED2) {

		unsigned n;

		switch (_system_type) {
			case MAV_TYPE_VTOL_DUOROTOR:
				n = 2;
				break;

			case MAV_TYPE_QUADROTOR:
			case MAV_TYPE_VTOL_QUADROTOR:
			case MAV_TYPE_VTOL_TILTROTOR:
				n = 4;
				break;

			case MAV_TYPE_VTOL_RESERVED2:
				// this is the standard VTOL / quad plane with 5 propellers
				n = 5;
				break;

			case MAV_TYPE_HEXAROTOR:
				n = 6;
				break;

			default:
				n = 8;
				break;
		case MAV_TYPE_VTOL_DUOROTOR:
			n = 2;
			break;

		case MAV_TYPE_QUADROTOR:
		case MAV_TYPE_VTOL_QUADROTOR:
		case MAV_TYPE_VTOL_TILTROTOR:
			n = 4;
			break;

		case MAV_TYPE_VTOL_RESERVED2:
			// this is the standard VTOL / quad plane with 5 propellers
			n = 5;
			break;

		case MAV_TYPE_HEXAROTOR:
			n = 6;
			break;

		default:
			n = 8;
			break;
		}

		for (unsigned i = 0; i < 16; i++) {
			if (_actuators[index].output[i] > PWM_DEFAULT_MIN / 2) {
				if (i < n) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for rotors */
					msg.controls[i] = (_actuators[index].output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for other channels */
					msg.controls[i] = (_actuators[index].output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
				}

			} else {
				/* send 0 when disarmed and for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}

	} else {
		/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

		for (unsigned i = 0; i < 16; i++) {
			if (_actuators[index].output[i] > PWM_DEFAULT_MIN / 2) {
				if (i != 4) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for normal channels */
					msg.controls[i] = (_actuators[index].output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for throttle */
					msg.controls[i] = (_actuators[index].output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
				}

			} else {
				/* set 0 for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}
	}

	msg.mode = mode_flag_custom;
	msg.mode |= (armed) ? mode_flag_armed : 0;
	msg.flags = 0;
}

void Communicator::send_controls(int _actuator_outputs_sub[ORB_MULTI_MAX_INSTANCES])
{
	int i;

	for (i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

		if (_actuator_outputs_sub[i] < 0 || _actuators[i].timestamp == 0) {
			continue;
		}

		mavlink_hil_actuator_controls_t hil_act_control = {};
		mavlink_message_t message = {};
		pack_actuator_message(hil_act_control, i);
		mavlink_msg_hil_actuator_controls_encode(_vehicle_status.system_id, 200,
				&message, &hil_act_control);
		send_mavlink_message(message);

		classMeas.insert_control_timestamp(hil_act_control.time_usec);
	}
}

void Communicator::send_mavlink_message(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t bufLen = 0;

	// convery mavlink message to raw data
	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	// send data
	ssize_t len = sendto(sock, buf, bufLen, 0, (struct sockaddr*)&remAddr, sizeof(struct sockaddr_in));

	if (len <= 0) {
		perror("SendError");
		PX4_WARN("Failed sending mavlink message");
		PX4_ERR("poll error %d, %d", len, errno);
	}
}

void Communicator::InitializePort(const char* ip_addr, uint32_t w_port)
{
       	write_port = w_port;
	PX4_INFO("Communicator: IP = %s | Port = %d", ip_addr, w_port);

	// Endpoint for communication file descriptor
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed\n");
		return;
	}

	// Initialize remote sockaddr_in structure
	memset(&remAddr, 0, sizeof(remAddr));

	remAddr.sin_family = AF_INET;
	remAddr.sin_port = htons(write_port);
	remAddr.sin_addr.s_addr = inet_addr(ip_addr);

	// Perfoming a non blocking access
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	{
		PX4_WARN("Communicator::InitializePort error: unable to set nonblocking");
		close(sock);
	}
	// Initialize remote sockaddr_in structure
	memset(&remAddr, 0, sizeof(remAddr));

	remAddr.sin_family = AF_INET;
	remAddr.sin_port = htons(write_port);
	remAddr.sin_addr.s_addr = inet_addr(ip_addr);

	// Perfoming a non blocking access
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	{
		PX4_WARN("Communicator::InitializePort error: unable to set nonblocking");
		close(sock);
		exit(EXIT_FAILURE);
	}
}


int communicator_main(int argc, char *argv[])
{
	return Communicator::main(argc, argv);
}
