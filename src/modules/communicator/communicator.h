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

#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <px4_module.h>
#include <px4_module_params.h>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>

#include <v2.0/mavlink_types.h>
#include <v2.0/common/mavlink.h>

extern "C" __EXPORT int communicator_main(int argc, char *argv[]);


class Communicator : public ModuleBase<Communicator>, public ModuleParams
{
public:
	Communicator(const char* ip_addr, uint32_t w_port);

	virtual ~Communicator() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Communicator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** Pack the control message */
	void pack_actuator_message(mavlink_hil_actuator_controls_t &msg, unsigned index);

	/** Send controls data */
	void send_controls(int _actuator_outputs_sub[ORB_MULTI_MAX_INSTANCES]);




	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

	void poll_topics(int _actuator_outputs_sub[ORB_MULTI_MAX_INSTANCES], int _vehicle_status_sub);

	void InitializePort(const char* ip_addr, uint32_t w_port);

	void send_mavlink_message(const mavlink_message_t &aMsg);

	struct actuator_outputs_s _actuators[ORB_MULTI_MAX_INSTANCES];
	struct vehicle_status_s _vehicle_status;

	// For param MAV_TYPE
	int32_t _system_type;
	int32_t _system_id;

	/**
	 * Communication part.
	 */
	int sock; /*!< File descriptor that refers to the END point of the communication*/
	char target_ip[16]; /*!< String containing the Remote Address IP*/
	struct sockaddr_in remAddr; /*!< Remote address*/
	int write_port; /*! Write Port */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _sys_autoconfig  /**< another parameter */
	)
};

