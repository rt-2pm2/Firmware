px4_add_board(
	PLATFORM posix
	VENDOR px4
	MODEL sitl
	LABEL hil 
	TESTING

	DRIVERS
		differential_pressure
		distance_sensor
		batt_smbus
		camera_trigger
		gps
		pwm_out_sim

	MODULES
		vmount
		sensors
		camera_feedback
		commander
		events
		land_detector
		load_mon
		mavlink
		navigator
		replay
		simulator	
		# Estimation modules
		#
		attitude_estimator_q
		ekf2
		local_position_estimator
		position_estimator_inav
		wind_estimator
		# Vehicle Control
		#
		fw_att_control
		fw_pos_control_l1
		gnd_att_control
		gnd_pos_control
		mc_att_control
		mc_pos_control
		vtol_att_control
		# Logging
		#
		logger
		# Simulation
		# This moduls provides more accurate timing 
		# in the sending of the control data
		#
		communicator
		# Library modules
		#
		dataman
		landing_target_estimator

	SYSTEMCMDS
		esc_calib
		led_control
		mixer
		motor_ramp
		param
		perf
		pwm
		reboot
		shutdown
		sd_bench
		top
		topic_listener
		tune_control
		ver
		tests
	
	EXAMPLES

	#
	# OBC challenge
	#
		bottle_drop

	#
	# Rover apps
	#
		rover_steering_control

	#
	# HippoCampus example (AUV from TUHH)
	#
		uuv_example_app

	#
	# Segway
	#
		segway

		dyn_hello

	#
	# Demo apps
	#

	# Tutorial code from
	# https://px4.io/dev/px4_simple_app
		px4_simple_app
	
	# Tutorial code from
	# https://px4.io/dev/debug_values
		px4_mavlink_debug

	# Tutorial code from
	# https://px4.io/dev/example_fixedwing_control
		fixedwing_control


	)

set(config_sitl_viewer jmavsim CACHE STRING "viewer for sitl")
set_property(CACHE config_sitl_viewer PROPERTY STRINGS "jmavsim;none")

set(config_sitl_debugger disable CACHE STRING "debugger for sitl")
set_property(CACHE config_sitl_debugger PROPERTY STRINGS "disable;gdb;lldb")

# If the environment variable 'replay' is defined, we are building with replay
# support. In this case, we enable the orb publisher rules.
set(REPLAY_FILE "$ENV{replay}")
if(REPLAY_FILE)
	message("Building with uorb publisher rules support")
	add_definitions(-DORB_USE_PUBLISHER_RULES)
endif()
