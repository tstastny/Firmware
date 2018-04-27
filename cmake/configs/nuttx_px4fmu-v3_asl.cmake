include(configs/nuttx_px4fmu-v3_default)

# Temporary fix for https://github.com/PX4/Firmware/issues/9116
# Remove this when a fix has been merge to the PX4 repo
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

list(REMOVE_ITEM config_module_list
	drivers/batt_smbus
	drivers/imu/bmi160
	drivers/px4flow
	drivers/tap_esc
	
	modules/attitude_estimator_q
	modules/gnd_att_control
	modules/gnd_pos_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control
	
	examples/bottle_drop
	examples/rover_steering_control
	examples/segway
	examples/px4_simple_app
	examples/px4_mavlink_debug
	examples/fixedwing_control
	examples/hwtest
)

list(APPEND config_module_list
	drivers/bat_mon
	drivers/bat_mon/bq78350
)
