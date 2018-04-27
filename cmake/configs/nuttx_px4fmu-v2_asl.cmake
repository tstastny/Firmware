include(configs/nuttx_px4fmu-v2_default)

# Temporary fix for https://github.com/PX4/Firmware/issues/9116
# Remove this when a fix has been merge to the PX4 repo
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

list(REMOVE_ITEM config_module_list
	drivers/px4flow
	
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control
)

list(APPEND config_module_list
	drivers/imu/adis16448
)