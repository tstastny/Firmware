## Specific configurations and differences of the ASL firmware

### Initial platform setup

 - Copy the whole content of asl_configdata/startscripts/ into an /etc/ folder on a microsd card
 - Bootup QGC. Flash our (!) firmware once. This has to be done via USB (QGC learns our firmware configuration including platform types and parameter descriptions via this process, see https://docs.qgroundcontrol.com/en/SetupView/Firmware.html)
 - If no or only the default parameter file is available, then in the QGC Settings -> Airframe -> XXX Plane sections, you have to select our ASL_XXX platform. After that, configure the other parameters as required and you are ready to go!
 
### Start scripts and config files

 - We have our own (much condensed) start script, located in startscripts/rc.txt. Most configuration options (Airplane type, HIL, onboard computer type) are now set via QGroundControl parameters and _not_ via the startup script anymore. 
 - The telemetry stream options can be configured in a separate file (startscripts/telem/telem.txt).
 - Our default logger configuration is located in startscripts/logging/logger_topics.txt

### Parameters

The parameters provided here shall just be a _basic and valid_ set of parameters, but they are _not_ tuned to any platform! Please use the parameters that we should _always_ store in the respective flight test folder on the group server after each flight. 

### Build process

Please build the targets `px4fmu-v2_asl` and `px4fmu-v3_asl` instead of `px4fmu-v2_default` and `px4fmu-v3_default`. This will build the firmware ONLY with the modules that we actually need (the config is done neatly in the respective /cmake/configs/... files by subtracting/adding packages from the default makefiles).

### Overall code contribution process

Put simply, please stay _as close as possible_ to the main Pixhawk repository: 
 - Develop new features using the pixhawk master branch such that these can be merged into our branch later on. Same applies to issues: Only issues that concern ASL-exclusive functionality should be posted in our repository.  
 - For features that do not go into the Pixhawk master (which shouldn't be many), we will use a strict _pull-request-to-master_ workflow here.

### Branches ###
 - stable: Current stable release, flight tested
 - master: Current main release, i.e. under development but bench-tested and thus operational. Please do not push directly to master, but use Pull Requests (PR) to contribute to master (see below!).
 - features/MYFEATURENAME: A new feature branch. All new features shall be added like this.
 - fix/MYFIX: A new fix. All new fixes shall be added like this.






