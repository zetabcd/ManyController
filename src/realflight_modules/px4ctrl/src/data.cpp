// sun: 本文件保存早期节点入口和 PX4 消息字段对照资料，当前未加入 CMake 构建目标；
// sun: 阅读时可将其视为接口参考，实际运行入口位于 px4ctrl_node.cpp。

int main(void)
{
    flag_rc_is_received = true;//临时量，判断遥控器是否接收到


 
	qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

	/*
	uint64 timestamp # time since system start (microseconds)

uint64 armed_time # Arming timestamp (microseconds)
uint64 takeoff_time # Takeoff timestamp (microseconds)

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED    = 2

uint8 latest_arming_reason
uint8 latest_disarming_reason
uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
uint8 ARM_DISARM_REASON_RC_STICK = 1
uint8 ARM_DISARM_REASON_RC_SWITCH = 2
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
uint8 ARM_DISARM_REASON_MISSION_START = 5
uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6
uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
uint8 ARM_DISARM_REASON_KILL_SWITCH = 9
uint8 ARM_DISARM_REASON_LOCKDOWN = 10
uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11
uint8 ARM_DISARM_REASON_SHUTDOWN = 12
uint8 ARM_DISARM_REASON_UNIT_TEST = 13

uint64 nav_state_timestamp # time when current nav_state activated

uint8 nav_state_user_intention                  # Mode that the user selected (might be different from nav_state in a failsafe situation)

uint8 nav_state                                 # Currently active mode
uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode
uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode
uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
uint8 NAVIGATION_STATE_POSITION_SLOW = 6
uint8 NAVIGATION_STATE_FREE5 = 7
uint8 NAVIGATION_STATE_FREE4 = 8
uint8 NAVIGATION_STATE_FREE3 = 9
uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode
uint8 NAVIGATION_STATE_FREE2 = 11
uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode
uint8 NAVIGATION_STATE_FREE1 = 16
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
uint8 NAVIGATION_STATE_EXTERNAL1 = 23
uint8 NAVIGATION_STATE_EXTERNAL2 = 24
uint8 NAVIGATION_STATE_EXTERNAL3 = 25
uint8 NAVIGATION_STATE_EXTERNAL4 = 26
uint8 NAVIGATION_STATE_EXTERNAL5 = 27
uint8 NAVIGATION_STATE_EXTERNAL6 = 28
uint8 NAVIGATION_STATE_EXTERNAL7 = 29
uint8 NAVIGATION_STATE_EXTERNAL8 = 30
uint8 NAVIGATION_STATE_MAX = 31

uint8 executor_in_charge                        # Current mode executor in charge (0=Autopilot)

uint32 valid_nav_states_mask                    # Bitmask for all valid nav_state values
uint32 can_set_nav_states_mask                  # Bitmask for all modes that a user can select

# Bitmask of detected failures
uint16 failure_detector_status
uint16 FAILURE_NONE = 0
uint16 FAILURE_ROLL = 1              # (1 << 0)
uint16 FAILURE_PITCH = 2             # (1 << 1)
uint16 FAILURE_ALT = 4               # (1 << 2)
uint16 FAILURE_EXT = 8               # (1 << 3)
uint16 FAILURE_ARM_ESC = 16          # (1 << 4)
uint16 FAILURE_BATTERY = 32          # (1 << 5)
uint16 FAILURE_IMBALANCED_PROP = 64  # (1 << 6)
uint16 FAILURE_MOTOR = 128           # (1 << 7)

uint8 hil_state
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# If it's a VTOL, then the value will be VEHICLE_TYPE_ROTARY_WING while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING when flying as a fixed-wing
uint8 vehicle_type
uint8 VEHICLE_TYPE_UNKNOWN = 0
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3
uint8 VEHICLE_TYPE_AIRSHIP = 4

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe

bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control
uint8 failsafe_defer_state # one of FAILSAFE_DEFER_STATE_*

# Link loss
bool gcs_connection_lost              # datalink to GCS lost
uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events
bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

# VTOL flags
bool is_vtol             # True if the system is VTOL capable
bool is_vtol_tailsitter  # True if the system performs a 90° pitch down rotation during transition from MC to FW
bool in_transition_mode  # True if VTOL is doing a transition
bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW

# MAVLink identification
uint8 system_type  # system type, contains mavlink MAV_TYPE
uint8 system_id    # system id, contains MAVLink's system ID field
uint8 component_id # subsystem / component id, contains MAVLink's component ID field

bool safety_button_available # Set to true if a safety button is connected
bool safety_off # Set to true if safety is off

bool power_input_valid                            # set if input power is valid
bool usb_connected                                # set to true (never cleared) once telemetry received from usb link

bool open_drone_id_system_present
bool open_drone_id_system_healthy

bool parachute_system_present
bool parachute_system_healthy

bool avoidance_system_required                    # Set to true if avoidance system is enabled via COM_OBS_AVOID parameter
bool avoidance_system_valid                       # Status of the obstacle avoidance system

bool rc_calibration_in_progress
bool calibration_enabled

bool pre_flight_checks_pass             # true if all checks necessary to arm pass
	*/
	vehicle_status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
								   "/fmu/out/vehicle_status", 
								   qos,
								   std::bind(&States_Data_t::feed, &fsm.sta_data, std::placeholders::_1));	

	/*
	uint64 timestamp                        # time since system start (microseconds)
bool connected                          # Whether or not a battery is connected, based on a voltage threshold
float32 voltage_v                       # Battery voltage in volts, 0 if unknown
float32 voltage_filtered_v      # Battery voltage in volts, filtered, 0 if unknown
float32 current_a                       # Battery current in amperes, -1 if unknown
float32 current_filtered_a      # Battery current in amperes, filtered, 0 if unknown
float32 current_average_a       # Battery current average in amperes (for FW average in level flight), -1 if unknown
float32 discharged_mah          # Discharged amount in mAh, -1 if unknown
float32 remaining                       # From 1 to 0, -1 if unknown
float32 scale                           # Power scaling factor, >= 1, or -1 if unknown
float32 time_remaining_s        # predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown
float32 temperature                     # temperature of the battery. NaN if unknown
uint8 cell_count                        # Number of cells, 0 if unknown

uint8 BATTERY_SOURCE_POWER_MODULE = 0
uint8 BATTERY_SOURCE_EXTERNAL = 1
uint8 BATTERY_SOURCE_ESCS = 2
uint8 source                            # Battery source
uint8 priority                          # Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
uint16 capacity                         # actual capacity of the battery
uint16 cycle_count                      # number of discharge cycles the battery has experienced
uint16 average_time_to_empty    # predicted remaining battery capacity based on the average rate of discharge in min
uint16 serial_number            # serial number of the battery pack
uint16 manufacture_date         # manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512
uint16 state_of_health          # state of health. FullChargeCapacity/DesignCapacity, 0-100%.
uint16 max_error                        # max error, expected margin of error in % in the state-of-charge calculation with a range of 1 to 100%
uint8 id                                        # ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.
uint16 interface_error          # interface error counter

float32[14] voltage_cell_v              # Battery individual cell voltages, 0 if unknown
float32 max_cell_voltage_delta  # Max difference between individual cell voltages

bool is_powering_off                    # Power off event imminent indication, false if unknown
bool is_required                        # Set if the battery is explicitly required before arming


uint8 BATTERY_WARNING_NONE = 0          # no battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1           # warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2      # critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3     # immediate landing required
uint8 BATTERY_WARNING_FAILED = 4        # the battery has failed completely
uint8 BATTERY_STATE_UNHEALTHY = 6 # Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in faults field.
uint8 BATTERY_STATE_CHARGING = 7 # Battery is charging

uint8 BATTERY_FAULT_DEEP_DISCHARGE = 0 # Battery has deep discharged
uint8 BATTERY_FAULT_SPIKES = 1 # Voltage spikes
uint8 BATTERY_FAULT_CELL_FAIL= 2 # One or more cells have failed
uint8 BATTERY_FAULT_OVER_CURRENT = 3 # Over-current
uint8 BATTERY_FAULT_OVER_TEMPERATURE = 4 # Over-temperature
uint8 BATTERY_FAULT_UNDER_TEMPERATURE = 5 # Under-temperature fault
uint8 BATTERY_FAULT_INCOMPATIBLE_VOLTAGE = 6 # Vehicle voltage is not compatible with battery one
uint8 BATTERY_FAULT_INCOMPATIBLE_FIRMWARE = 7 # Battery firmware is not compatible with current autopilot firmware
uint8 BATTERY_FAULT_INCOMPATIBLE_MODEL = 8 # Battery model is not supported by the system
uint8 BATTERY_FAULT_HARDWARE_FAILURE = 9 # hardware problem
uint8 BATTERY_WARNING_OVER_TEMPERATURE = 10 # Over-temperature
uint8 BATTERY_FAULT_COUNT = 11 # Counter - keep it as last element!

uint16 faults           # Smart battery supply status/fault flags (bitmask) for health indication.
uint32 custom_faults    # Bitmask indicating smart battery internal manufacturer faults, those are not user actionable.
uint8 warning           # Current battery warning
uint8 mode              # Battery mode. Note, the normal operation mode

uint8 BATTERY_MODE_UNKNOWN = 0 # Battery does not support a mode, or if it does, is operational
uint8 BATTERY_MODE_AUTO_DISCHARGING = 1 # Battery is auto discharging (towards storage level)
uint8 BATTERY_MODE_HOT_SWAP = 2 # Battery in hot-swap mode
uint8 BATTERY_MODE_COUNT = 3 # Counter - keep it as last element (once we're fully migrated to events interface we can just comment this)!


uint8 MAX_INSTANCES = 4

float32 average_power               # The average power of the current discharge
float32 available_energy            # The predicted charge or energy remaining in the battery
float32 full_charge_capacity_wh     # The compensated battery capacity
float32 remaining_capacity_wh       # The compensated battery capacity remaining
float32 design_capacity             # The design capacity of the battery
uint16 average_time_to_full         # The predicted remaining time until the battery reaches full charge, in minutes
uint16 over_discharge_count         # Number of battery overdischarge
float32 nominal_voltage             # Nominal voltage of the battery pack
	*/  
	battery_status_position_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
								   "/fmu/out/battery_status", 
								   qos,
								   std::bind(&Battery_Data_t::feed, &fsm.bat_data, std::placeholders::_1)); 

	/*
	uint64 timestamp                                # time since system start (microseconds)

int32 RELATIVE_TIMESTAMP_INVALID = 2147483647   # (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid

# gyro timstamp is equal to the timestamp of the message
float32[3] gyro_rad                     # average angular rate measured in the FRD body frame XYZ-axis in rad/s over the last gyro sampling period
uint32 gyro_integral_dt                 # gyro measurement sampling period in microseconds

int32 accelerometer_timestamp_relative  # timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
float32[3] accelerometer_m_s2           # average value acceleration measured in the FRD body frame XYZ-axis in m/s^2 over the last accelerometer sampling period
uint32 accelerometer_integral_dt        # accelerometer measurement sampling period in microseconds

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint8 accelerometer_clipping    # bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame
uint8 gyro_clipping             # bitfield indicating if there was any gyro clipping (per axis) during the integration time frame

uint8 accel_calibration_count   # Calibration changed counter. Monotonically increases whenever accelermeter calibration changes.
uint8 gyro_calibration_count    # Calibration changed counter. Monotonically increases whenever rate gyro calibration changes.
	*/							   
	sensor_combined_subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
								   "/fmu/out/sensor_combined", 
								   qos,
								   std::bind(&Sensor_Data_t::feed, &fsm.sens_data, std::placeholders::_1));		

	/*
	# The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)

uint64 timestamp                # time since system start (microseconds)

uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[4] q                    # Quaternion rotation from the FRD body frame to the NED earth frame
float32[4] delta_q_reset        # Amount by which quaternion has changed during last reset
uint8 quat_reset_counter        # Quaternion reset counter
	*/							   
	vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
								   "/fmu/out/vehicle_attitude", 
								   qos,
								   std::bind(&Attitude_Data_t::feed, &fsm.att_data, std::placeholders::_1));	
								   
	/*
	# Fused local position in NED.
# The coordinate system origin is the vehicle position at the time when the EKF2-module was started.

uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

bool xy_valid                           # true if x and y are valid
bool z_valid                            # true if z is valid
bool v_xy_valid                         # true if vx and vy are valid
bool v_z_valid                          # true if vz is valid

# Position in local NED frame
float32 x                               # North position in NED earth-fixed frame, (metres)
float32 y                               # East position in NED earth-fixed frame, (metres)
float32 z                               # Down position (negative altitude) in NED earth-fixed frame, (metres)

# Position reset delta
float32[2] delta_xy                     # Amount of lateral shift of position estimate in latest reset (in x and y) [m]
uint8 xy_reset_counter                  # Index of latest lateral position estimate reset
float32 delta_z                         # Amount of vertical shift of position estimate in latest reset [m]
uint8 z_reset_counter                   # Index of latest vertical position estimate reset

# Velocity in NED frame
float32 vx                              # North velocity in NED earth-fixed frame, (metres/sec)
float32 vy                              # East velocity in NED earth-fixed frame, (metres/sec)
float32 vz                              # Down velocity in NED earth-fixed frame, (metres/sec)
float32 z_deriv                         # Down position time derivative in NED earth-fixed frame, (metres/sec)

# Velocity reset delta
float32[2] delta_vxy                    # Amount of lateral shift of velocity estimate in latest reset (in x and y) [m/s]
uint8 vxy_reset_counter                 # Index of latest vertical velocity estimate reset
float32 delta_vz                        # Amount of vertical shift of velocity estimate in latest reset [m/s]
uint8 vz_reset_counter                  # Index of latest vertical velocity estimate reset

# Acceleration in NED frame
float32 ax        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az        # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

float32 heading                         # Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
float32 heading_var
float32 unaided_heading                 # Same as heading but generated by integrating corrected gyro data only
float32 delta_heading                   # Heading delta caused by latest heading reset [rad]
uint8 heading_reset_counter             # Index of latest heading reset
bool heading_good_for_control

float32 tilt_var

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
bool xy_global                          # true if position (x, y) has a valid global reference (ref_lat, ref_lon)
bool z_global                           # true if z has a valid global reference (ref_alt)
uint64 ref_timestamp                    # Time when reference position was set since system start, (microseconds)
float64 ref_lat                         # Reference point latitude, (degrees)
float64 ref_lon                         # Reference point longitude, (degrees)
float32 ref_alt                         # Reference altitude AMSL, (metres)

# Distance to surface
float32 dist_bottom                     # Distance from from bottom surface to ground, (metres)
bool dist_bottom_valid                  # true if distance to bottom surface is valid
uint8 dist_bottom_sensor_bitfield       # bitfield indicating what type of sensor is used to estimate dist_bottom
uint8 DIST_BOTTOM_SENSOR_NONE = 0
uint8 DIST_BOTTOM_SENSOR_RANGE = 1      # (1 << 0) a range sensor is used to estimate dist_bottom field
uint8 DIST_BOTTOM_SENSOR_FLOW = 2       # (1 << 1) a flow sensor is used to estimate dist_bottom field (mostly fixed-wing use case)

float32 eph                             # Standard deviation of horizontal position error, (metres)
float32 epv                             # Standard deviation of vertical position error, (metres)
float32 evh                             # Standard deviation of horizontal velocity error, (metres/sec)
float32 evv                             # Standard deviation of vertical velocity error, (metres/sec)

bool dead_reckoning                     # True if this position is estimated through dead-reckoning

# estimator specified vehicle limits
float32 vxy_max                         # maximum horizontal speed - set to 0 when limiting not required (meters/sec)
float32 vz_max                          # maximum vertical speed - set to 0 when limiting not required (meters/sec)
float32 hagl_min                        # minimum height above ground level - set to 0 when limiting not required (meters)
float32 hagl_max                        # maximum height above ground level - set to 0 when limiting not required (meters)
	*/							   
	vehicle_local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
								   "/fmu/out/vehicle_local_position", 
								   qos,
								   std::bind(&LocalPose_Data_t::feed, &fsm.pose_data, std::placeholders::_1));
	// 客户端
	/*
	VehicleCommand request
        uint64 timestamp                                        #
        uint16 VEHICLE_CMD_CUSTOM_0 = 0                         #
        uint16 VEHICLE_CMD_CUSTOM_1 = 1                         #
        uint16 VEHICLE_CMD_CUSTOM_2 = 2                         #
        uint16 VEHICLE_CMD_NAV_WAYPOINT = 16                    #
        uint16 VEHICLE_CMD_NAV_LOITER_UNLIM = 17                #
        uint16 VEHICLE_CMD_NAV_LOITER_TURNS = 18                #
        uint16 VEHICLE_CMD_NAV_LOITER_TIME = 19                 #
        uint16 VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20            #
        uint16 VEHICLE_CMD_NAV_LAND = 21                        #
        uint16 VEHICLE_CMD_NAV_TAKEOFF = 22                     #
        uint16 VEHICLE_CMD_NAV_PRECLAND = 23                    #
        uint16 VEHICLE_CMD_DO_ORBIT = 34                        #
        uint16 VEHICLE_CMD_DO_FIGUREEIGHT = 35                  #
        uint16 VEHICLE_CMD_NAV_ROI = 80                         #
        uint16 VEHICLE_CMD_NAV_PATHPLANNING = 81                # Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goa
        uint16 VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84                #
        uint16 VEHICLE_CMD_NAV_VTOL_LAND = 85                   #
        uint16 VEHICLE_CMD_NAV_GUIDED_LIMITS = 90               #
        uint16 VEHICLE_CMD_NAV_GUIDED_MASTER = 91               #
        uint16 VEHICLE_CMD_NAV_DELAY = 93                               #
        uint16 VEHICLE_CMD_NAV_LAST = 95                        #
        uint16 VEHICLE_CMD_CONDITION_DELAY = 112                #
        uint16 VEHICLE_CMD_CONDITION_CHANGE_ALT = 113           #
        uint16 VEHICLE_CMD_CONDITION_DISTANCE = 114             #
        uint16 VEHICLE_CMD_CONDITION_YAW = 115                  #
        uint16 VEHICLE_CMD_CONDITION_LAST = 159                 #
        uint16 VEHICLE_CMD_CONDITION_GATE = 4501                #
        uint16 VEHICLE_CMD_DO_SET_MODE = 176                    #
        uint16 VEHICLE_CMD_DO_JUMP = 177                        #
        uint16 VEHICLE_CMD_DO_CHANGE_SPEED = 178                #
        uint16 VEHICLE_CMD_DO_SET_HOME = 179                    #
        uint16 VEHICLE_CMD_DO_SET_PARAMETER = 180               #
        uint16 VEHICLE_CMD_DO_SET_RELAY = 181                   #
        uint16 VEHICLE_CMD_DO_REPEAT_RELAY = 182                #
        uint16 VEHICLE_CMD_DO_REPEAT_SERVO = 184                #
        uint16 VEHICLE_CMD_DO_FLIGHTTERMINATION = 185           #
        uint16 VEHICLE_CMD_DO_CHANGE_ALTITUDE = 186             #
        uint16 VEHICLE_CMD_DO_SET_ACTUATOR = 187                #
        uint16 VEHICLE_CMD_DO_LAND_START = 189                  #
        uint16 VEHICLE_CMD_DO_GO_AROUND = 191                   #
        uint16 VEHICLE_CMD_DO_REPOSITION = 192                  # Reposition to specific WGS84 GPS position. |Ground speed [m/s] |Bitmask |Loiter radius [m] for planes |Yaw    [deg] |Latitude     |Longitude |Altitude
        uint16 VEHICLE_CMD_DO_PAUSE_CONTINUE = 193
        uint16 VEHICLE_CMD_DO_SET_ROI_LOCATION = 195            #
        uint16 VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196       #
        uint16 VEHICLE_CMD_DO_SET_ROI_NONE = 197                #
        uint16 VEHICLE_CMD_DO_CONTROL_VIDEO = 200               #
        uint16 VEHICLE_CMD_DO_SET_ROI = 201                     #
        uint16 VEHICLE_CMD_DO_DIGICAM_CONTROL=203
        uint16 VEHICLE_CMD_DO_MOUNT_CONFIGURE=204               #
        uint16 VEHICLE_CMD_DO_MOUNT_CONTROL=205                 #
        uint16 VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST=206            #
        uint16 VEHICLE_CMD_DO_FENCE_ENABLE=207                  #
        uint16 VEHICLE_CMD_DO_PARACHUTE=208                     #
        uint16 VEHICLE_CMD_DO_MOTOR_TEST=209                    # motor test command |Instance (1, ...)| throttle type| throttle| timeout [s]| Motor count | Test order| Empt
        uint16 VEHICLE_CMD_DO_INVERTED_FLIGHT=210               #
        uint16 VEHICLE_CMD_DO_GRIPPER = 211                     #
        uint16 VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL=214        #
        uint16 VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT=220            #
        uint16 VEHICLE_CMD_DO_GUIDED_MASTER=221                 #
        uint16 VEHICLE_CMD_DO_GUIDED_LIMITS=222                 #
        uint16 VEHICLE_CMD_DO_LAST = 240                        #
        uint16 VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241          #
        uint16 PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION = 3#
        uint16 VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242   #
        uint16 VEHICLE_CMD_PREFLIGHT_UAVCAN = 243               #
        uint16 VEHICLE_CMD_PREFLIGHT_STORAGE = 245              #
        uint16 VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246      #
        uint16 VEHICLE_CMD_OBLIQUE_SURVEY=260                   #
        uint16 VEHICLE_CMD_DO_SET_STANDARD_MODE=262             #
        uint16 VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION = 283      #
        uint16 VEHICLE_CMD_MISSION_START = 300                  #
        uint16 VEHICLE_CMD_ACTUATOR_TEST = 310                  # Actuator testing command|value [-1,1]|timeout [s]|Empty|Empty|output functio
        uint16 VEHICLE_CMD_CONFIGURE_ACTUATOR = 311             #
        uint16 VEHICLE_CMD_COMPONENT_ARM_DISARM = 400           #
        uint16 VEHICLE_CMD_RUN_PREARM_CHECKS = 401              #
        uint16 VEHICLE_CMD_INJECT_FAILURE = 420                 #
        uint16 VEHICLE_CMD_START_RX_PAIR = 500                  #
        uint16 VEHICLE_CMD_REQUEST_MESSAGE = 512                #
        uint16 VEHICLE_CMD_SET_CAMERA_MODE = 530                #
        uint16 VEHICLE_CMD_SET_CAMERA_ZOOM = 531                #
        uint16 VEHICLE_CMD_SET_CAMERA_FOCUS = 532
        uint16 VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000    #
        uint16 VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001   #
        uint16 VEHICLE_CMD_IMAGE_START_CAPTURE = 2000           #
        uint16 VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003            #
        uint16 VEHICLE_CMD_VIDEO_START_CAPTURE = 2500           #
        uint16 VEHICLE_CMD_VIDEO_STOP_CAPTURE = 2501            #
        uint16 VEHICLE_CMD_LOGGING_START = 2510                 #
        uint16 VEHICLE_CMD_LOGGING_STOP = 2511                  #
        uint16 VEHICLE_CMD_CONTROL_HIGH_LATENCY = 2600          #
        uint16 VEHICLE_CMD_DO_VTOL_TRANSITION = 3000            #
        uint16 VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST = 3001     #
        uint16 VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001       #
        uint16 VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002       #
        uint16 VEHICLE_CMD_FIXED_MAG_CAL_YAW = 42006            #
        uint16 VEHICLE_CMD_DO_WINCH = 42600                     #
        uint16 VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE = 43003 #
        uint32 VEHICLE_CMD_PX4_INTERNAL_START    = 65537        #
        uint32 VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN = 100000       #
        uint32 VEHICLE_CMD_SET_NAV_STATE = 100001               #
        uint8 VEHICLE_MOUNT_MODE_RETRACT = 0                    #
        uint8 VEHICLE_MOUNT_MODE_NEUTRAL = 1                    #
        uint8 VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2          #
        uint8 VEHICLE_MOUNT_MODE_RC_TARGETING = 3               #
        uint8 VEHICLE_MOUNT_MODE_GPS_POINT = 4                  #
        uint8 VEHICLE_MOUNT_MODE_ENUM_END = 5                   #
        uint8 VEHICLE_ROI_NONE = 0                              #
        uint8 VEHICLE_ROI_WPNEXT = 1                            #
        uint8 VEHICLE_ROI_WPINDEX = 2                           #
        uint8 VEHICLE_ROI_LOCATION = 3                          #
        uint8 VEHICLE_ROI_TARGET = 4                            #
        uint8 VEHICLE_ROI_ENUM_END = 5
        uint8 PARACHUTE_ACTION_DISABLE = 0
        uint8 PARACHUTE_ACTION_ENABLE = 1
        uint8 PARACHUTE_ACTION_RELEASE = 2
        uint8 FAILURE_UNIT_SENSOR_GYRO = 0
        uint8 FAILURE_UNIT_SENSOR_ACCEL = 1
        uint8 FAILURE_UNIT_SENSOR_MAG = 2
        uint8 FAILURE_UNIT_SENSOR_BARO = 3
        uint8 FAILURE_UNIT_SENSOR_GPS = 4
        uint8 FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5
        uint8 FAILURE_UNIT_SENSOR_VIO = 6
        uint8 FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7
        uint8 FAILURE_UNIT_SENSOR_AIRSPEED = 8
        uint8 FAILURE_UNIT_SYSTEM_BATTERY = 100
        uint8 FAILURE_UNIT_SYSTEM_MOTOR = 101
        uint8 FAILURE_UNIT_SYSTEM_SERVO = 102
        uint8 FAILURE_UNIT_SYSTEM_AVOIDANCE = 103
        uint8 FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104
        uint8 FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105
        uint8 FAILURE_TYPE_OK = 0
        uint8 FAILURE_TYPE_OFF = 1
        uint8 FAILURE_TYPE_STUCK = 2
        uint8 FAILURE_TYPE_GARBAGE = 3
        uint8 FAILURE_TYPE_WRONG = 4
        uint8 FAILURE_TYPE_SLOW = 5
        uint8 FAILURE_TYPE_DELAYED = 6
        uint8 FAILURE_TYPE_INTERMITTENT = 7
        uint8 SPEED_TYPE_AIRSPEED = 0
        uint8 SPEED_TYPE_GROUNDSPEED = 1
        uint8 SPEED_TYPE_CLIMB_SPEED = 2
        uint8 SPEED_TYPE_DESCEND_SPEED = 3
        int8 ARMING_ACTION_DISARM = 0
        int8 ARMING_ACTION_ARM = 1
        uint8 GRIPPER_ACTION_RELEASE = 0
        uint8 GRIPPER_ACTION_GRAB = 1
        uint8 ORB_QUEUE_LENGTH = 8
        float32 param1                  #
        float32 param2                  #
        float32 param3                  #
        float32 param4                  #
        float64 param5                  #
        float64 param6                  #
        float32 param7                  #
        uint32 command                  #
        uint8 target_system             #
        uint8 target_component          #
        uint8 source_system             #
        uint16 source_component #
        uint8 confirmation              #
        bool from_external
        uint16 COMPONENT_MODE_EXECUTOR_START = 1000
---
VehicleCommandAck reply
        uint64 timestamp                #
        uint8 VEHICLE_CMD_RESULT_ACCEPTED = 0                   #
        uint8 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1       #
        uint8 VEHICLE_CMD_RESULT_DENIED = 2                     #
        uint8 VEHICLE_CMD_RESULT_UNSUPPORTED = 3                #
        uint8 VEHICLE_CMD_RESULT_FAILED = 4                     #
        uint8 VEHICLE_CMD_RESULT_IN_PROGRESS = 5                #
        uint8 VEHICLE_CMD_RESULT_CANCELLED = 6                  #
        uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
        uint16 ARM_AUTH_DENIED_REASON_NONE = 1
        uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
        uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
        uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
        uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5
        uint8 ORB_QUEUE_LENGTH = 4
        uint32 command                                          #
        uint8 result                                            #
        uint8 result_param1                                     # Also used as progress[%], it can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRE
        int32 result_param2                                     #
        uint8 target_system
        uint16 target_component                                 #
        bool from_external                                      #
	*/
	fsm.vehicle_command_client = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");






    //这个数据极其重要，控制了状态机的节奏
        class RC_Data_t
{
public:
    // sun: 四个主通道均归一化到 [-1, 1]；油门会经过悬停点重映射。
    double roll;
    double pitch;
    double yaw;
    double throttle; // [-1,1]
    GEARS aux1;
    GEARS aux2;
    GEARS aux3;
    GEARS aux4;
    GEARS aux5;
    GEARS aux6;
    bool sticks_moving;
    double hover_percentage;

    uint64_t timestamp;
    // sun: timestamp 来自 PX4/仿真端，rcv_stamp 是本 ROS 节点实际收到消息的时刻。
    rclcpp::Time rcv_stamp;

    GEARS aux1_last_;
    GEARS aux2_last_;
    GEARS aux3_last_;
    GEARS aux4_last_;
    GEARS aux5_last_;
    GEARS aux6_last_;

    bool aux1_changed;
    bool aux2_changed;
    bool aux3_changed;
    bool aux4_changed;
    bool aux5_changed;
    bool aux6_changed;

    bool aux2_has_downed;

    // sun: feed() 同时更新当前开关值、边沿变化标志和上一帧状态。
    RC_Data_t(PX4ControlNode &);
#ifdef SIMULATION
    void feed(const joy_msgs::msg::JoyStick::UniquePtr msg);
#else
    void feed(const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg);
#endif

private:
    PX4ControlNode& px4controlnode_;

};




    return 0;
}
