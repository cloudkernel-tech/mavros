# raw servo values for direct actuator controls
#
# about groups, mixing and channels:
# https://pixhawk.org/dev/mixing

# constant for mixer group
uint8 PX4_MIX_FLIGHT_CONTROL = 0
uint8 PX4_MIX_FLIGHT_CONTROL_VTOL_ALT = 1
uint8 PX4_MIX_PAYLOAD = 2
uint8 PX4_MIX_MANUAL_PASSTHROUGH = 3
#uint8 PX4_MIX_FC_MC_VIRT = 4
#uint8 PX4_MIX_FC_FW_VIRT = 5

# constant for kerloud rover direct control
uint8 ROVER_YAW_CHANNEL_CONTROL_INDEX = 2
uint8 ROVER_THROTTLE_CHANNEL_CONTROL_INDEX = 3

std_msgs/Header header
uint8 group_mix
float32[8] controls
uint8 flag_rover_mode
