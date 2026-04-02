# This message was automatically generated
# https://github.com/nrjl/lcm_to_ros, nicholas.lawrance@oregonstate.edu
int64 timestamp
int64 seq_num
byte error_flag
byte motion_control_fsm
int32[3] motion_control_error_stack
byte[6] hard_error_leg_left
byte[6] hard_error_leg_right
byte[7] hard_error_arm_left
byte[7] hard_error_arm_right
byte[2] hard_error_waist
byte[2] hard_error_head

byte[2] hard_error_hand
byte[2] hard_error_gripper

byte[2] hard_error_leg_ft
byte[2] hard_error_arm_ft

byte[8] hard_error_ecat_stack
byte hard_error_imu
byte hard_error_power
int32[3] hard_error_other_stack
byte[3] ecat_node_num
