[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE  | DEFAULT JOINT
/dev/ttyUSB0 | 2000000   | r_sho_pitch

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL            | PROTOCOL | DEV NAME         | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | XM540-W270     | 2.0      | r_sho_pitch      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 2   | XM540-W270     | 2.0      | l_sho_pitch      | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 3   | XM540-W270     | 2.0      | r_sho_roll       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 4   | XM540-W270     | 2.0      | l_sho_roll       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 5   | XM540-W270     | 2.0      | r_el             | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 6   | XM540-W270     | 2.0      | l_el             | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 7   | XM540-W270     | 2.0      | r_el_yaw         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 8   | XM540-W270     | 2.0      | l_el_yaw         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 9   | XM540-W270     | 2.0      | r_hip_yaw        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 10  | XM540-W270     | 2.0      | l_hip_yaw        | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 11  | XM540-W270     | 2.0      | r_hip_roll       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 12  | XM540-W270     | 2.0      | l_hip_roll       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 13  | XM540-W270     | 2.0      | r_knee           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 14  | XM540-W270     | 2.0      | l_knee           | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 15  | XM540-W270     | 2.0      | r_ank_pitch_back | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 16  | XM540-W270     | 2.0      | l_ank_pitch_back | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 17  | XM540-W270     | 2.0      | r_ank            | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 18  | XM540-W270     | 2.0      | l_ank            | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 19  | XM540-W270     | 2.0      | r_ank_roll       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 20  | XM540-W270     | 2.0      | l_ank_roll       | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 21  | XM540-W270     | 2.0      | head_pan         | present_position, position_p_gain, position_i_gain, position_d_gain
dynamixel | /dev/ttyUSB0 | 22  | XM540-W270     | 2.0      | head_tilt        | present_position, position_p_gain, position_i_gain, position_d_gain
sensor    | /dev/ttyUSB0 | 200 | OPEN-CR          | 2.0      | open-cr          | button, present_voltage, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, roll, pitch, yaw
