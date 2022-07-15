rostopic pub -1 /humanoid/l_ank_pitch_back_joint_position_controller/command std_msgs/Float64 "data: 0.8"
rostopic pub -1 /humanoid/l_knee_joint_position_controller/command std_msgs/Float64 "data: -0"
rostopic pub -1 /humanoid/l_hip_roll_joint_position_controller/command std_msgs/Float64 "data: 0"
rostopic pub -1 /humanoid/l_ank_roll_joint_position_controller/command std_msgs/Float64 "data: 0"
rostopic pub -1 /humanoid/l_hip_yaw_joint_position_controller/command std_msgs/Float64 "data: -0"

