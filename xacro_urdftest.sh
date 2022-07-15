clear

source devel/setup.bash

roslaunch urdf_tutorial display.launch model:='$(find humanoid_description)/urdf/humanoid_backup.xacro'

