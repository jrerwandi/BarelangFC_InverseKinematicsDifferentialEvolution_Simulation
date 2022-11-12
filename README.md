# BarelangFC_InverseKinematicsDifferentialEvolution_Simulation
 
**Setup Workspace**

Install dependencies

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install ros-noetic-ros-control
sudo apt install ros-noetic-ros-controllers
```

Clone simulation package

```bash
cd ~
mkdir -p barelangfc/src
cd barelangfc/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git
git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Utility.git
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
git clone https://github.com/BarelangFC/BarelangFC-AdultSize-Simulation.git
git clone https://github.com/BarelangFC/BarelangFC_InverseKinematicsDifferentialEvolution_Simulation.git
cd ..
catkin_make
source devel/setup.bash 
```

Create executable
```bash
cd ~
cd barelangfc/src/BarelangFC_InverseKinematicsDifferentialEvolution_Simulation/ik_arm_solver_de/InverseKinematics/
chmod +x forward_random_kanan.py
chmod +x forward_random_kiri.py
chmod +x IK_Arm_Kanan.py
chmod +x IK_Arm_Kiri.py
cd ~
cd barelangfc/src/BarelangFC_InverseKinematicsDifferentialEvolution_Simulation/ik_arm_solver_de/marker/
chmod +x random_marker_kanan.py
chmod +x random_marker_kiri.py
```

**Terminal 1**
**Running Simulation**
note : after gazebo's open, click play on beside real time factor
```bash

roslaunch humanoid_gazebo humanoid_gazebo.launch

```

**Terminal 2**

```bash
cd ~
cd barelangfc
source devel/setup.bash
roslaunch humanoid_description humanoid_display.launch
```

**Terminal 3**

```bash
cd ~
cd barelangfc
source devel/setup.bash
roslaunch humanoid_manager humanoid_gazebo.launch
```

**Terminal 4**

```bash
cd ~
cd barelangfc
source devel/setup.bash
rosrun ik_arm_solver_de forward_random_kiri.py
```

**Terminal 5**

```bash
cd ~
cd barelangfc
source devel/setup.bash
rosrun ik_arm_solver_de random_maker_kiri.py
```

**Terminal 6**

```bash
cd ~
cd barelangfc
source devel/setup.bash
rosrun ik_arm_solver_de IK_Arm_kiri.py
```


# Requirement
1. Python 3
2. ros noetic 
4. numpy


**Folder Simulation Package**

Download this folder fo running simulation\
https://drive.google.com/file/d/1sk5Oou0sWBBAGiBrHiLFfxB44LGY8bKr/view?usp=sharing

**Poster**
![This is an image](https://github.com/jrerwandi/BarelangFC_InverseKinematicsDifferentialEvolution_Simulation/ik_arm_solver_de/Poster\ /DE.jpeg)


**Video**

https://youtu.be/Jzk4io5-jXo
