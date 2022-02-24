# Project Background
This project is a modularized software framework for generic robot development and research. The modularized platform separates low level components that vary from robot to robot from the high level logic that does not vary across robots. The low level components include processes to communicate with motors and sensors on the robot, including the camera. The high level components include the state machines that control how the robots move around and process sensor data. By separating into these levels, we achieve a more adaptable system that is easily ported to different robots.

The project began with the University of Pennsylvania RoboCup code base from the 2011 RoboCup season and is continuing to evolve into an ever more generalized and versatile robot software framework.  The DARPA Robotics Challenge also pushed development

## Copyright

All code sources associated with this project are:

* (c) 2013 Stephen McGill, Seung-Joon Yi, Daniel D. Lee, Yida Zhang, Spyridon Karachalios, Qin He, Jordan Brindza.
* (c) 2014 Stephen McGill, Seung-Joon Yi, Daniel D. Lee, Yida Zhang, Qin He, Larry Vadakedathu
* (c) 2015 Stephen McGill, Seung-Joon Yi, Daniel D. Lee, Bhoram Lee
* (c) 2016 Stephen McGill, Seung-Joon Yi, Daniel D. Lee, Jinwook Huh
* (c) 2017 Seung-Joon Yi
* (c) 2018 Seung-Joon Yi
* (c) 2019 Seung-Joon Yi
* (c) 2020 Seung-Joon Yi

* Exceptions are noted on a per file basis.

### Contact Information

* seungjoon.yi@pusan.ac.kr

# Ubuntu Setup

Download the 16.04.6 LTS desktop image from [Ubuntu](http://www.ubuntu.com/download/desktop)
Install with username `sj`.

# Download or copy webots simulator

cd ~/Downloads
wget https://github.com/cyberbotics/webots/releases/download/R2020b/webots-R2020b-x86-64_ubuntu-16.04.tar.bz2

# Basic setup for Ubuntu 16.04 / ROS Kinetic
```
./install_nuc.sh  
```
# Basic setup for Ubuntu 18.04 / ROS Melodic
```
./install_nuc_1804.sh
```

# Compile for each robot

```
make "robot_name"
```

* Panda : Franka Emika Panda 7DOF arm
* UR5: Universal Robotics UR5E arm
* TB3 : Turtlebot
* TB3Arm : Turtlebot Home Service robot (100mm mecanum wheel + dynamixel gripper)
* HSR: Toyota HSR
* KMOXI: K-MOXI Service Robot (RMD-X8 servo + 150mm meacnum wheel + kinova arm)
* BK1: Bakwi- Killer 1 (RMD servo + 150mm meacnum wheel + 1DOF lift joint + 4DOF arm)
* M2: small quadruped robot
* PADUK1: quadruped robot (RMD-X8 servos)


Webots tested: Panda, UR5, TB3Arm, PADUK1,



# Run webots simulation
```
./webots_start.lua
```

# Run robot codes
```
./start_robot.lua
```





### NVIDIA driver and CUDA setup ###
Press Ctrl+alt+F1
sudo service lightdm stop
```
cd ~/Downloads
wget  http://us.download.nvidia.com/XFree86/Linux-x86_64/390.77/NVIDIA-Linux-x86_64-390.77.run
chmod +x NVIDIA-Linux-x86_64-390.77.run
./NVIDIA-Linux-x86_64-390.77.run -x
cd ../NVIDIA-Linux-x86_64-390.77
sudo ./nvidia-installer
```

Cuda 9.1 setup (for GTX 10xx GPU)
```
wget https://developer.nvidia.com/compute/cuda/9.1/Prod/local_installers/cuda_9.1.85_387.26_linux
chmod +x cuda_9.1.85_387.26_linux
./cuda_9.1.85_387.26_linux -extract ~/Downloads/cuda
cd cuda
sudo ./cuda-linux.9.1.85-2308092.run
```

Cuda 10.1 setup (for RTX 20xx GPU)
```
wget https://developer.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.105_418.39_linux.run
sudo ./cuda_10.1.105_418.39_linux.run
```

### Compile ros packages
```
cd ~/Desktop/ARAICodes/catkin_ws2
catkin_make -DCMAKE_BUILD_TYPE=Release
cd src/darknet_ros/darknet
make
```

https://github.com/AlexeyAB/darknet
