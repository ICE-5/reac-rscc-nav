Use Velodyne Puck or Velodyne Puck Lite with Xsens MTi-200 VRU, mount the sensors side by side with their cables pointing in the same direction, build customized cable as in "wiring_diagram.pdf", connect the sensors to a Ubuntu 20.04 computer (recommend latest i7 CPU) installed with ROS Noetic, install MT SDK and configure IMU settings with MT Manager, and set computer IP to 192.168.1.100

Install Pcap library

  sudo apt-get install libpcap-dev

Unzip laser_odometry folder, then in a terminal go to the folder

  catkin_make

  source devel/setup.sh

Launch processing with

  roslaunch loam_velodyne_16 loam_velodyne_16.launch

Registered scans and sensor frame should show in RVIZ

In a web browser, log on to 192.168.1.201 to check time synchronization status

If using with ground-based collision avoidance at

  https://github.com/jizhang-cmu/ground_based_autonomy_basic

comment out

  <!--include file="$(find vehicle_simulator)/launch/vehicle_simulator.launch" /-->

and uncomment

  <include file="$(find loam_interface)/launch/loam_interface.launch" />

in 'ground_based_autonomy_basic/src/vehicle_simulator/launch/system.launch'. Set 'flipStateEstimation = false' and 'reverseTF = true' in 'ground_based_autonomy_basic/src/loam_interface/launch/loam_interface.launch'.
