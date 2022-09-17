#! /bin/bash
export ROS_DOMAIN_ID=33
source /opt/lc/install/setup.bash
ros2 launch  process_launchers scan_assembler.launch.py 
