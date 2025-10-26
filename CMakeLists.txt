cmake_minimum_required(VERSION 3.0.2)
project(swarm-bot-surveyor)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs)
catkin_package()
install(PROGRAMS src/swarm_node.py src/slam_processor.py src/exporter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
