cmake_minimum_required(VERSION 3.0.2)
project(rl_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  gazebo_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()
