cmake_minimum_required(VERSION 3.0.2)
project(ros_helper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
  message_generation
  )

catkin_python_setup()

add_message_files(
  FILES
  MultiJoy.msg
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs#   visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp rospy sensor_msgs std_msgs tf2_ros visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/rosbags_to_csv.py
  scripts/multijoy_node.py
  scripts/save_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_nosetests(test/test_transforms.py)
catkin_add_nosetests(test/test_config.py)
