# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_precise_trajectory)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS std_msgs rospy trajectory_msgs geometry_msgs sensor_msgs tf actionlib actionlib_msgs pr2_controllers_msgs pr2_gripper_sensor_msgs message_generation message_runtime)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_find_ros_package(actionlib_msgs)




# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
add_message_files(
  FILES
  # TODO: List your msg files here
)

add_action_files(
  FILES
  GripperSequence.action
  MoveSequence.action
)


generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy trajectory_msgs geometry_msgs sensor_msgs tf actionlib actionlib_msgs pr2_controllers_msgs pr2_gripper_sensor_msgs
    CATKIN_DEPENDS message_runtime# TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

