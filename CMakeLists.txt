# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_precise_trajectory)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rospy trajectory_msgs geometry_msgs sensor_msgs tf actionlib actionlib_msgs pr2_controllers_msgs pr2_gripper_sensor_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
add_message_files(
  FILES
  # TODO: List your msg files here
)


# CATKIN_MIGRATION
# use bloom tool
# rosbuild_make_distribution(0.1.0)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy trajectory_msgs geometry_msgs sensor_msgs tf actionlib actionlib_msgs pr2_controllers_msgs pr2_gripper_sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

