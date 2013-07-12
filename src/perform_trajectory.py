#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory.converter import load_trajectory
from pr2_precise_trajectory.full_controller import FullPr2Controller
import sys
import yaml

if __name__ == '__main__':
    rospy.init_node('perform_trajectory')

    movements = load_trajectory(sys.argv[1])
    controller = FullPr2Controller()
    controller.do_action(movements)
    rospy.spin()

