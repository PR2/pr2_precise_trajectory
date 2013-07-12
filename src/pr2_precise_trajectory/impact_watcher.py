import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_controllers_msgs.msg import *
from pr2_gripper_sensor_msgs.msg import *
from actionlib import SimpleActionClient, SimpleGoalState
from sensor_msgs.msg import JointState
import trajectory_msgs.msg

class ImpactWatcher:
    def __init__(self, names=['r_gripper_sensor_controller'], rate=200):
        self.accel = rospy.get_param('acceleration_trigger', 70.0)
        self.slip = rospy.get_param('slip_trigger', 0.008)
        self.trigger = PR2GripperEventDetectorCommand.ACC  # use just acceleration as our contact signal
        self.clients = {}
        for name in names:
            self.clients[name] = SimpleActionClient("%s/event_detector"%name, PR2GripperEventDetectorAction)
            #wait for the action servers to come up 
            rospy.loginfo("[IMPACT] Waiting for %s controllers"%name)
            self.clients[name].wait_for_server()
            rospy.loginfo("[IMPACT] Got %s controllers"%name)

        self.rate = rospy.Rate(rate)

    def wait_for_impact(self):
        place_goal = PR2GripperEventDetectorGoal()
        place_goal.command.trigger_conditions                  = self.trigger
        place_goal.command.acceleration_trigger_magnitude = self.accel  # m/^2
        place_goal.command.slip_trigger_magnitude            = self.slip     # slip gain

        for name, client in self.clients.iteritems():
            client.send_goal(place_goal)

        #wait for a slap
        while(not self.is_slap_done() and not rospy.is_shutdown()):
            self.rate.sleep()

    def is_slap_done(self):
        done = True
        for name, client in self.clients.iteritems():
            done = done and client.get_state() > SimpleGoalState.ACTIVE
        return done


