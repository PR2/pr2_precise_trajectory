import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory import *
from pr2_precise_trajectory.msg import *
from pr2_controllers_msgs.msg import *
from actionlib import SimpleActionClient, SimpleActionServer

OPEN = 0.09
CLOSED = 0.002

ACTION_NAME = '/%s_gripper_controller/gripper_sequence'

class GripperController:
    def __init__(self, hand):
        arm = hand[0]
        self.effort = -1.0

        self.server = SimpleActionServer(ACTION_NAME % arm, GripperSequenceAction, self.execute, False) 
        self.server.start()

        self.sub_client= SimpleActionClient("%s_gripper_controller/gripper_action"%arm, Pr2GripperCommandAction)
        #wait for the action servers to come up 
        rospy.loginfo("[GRIPPER] Waiting for %s controllers"%arm)
        self.sub_client.wait_for_server()
        rospy.loginfo("[GRIPPER] Got %s controllers"%arm)

        self.client = SimpleActionClient(ACTION_NAME % arm, GripperSequenceAction)
        rospy.loginfo("[GRIPPER] Waiting for self client")
        self.client.wait_for_server()
        rospy.loginfo("[GRIPPER] Got self client")

    def send_goal(self, goal):
        self.client.send_goal(goal)

    def execute(self, goal):
        rate = rospy.Rate(10)
        while rospy.Time.now() < goal.header.stamp:
            rate.sleep()

        for position, time in zip(goal.positions, goal.times):
            self.change_position(position, False)
            rospy.sleep( time ) 
        self.server.set_succeeded(GripperSequenceResult())
        self.change_position(position, False, 0.0)
        

    def change_position(self, position, should_wait=True, effort=None):
        if effort is None:
            effort = self.effort

        gg = Pr2GripperCommandGoal()
        gg.command.position = position
        gg.command.max_effort = effort
        self.sub_client.send_goal(gg)
        if should_wait:
            self.sub_client.wait_for_result()

