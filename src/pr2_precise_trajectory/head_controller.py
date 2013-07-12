import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from actionlib import SimpleActionClient, SimpleGoalState
import trajectory_msgs.msg

HEAD_JOINTS = ['head_pan_joint', 'head_tilt_joint']

class HeadController:
    def __init__(self):
        self.client = SimpleActionClient('/head_traj_controller/joint_trajectory_action', JointTrajectoryAction)
        #wait for the action servers to come up 
        rospy.loginfo("[HEAD] Waiting for controller")
        self.client.wait_for_server()
        rospy.loginfo("[HEAD] Got controller")

    def start_trajectory(self, trajectory, set_time_stamp=True, wait=True):
        """Creates an action from the trajectory and sends it to the server"""
        goal = JointTrajectoryGoal()
        goal.trajectory = trajectory
        if set_time_stamp:
            goal.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(goal)

        if wait:
            self.wait()

    def wait(self):
        self.client.wait_for_result()

    def is_done(self):
        return self.client.get_state() > SimpleGoalState.ACTIVE

