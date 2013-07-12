import roslib; roslib.load_manifest('pr2_precise_trajectory')
import rospy
from pr2_precise_trajectory import *
from pr2_precise_trajectory.arm_controller import *
from pr2_precise_trajectory.gripper_controller import *
from pr2_precise_trajectory.base_controller import *
from pr2_precise_trajectory.head_controller import *
from pr2_precise_trajectory.impact_watcher import *
from pr2_precise_trajectory.joint_watcher import *
from pr2_precise_trajectory.converter import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def transition_split(movements):
    first = movements[0]
    chunks = [[first]]
    last_transition = first.get('transition', 'wait')
    
    for move in movements[1:]:
        transition = move.get('transition', 'wait')
        if transition==last_transition and transition=='wait':
            chunks[-1].append(move)
        else:
            last_transition = transition
            chunks.append( [move] )
    return chunks
    

class FullPr2Controller:
    def __init__(self, keys=[LEFT, RIGHT, HEAD, BASE, LEFT_HAND, RIGHT_HAND], impact=True):
        self.keys = keys
        self.arms = {}

        joint_map = {}
        for arm in [LEFT, RIGHT]:
            if arm not in keys:
                continue
            self.arms[arm] = ArmController(arm)
            joint_map[arm] = get_arm_joint_names(arm)

        self.hands = {}
        for hand in [LEFT_HAND, RIGHT_HAND]:
            if hand not in keys:
                continue
            self.hands[hand] = GripperController(hand)
            joint_map[hand] = ['%s_gripper_joint'%hand[0]]

        if HEAD in keys:
            self.head = HeadController() 
            joint_map[HEAD] = ['head_pan_joint', 'head_tilt_joint']
        else:
            self.head = None

        self.impacts = ImpactWatcher(['%s_gripper_sensor_controller'%arm for arm in self.arms.keys()]) if impact else None
        self.joint_watcher = JointWatcher(joint_map)

        if BASE in keys:
            self.base = BaseController()
            self.joint_watcher.add_tf(self.base.tf)
        else:
            self.base = None

    def do_action(self, movements):
        if len(movements)==0:
            return

        chunks = transition_split(movements)

        for ms in chunks:
            clients = []
            for key in self.keys:
                sub = precise_subset(ms, key)
                if len(sub)==0:
                    continue
                if key==BASE:
                    seq = simple_to_move_sequence(sub)
                    self.base.send_goal(seq)
                    clients.append(self.base.client)
                elif key==LEFT_HAND or key==RIGHT_HAND:
                    seq = simple_to_gripper_sequence(sub, key)
                    self.hands[key].send_goal(seq)
                    clients.append(self.hands[key].client)
                else:
                    traj = simple_to_message(sub, key)
                    if key in self.arms:
                        self.arms[key].start_trajectory(traj, wait=False)
                        clients.append(self.arms[key].client)
                    elif key==HEAD:
                        self.head.start_trajectory(traj, wait=False)
                        clients.append(self.head.client)

            transition = ms[0].get('transition', 'wait')
            if transition=='wait':
                for client in clients:
                    client.wait_for_result()
            elif transition=='impact':
                rospy.sleep(.1)
                self.impacts.wait_for_impact()
                self.stop_arm()

    def stop_arm(self, time=0.1):
        for arm in self.arms:
            trajectory = simple_to_message_single(self.joint_watcher.get_positions(arm), time, arm)
            self.arms[arm].start_trajectory(trajectory, wait=False)
        rospy.sleep(time)


