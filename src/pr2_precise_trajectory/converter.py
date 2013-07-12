import roslib; roslib.load_manifest('pr2_precise_trajectory')
from pr2_precise_trajectory import *
from pr2_precise_trajectory.arm_controller import get_arm_joint_names
from pr2_precise_trajectory.head_controller import HEAD_JOINTS
from pr2_precise_trajectory.msg import *
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pickle
import yaml
import rospy

def load_trajectory(filename):
    if ".traj" in filename:
        trajectory = pickle.load(open(filename, 'r'))
        return trajectory_to_simple( trajectory )
    elif '.yaml' in filename:
        return yaml.load( open(filename, 'r'))
    else:
        print "Unknown file type"
        return None

def save_trajectory(trajectory, filename, width=1000):
    f = open(filename, 'w')
    f.write(yaml.dump(trajectory, width=width))
    f.close()


def simple_to_message_single(angles, duration, key):
    movements = {key: angles, TIME: duration}
    return simple_to_message([movements], key)

def simple_to_message(movements, key):
    trajectory = JointTrajectory()
    if key==HEAD:
        trajectory.joint_names = HEAD_JOINTS
    else:
        trajectory.joint_names = get_arm_joint_names(key)
    trajectory.header.stamp = rospy.Time.now()
    t=0
    for move in precise_subset(movements, key):
        pt = JointTrajectoryPoint()
        pt.positions = move[key]
        t+= get_time(move)
        pt.time_from_start = rospy.Duration(t)
        #pt.velocities = [0.0]*7
        trajectory.points.append(pt)
    return trajectory

def simple_to_move_sequence(movements, frame="/map", now=None, delay=0.0):
    nav_goal = MoveSequenceGoal()
    nav_goal.header.frame_id = frame
    for move in precise_subset(movements, BASE):
        t = get_time(move)
        pose = move[BASE]
        nav_goal.times.append(t-delay)
        p = Pose()
        p.position.x = pose[0]
        p.position.y = pose[1]
        q = quaternion_from_euler(0, 0, pose[2])
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        nav_goal.poses.append(p)
    if now is None:
        now = rospy.Time.now()
    nav_goal.header.stamp = now + rospy.Duration(delay)
    return nav_goal

def simple_to_gripper_sequence(movements, hand, now=None):
    goal = GripperSequenceGoal()
    for move in precise_subset(movements, hand):
        position = move[hand]
        t = get_time(move)
        goal.times.append(t)
        goal.positions += position
    if now is None:
        now = rospy.Time.now()
    goal.header.stamp = now
    return goal


def trajectory_to_simple(trajectory, fill_missing_with_zeros=True):
    indexes = {}
    for arm in [LEFT, RIGHT]:
        idx = []
        found = 0
        missing = 0
        for name in get_arm_joint_names(arm):
            if name not in trajectory.joint_names:
                idx.append(None)
                missing += 1
            else:
                idx.append(trajectory.joint_names.index(name))
                found += 1
            
        if found>0 and (fill_missing_with_zeros or missing == 0):
            indexes[arm] = idx

    if len(indexes)==0:
        print "ERROR: Neither arm defined"
        return []

    arr = []
    last_time = 0.0
    for point in trajectory.points:
        m = {}
        for arm, idx in indexes.iteritems():
            m[arm] = []
            for index in idx: 
                if index==None:
                    m[arm].append(0.0)
                else:
                    m[arm].append( point.positions[index] ) 
            #TODO Velocity
        time = point.time_from_start.to_sec()
        m[TIME] = time - last_time
        last_time = time
        arr.append(m)
    return arr
    
def simple_to_joint_states(movements, start_time=None):
    arr = []
    if start_time is None:
        start_time = rospy.Time.now()
    for move in movements:
        start_time += rospy.Duration( get_time(move) )
        state = JointState()
        state.header.stamp = start_time
        for arm in [LEFT, RIGHT]:
            if arm not in move:
                continue
            state.name += get_arm_joint_names(arm)
            state.position += move[arm]
        arr.append(state)
    return arr

def tprint(movements):
    for move in movements:
        print "%0.4f"% get_time(move) , 
        PRESET = [LEFT, RIGHT, HEAD, BODY]
        for key in PRESET:
            if key in move:
                j = ["%.3f"%x for x in move[key] ]
                print "%s: [%s]"%(key,",".join(j)),
        for x,a in move.iteritems():
            if x not in PRESET + [TIME]:
                print "%s: %s"%(x,str(a)),
        print 
