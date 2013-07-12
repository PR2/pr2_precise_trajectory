import roslib; roslib.load_manifest('pr2_precise_trajectory')
from pr2_precise_trajectory import *
import rospy
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
import tf

class JointWatcher:
    def __init__(self, name_map):
        self.name_map = name_map
        self.key_map = {}
        for key, names in name_map.iteritems():
            for name in names:
                self.key_map[name] = key

        self.state = {}
        self.data = []
        self.start_time = None
        self.done = False

        self.tf = None
        self.frame = None

        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb)        

    def add_tf(self, tf, frame='/map'):
        self.tf = tf
        self.frame = frame
        self.key_map[BASE] = ['x', 'y', 'theta']

    def joint_cb(self, msg):
        self.state = {}
        for key, names in self.name_map.iteritems():
            pos = []
            for name in names:
                i = msg.name.index(name) 
                pos.append( msg.position[i] )
            self.state[key] = pos

        if BASE in self.key_map and self.tf:
            attempts = 0
            while attempts < 10:
                try:
                    (trans,rot) = self.tf.lookupTransform(self.frame, '/base_footprint', rospy.Time(0))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    attempts += 1
                    continue
            if attempts < 10:
                euler = euler_from_quaternion(rot)
                self.state[BASE] = [trans[0], trans[1], euler[2]]
            else:
                rospy.logerr("TF ERROR!")

        self.state[TIME] = msg.header.stamp

        if self.start_time is not None and not self.done:
            self.data.append(self.state)

    def get_positions(self, key):
        return self.state[key]

    def get_state(self):
        return self.state

    def record(self):
        self.start_time = rospy.Time.now()
        self.data = []
        self.done = False
    
    def stop(self, delay=0.0):
        self.done = True
        last = None
        for move in self.data:
            if last is None:
                last = move[TIME]
                move[TIME] = delay+ 0.0
            else:
                temp = move[TIME]
                move[TIME] = (temp - last).to_sec()
                last = temp
        return self.data

