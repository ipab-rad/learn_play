#!/usr/bin/python

import sys
import rospy
import rospkg
import baxter_interface
import tf

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from sensor_msgs.msg import (
    Image,
    JointState,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



import learn_play

class Calibrate(object):
    
    def __init__(self, limb):
        self._rp = rospkg.RosPack()
        self._config_path = self._rp.get_path('learn_play') + '/config/'

        self._limb = limb
        self._baxter_limb = baxter_interface.Limb(self.limb)

        self._default_pos = {}
        self._square_length = 50 # mm - this will also be the offset


        self._dash_io = baxter_interface.DigitalIO(self.limb + '_upper_button')
        self._circle_io = baxter_interface.DigitalIO(self.limb + '_lower_button')
        self._dash_io.state_changed.connect(self._register_pos_one)
        self.circle_io.state_changed.connect(self._register_pos_two)
        
        
        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        
    def _find_joint_position(self, pose):
        ik_request - SolvePositionIKRequest()
        goal_pose = Pose()
        goal_pose.position = pose['position']
        goal_pose.orientation = pose['orientation']
        
        hdr = Header(stamp=rospy.Time.Now(), frame_id='base')
        pose_request = PoseStamped(header=hdr, pose=goal_pose)
        ikreq.pose_stamp.append(pose_request)
        resp = self._iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _find_approach(self, pose, offset):
        ik_request = SolvePositionIKRequest()
        # Add 5 cm offset in Z direction
        try:
            pose['position'] = Point(x=pose['position'][0],
                                     y=pose['position'][1],
                                     z=pose['position'][2] + offset
                                     )
        except Exception:
            pose['position'] = Point(x=pose['position'].x,
                                     y=pose['position'].y,
                                     z=pose['position'].z + offset
                                     )
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_request = PoseStamped(header=hdr, pose=approach_pose)
        ikreq.pose_stamp.append(pose_request)
        resp = self._iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _blink_light(io_component=self.limb + '_itb_light_outer'):
        """Blinks a Digital Output on then off."""
        # rospy.loginfo("Blinking Digital Output: %s", io_component)
        b = baxter_interface.digital_io.DigitalIO(io_component)
        
        print "Initial state: ", b.state

        # turn on light
        b.set_output(True)
        rospy.sleep(1)
        # print "New state: ", b.state

        # reset output
        b.set_output(False)
        rospy.sleep(1)
        # print "Final state:", b.state

def main():
    rospy.init_node("learn_play_calibrate")
    
    rs = baxter_interface.RobotEnable()
    rs.enable()
    cal = Calibrate("left")
    
    

if __name__ == "__main__":
    sys.exit(main())
