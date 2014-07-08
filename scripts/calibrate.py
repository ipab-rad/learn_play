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

        self.pick_pos = {} # picking position
        self.br_pos = {} # bottom right position (0,7)
        self._dash_io = baxter_interface.DigitalIO(self.limb + '_upper_button')
        self._circle_io = baxter_interface.DigitalIO(self.limb + '_lower_button')
        self._dash_io.state_changed.connect(self._register_pos_one)
        self.circle_io.state_changed.connect(self._register_pos_two)
        
        
        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        circle_io.state_changed.connect(self._pick)
        dash_io.state_changed.connect(self._place)

        
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

    def pick_point(self, value):
        if value:
            
    def _save_file(self, file):
        print "Saving your positions to file!"
        f = open(file, 'w')
        f.write('camera=' + str(self.camera_jp) + '\n')
        f.write('pick=' + str(self.pick_location) + '\n')
        f.write('pick_approach=' + str(self.pick_approach) + '\n')
        f.write('neutral=' + str(self.neutral_jp) + '\n')
        f.close()

    def get_locations(self):
        good_input = False
        while not good_input:
            self.read_file = raw_input("Are you sure you really want to"
                                       " overwrite your previous changes"
                                       "(y/n)?")
            if self.read_file != 'y' and self.read_file != 'n':
                print "You must answer 'y' or 'n'"
            
            elif self.read_file == 'n':
                print "Alright then: using previous values."
                good_input = True
            
            else:
                print ("Move the " + self._limb + " arm to the default "
                       "position (for picking) and "
                       "press Circle button ")
                while(len(self.pick_pos) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Default gripping position - Registered.")

                print ("Move same arm to (0, 7) position and press the"
                       "dash button to record")
                while(len(self.br_pos) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("(0, 7) playing position - Registered.")
                
                good_input = True



def main():
    rospy.init_node("learn_play_calibrate")
    
    rs = baxter_interface.RobotEnable()
    rs.enable()
    cal = Calibrate("left")
    cal.pick_point()
    

if __name__ == "__main__":
    sys.exit(main())
