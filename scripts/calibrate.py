#!/usr/bin/python

import sys
import rospy
import rospkg
import baxter_interface
import tf
from copy import deepcopy

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
        self._baxter_limb = baxter_interface.Limb(self._limb)
        self._neutral_pos = {}
        self._default_pos = {}
        self._square_length = 50 # mm - this will also be the offset

        self._neutral_bool = False
        self._default_bool = True

        self.chess_pos = {} # dict of list (tuple) of dict (sigh)
        # for x in xrange(8):
        #     for y in xrange(8):
        #       self.chess_pos[(x,y)] = []

        self.pick_pos = {} # picking position
        self.br_pos = {} # bottom right position (7,0)
        self._the_pose = Pose()
        self._should_io = baxter_interface.DigitalIO(self._limb + '_shoulder_button')
        self._dash_io = baxter_interface.DigitalIO(self._limb + '_upper_button')
        self._circle_io = baxter_interface.DigitalIO(self._limb + '_lower_button')
        
        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        self._circle_io.state_changed.connect(self._default_point)
        self._should_io.state_changed.connect(self._neutral_point)
        self._dash_io.state_changed.connect(self._bottom_right_point)

        
    # def _find_joint_position(self, pose):
    #     ik_request = SolvePositionIKRequest()
    #     goal_pose = Pose()
    #     goal_pose.position = pose['position']
    #     goal_pose.orientation = pose['orientation']

    #     hdr = Header(stamp=rospy.Time.Now(), frame_id='base')
    #     pose_request = PoseStamped(header=hdr, pose=goal_pose)
    #     ikreq.pose_stamp.append(pose_request)
    #     resp = self._iksvc(ik_request)
    #     return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _find_joint_position(self, pose, x_off = 0.0, y_off = 0.0, z_off = 0.0):
        ik_request = SolvePositionIKRequest()
        # try:
        #     print 
        #     pose['position'] = Point(x=pose['position'][0] + x_off,
        #                              y=pose['position'][1] + y_off,
        #                              z=pose['position'][2] + z_off
        #                              )
        # except Exception:
        the_pose = deepcopy(pose)
        the_pose['position'] = Point(x=pose['position'].x + x_off,
                                 y=pose['position'].y + y_off,
                                 z=pose['position'].z + z_off
                                 )
        print the_pose
        approach_pose = Pose()
        approach_pose.position = the_pose['position']
        approach_pose.orientation = the_pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ik_request.pose_stamp.append(pose_req)
        resp = self._iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _blink_light(self, io_component='left_itb_light_outer'):
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

    def _default_point(self, value):
        if value:                
            if len(self._default_pos) == 0:
                # Record default position
                print 'Recording default location'
                self._default_pos[0] = self._baxter_limb.joint_angles()
                self._default_pos[1] = self._find_joint_position(
                                         self._baxter_limb.endpoint_pose(),
                                         z_off = 0.10)
                self._neutral_bool = True
                self._default_bool = False
    
    def _neutral_point(self, value):
        if value:                
            if len(self._neutral_pos) == 0:
                # Record default position
                print 'Recording neutral location'
                self._neutral_pos = self._baxter_limb.joint_angles()
                self._neutral_bool = False

    def _bottom_right_point(self, value): # 7, 0
        if value:                
            if len(self.br_pos) == 0:
                # Record default position
                print 'Recording pick location'
                self.br_pos[0] = self._baxter_limb.joint_angles()
                self._the_pose = self._baxter_limb.endpoint_pose()
                self.br_pos[1] = self._find_joint_position(
                                         self._the_pose,
                                         z_off = 0.10)
                # print "done"

    def generate_positions(self):
        if len(self.br_pos) != 0:
            # go to 7,0 position
            cur_bottom_pose = self._the_pose
            print cur_bottom_pose
            # cur_top_pose = self.br_pos[1]
            # self._baxter_limb.move_to_joint_positions(self.br_pos[0])
            for y in range(8):
                for x in range(8):
                    x_o = -0.05*(7-x)
                    y_o = -1 * (y * 0.05)
                    t = (x,y)
                    one = self._find_joint_position(
                        cur_bottom_pose,
                        x_off = x_o,
                        y_off = y_o
                    )
                    
                    two = self._find_joint_position(
                        cur_bottom_pose,
                        x_off = x_o,
                        y_off = y_o,
                        z_off = 0.10
                    )
                    rospy.sleep(0.2)
                    self.chess_pos[t] = [one, two]
                    print "-------------------"
                    print t
                    print self.chess_pos[t][0]
                    print t
                    print self.chess_pos[t][1]

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
                while(len(self._default_pos) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Default gripping position - Registered.")

                print ("Move the " + self._limb + " arm to the neutral "
                       "position (for picking) and "
                       "press Circle button ")
                while(len(self._neutral_pos) == 0 and not rospy.is_shutdown()):
                    # print "lol"
                    rospy.sleep(0.1)
                print ("Default gripping position - Registered.")

                print ("Move same arm to (7,0) position and press the"
                       "dash button to record")
                while(len(self.br_pos) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("(0, 7) playing position - Registered.")
                self.generate_positions()
                
                good_input = True



def main():
    rospy.init_node("learn_play_calibrate")
    
    rs = baxter_interface.RobotEnable()
    rs.enable()
    cal = Calibrate("left")
    cal.get_locations()
    for i in range(8):
        for k in range(8):
            print "Go to pos", i, ",", k
            cal._baxter_limb.move_to_joint_positions(cal._neutral_pos)
            cal._baxter_limb.move_to_joint_positions(cal.chess_pos[(i, k)][0])
    # cal._baxter_limb.move_to_joint_positions(cal._neutral_pos)
    # l = cal.chess_pos[(0,0)][0]
    # print l
    # cal._baxter_limb.move_to_joint_positions(l)
    # cal._baxter_limb.move_to_joint_positions(cal._neutral_pos)
    # l2 = cal.chess_pos[(0,0)][0]
    # print l2
    # cal._baxter_limb.move_to_joint_positions(l2)
    # cal._baxter_limb.move_to_joint_positions(cal._neutral_pos)
    # l3 = cal.br_pos[1]
    # print l3
    # cal._baxter_limb.move_to_joint_positions(l3)
    # cal._baxter_limb.move_to_joint_positions(cal._neutral_pos)
    # l4 = cal.br_pos[0]
    # print l4
    # cal._baxter_limb.move_to_joint_positions(l4)

if __name__ == "__main__":
    sys.exit(main())
