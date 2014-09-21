#!/usr/bin/python

import sys
import rospy
import rospkg
import baxter_interface

from copy import deepcopy
import time

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    # Quaternion,
)

from std_msgs.msg import Header

# from sensor_msgs.msg import (
#     # Image,
#     # JointState,
#     )

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class Ik_test(object):

    def __init__(self, limb="left"):
        self._rp = rospkg.RosPack()

        self._limb = limb
        self._baxter_limb = baxter_interface.Limb(self._limb)

        self._some_pos = {}
        self._the_pose = Pose()

        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()
        self._read_path = self._rpb.get_path('learn_play') + "read.txt"

    def go_to_pos(self, x, y):
        # a_pos = self._baxter_limb.joint_angles()
        print "Well"
        s_pose = self._baxter_limb.endpoint_pose()
        print s_pose
        r_pos = self._find_joint_position(
            s_pose,
            x_off=x,
            y_off=y)
        print r_pos
        self._baxter_limb.move_to_joint_positions(r_pos)

    def _find_joint_position(self, pose, x_off=0.0, y_off=0.0, z_off=0.0):
        '''
        Finds the joint position of the arm given some pose and the
        offsets from it (to avoid opening the structure all the time
        outside of the function).
        '''

        ik_request = SolvePositionIKRequest()
        the_pose = deepcopy(pose)
        the_pose['position'] = Point(x=pose['position'].x + x_off,
                                     y=pose['position'].y + y_off,
                                     z=pose['position'].z + z_off)
        approach_pose = Pose()
        approach_pose.position = the_pose['position']
        approach_pose.orientation = the_pose['orientation']
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ik_request.pose_stamp.append(pose_req)
        resp = self._iksvc(ik_request)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def move_to_offs(self, x, y, z):
        """
        In cm.
        """

        # x = raw_input("x: ")
        # y = raw_input("y: ")
        # z = raw_input("z: ")

        self.go_to_pos(float(x)/100.0, float(y)/100.0, float(z)/100.0)

    def read_file(self, path):
        f = open(path, "rt")
        where = f.tell()
        line = f.readline()
        if not line:
            time.sleep(1)
            f.seek(where)
        else:
            return line.split(" ")


def main():

    """
    Expected x y z
    """

    rospy.init_node("ik_test")
    rs = baxter_interface.RobotEnable()
    rs.enable()
    ikt = Ik_test("left")
    the_id = ""
    while (1):
        l = ikt.read_file()
        if the_id != l[0]:
            print "New value"
            x = float(l[1])
            y = float(l[2])
            z = float(l[3])
            ikt.move_to_offs(x, y, z)


if __name__ == "__main__":
    sys.exit(main())
