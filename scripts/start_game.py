#!/usr/bin/python

import sys

import rospkg
import rospy
from std_msgs.msg import (
    String,
    Image,
)
import baxter_interface

import cv
import cv_bridge
import learn_play


class LearnPlay(object):
    """This class defines the actual game structure (and procedure). It
    receives input from the vision system, input that is then sent to
    the AI which returns the next action to do (which then this class
    will carry on).

    """
    def __init__(self, limb):
        # Limb is the one that is NOT used by the vision client
        # Grid state
        #  0 - free
        #  1 - occupied

        self._rp = rospkg.RosPack()
        self._config_path = self._rp.get_path('learn_play') + '/config/'
        self._images_path = self._rp.get_path('learn_play') + '/share/images/'
        self._config_file_path = self._config_path + 'positions.config'
        self._good_face_path = self._images_path + "good_face.jpg"
        self._angry_face_path = self._images_path + "angry_face.jpg"

        self._limb = limb
        self._baxter_limb = baxter_interface.Limb(self._limb)
        self._baxter_gripper = baxter_interface.Gripper(self._limb)
        print "Calibrating gripper..."
        self._baxter_gripper.calibrate()

        self._default_pos = {}
        self._neutral_pos = {}
        self._chess_pos = {}
        self.picked = False

        self._no_squares = 8

        self.game_off = False
        self._turn = 0  # 1, 2 = baxter, human
        self._no_pieces = 0  # Number of pieces on board
        if (not self._check_config()):
            exit(0)
        self._read_config(self._config_file_path)

        self._grid = None
        vision_topic = "/vision/learn_play_state/"
        _grid_sub = rospy.Subscriber(vision_topic,
                                     String,
                                     self._on_state)
        err_msg = ("Failed waiting for vision to "
                   "be published on topic %s - start vision node" %
                   vision_topic)

        # Wait for vision node to start
        it = rospy.Time.now()
        while(not rospy.is_shutdown()):
            rospy.sleep(0.1)
            if self._grid is not None:
                break
            if rospy.Time.now() - it > rospy.Duration(2):
                print err_msg
                sys.exit(1)
            it = rospy.Time.now()

    def _on_state(self, msg):
        data = eval(msg.data)
        self._grid = data["baxter_count"]

    def _check_config(self):
        ri = ""
        while (1):
            ri = raw_input("Have you calibrated the arm? [y/n] ")
            if ri.lower() == "y":
                print "Awesome. Carry on."
                return True
            elif ri.lower() == "n":
                print ">> run `rosrun learn_play calibrate.py` first! <<"
                return False

    def _read_config(self, file):
        """
        Read positions from config file.
        """

        print "Reading positions from file."
        f = open(file, 'r')
        lines = f.readlines()
        splitted = [line.split("=") for line in lines]
        self._default_pos = eval(splitted.pop(0)[1])
        self._neutral_pos = eval(splitted.pop(0)[1])
        for x in range(8):
            for y in range(8):
                # This must be really slow
                self._chess_pos[(x, y)] = eval(splitted.pop(0)[1])
        print "Positions are in memory."
        f.close()

    def gripper_open(self, percentage):
        if percentage < 100:
            return self._baxter_gripper.command_position(percentage)
        else:
            return False

    def pick_piece(self, pos="default"):
        if pos != "default":
            print "Cannot use any other position!"
            return
        if self.picked:
            print "Cannot pick a piece while holding another one!"
            return
        self._baxter_limb.move_to_joint_positions(
            self._neutral_pos
        )
        print rospy.Time.now(), "Picking piece..."
        self._baxter_limb.move_to_joint_positions(
            self._default_pos[1]
        )
        self._baxter_limb.move_to_joint_positions(
            self._default_pos[0]
        )
        rospy.sleep(0.5)  # sigh
        self._baxter_gripper.close()
        self._baxter_limb.move_to_joint_positions(
            self._default_pos[1]
        )
        self.picked = True
        print "Piece has been picked."

    def place_piece(self, pos):
        if not self.picked:
            print "Pick a piece first!"
            return
        try:
            self._baxter_limb.move_to_joint_positions(
                self._neutral_pos
            )
            print rospy.Time.now(), "Placing piece to", pos, "..."
            self._baxter_limb.move_to_joint_positions(
                self._chess_pos[pos][1])
            # closed
            self._baxter_limb.set_joint_position_speed(0.05)
            self._baxter_limb.move_to_joint_positions(
                self._chess_pos[pos][0])
            # self._baxter_gripper.open()
            self.gripper_open(50)
            print "Piece has been placed."
            self.picked = False
            rospy.sleep(0.5)  # sigh
            self._baxter_limb.set_joint_position_speed(0.05)
            self._baxter_limb.move_to_joint_positions(
                self._chess_pos[pos][1])
            self._baxter_limb.set_joint_position_speed(0.5)
            self._baxter_gripper.open()
        except Exception:
            print "Unknown location!"  # This sucks. Seriously.

    def make_diag(self):
        for i in range(8):
            self.pick_piece()
            self.place_piece((i, i))

    def check_diag(self, buf):
        for i in range(8):
            if not (i, i) in buf:
                return [False, (i, i)]
        return [True, (-1, -1)]  # mmmmh

    def fill_diag(self):
        (b, t) = self.check_diag(self._grid)
        if not b:
            self.send_image(self._angry_face_path)
            self.pick_piece()
            self.place_piece(t)
        else:
            self.send_image(self._good_face_path)

    def send_image(self, path):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        img = cv.LoadImage(path)
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(1)


def main():

    limb = "right"
    rospy.init_node('learn_play_%s' % (limb))
    lp = LearnPlay(limb)
    while(1):
        lp.fill_diag()


if __name__ == "__main__":
    main()
