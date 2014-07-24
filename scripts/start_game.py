#!/usr/bin/python

import argparse
import sys
from copy import deepcopy
import os
import rospkg
import rospy
from std_msgs.msg import String
import baxter_interface

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
        self._config_path = self._rp.get_path('learn_play') + '/config/positions.config'

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
        self._grid = ([[0 for _i in range(self._no_squares)] 
                      for _j in range(self._no_squares)])
        
        self.game_off  = False
        self._turn = 0 # 1, 2 = baxter, human
        self._no_pieces = 0 # Number of pieces on board
        if (not self._check_config()):
            exit(0)
        self._read_config()

        


    def _check_config(self):
        ri = ""
        while (1):
            ri = raw_input("Have you calibrated the arm? [y/n]")
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
                self._chess_pos[(x,y)] = eval(splitted.pop(0)[1]) 
        print "Positions are in memory."
        f.close()


    def pick_piece(self, pos = "default"):
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
        rospy.sleep(0.5) #sigh
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
            self._baxter_limb.move_to_joint_positions(
                self._chess_pos[pos][0])
            self._baxter_gripper.open()
            print "Piece has been placed."
            self.picked = False
            rospy.sleep(0.5) # sigh
            self._baxter_limb.move_to_joint_positions(
                self._chess_pos[pos][1])
        except Exception:
            print "Unknown location!" # This sucks. Seriously.
                
        
def main():
    
    limb = "left"
    rospy.init_node('rsdk_learn_play_%s' % (limb)) 
    lp = LearnPlay(limb)
    while(1):
        for i in range(8):
            lp.pick_piece()
            lp.place_piece((i,i))
            
    
    # 1 get data from vision
    # 2 start arms stuff
    # 3 wait for user pieces (3?)
    # 4 calculates next move
    # 5 make move
    # 6 goto 3

if __name__ == "__main__":
    main()
