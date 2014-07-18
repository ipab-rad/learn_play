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

        self._no_squares = 8
        self._grid = ([[0 for _i in range(self._no_squares)] 
                      for _j in range(self._no_squares)])
        self._positions_top = {}
                
        # for i in xrange(len(self.grid)):
        #     print self.grid[i]
            
        self.game_off  = False
        self._turn = 0 # 1, 2 = baxter, human
        self._no_pieces = 0 # Number of pieces on board


    def place_piece(self, pos = 0):
        print rospy.Time.now(), "Placing piece to", pos

        if (pos != 0):
            self._baxter_limb.move_to_joint_positions(
                self._positions_top[pos])
            # closed
            self._baxter_limb.move_to_joint_positions(
                self._positions_bottom[pos])
            self._baxter_gripper.open()
            rospy.sleep(0.5)
            self._baxter_limb.move_to_joint_positions(
                self._positions_top[pos])
        else:
            print "Specify place!"
                
        
def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--limb', dest='limb', choices=['left', 'right'],
                        required=True, help='limb to control')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_learn_play_%s' % (args.limb,)) 
    lp = LearnPlay(args.limb)
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
