#!/usr/bin/python

import argparse

import rospy
import rospkg

import learn_play

import baxter_interface

def main():
    limb = "right"
    
    rospy.init_node('learn_play_vision_%s' % (limb))
    
    baxter_limb = baxter_interface.Limb(limb)    
    # todo: make configurable
    starting_pos = {'right_s0': 0.5092816209960938,
                    'right_s1': -1.20800986907959,
                    'right_w0': -0.009203884716796876,
                    'right_w1': 1.7054031389831543,
                    'right_w2': -1.0177962515991212,
                    'right_e0': 0.7853981625,
                    'right_e1': 1.038121497015381}

    baxter_limb.move_to_joint_positions(starting_pos)

    lpv = learn_play.LPVision(limb)
    while not rospy.is_shutdown():
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
