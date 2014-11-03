#!/usr/bin/python

import rospy

import learn_play

import baxter_interface


def main():
    limb = "left"
    rospy.init_node('learn_play_vision_%s' % (limb))
    baxter_limb = baxter_interface.Limb(limb)
    starting_pos = {'left_e0': -0.24620391617431642,
                    'left_e1': -0.03873301484985352,
                    'left_s0': -1.0902768437438966,
                    'left_s1': -0.8329515668701173,
                    'left_w0': -0.39269908125,
                    'left_w1': 2.093500277874756,
                    'left_w2': 0.7616214603149415}
    baxter_limb.move_to_joint_positions(starting_pos)
    lpv = learn_play.LPVision(limb)
    # lpv.check_board()
    while not rospy.is_shutdown():
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
