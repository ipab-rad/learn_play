#!/usr/bin/python

import rospy

import learn_play

import baxter_interface


def main():
    limb = "right"

    rospy.init_node('learn_play_vision_%s' % (limb))

    baxter_limb = baxter_interface.Limb(limb)
    starting_pos = {'right_s0': -0.03873301484985352,
                    'right_s1': -0.757019517956543,
                    'right_w0': -0.794602047216797,
                    'right_w1': 1.7203594516479492,
                    'right_w2': 0.09203884716796876,
                    'right_e0': 1.2597817206115725,
                    'right_e1': 0.9794467319458009}
    baxter_limb.move_to_joint_positions(starting_pos)
    lpv = learn_play.LPVision(limb)
    while not rospy.is_shutdown():
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
