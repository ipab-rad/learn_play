#!/usr/bin/python

import argparse

import rospy

import learn_play


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--limb', dest='limb', choices=['left', 'right'],
                        required=True, help='limb to control')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('learn_play_vision_%s' % (args.limb))

    c4v = learn_play.LPVision(args.limb)

    while not rospy.is_shutdown():
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
