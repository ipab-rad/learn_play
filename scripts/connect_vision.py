#!/usr/bin/python

import argparse

import rospy

import connect_four


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--limb', dest='limb', choices=['left', 'right'],
                        required=True, help='limb to control')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('connect_four_vision_%s' % (args.limb))

    c4v = connect_four.LPVision(args.limb)

    while not rospy.is_shutdown():
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
