#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image


print "setup..."
rospy.init_node("my_cam_bl")
display_pub = rospy.Publisher('/robot/xdisplay', Image)


def republish(msg):
        """
            Sends the camera image to baxter's display
        """
        print "lulz"
        display_pub.publish(msg)


print("LOL")
right_camera = baxter_interface.CameraController("right_hand_camera")

right_camera.resolution = [960, 600]
right_camera.open()
camera_name = "right_hand_camera"
camera_topic = '/cameras/' + camera_name + '/image'
print "LAL"
sub = rospy.Subscriber(camera_topic,
                       Image, republish, None, 1)
while not rospy.is_shutdown():
    print "LEL"
    rospy.sleep(1.0)

# 365, 235     700,235

# 358, 593     706, 593
