import argparse
import threading
from copy import deepcopy
from os import system

import numpy as np
import cv2
from cv2 import cv
from cv_bridge import CvBridge

import rospy
import baxter_interface

from geometry_msgs.msg import (
  PolygonStamped,
)

from sensor_msgs.msg import (
  Image,
)

from std_msgs.msg import (
  String,
)

# TODO: Document all the things.

class LPVision(object):
    def __init__(self, limb):
        # Start camera
        self._side = limb
        self._camera_name = self._side + "_hand_camera"
        print ("Opening " + self._camera_name + "...")
        self._camera = baxter_interface.CameraController(self._camera_name)
        self._camera.open()
        self._camera.resolution = [1280, 800]
        self._camera.gain = 30

        self._side_roi = 400
        self._no_squares = 8
        # 400 / 8 = 50
        self._square_side_roi = self._side_roi / self._no_squares

        self.grid = [[0 for _i in range(self._no_squares)] for _j in range(self._no_squares)]
        print self.grid
        self.cv_image = None
        self._bridge = CvBridge()

        # self.yellow_sample = ()
        self.red_sample = ()
        self.blue_sample = ()

        # self.yellows = None
        self.reds = None
        self.blues = None

        # more central
        # roi = region of interest
        self._roi_points = [[100, 200], [100, 100], [200, 100], [200, 200]] # magic
        self._roi_move = False
        self._point_selected = -1
        self._gain_slider = 30
        self._red_thresh = 30
        # self._yellow_thresh = 100
        self._slider_time = rospy.Time.now()
        self._gain_set = False
        self._text = ['X', 'Y', 'R', 'G', 'B']
        self._pixel = {}
        # initialize data structure
        for label in self._text:
            self._pixel[label] = 0.0
        self._vector = {}

        self._grid = [[0 for _i in range(self._no_squares)] for _j in range(self._no_squares)]
        self._pnts = [[0 for i in range(self._square_side_roi + 1)] for j in range(self._square_side_roi + 1)]

        self._pieces_positions = []

        # initialize images
        self._np_image = np.zeros((self._side_roi, self._side_roi, 3), np.uint8)
        self._image_grid = np.zeros((self._side_roi, self._side_roi, 3), np.uint8)
        # self._yellow = np.zeros((self._side_roi, self._side_roi), np.uint8)
        self._red = np.zeros((self._side_roi, self._side_roi), np.uint8)
        self._projected = np.zeros((self._side_roi, self._side_roi, 3), np.uint8)

        self.subLock = threading.Lock()

        camera_topic = '/cameras/' + self._camera_name + '/image'
        _camera_sub = rospy.Subscriber(
            camera_topic,
            Image,
            self._on_camera)

        roi_topic = '/learn_play/localize/grid_pixels'
        _roi_sub = rospy.Subscriber(
            roi_topic,
            PolygonStamped,
            self._on_roi)

        board_state_topic = '/vision/learn_play_state'
        self._board_state_pub = rospy.Publisher(
            board_state_topic,
            String)

        print ' - All set - '
        print " - - - - - - - "
        self._process_images()

    def _process_images(self):

        print " - Starting to process images! - "
        while not rospy.is_shutdown():

            # gain changed from slider settled
            if (rospy.Time.now() - self._slider_time > rospy.Duration(2.0)
                and self._gain_set == True):
                self._gain_set = False
                print 'Setting GAIN!'
                self._camera.gain = self._gain_slider

            # process red/yellow image
            self._show_image()
            self._project_roi()
            # self._filter_yellow()
            self._filter_red()
            self._process_colors(deepcopy(self._red))
            self._update_image_grid()

            # publish state
            self._pub_state()
            rospy.sleep(0.1)

    def _show_image(self):

        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()

        # draw circles
        for idx, points in enumerate(self._roi_points):
            cv2.circle(local_image, (points[0], points[1]), 5, (255, 0, 0), 2)

        # draw green lines
        cv2.polylines(local_image, np.int32([np.array(self._roi_points)]),
                      1, (0, 255, 0), 2)

        cv.ShowImage("Learn Play game RGB", cv.fromarray(local_image))

        cv.SetMouseCallback("Learn Play game RGB", self._on_mouse_click, 0)
        cv.CreateTrackbar("Gain", "Learn Play game RGB", self._gain_slider,
                          100, self._on_gain_slider)
        cv.CreateTrackbar("Red Threshold", "Learn Play game RGB",
                          self._red_thresh, 500, self._on_red_slider)
        # TODO: create HSV bar for colour
        cv.WaitKey(3)

        # projects roi chosen by user

    def _project_roi(self):
        warped_in = np.float32([np.array(self._roi_points)])
        project_out = np.float32([[0, 0], [self._side_roi, 0], [self._side_roi, self._side_roi], [0, self._side_roi]])
        M = cv2.getPerspectiveTransform(warped_in, project_out)
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()
        self._projected = cv2.warpPerspective(local_image, M, (self._side_roi, self._side_roi))
        # ENHANCE! *puts glasses on*
        self._blurred = cv2.GaussianBlur(self._projected, (0,0), 3)
        self._projected = cv2.addWeighted(self._projected, 1.5, self._blurred, -0.5, 0, self._projected)

    def _filter_red(self):
        # Finds red colors in HSV space
        hsv = cv2.cvtColor(self._projected, cv2.COLOR_BGR2HSV)
        # lower_red = np.array([165, 60, 60])
        # upper_red = np.array([180, 255, 255])
        lower_orange = np.array([3, 50, 50]) # TODO: those suck.
        upper_orange = np.array([16, 255, 255])
        self._red = cv2.inRange(hsv, lower_orange, upper_orange)
        cv.ShowImage('Orange', cv.fromarray(self._red))
        # print rospy.Time.now()

    def _process_colors(self, red):
        # look down each column building up from bottom
        self._grid = [[0 for _i in range(self._no_squares)] for _j in range(self._no_squares)]
        # print self._grid
        # print '----------'
        self._image_grid = deepcopy(self._projected)
        # print self._image_grid
        self._pieces_positions = []
        for col in xrange(self._no_squares):
            cur_row = True
            x_offset = self._square_side_roi * col
            # print "x = ", x_offset
            # Look from the bottom up checking if piece is there
            for row in xrange(self._no_squares):
                if cur_row == True: # runs first time
                    y_offset = self._square_side_roi * row
                    # print "y = ", y_offset
                    red_cnt = 0
                    # yellow_cnt = 0
                    # look though each pixel in current grid location
                    if len(red) != self._side_roi:
                        print 'BAILING - IMAGE SIZE IS UNEXPECTED'
                        return

                    for y in xrange(0, self._square_side_roi, 2):
                        for x in xrange(0, self._square_side_roi, 2):
                            if red[y + y_offset, x + x_offset] == 255:
                                red_cnt += 1
                                if red_cnt > self._red_thresh: # Speed tweak 
                                    cv2.putText(self._image_grid,
                                    'o',
                                    (x_offset + 20, y_offset + 40),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                    2,
                                    (0, 0, 255))
                                    self._grid[row][col] = 1
                                    self._pieces_positions.append((row,col)) 
                                    break
                        if red_cnt > self._red_thresh:
                            break # (sigh)
                                # else:
                                #     # print "what"
                                #     cur_row = False
                                #     break

    def _update_image_grid(self):
        for idx in xrange(1, self._no_squares):
            cv2.line(self._image_grid, (self._square_side_roi * idx, 0), (self._square_side_roi * idx, self._side_roi),
                     (0, 255, 0), 1)
            cv2.line(self._image_grid, (0, self._square_side_roi * idx), (self._side_roi, self._square_side_roi * idx),
                     (0, 255, 0), 1)
            cv2.line(self._image_grid, (self._square_side_roi * self._no_squares, 0), (self._square_side_roi * self._no_squares, self._side_roi),
                     (0, 255, 0), 1)
            cv.ShowImage('Board State', cv.fromarray(self._image_grid))

    def _transform_positions(self, positions):
        return [(y, 7 - x) for (x,y) in positions]

    def _pub_state(self):
        state = dict()
        self._pieces_positions = self._transform_positions(self._pieces_positions)
        print rospy.Time.now(), " - ", self._pieces_positions
        state['baxter_count'] = self._pieces_positions
        # state['user_count'] = self._user_cnt
        state['board'] = self._grid
        self._board_state_pub.publish(str(state))

    def _on_roi(self, data):
        if data.polygon.points:
            for idx, point in enumerate(data.polygon.points):
                self._roi_points[3 - idx] = [int(point.x), int(point.y)]

    def _on_camera(self, data):
        try:
            self.cv_image = self._bridge.imgmsg_to_cv(data, "bgr8")
            local_image = np.asarray(self.cv_image)
        except Exception:
            print 'Cannot get image from Baxter'

        self.subLock.acquire(True)
        self._np_image = deepcopy(local_image)
        self.subLock.release()

    def _on_gain_slider(self, pos):
        self._gain_slider = pos
        self._gain_set = True
        self._slider_time = rospy.Time.now()

    def _on_red_slider(self, pos):
        self._red_thresh = pos

    def _on_mouse_click(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            width, height = cv.GetSize(self.cv_image)
            for idx, points in enumerate(self._roi_points):
                if (x <= points[0] + 5 and x >= points[0] - 5
                    and y <= points[1] + 5 and y >= points[1] - 5):
                    self._roi_move = True
                    self._point_selected = idx

        elif event == cv.CV_EVENT_MOUSEMOVE and self._roi_move:
            self._roi_points[self._point_selected] = [x, y]

        elif event == cv.CV_EVENT_LBUTTONUP and self._roi_move:
            self._roi_move = False
