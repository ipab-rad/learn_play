import threading
from copy import deepcopy
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


class LPVision(object):
    def __init__(self, limb):
        self._side = limb
        self._camera_name = self._side + "_hand_camera"
        print ("Opening " + self._camera_name + "...")
        self._camera = baxter_interface.CameraController(self._camera_name)
        self._camera.open()
        self._camera.resolution = [640, 400]
        self._camera.gain = 20
        self._side_roi = 400
        self._side_other_roi = 50
        self._no_squares = 8
        # 400 / 8 = 50
        self._square_side_roi = self._side_roi / self._no_squares
        self.grid = [[0 for _i in range(self._no_squares)]
                     for _j in range(self._no_squares)]
        print self.grid
        self.cv_image = None
        self._bridge = CvBridge()
        self.red_sample = ()
        self.blue_sample = ()
        self.reds = None
        self.blues = None
        self._roi_points = [[100, 200],
                            [200, 200],
                            [200, 100],
                            [100, 100]]  # magic (clockwise)
        self._other_roi_points = [[375, 400],
                                  [425, 400],
                                  [425, 350],
                                  [375, 350]]  # magic (clockwise)
        self._is_pickable = False
        self._roi_move = False
        self._point_selected = -1
        self._gain_slider = 30
        self._high_colour_slider = 22
        self._low_colour_slider = 2
        self._low_colour = np.array([self._low_colour_slider, 50, 50])
        self._high_colour = np.array([self._high_colour_slider, 255, 255])
        self._inrange_colour_thresh = 30
        # self._yellow_thresh = 100
        self._slider_time = rospy.Time.now()
        self._gain_set = False
        self._text = ['X', 'Y', 'R', 'G', 'B']
        self._pixel = {}
        for label in self._text:
            self._pixel[label] = 0.0
        self._vector = {}
        self._grid = [[0 for _i in range(self._no_squares)]
                      for _j in range(self._no_squares)]
        self._pnts = [[0 for i in range(self._square_side_roi + 1)]
                      for j in range(self._square_side_roi + 1)]
        self._pieces_positions = []
        self._np_image = np.zeros((self._side_roi, self._side_roi, 3),
                                  np.uint8)
        self._image_grid = np.zeros((self._side_roi, self._side_roi, 3),
                                    np.uint8)
        self._inrange_colour = np.zeros((self._side_roi, self._side_roi),
                             np.uint8)
        self._projected = np.zeros((self._side_roi, self._side_roi, 3),
                                   np.uint8)
        self._other_projected = np.zeros((self._side_other_roi,
                                          self._side_other_roi,
                                          3),
                                         np.uint8)
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
        rest = {'right_e0': 1.3042671634094238,
                'right_e1': 0.9602719721191407,
                'right_s0': -0.5361262847534181,
                'right_s1': -0.3535825712036133,
                'right_w0': -1.1048496612121583,
                'right_w1': 1.54203418526001,
                'right_w2': -0.5004612314758301}
        self._other_arm = baxter_interface.Limb("right")
        self._other_arm.move_to_joint_positions(rest)
        print ' - All set - '
        print " - - - - - - - "
        self._process_images()

    def _process_images(self):
        print " - Starting to process images! - "
        while not rospy.is_shutdown():
            t = rospy.Time.now()
            delta = rospy.Duration(2.0)
            # gain changed from slider settled
            if (t - self._slider_time > delta and self._gain_set):
                self._gain_set = False
                print 'Setting GAIN!'
                self._camera.gain = self._gain_slider
            # process red/yellow image
            self._show_image()
            self._project_roi()
            # self._filter_yellow()
            self._filter_red()
            self._process_colors(deepcopy(self._inrange_colour))
            self._update_image_grid()
            self._is_pickable = self._project_other_roi()
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
        cv2.polylines(local_image, np.int32([np.array(
            self._other_roi_points)]),
            1, (0, 255, 0), 2)
        cv.ShowImage("Learn Play game RGB", cv.fromarray(local_image))
        cv.SetMouseCallback("Learn Play game RGB", self._on_mouse_click, 0)
        cv.CreateTrackbar("Gain", "Learn Play game RGB", self._gain_slider,
                          100, self._on_gain_slider)
        cv.CreateTrackbar("Red Threshold", "Learn Play game RGB",
                          self._inrange_colour_thresh, 500,
                          self._on_red_slider)
        cv.CreateTrackbar("High red", "Learn Play game RGB",
                          self._high_colour_slider,
                          40, self._on_high_colour_slider)
        cv.CreateTrackbar("Low red", "Learn Play game RGB",
                          self._low_colour_slider,
                          40, self._on_low_colour_slider)
        cv.WaitKey(3)

    def _project_roi(self):
        warped_in = np.float32([np.array(self._roi_points)])
        project_out = np.float32([[0, 0],
                                  [self._side_roi, 0],
                                  [self._side_roi, self._side_roi],
                                  [0, self._side_roi]])
        M = cv2.getPerspectiveTransform(warped_in, project_out)
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()
        self._projected = cv2.warpPerspective(local_image,
                                              M,
                                              (self._side_roi,
                                               self._side_roi))
        self._blurred = cv2.GaussianBlur(self._projected, (0, 0), 3)
        self._projected = cv2.addWeighted(self._projected,
                                          1.5,
                                          self._blurred,
                                          -0.5,
                                          0,
                                          self._projected)

    def _filter_red(self):
        """
        Finds red colors in HSV space
        """
        hsv = cv2.cvtColor(self._projected, cv2.COLOR_BGR2HSV)
        self._inrange_colour = cv2.inRange(hsv, self._low_colour,
                                           self._high_colour)
        cv.ShowImage('Orange', cv.fromarray(self._inrange_colour))

    def _process_colors(self, red):
        # look down each column building up from bottom
        self._grid = [[0 for _i in range(self._no_squares)]
                      for _j in range(self._no_squares)]
        self._image_grid = deepcopy(self._projected)
        self._pieces_positions = []
        for col in xrange(self._no_squares):
            cur_row = True
            x_offset = self._square_side_roi * col
            # Look from the bottom up checking if piece is there
            for row in xrange(self._no_squares):
                if cur_row:  # runs first time
                    y_offset = self._square_side_roi * row
                    # print "y = ", y_offset
                    red_cnt = 0
                    # look though each pixel in current grid location
                    if len(red) != self._side_roi:
                        print 'BAILING - IMAGE SIZE IS UNEXPECTED'
                        return
                    for y in xrange(0, self._square_side_roi, 2):
                        for x in xrange(0, self._square_side_roi, 2):
                            if red[y + y_offset, x + x_offset] == 255:
                                red_cnt += 1
                                if red_cnt > self._inrange_colour_thresh:  # Speed tweak
                                    cv2.putText(self._image_grid,
                                                'o',
                                                (x_offset + 20, y_offset + 40),
                                                cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                                2,
                                                (0, 0, 255))
                                    self._grid[row][col] = 1
                                    self._pieces_positions.append((row, col))
                                    break
                        if red_cnt > self._inrange_colour_thresh:
                            break  # (sigh)

    def _update_image_grid(self):
        for idx in xrange(1, self._no_squares):
            cv2.line(self._image_grid,
                     (self._square_side_roi * idx, 0),
                     (self._square_side_roi * idx, self._side_roi),
                     (0, 255, 0),
                     1)
            cv2.line(self._image_grid,
                     (0, self._square_side_roi * idx),
                     (self._side_roi, self._square_side_roi * idx),
                     (0, 255, 0),
                     1)
            cv2.line(self._image_grid,
                     (self._square_side_roi * self._no_squares, 0),
                     (self._square_side_roi * self._no_squares,
                      self._side_roi),
                     (0, 255, 0),
                     1)
            cv.ShowImage('Board State', cv.fromarray(self._image_grid))

    def _transform_positions(self, positions):
        return [(y, 7 - x) for (x, y) in positions]

    def _pub_state(self):
        state = dict()
        self._pieces_positions = self._transform_positions(
            self._pieces_positions)
        print rospy.Time.now(), " - ", self._pieces_positions
        state['baxter_count'] = self._pieces_positions
        # state['user_count'] = self._user_cnt
        state['board'] = self._grid
        state['picking_state'] = self._is_pickable
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
        self._inrange_colour_thresh = pos

    def _on_high_colour_slider(self, pos):
        self._high_colour_slider = pos
        self._high_colour = np.array([self._high_colour_slider, 255, 255])

    def _on_low_colour_slider(self, pos):
        self._low_colour_slider = pos
        self._low_colour = np.array([self._low_colour_slider, 50, 50])

    def _on_mouse_click(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            width, height = cv.GetSize(self.cv_image)
            for idx, points in enumerate(self._roi_points):
                if (x <= points[0] + 5 and
                        x >= points[0] - 5 and
                        y <= points[1] + 5 and
                        y >= points[1] - 5):
                    self._roi_move = True
                    self._point_selected = idx

        elif event == cv.CV_EVENT_MOUSEMOVE and self._roi_move:
            self._roi_points[self._point_selected] = [x, y]

        elif event == cv.CV_EVENT_LBUTTONUP and self._roi_move:
            self._roi_move = False

    def check_for_pick(self):
        """
        Returns Bool
        """
        local_image = deepcopy(self._np_image)

    def _project_other_roi(self):
        warped_in = np.float32([np.array(self._other_roi_points)])
        project_out = np.float32([[0, 0],
                                  [self._side_other_roi, 0],
                                  [self._side_other_roi, self._side_other_roi],
                                  [0, self._side_other_roi]])
        M = cv2.getPerspectiveTransform(warped_in, project_out)
        self.subLock.acquire(True)
        local_image = deepcopy(self._np_image)
        self.subLock.release()
        self._other_projected = cv2.warpPerspective(local_image,
                                                    M,
                                                    (self._side_other_roi,
                                                     self._side_other_roi))
        # Finds red colors in HSV space
        hsv = cv2.cvtColor(self._other_projected, cv2.COLOR_BGR2HSV)

        self._inrange_colour = cv2.inRange(hsv, self._low_colour,
                                           self._high_colour)

        cv.ShowImage('Colour', cv.fromarray(self._inrange_colour))
        # the following can probably be optimized
        red_cnt = 0
        for x in range(self._side_other_roi):
            for y in range(self._side_other_roi):
                if red_cnt > self._inrange_colour_thresh:  # Speed tweak
                    return True
                else:
                    if self._inrange_colour[x, y] == 255:
                        red_cnt += 1
        return False
