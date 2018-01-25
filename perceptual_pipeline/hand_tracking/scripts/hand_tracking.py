#!/usr/bin/env python
# license removed for brevity

import sys

import rospy
import rospkg

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters







def handleTrackbarChanges(obj):
    params = obj.getTrackbarParams()

def trackbar_callback(x):
    pass

def createHSVTrackbars():
    cv2.namedWindow('test')

    cv2.createTrackbar('h_min', 'test', 0, 179, trackbar_callback)
    cv2.createTrackbar('h_max', 'test', 0, 179, trackbar_callback)
    cv2.createTrackbar('s_min', 'test', 0, 255, trackbar_callback)
    cv2.createTrackbar('s_max', 'test', 0, 255, trackbar_callback)
    cv2.createTrackbar('v_min', 'test', 0, 255, trackbar_callback)
    cv2.createTrackbar('v_max', 'test', 0, 255, trackbar_callback)

    cv2.setTrackbarPos('h_min', 'test', 32)
    cv2.setTrackbarPos('h_max', 'test', 46)
    cv2.setTrackbarPos('s_min', 'test', 55)
    cv2.setTrackbarPos('s_max', 'test', 175)
    cv2.setTrackbarPos('v_min', 'test', 180)
    cv2.setTrackbarPos('v_max', 'test', 255)


    # some more stuff





class HandTracking:
    kernel = np.ones([7,7])

    color_img = None
    depth_img = None
    show_img = None

    def __init__(self):
        # Image bridge, RGB-D subscriber and sync policy
        self.bridge = CvBridge()
        self.rgb_sub= message_filters.Subscriber('/kinect2/qhd/image_color_rect', Image)
        self.depth_sub= message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 100)
        self.ts.registerCallback(self.image_cb)



    def get_min_track_bar_values(self):
        h = cv2.getTrackbarPos('h_min', 'test')
        s = cv2.getTrackbarPos('s_min', 'test')
        v = cv2.getTrackbarPos('v_min', 'test')
        return np.array([h,s,v], dtype='uint8')

    def get_max_track_bar_values(self):
        h = cv2.getTrackbarPos('h_max', 'test')
        s = cv2.getTrackbarPos('s_max', 'test')
        v = cv2.getTrackbarPos('v_max', 'test')
        return np.array([h,s,v], dtype='uint8')

    # Image callback function (main loop)
    def image_cb(self, rgb_msg, depth_msg):

        try:
            color_img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            depth_img = np.array(depth_img, dtype=np.float32)
        except CvBridgeError as e:
            print(e)
            return
        else:
            self.color_img_rgb = color_img

            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
            depth_img = cv2.blur(depth_img,(7,7))

            # depth_img = cv2.normalize(depth_img, depth_img, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32FC1)

            color_img = cv2.blur(color_img,(5,5))
            mask = np.zeros( (color_img.shape[0],color_img.shape[1],1), np.uint8)
            low = self.get_min_track_bar_values()
            high = self.get_max_track_bar_values()

            mask = cv2.bitwise_or(mask,cv2.inRange(color_img, low, high))
            mask = cv2.dilate(mask, self.kernel, iterations=1)
            mask = cv2.erode(mask, self.kernel, iterations=1)
            _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            cut_contours = []
            for c in contours:
                c_moments = cv2.moments(c)
                if c_moments['m00'] and c_moments['m00']>500:# and c_moments['m00']>:
                    cut_contours.append(c)

            if cut_contours:
                hull = np.concatenate(cut_contours)
                hull = cv2.convexHull(hull)

                hull_moments = cv2.moments(hull)

                hull_x = int(hull_moments['m10']/hull_moments['m00'])
                hull_y = int(hull_moments['m01']/hull_moments['m00'])
                # print(hull_x)
                hull_center_depth = depth_img[hull_y][hull_x]


                mask = np.zeros( (depth_img.shape[0],depth_img.shape[1],1), np.uint8)

                cv2.drawContours(mask, [hull], -1, 255, -1)

                print(hull_center_depth)
                depth_hull = depth_img
                if hull_center_depth:
                    depth_hull = np.where(  (depth_hull>=hull_center_depth-25) &   (depth_hull<=hull_center_depth+25), 255, 0)

                    depth_hull= depth_hull.astype(np.uint8)
                    # print(type(mask))
                    # print(type(depth_hull))

                    depth_hull = np.expand_dims(depth_hull, axis=-1)

                    mask = cv2.bitwise_and(mask, depth_hull)










            self.show_img = self.color_img_rgb
            #self.show_img = mask
            # if cut_contours:
            #     cv2.drawContours(self.show_img, [hull], -1, (0,255,0), 1)
            if contours:
                cv2.drawContours(self.show_img, cut_contours, -1, (0,255,0), 1)

    def showImages(self):
        if self.show_img is not None:
            cv2.imshow('test', self.show_img)


# Main fn
def main(args):
    createHSVTrackbars()
    ht = HandTracking()

    rospy.init_node('hand_tracking_node', anonymous=True)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ht.showImages()
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
