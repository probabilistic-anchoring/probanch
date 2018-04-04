#!/usr/bin/env python

import sys

import rospy
import rospkg
import cv2

from cv_bridge import CvBridge, CvBridgeError
import message_filters


import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers






class AlvarTFBroadcaster():
    bridge = CvBridge()
    show_image_bool = False


    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    origin_link = "ar_marker_99"
    kinect2_link = "kinect2_link"

    def __init__(self):
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback=self.broadcast_base_link)
        rospy.Subscriber('kinect2/hd/image_color', Image, callback=self.show_image)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)


    def broadcast_base_link(self, msg):
        t = self.tf_buffer.lookup_transform(
            self.origin_link,
            self.kinect2_link,
            rospy.Time.now(),
            rospy.Duration(1)  # Timeout
            )

        t.header.frame_id = 'base_link'
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = 'kinect2_link'
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish([t])


        # trans = t.transform.translation
        # rot = t.transform.rotation
        # launch_transform = "{x} {y} {z} {rx} {ry} {rz} {w}".format(x=trans.x, y=trans.y, z=trans.z, rx=rot.x, ry=rot.y, rz=rot.z, w=rot.w)
        # print(launch_transform)

        return

    def show_image(self, msg):
        if self.show_image_bool:
            try:
              cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
              print(e)
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

def main(args):
    try:
        rospy.init_node('transform_node', anonymous=True)
        AlvarTFBroadcaster()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("AR image transformer node terminated.")

if __name__ == '__main__':
    main(sys.argv)
