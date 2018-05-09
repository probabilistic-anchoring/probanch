#!/usr/bin/env python
import os

import rospy
import rospkg
from dc_pybridge import DCUtil

from anchor_msgs.msg import AnchorArray
from anchor_msgs.msg import LogicAnchor
from anchor_msgs.msg import LogicAnchorArray

from geometry_msgs.msg import Point






class RelTrack():

    def __init__(self, model_file, n_samples):
        self.util = DCUtil(model_file, n_samples)
        self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, self.process_anchors)
        self.pub = rospy.Publisher('logic_anchors', LogicAnchorArray, queue_size=10)



    def process_anchors(self, msg):
        observations = self.make_observations(msg.anchors)

        self.util.step(observations);
        anchors = self.util.querylist("A_ID", "current(rv(A_ID))~=_")
        anchors = anchors.args_ground


        la_array = self.make_LogicAnchorArray(anchors)
        # print(la_array)
        # self.pub


    def make_observations(self, anchors):
        observations = []
        print("\n")
        for a in anchors:
            obs = []

            if not "glasses" in a.caffe.symbols[0:4]:
                print("anchor IDs: {}".format(a.id))
                position = a.position.data.pose.position
                bbox = a.shape.data
                color = a.color.symbols[0]
                obs.append("observation(anchor_r('{}'))~=({},{},{})".format(a.id, position.x, position.y, position.z))
                obs.append("observation(anchor_bb('{}'))~=({},{},{})".format(a.id, bbox.x, bbox.y, bbox.z))
                obs.append("observation(anchor_c('{}'))~={}".format(a.id, color))
                obs = ','.join(obs)
                observations.append(obs)
        observations = ','.join(observations)

        return observations


    def make_LogicAnchorArray(self, anchors):
        la_array =LogicAnchorArray()

        point = Point()
        for a in anchors:
            la = LogicAnchor()
            la.id = a

            particle_positions = self.util.querylist("(X,Y,Z)", "current(rv(A_ID))~=(X,_,Y,_,Z,_)")
            particle_positions = particle_positions.args_ground
            for p in particle_positions:
                point = Point()
                x,y,z = p.split(",")
                point.x = float(x)
                point.y = float(y)
                point.z = float(z)

                la.particle_positions.append(point)
            la_array.anchors.append(la)

        return la_array

if __name__ == "__main__":
    rospy.init_node("rel_track_node")
    path = rospkg.RosPack().get_path('reasoning')
    model_file = os.path.join(path, 'models/dc_model.pl')
    N_SAMPLES = 10

    rel_track = RelTrack(model_file, N_SAMPLES)

    rospy.spin()
