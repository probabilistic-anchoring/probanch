#!/usr/bin/env python
import os

import rospy
import rospkg
from dc_pybridge import DCUtil

from anchor_msgs.msg import AnchorArray
from anchor_msgs.msg import LogicAnchor
from anchor_msgs.msg import LogicAnchorArray
from anchor_msgs.msg import LogicHiddenAnchorsID

from geometry_msgs.msg import Point






class RelTrack():

    def __init__(self, model_file, n_samples):
        self.util = DCUtil(model_file, n_samples)
        self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, self.process_anchors)
        self.logic_anchors_publisher = rospy.Publisher('logic_anchors', LogicAnchorArray, queue_size=10)
        self.logic_hidden_anchorsID_publisher = rospy.Publisher('logic_anchors', LogicHiddenAnchorsID, queue_size=10)



    def process_anchors(self, msg):
        observations = self.make_observations(msg.anchors)

        self.util.step(observations);
        anchors = self.util.querylist("A_ID", "current(rv(A_ID))~=_")
        anchors = anchors.args_ground


        la_array = self.make_LogicAnchorArray(msg.anchors)
        hidden_anchors = self.make_HiddenAnchorsID()

        self.logic_anchors_publisher.publish(la_array)
        self.logic_hidden_anchorsID_publisher.publish(hidden_anchors)


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
                obs.append("observation(anchor_r('{A_ID}'))~=({X},{Y},{Z})".format(A_ID=a.id, X=position.x, Y=position.y, Z=position.z))
                obs.append("observation(anchor_bb('{A_ID}'))~=({BBX},{BBY},{BBZ})".format(A_ID=a.id, BBX=bbox.x, BBY=bbox.y, BBZ=bbox.z))
                obs.append("observation(anchor_c('{A_ID}'))~={C}".format(A_ID=a.id, C=color))
                obs = ','.join(obs)
                observations.append(obs)
        observations = ','.join(observations)

        return observations


    def make_LogicAnchorArray(self, anchors):
        la_array =LogicAnchorArray()
        point = Point()
        for a in anchors:
            la = LogicAnchor()
            la.id = a.id

            particle_positions = self.util.querylist("(X,Y,Z)", "current(rv('{A_ID}'))~=(X,_,Y,_,Z,_)".format(A_ID=la.id))
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

    def make_HiddenAnchorsID(self):
        hidden_anchors = LogicHiddenAnchorsID()
        ha = self.util.querylist("A_ID", "current(hidden(A_ID))")
        hidden_anchors.ids = ha.args_ground
        return hidden_anchors



if __name__ == "__main__":
    rospy.init_node("rel_track_node")
    path = rospkg.RosPack().get_path('reasoning')
    model_file = os.path.join(path, 'models/dc_model.pl')
    N_SAMPLES = 200

    rel_track = RelTrack(model_file, N_SAMPLES)

    rospy.spin()
