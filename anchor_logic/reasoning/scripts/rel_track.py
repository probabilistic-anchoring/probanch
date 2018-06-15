#!/usr/bin/env python
import os

import rospy
import rospkg
from dc_pybridge import DCUtil

from anchor_msgs.msg import AnchorArray
from anchor_msgs.msg import LogicAnchor
from anchor_msgs.msg import LogicAnchorArray
from anchor_msgs.msg import PositionAttribute

'''
# aspo: I think that this messge should be used instead (?):
from anchor_msgs.msg import PositionAttribute

# ...also, se lines 86-90 below.
'''





class RelTrack():

    def __init__(self, model_file, n_samples):
        self.util = DCUtil(model_file, n_samples)
        self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, callback = self.process_anchors)
        self.logic_anchors_publisher = rospy.Publisher('logic_anchors', LogicAnchorArray, queue_size=10)



    def process_anchors(self, msg):
        observations = self.make_observations(msg.anchors)

        self.util.step(observations);
        anchors = self.util.querylist("A_ID", "current(rv(A_ID))~=_")
        anchors = anchors.args_ground

        la_array = self.make_LogicAnchorArray(msg.anchors)
        self.logic_anchors_publisher.publish(la_array)


    def make_observations(self, anchors):
        observations = []
        for a in anchors:
            obs = []
            # print(a.caffe.symbols)
            if self.filter(a):
                # print("anchor IDs: {}".format(a.id))
                position = a.position.data.pose.position
                bbox = a.shape.data

                color = a.color.symbols[0]

                obs.append("observation(anchor_r('{A_ID}'))~=({X},{Y},{Z})".format(A_ID=a.id, X=position.x, Y=position.y, Z=position.z))
                obs.append("observation(anchor_bb('{A_ID}'))~=({BBX},{BBY},{BBZ})".format(A_ID=a.id, BBX=bbox.x, BBY=bbox.y, BBZ=bbox.z))
                obs.append("observation(anchor_c('{A_ID}'))~={C}".format(A_ID=a.id, C=color))
                if self.is_hand(a):
                    print(a.id,33333333)
                    obs.append("observation(anchor_hand('{A_ID}'))~=true".format(A_ID=a.id))

                obs = ','.join(obs)
                observations.append(obs)
        observations = ','.join(observations)

        return observations


    def make_LogicAnchorArray(self, anchors):
        la_array = LogicAnchorArray()
        for a in anchors:
            if self.filter(a):

                la = LogicAnchor()

                la.id = a.id

                particle_positions = self.util.querylist("(X,Y,Z)", "current(rv('{A_ID}'))~=(X,_,Y,_,Z,_)".format(A_ID=la.id))
                particle_positions = particle_positions.args_ground
                for p in particle_positions:
                    x,y,z = p.split(",")
                    position = PositionAttribute()

                    position.data.pose.position.x = float(x)
                    position.data.pose.position.y = float(y)
                    position.data.pose.position.z = float(z)


                    la.particle_positions.append(position)

                observed = self.util.query("current(observed('{A_ID}'))".format(A_ID=la.id))

                la.observed = bool(observed.probability)
                la.color.symbols = a.color.symbols
                la.color.predictions = a.color.predictions

                la_array.anchors.append(la)
        is_hand = self.util.querylist("A_ID","current(is_hand(A_ID))")
        print(is_hand)

        return la_array


    def filter(self, anchor):
        if "glasses" in anchor.caffe.symbols[0:4]:
            return False
        elif not anchor.caffe.symbols:
            return False
        else:
            return True


    def is_hand(self, anchor):
        if "glove" in anchor.caffe.symbols[0:4]:
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node("rel_track_node")
    path = rospkg.RosPack().get_path('reasoning')
    model_file = os.path.join(path, 'models/dc_model.pl')
    N_SAMPLES = 100

    rel_track = RelTrack(model_file, N_SAMPLES)

    rospy.spin()
