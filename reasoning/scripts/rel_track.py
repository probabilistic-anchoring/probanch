#!/usr/bin/env python
import os

import rospy
import rospkg
from pydc import DDC

from anchor_msgs.msg import AnchorArray
from anchor_msgs.msg import LogicAnchor
from anchor_msgs.msg import LogicAnchorArray
from anchor_msgs.msg import PositionAttribute




class RelTrack():

    def __init__(self, model_file, n_samples):
        self.ddc = DDC(model_file, n_samples)
        self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, callback=self.process_anchors)
        self.logic_anchors_publisher = rospy.Publisher('logic_anchors', LogicAnchorArray, queue_size=10)



    def process_anchors(self, msg):
        observations = self.make_observations(msg.anchors)
        self.ddc.step(observations);

        la_array = self.make_LogicAnchorArray(msg.anchors)
        self.logic_anchors_publisher.publish(la_array)


    def make_observations(self, anchors):
        observations = []
        for a in anchors:
            obs = []

            if self.filter(a):
                position = a.position.data.pose.position
                bbox = a.shape.data
                color = a.color.symbols[0]
                #TODO make probabilistic with prediciont socres
                caffe = a.caffe.symbols[0]

                obs.append("observation(anchor_r('{A_ID}'))~=({X},{Y},{Z})".format(A_ID=a.id, X=position.x, Y=position.y, Z=position.z))
                obs.append("observation(anchor_bb('{A_ID}'))~=({BBX},{BBY},{BBZ})".format(A_ID=a.id, BBX=bbox.x, BBY=bbox.y, BBZ=bbox.z))
                obs.append("observation(anchor_c('{A_ID}'))~={C}".format(A_ID=a.id, C=color))
                obs.append("observation(anchor_caffe('{A_ID}'))~={Caffe}".format(A_ID=a.id, Caffe=caffe))
                if self.is_hand(a):
                    obs.append("observation(anchor_hand('{A_ID}'))~=true".format(A_ID=a.id))

                obs = ','.join(obs)
                observations.append(obs)
        observations = ','.join(observations)

        return observations


    def make_LogicAnchorArray(self, anchors_observed):
        anchor_ids_observed = [a.id for a in anchors_observed]

        la_array = LogicAnchorArray()
        
        # for a in anchors_observed:
        #     la = LogicAnchor()
        #
        #     la.id = a.id
        #
        #     la.observed = True
        #
        #     la.color.symbols = a.color.symbols
        #     la.color.predictions = a.color.predictions
        #
        #     particle_positions = self.ddc.querylist("(X,Y,Z)", "current(rv('{A_ID}'))~=(X,_,Y,_,Z,_)".format(A_ID=la.id))
        #     for p in particle_positions:
        #         x,y,z = p.split(",")
        #         position = PositionAttribute()
        #
        #         position.data.pose.position.x = float(x)
        #         position.data.pose.position.y = float(y)
        #         position.data.pose.position.z = float(z)
        #
        #         la.particle_positions.append(position)
        #
        #
        #     la_array.anchors.append(la)



        anchor_ids = self.ddc.querylist("A_ID", "current(anchor(A_ID))")
        anchor_ids = anchor_ids.keys()


        for a_id in anchor_ids:
            if a_id not in anchor_ids_observed:
                la.id = a_id

                la.observed = False

                color = self.ddc.querylist("Color", "current(color('{A_ID}'))~=Color".format(A_ID=la.id))
                color = color.keys()
                la.color.symbols = color
                particle_positions = self.ddc.querylist("(X,Y,Z)", "current(rv('{A_ID}'))~=(X,_,Y,_,Z,_)".format(A_ID=la.id))
                for p in particle_positions:
                    x,y,z = p.split(",")
                    position = PositionAttribute()
                    position.data.pose.position.x = float(x)
                    position.data.pose.position.y = float(y)
                    position.data.pose.position.z = float(z)
                    la.particle_positions.append(position)


                la_array.anchors.append(la)


                # caffe = self.ddc.querylist("Caffe","(current(caffe('{A_ID}'))~=Caffe)".format(A_ID=la.id))
                # caffe = caffe.keys()
                # print(caffe)
                #
                # hidden = self.ddc.query("current(hidden('{A_ID}',_))".format(A_ID=la.id))

        return la_array


    def filter(self, anchor):
        if "glasses" in anchor.caffe.symbols[0:4]:
            return False
        elif "banana" in anchor.caffe.symbols[0:1]:
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
    N_SAMPLES = 200

    rel_track = RelTrack(model_file, N_SAMPLES)

    rospy.spin()
