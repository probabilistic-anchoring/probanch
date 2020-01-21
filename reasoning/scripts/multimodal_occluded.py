#!/usr/bin/env python
import os
import sys
import numpy as np
import pickle

import rospy
import rospkg
from pydc import DDC

from anchor_msgs.msg import AnchorArray
from anchor_msgs.msg import LogicAnchor
from anchor_msgs.msg import LogicAnchorArray
from anchor_msgs.msg import PositionAttribute
from anchor_msgs.msg import RemovedAnchorArray

class State(object):
    def __init__(self, time, run):
        self.time = time
        self.run = run
        self.anchors = {}
        self.flag_to_occluded = False

    def __repr__(self):
        return "State(run: {}, time: {}, anchors: {})".format(self.run, self.time, self.anchors.keys())

class AnchorInfo(object):
    def __init__(self, mean_std, observed, occluded):
        self.mean_std = mean_std
        self.observed = observed
        self.occluded = occluded


class MultimodalOccluded():

    def __init__(self, model_file, n_samples):
        self.previous = {}
        self.current = {}
        self.ddc = DDC(model_file, n_samples)
        self.anchors_to_remove = []
        self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, callback=self.process_anchors)
        self.removed_anchors_sub = rospy.Subscriber('anchors_removed', RemovedAnchorArray, callback=self.remove_anchors)
        self.logic_anchors_publisher = rospy.Publisher('logic_anchors', LogicAnchorArray, queue_size=10)


    # New callback function for removing anchors (from DDC database)
    def remove_anchors(self, msg):
        self.anchors_to_remove += msg.ids

    def process_anchors(self, msg):
        observations = self.make_observations(msg.anchors)
        self.ddc.step(observations);

        la_array = self.make_LogicAnchorArray(msg.anchors)
        self.logic_anchors_publisher.publish(la_array)


    def make_observations(self, anchors):
        observations = []
        for a_id in self.anchors_to_remove:
            observations.append("observation(remove_anchor('{A_ID}'))~=true".format(A_ID=a_id))

        aids= [a.id for a in anchors]

        for a in anchors:
            if not a.id in self.anchors_to_remove:
                obs = []
                if self.filter(a):
                    position = a.position.data.pose.position
                    bbox = a.size.data
                    color = a.color.symbols[0]
                    #TODO make probabilistic with prediciont socres
                    category = a.category.symbols[0]

                    obs.append("observation(anchor_r('{A_ID}'))~=({X},{Y},{Z})".format(A_ID=a.id, X=position.x, Y=position.y, Z=position.z))
                    obs.append("observation(anchor_bb('{A_ID}'))~=({BBX},{BBY},{BBZ})".format(A_ID=a.id, BBX=bbox.x, BBY=bbox.y, BBZ=bbox.z))
                    obs.append("observation(anchor_c('{A_ID}'))~={C}".format(A_ID=a.id, C=color))
                    obs.append("observation(anchor_category('{A_ID}'))~={Category}".format(A_ID=a.id, Category=category))


                    obs = ','.join(obs)
                    observations.append(obs)
        observations = ','.join(observations)

        self.anchors_to_remove = []
        return observations


    def make_LogicAnchorArray(self, anchors_observed):
        anchor_ids_observed = [a.id for a in anchors_observed]

        la_array = LogicAnchorArray()
        anchor_ids = self.ddc.querylist("A_ID", "(current(anchor(A_ID)), current(occluded_by(A_ID,_)))")

        # for quickly testin (more often then not anchors are not occluded)
        # but do not publish gives fault in the anchor management node?
        # anchor_ids = self.ddc.querylist("A_ID", "(current(anchor(A_ID)), \+current(occluded_by(A_ID,_)))")
        anchor_ids = [a_id.decode("UTF-8") for a_id in anchor_ids.keys()]

        for a_id in anchor_ids:
            # probably do not need this check
            if a_id not in anchor_ids_observed:
                la = LogicAnchor()
                la.id = a_id
                la.observed = False

                color = self.ddc.querylist("Color", "( current(color('{A_ID}'))~=Color)".format(A_ID=la.id))
                color = [c.decode("UTF-8") for c in color.keys()]
                la.color.symbols = color
                particle_positions = self.ddc.querylist("(X,Y,Z)", "(current(rv('{A_ID}'))~=(X,_,Y,_,Z,_))".format(A_ID=la.id))
                for p in particle_positions:
                    p = p.decode("UTF-8")
                    x,y,z = p.split(",")
                    position = PositionAttribute()
                    position.data.pose.position.x = float(x)
                    position.data.pose.position.y = float(y)
                    position.data.pose.position.z = float(z)
                    la.particle_positions.append(position)

                la_array.anchors.append(la)
        return la_array


    def filter(self, anchor):
        # print(anchor.category.symbols[0])
        # print(anchor.id)
        if "glasses" in anchor.category.symbols[0:4]:
            return False
        elif "glass" in anchor.category.symbols[0:4]:
            return False
        elif "glove" in anchor.category.symbols[0:2]:
            if "black" in anchor.color.symbols[0:1]:
                return False
            else:
                return False
        elif "keyboard" in anchor.category.symbols[0:1]:
            return False
        # elif "skin" in anchor.category.symbols[0:1]:
        #     return False
        # elif "skin" in anchor.category.symbols[0:4]:
        #     return False
        elif "candle" in anchor.category.symbols[0:2]:
            return False
        elif "beaker" in anchor.category.symbols[0:2]:
            return False
        elif "case" in anchor.category.symbols[0:2]:
            return False
        elif "spatual" in anchor.category.symbols[0:2]:
            return False
        elif "tape_measure" in anchor.category.symbols[0:2]:
            return False
        elif "flashlight" in anchor.category.symbols[0:1]:
            return False
        # elif "melon" in anchor.category.symbols[0:1]:
        #     return False
        # elif "mango" in anchor.category.symbols[0:1]:
        #     return False
        # elif "squash" in anchor.category.symbols[0:2]:
        #     return False
        # elif not anchor.category.symbols:
        #     return False

        else:
            # print(anchor.category.symbols[0], anchor.color.symbols[0])
            return True
