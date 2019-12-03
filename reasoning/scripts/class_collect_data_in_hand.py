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



class State(object):
    def __init__(self, time, run):
        self.time = time
        self.run = run
        self.anchors = {}
        self.flag_to_hand = False

    def __repr__(self):
        return "State(run: {}, time: {}, anchors: {})".format(self.run, self.time, self.anchors.keys())

class AnchorInfo(object):
    def __init__(self, mean_std, is_hand, observed, in_hand):
        self.mean_std = mean_std
        self.is_hand = is_hand
        self.observed = observed
        self.in_hand = in_hand


class CollectDataInHand():

    def __init__(self, model_file, n_samples, run):
        self.flag_process_data = True
        self.run = run
        self.previous = {}
        self.current = {}
        self.ddc = DDC(model_file, n_samples)
        self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, callback=self.process_anchors)
        self.logic_anchors_publisher = rospy.Publisher('logic_anchors', LogicAnchorArray, queue_size=10)



    def process_anchors(self, msg):
        observations = self.make_observations(msg.anchors)
        self.ddc.step(observations);

        la_array = self.make_LogicAnchorArray(msg.anchors)
        self.collect_data(msg.anchors)
        if self.flag_process_data:
            self.process_data()
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
        # print(observations)

        return observations


    def make_LogicAnchorArray(self, anchors_observed):
        anchor_ids_observed = [a.id for a in anchors_observed]

        la_array = LogicAnchorArray()

        # for a in anchors_observed:
        #
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



        anchor_ids = self.ddc.querylist("A_ID", "(current(hidden(A_ID,_)), current(anchor(A_ID)))")
        anchor_ids = anchor_ids.keys()


        for a_id in anchor_ids:
            if a_id not in anchor_ids_observed:
                la = LogicAnchor()
                la.id = a_id

                la.observed = False

                color = self.ddc.querylist("Color", "(current(hidden(A_ID,_)), current(color('{A_ID}'))~=Color)".format(A_ID=la.id))
                color = color.keys()
                la.color.symbols = color
                particle_positions = self.ddc.querylist("(X,Y,Z)", "(current(hidden(A_ID,_)),  current(hidden(A_ID,_)), current(rv('{A_ID}'))~=(X,_,Y,_,Z,_))".format(A_ID=la.id))
                for p in particle_positions:
                    x,y,z = p.split(",")
                    position = PositionAttribute()
                    position.data.pose.position.x = float(x)
                    position.data.pose.position.y = float(y)
                    position.data.pose.position.z = float(z)
                    la.particle_positions.append(position)


                la_array.anchors.append(la)

                # print(la_array)
                # caffe = self.ddc.querylist("Caffe","(current(caffe('{A_ID}'))~=Caffe)".format(A_ID=la.id))
                # caffe = caffe.keys()
                # print(caffe)
                #
                # hidden = self.ddc.query("current(hidden('{A_ID}',_))".format(A_ID=la.id))

        # print(len(la_array.anchors))
        return la_array


    def filter(self, anchor):
        if "glasses" in anchor.caffe.symbols[0:4]:
            return False
        elif "banana" in anchor.caffe.symbols[0:1]:
            return False
        elif "skin" in anchor.caffe.symbols[0:2]:
            return False
        elif "squash" in anchor.caffe.symbols[0:2]:
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


    def collect_data(self, anchors_observed):
        time_step = self.ddc.querylist("T", "(current(time(T)))")
        time_step = int(time_step.keys()[0])

        anchor_ids = self.ddc.querylist("A_ID", "(current(anchor(A_ID)))")
        anchor_ids = anchor_ids.keys()



        self.current = State(time_step, self.run)
        flag_to_hand = False
        for a_id in anchor_ids:
            position_mean_std = self.collect_position(a_id)
            is_hand = self.collect_is_hand(a_id)
            observed = self.collect_observed(a_id)
            in_hand = self.collect_in_hand(a_id)
            if in_hand:
                flag_to_hand = True

            # print(observed)
            # print(in_hand)
            self.current.anchors[a_id] = AnchorInfo(position_mean_std, is_hand, observed, in_hand)
        self.current.flag_to_hand = flag_to_hand

    def collect_position(self, a_id):
        particle_coordinates = self.ddc.querylist("(X,VX,Y,VY,Z,VZ)", "(current(rv('{A_ID}'))~=(X,VX,Y,VY,Z,VZ))".format(A_ID=a_id))
        particle_coordinates = particle_coordinates.keys()
        x_pos = []
        y_pos = []
        z_pos = []
        for rv in particle_coordinates:
            rv = rv.strip("'").split(",")
            x_pos.append(float(rv[0]))
            y_pos.append(float(rv[2]))
            z_pos.append(float(rv[4]))
        x_pos = np.array(x_pos)
        y_pos = np.array(y_pos)
        z_pos = np.array(z_pos)

        x_mean = x_pos.mean()
        y_mean = y_pos.mean()
        z_mean = z_pos.mean()

        x_std = x_pos.std()
        y_std = y_pos.std()
        z_std = z_pos.std()
        return ((x_mean, x_std), (y_mean, y_std), (z_mean, z_std))

    def collect_is_hand(self, a_id):
        is_hand = self.ddc.query("(current(is_hand('{A_ID}')))".format(A_ID=a_id))
        return is_hand

    def collect_observed(self, a_id):
        observed = self.ddc.query("(current(observed('{A_ID}')))".format(A_ID=a_id))
        return observed

    def collect_in_hand(self, a_id):
        in_hand = self.ddc.querylist("A_Hand", "(current(in_hand('{A_ID}',A_Hand)))".format(A_ID=a_id))
        if not in_hand.keys():
            return None
        else:
            hand_id = in_hand.keys()[0]
            return hand_id

    def process_data(self):
        if self.previous:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            data_dir = os.path.join(dir_path, "learn_in_hand_data")
            data_file = os.path.join(data_dir, "run{}_time{}.pickle".format(self.current.run, self.current.time))
            data = {"current": self.current, "previous": self.previous}



            if self.current.time>0:
                with open(data_file, 'wb') as handle:
                    pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

            if self.current.flag_to_hand:
                self.flag_process_data = False

        self.previous = self.current
