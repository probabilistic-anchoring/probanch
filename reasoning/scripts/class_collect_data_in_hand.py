#!/usr/bin/env python
import os
import sys


import rospy
import rospkg
from pydc import DDC

from anchor_msgs.msg import AnchorArray
from anchor_msgs.msg import LogicAnchor
from anchor_msgs.msg import LogicAnchorArray
from anchor_msgs.msg import PositionAttribute



class AnchorInfo(object):
    def __init__(self, coordinates, is_hand, hidden, observed, in_hand, time):
        self.coordinates = coordinates
        self.is_hand = is_hand
        self.hidden = hidden
        self.observed = observed
        self.in_hand = in_hand
        self.time = time


class CollectDataInHand():

    def __init__(self, model_file, n_samples, run):
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
        anchor_ids = self.ddc.querylist("A_ID", "(current(anchor(A_ID)))")
        anchor_ids = anchor_ids.keys()

        time = self.ddc.querylist("T", "(current(time(T)))")
        particle_time = []
        for t in time.keys():
            particle_time.append(t)

        for a_id in anchor_ids:
            anchor_key = "{}_{}_{}".format(a_id, time, self.run)

            particle_coordinates = self.ddc.querylist("(X,VX,Y,VY,Z,VZ)", "(current(rv('{A_ID}'))~=(X,VX,Y,VY,Z,VZ))".format(A_ID=a_id))
            particle_coordinates = particle_coordinates.keys()

            is_hand = self.ddc.querylist("(B,RN)", "(current(data_is_hand('{A_ID}'))~=(B,RN))".format(A_ID=a_id))
            particle_is_hand = []
            print(len(is_hand))
            for h in is_hand.keys():
                ph = int(h.split(",")[0])
                particle_is_hand.append(ph)

            observed = self.ddc.querylist("(B,RN)", "(current(data_observed('{A_ID}'))~=(B,RN))".format(A_ID=a_id))
            particle_observed = []
            for h in observed.keys():
                ph = int(h.split(",")[0])
                particle_observed.append(ph)


            hidden = self.ddc.querylist("(B,RN)", "(current(data_hidden('{A_ID}'))~=(B,RN))".format(A_ID=a_id))
            # print(hidden)
            particle_hidden = []
            # for h in hidden.keys():
            #     ph = int(h.split(",")[0])
            #     particle_hidden.append(ph)


            in_hand = self.ddc.querylist("(B,A_ID_Hider,RN)", "(current(data_in_hand('{A_ID}'))~=(B,A_ID_Hider,RN))".format(A_ID=a_id))
            # print(in_hand)
            particle_in_hand = []
            # for h in in_hand.keys():
            #     ph = h.split(",")[0].strip("'")
            #     if ph.isdigit():
            #         ph = int(ph)
            #     else:
            #         ph = "a"+ph
            #     particle_in_hand.append(ph)




            self.current[a_id] = AnchorInfo(particle_coordinates, particle_is_hand, particle_hidden, particle_observed, particle_in_hand, particle_time)


    def process_data(self):
        with open("dc_data_run{}.pl".format(self.run), "a+") as dc_data_file:
            with open("pl_data_run{}.pl".format(self.run), "a+") as pl_data_file:
                for a_id in self.current:
                    for i in range(0,len(self.current[a_id].time)):
                        formatted_anchor_id = "a{ID}_{Time}_{Particle}_{Run}".format(ID=a_id,Time=self.current[a_id].time,Particle=i,Run=self.run)

                        dc_data_file.write("anchor_t1({}):=true.".format(formatted_anchor_id))
                        dc_data_file.write("\n")
                        pl_data_file.write("anchor_t1({}).".format(formatted_anchor_id))
                        pl_data_file.write("\n")

                        if self.current[a_id].is_hand[i]:
                            dc_data_file.write("is_hand_t1({}):=true.".format(formatted_anchor_id))
                            dc_data_file.write("\n")
                            pl_data_file.write("is_hand_t1({}).".format(formatted_anchor_id))
                            pl_data_file.write("\n")

                        if self.current[a_id].observed[i]:
                            dc_data_file.write("observed_t1({}):=true.".format(formatted_anchor_id))
                            dc_data_file.write("\n")
                            pl_data_file.write("observed_t1({}).".format(formatted_anchor_id))
                            pl_data_file.write("\n")

                        # if self.current[a_id].hidden[i]:
                        #     dc_data_file.write("is_hidden_t1({}):=true.".format(formatted_anchor_id))
                        #     dc_data_file.write("\n")
                        #     pl_data_file.write("is_hidden_t1({}).".format(formatted_anchor_id))
                        #     pl_data_file.write("\n")

                        # if self.current[a_id].in_hand[i]:
                        #     hider_a_id = self.current[a_id].in_hand[i]
                        #     formatted_anchor_id_hider = "a{ID}_{Time}_{Particle}_{Run}".format(ID=hider_a_id,Time=self.current[a_id].time,Particle=i,Run=self.run)
                        #     dc_data_file.write("in_hand_t1({},{}):=true.".format(formatted_anchor_id, formatted_anchor_id_hider))
                        #     dc_data_file.write("\n")
                        #     pl_data_file.write("in_hand_t1({}).".format(formatted_anchor_id, formatted_anchor_id_hider))
                        #     pl_data_file.write("\n")

                        # dc_data_file.write("rv_t1(a{ID}_{Time}_{Particle}_{Run})~.".format(ID=a_id,Time=self.current[a_id].coordinates,Particle=i,Run=self.run))
                        # dc_data_file.write("\n")

                        if a_id in self.previous:
                            dc_data_file.write("anchor_t0({}):=true.".format(formatted_anchor_id))
                            dc_data_file.write("\n")
                            pl_data_file.write("anchor_t0({}).".format(formatted_anchor_id))
                            pl_data_file.write("\n")


                            if self.previous[a_id].is_hand[i]:
                                dc_data_file.write("is_hand_t0({}):=true.".format(formatted_anchor_id))
                                dc_data_file.write("\n")
                                pl_data_file.write("is_hand_t0({}).".format(formatted_anchor_id))
                                pl_data_file.write("\n")

                            if self.previous[a_id].observed[i]:
                                dc_data_file.write("observed_t0({}):=true.".format(formatted_anchor_id))
                                dc_data_file.write("\n")
                                pl_data_file.write("observed_t0({}).".format(formatted_anchor_id))
                                pl_data_file.write("\n")

                            # if self.previous[a_id].hidden[i]:
                            #     dc_data_file.write("is_hidden_t0({}):=true.".format(formatted_anchor_id))
                            #     dc_data_file.write("\n")
                            #     pl_data_file.write("is_hidden_t0({}).".format(formatted_anchor_id))
                            #     pl_data_file.write("\n")

                            # if self.previous[a_id].in_hand[i]:
                            #     hider_a_id = self.previous[a_id].in_hand[i]
                            #     formatted_anchor_id_hider = "a{ID}_{Time}_{Particle}_{Run}".format(ID=hider_a_id,Time=self.current[a_id].time,Particle=i,Run=self.run)
                            #     dc_data_file.write("in_hand_t0({},{}):=true.".format(formatted_anchor_id, formatted_anchor_id_hider))
                            #     dc_data_file.write("\n")
                            #     pl_data_file.write("in_hand_t0({}).".format(formatted_anchor_id, formatted_anchor_id_hider))
                            #     pl_data_file.write("\n")



                    dc_data_file.write("\n")
                    pl_data_file.write("\n")


                dc_data_file.write("%%%%%%%%%%%%%%%%%%%%%")
                dc_data_file.write("\n")

                pl_data_file.write("%%%%%%%%%%%%%%%%%%%%%")
                pl_data_file.write("\n")


        self.previous = self.current
