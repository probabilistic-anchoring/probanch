#!/usr/bin/env python
import os

import rospy
import rospkg
from dc_pybridge import DCUtil

from anchor_msgs.msg import AnchorArray





class RelTrack():

    def __init__(self,  path, model_file, n_samples):
        model_file = os.path.join(path, model_file)
        with open(model_file, 'r') as dc_file:
            model = dc_file.read()

        self.util = DCUtil()
        self.util.consult(model, n_samples)
        # self.anchors_sub = rospy.Subscriber('anchors', AnchorArray, self.process_anchors)
        # self.pub = rospy.Publisher('chatter', String, queue_size=10)



    # def process_anchors(self, msg):
    #     observations = self.make_observations(msg.anchors)
    #     # print(observations)
    #     self.util.step(observations);
    #     probabilities = self.util.querylist("New","current(asso(New))~=_");
    #     print(probabilities)
    #
    #
    # def make_observations(self, anchors):
    #     observations = []
    #     for a in anchors:
    #         obs = []
    #         position = a.position.data.pose.position
    #         bbox = a.shape.data
    #         color = a.color.symbols[0]
    #         obs.append("observation(anchor_r('{}'))~=({},{},{})".format(a.id, position.x, position.y, position.z))
    #         obs.append("observation(anchor_bb('{}'))~=({},{},{})".format(a.id, bbox.x, bbox.y, bbox.z))
    #         obs.append("observation(anchor_c('{}'))~={}".format(a.id, color))
    #         obs = ','.join(obs)
    #         observations.append(obs)
    #     observations = ','.join(observations)
    #     return observations





if __name__ == "__main__":
    rospy.init_node("rel_track_node")
    path = rospkg.RosPack().get_path('inference')
    rel_track = RelTrack(path, 'models/dc_model.pl', 200)
    # q1=rel_track.util.query("(current(p(o(1)))~=1)")
    # print(q1)
    # print(q1)
    # print(q1)
    # rel_track.util.step("")
    # q2=rel_track.util.query("(current(t(2))~=2)")
    # print(q2)
    # rel_track.util.step("")
    # # rel_track.query("(current(t(2))~=3)")
    # # ql = rel_track.util.querylist("ID","current(p(ID))~=1")
    # # q = rel_track.util.query("(current(p(o(1)))~=1)")
    # # print(ql)
    # # print(q)

    # rospy.spin()