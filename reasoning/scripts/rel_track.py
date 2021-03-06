#!/usr/bin/env python3
import os
import sys


import rospy
import rospkg



def argparser():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", type=str,
                        help="name+path of this file")

    parser.add_argument("-mode", type=str, help="which scenario to solve")
    parser.add_argument("-run", type=int, help="which run")



    return parser

if __name__ == "__main__":
    rospy.init_node("rel_track_node")
    path = rospkg.RosPack().get_path('reasoning')
    N_SAMPLES = 100

    python_argv = rospy.myargv(argv=sys.argv)
    parser = argparser()
    args = vars(parser.parse_args(python_argv))

    if args['mode']=="collect_data_occluded":
        RUN = args['run']
        from class_collect_data_occluded import CollectDataOccluded
        model_file = os.path.join(path, 'models/collect_data_occluded.pl')
        rel_track = CollectDataOccluded(model_file, N_SAMPLES, RUN)
    elif args['mode']=="occluded_learned":
        from occluded_learned import OccludedLearned
        model_file = os.path.join(path, 'models/occluded_learned.pl')
        rel_track = OccludedLearned(model_file, N_SAMPLES)
    elif args["mode"]=="multimodal_occluded":
        from multimodal_occluded import MultimodalOccluded
        model_file = os.path.join(path, 'models/multimodal_occluded.pl')
        rel_track = MultimodalOccluded(model_file, N_SAMPLES)
    elif args["mode"]=="occluded":
        from occluded import Occluded
        model_file = os.path.join(path, 'models/occluded.pl')
        rel_track = Occluded(model_file, N_SAMPLES)
    # if args['mode']=="shellgame":
    #     from class_shellgame import Shellgame
    #     model_file = os.path.join(path, 'models/shellgame.pl')
    #     rel_track = Shellgame(model_file, N_SAMPLES)
    # if args['mode']=="shellgame_multimodal":
    #     from class_shellgame_multimodal import ShellgameMultimodal
    #     model_file = os.path.join(path, 'models/shellgame_multimodal.pl')
    #     rel_track = ShellgameMultimodal(model_file, N_SAMPLES)
    # elif args['mode']=="behind_of":
    #     from class_behind_of import BehindOf
    #     model_file = os.path.join(path, 'models/behind_of.pl')
    #     rel_track = BehindOf(model_file, N_SAMPLES)
    # elif args['mode']=="in_hand":
    #     from class_in_hand import InHand
    #     model_file = os.path.join(path, 'models/in_hand.pl')
    #     rel_track = InHand(model_file, N_SAMPLES)


    rospy.spin()
