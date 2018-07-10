#!/usr/bin/env python
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

    return parser

if __name__ == "__main__":
    rospy.init_node("rel_track_node")
    path = rospkg.RosPack().get_path('reasoning')
    N_SAMPLES = 200

    python_argv = rospy.myargv(argv=sys.argv)
    parser = argparser()
    args = vars(parser.parse_args(python_argv))

    if args['mode']=="shellgame":
        from class_shellgame import Shellgame
        model_file = os.path.join(path, 'models/shellgame.pl')
        rel_track = Shellgame(model_file, N_SAMPLES)
    elif args['mode']=="behind_of":
        from class_behind_of import BehindOf
        model_file = os.path.join(path, 'models/behind_of.pl')
        rel_track = BehindOf(model_file, N_SAMPLES)
    elif args['mode']=="in_hand":
        from class_in_hand import InHand
        model_file = os.path.join(path, 'models/in_hand.pl')
        rel_track = InHand(model_file, N_SAMPLES)

    rospy.spin()
