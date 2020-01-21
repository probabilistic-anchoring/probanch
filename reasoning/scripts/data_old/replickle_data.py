import sys
sys.path.append("..")
import class_collect_data_occluded
import pickle
import os


def main():
    pass
    dir_path = os.path.dirname(os.path.realpath(__file__))
    data_dir = os.path.join(dir_path, "learn_occluded_by_data_frontiers_extra")
    data_repickled_dir = os.path.join(dir_path, "learn_occluded_by_data_frontiers_extra_repickled")


    pickle_files = [os.path.join(data_dir, f) for f in os.listdir(data_dir) ]


    for p_file_name in pickle_files:
        pickle_file = open(p_file_name,"rb")
        data = pickle.load(pickle_file)

        data_refactored = {}

        current_state = {}
        current_state["run"] = data["current"].run
        current_state["time"] = data["current"].time
        current_state["anchors"] = {}
        for a_id in data["current"].anchors:
            anchor = {}
            anchor_info = data["current"].anchors[a_id]
            anchor["mean_std"] = anchor_info.mean_std
            anchor["occluded_by"] = anchor_info.occluded
            anchor["observed"] = anchor_info.observed
            current_state["anchors"][a_id] = anchor



        previous_state = {}
        previous_state["run"] = data["previous"].run
        previous_state["time"] = data["previous"].time
        previous_state["anchors"] = {}
        for a_id in data["previous"].anchors:
            anchor = {}
            anchor_info = data["previous"].anchors[a_id]
            anchor["mean_std"] = anchor_info.mean_std
            anchor["occluded_by"] = anchor_info.occluded
            anchor["observed"] = anchor_info.observed
            previous_state["anchors"][a_id] = anchor


        data_refactored["current"] = current_state
        data_refactored["previous"] = previous_state

        run = data_refactored["current"]["run"]
        time = data_refactored["current"]["time"]


        data_repickled_file = os.path.join(data_repickled_dir, "run{}_time{}.pickle".format(run, time))
        with open(data_repickled_file, 'wb') as handle:
            pickle.dump(data_refactored, handle, protocol=pickle.HIGHEST_PROTOCOL)



    # print(pathlist)
    # for path in pathlist:
    # # for filename in os.listdir(data_dir):
    #     # data_file = os.path.join(data_dir, "run{}_time{}.pickle".format(self.current.run, self.current.time))
    #     # data_repickled_file = os.path.join(data_repickled_dir, "run{}_time{}.pickle".format(self.current.run, self.current.time))
    #     print(path)




if __name__ == "__main__":
    main()
