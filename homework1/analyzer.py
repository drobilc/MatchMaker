#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import glob

detectors = ["haar", "hog"]
video_ids = glob.glob("*.mp4")
factors = ["2", "4", "8"]
bw = ["True", "False"]

main_results = [] # detector | video | TP | FP | FN | processed_frames | TP/processed_frames

# open file for analysis results
result_file_name = "results.txt"
result_file = open(result_file_name, "a")

# repeat
for detector in detectors:
    for video_id in video_ids:
        for downscale_factor in factors:
            for bw_mode in bw:
                # sestavi ime fila
                data_file_name = detector + "_" + video_id + "_"  + downscale_factor + "_" + bw_mode + ".txt"

                # open file
                data_file = open(data_file_name, "r")
                lines = data_file.readlines()

                # calculate things
                main_results_line = []
                main_results_line.append(detector)
                main_results_line.append(video_id)
                main_results_line.append(lines[7]) # TP
                main_results_line.append(lines[6]-lines[7]) # FP
                main_results_line.append(lines[5]-lines[7]) # FN
                main_results_line.append(lines[5]) # processed frames
                main_results_line.append(lines[7]/lines[5]) # ratio

                # write to file
                result_file.write("{}\n".format(main_results_line))

                # close file
                data_file.close()