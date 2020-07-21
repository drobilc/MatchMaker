#!/usr/bin/env python
import glob

detectors = ["haar", "hog"]
# file_ids = glob.glob("data/*.txt")
faces = ["face01", "face02"]
angles = ["0deg", "30deg", "45deg", "60deg"]
factors = ["2", "4", "8"]
bw = ["True", "False"]

main_results = [] # detector | video | TP | FP | FN | processed_frames | TP/processed_frames

# open file for analysis results
result_file_name = "results.txt"
result_file = open(result_file_name, mode="a")

# repeat
for detector in detectors:
    for face in faces:
        for angle in angles:
            for downscale_factor in factors:
                for bw_mode in bw:
                    # sestavi ime fila
                    data_file_name = "data/" + detector + "_" + face + "_" + angle + "_"  + downscale_factor + "_" + bw_mode + ".txt"

                    # open file
                    data_file = open(data_file_name, mode="r")
                    lines = data_file.readlines()

                    # calculate things
                    main_results_line = []
                    main_results_line.append(detector)
                    main_results_line.append(face + "_" + angle)
                    main_results_line.append(lines[7].strip()) # TP
                    main_results_line.append(int(lines[6].strip()) - int(lines[7].strip())) # FP
                    main_results_line.append(int(lines[5].strip()) - int(lines[7].strip())) # FN
                    main_results_line.append(lines[5].strip()) # processed frames
                    main_results_line.append(int(lines[7].strip()) / int(lines[5].strip())) # ratio

                    # write to file
                    result_file.write("{}\n".format(main_results_line))

                    # close file
                    data_file.close()
result_file.close()