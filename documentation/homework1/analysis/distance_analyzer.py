#!/usr/bin/env python
import glob

angles = ["0deg", "30deg", "45deg", "60deg"]

main_results = [[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0]] # detector | video | resolution | TP | FP | FN | processed_frames | TP/processed_frames

# open file for analysis results
result_file_name = "distance_results.txt"
result_file = open(result_file_name, mode="a")

for file in glob.glob("data/*.txt"):
    data_file = open(file)
    lines = data_file.readlines()
   

    frames = len(lines) - 9
    fraction = frames / 12

    TP60 = 0
    TP55 = 0
    TP50 = 0
    TP45 = 0
    TP40 = 0
    TP35 = 0
    TP30 = 0
    TP25 = 0
    TP20 = 0
    TP15 = 0
    TP10 = 0
    TP05 = 0
    distances = []
    for i in range(1, frames + 1):
        j = frames + 1 - i
        index = i + 7

        if len(lines[index].strip()) > 2:
            if j > fraction * 11:
                TP60 += 1
            elif j > fraction * 10:
                TP55 += 1
            elif j > fraction * 9:
                TP50 += 1
            elif j > fraction * 8:
                TP45 += 1
            elif j > fraction * 7:
                TP40 += 1
            elif j > fraction * 6:
                TP35 += 1
            elif j > fraction * 5:
                TP30 += 1
            elif j > fraction * 4:
                TP25 += 1
            elif j > fraction * 3:
                TP20 += 1
            elif j > fraction * 2:
                TP15 += 1
            elif j > fraction * 1:
                TP10 += 1
            else:
                TP05 += 1

    distances = [TP05, TP10, TP15, TP20, TP25, TP30, TP35, TP40, TP45, TP50, TP55, TP60]
    if "60" in file:
        # prištej distances na pravi indeks
        for idx in range(0,12):
            main_results[3][idx] += distances[idx]
    elif "45" in file:
        # prištej distances na pravi indeks
        for idx in range(0,12):
            main_results[2][idx] += distances[idx]
    elif "30" in file:
        # prištej distances na pravi indeks
        for idx in range(0,12):
            main_results[1][idx] += distances[idx]
    else:
        # prištej distances na pravi indeks
        for idx in range(0,12):
            main_results[0][idx] += distances[idx]

    # close file
    data_file.close()

# write to file
result_file.write("{}\n".format(main_results))
                    
result_file.close()