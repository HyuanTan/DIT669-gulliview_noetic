#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tools

counter = 0
camera_id = "camera3"
folder = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/"
file_path = folder + str(counter) + "_full_rows.csv"
full_rows = tools.load_full_rows_from_csv(file_path)
output_path = folder + camera_id + "_" + str(counter)

tools.visualize_error(full_rows, save_path=output_path, cameraid=camera_id)

tools.visualize_radius_error(full_rows, save_path=output_path, cameraid=camera_id)


# file_list = [folder+"camera1_0_statistics.txt", folder+"camera0_0_statistics.txt"]
# tools.plot_boxplot(file_list, save_path=output_path, type_name="distance")