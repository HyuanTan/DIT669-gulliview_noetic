#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import Counter
import matplotlib.pyplot as plt
import tools
import copy

'''
pip install folium geopandas matplotlib

'''
global Flag_vis # For analysis
global Image_vi
Flag_vis = True

# Model = 0 # undistored,2k
Model = 1 # Birdview,2k

class CircleGridAutoPattern:
    def __init__(self):
        rospy.init_node("circle_grid_auto_pattern", anonymous=True)
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.bridge = CvBridge()
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.visualization_pub = rospy.Publisher("/circle_grid_visualization", Image, queue_size=10)

        rospy.loginfo(f"Subscribed to image topic: {self.image_topic}")
        self.counter = 0

    def is_red(self, frame, center):
        """
        Determine if the specified position is red.
        :param frame: Input image frame
        :param center: Pixel coordinates (x, y)
        :return: Boolean True if red, False otherwise
        """
        flag = False
        x, y = int(center[0]), int(center[1])
        b, g, r = frame[y, x]
        # if r > 150:
        if r > 140:
        # if r > 150 and g < 100 and b < 100:
        # if r > 120 and g < 100 and b < 100:
            flag = True
            # rospy.loginfo(f"r: {r}")

        return flag, r

    def is_white(self, frame, center):
        """
        Determine if the specified position is white.
        :param frame: Input image frame
        :param center: Pixel coordinates (x, y)
        :return: Boolean True if white, False otherwise
        """
        flag = False
        x, y = int(center[0]), int(center[1])
        b, g, r = frame[y, x]
        light = 100
        if r > light and g > light and b > light:
            flag = True
        return flag

    def encode_row_to_decimal(self, frame, row):
        """
        Encode a row of detected dots into a decimal value.
        :param frame: Input image frame
        :param row: Row of detected dots, used to calculate the decimal value.
        :return: Current row's decimal value
        """

        reverse_flag = True # If True: bottom-right is (0, 0), False: top-left is (0, 0)
        # sorted_row = sorted(row, key=lambda circle: circle[0][0])  # circle[0][0],left-up is（0，0）
        sorted_row = sorted(row, key=lambda circle: circle[0][0], reverse=reverse_flag) # Sort dots based on x-coordinate
        # rospy.loginfo(f"----------sorted_row size: {len(sorted_row)}")
        sum_decimal = 0
        visualize = False
        if Model == 0:
            front_scal = 0.2
            circle_size = 3
        elif Model == 1:
            front_scal = 0.5
            circle_size = 4
            front_scal2 = 0.3
        global Image_vi, Flag_vis
        # for i, (center, _) in enumerate(sorted_row):
        for i in range(len(sorted_row)):
            center, radius = sorted_row[i]
            x, y = int(center[0]), int(center[1])
            # rospy.loginfo(f"i: {i}, (x,y):{x, y}")
            flag, r = self.is_red(frame, (x, y))
            if flag:
                ##Encoding is shifted to the middle of the mat.
                sum_decimal += 2**(i-11)
                if visualize:
                    cv2.circle(frame, (x, y), circle_size, ((0,0, 255)), -1)
                    cv2.putText(
                            frame,f"{int(i)}", (x, y+10),  
                            cv2.FONT_HERSHEY_SIMPLEX,front_scal,(0, 0, 0),
                            1, cv2.LINE_AA,)
                    cv2.putText(
                        frame,f"{int(r)}", (x, y+25),  
                        cv2.FONT_HERSHEY_SIMPLEX,front_scal,(0, 0, 0),
                        1, cv2.LINE_AA,)
                elif Flag_vis:
                    cv2.circle(Image_vi, (x, y), int(radius), ((0,1,0)), 1)
                    cv2.putText(
                        Image_vi,f"{radius:.2f}", (x-10, y+25),  
                        cv2.FONT_HERSHEY_SIMPLEX,front_scal2,(0, 0, 0),
                        1, cv2.LINE_AA,)

            else:
                if visualize:
                    cv2.circle(frame, (x, y), circle_size, ((0,255,0)), -1)
                    cv2.putText(
                            frame,f"{int(i)}", (x, y+10),  
                            cv2.FONT_HERSHEY_SIMPLEX,front_scal,(0, 0, 255),
                            1, cv2.LINE_AA,)
                    cv2.putText(
                        frame,f"{int(r)}", (x, y+25),  
                        cv2.FONT_HERSHEY_SIMPLEX,front_scal,(0, 0, 0),
                        1, cv2.LINE_AA,)
                elif Flag_vis:
                    cv2.circle(Image_vi, (x, y), int(radius), ((0,255,0)), 1)
                    cv2.putText(
                        Image_vi,f"{radius:.2f}", (x-10, y+25),  
                        cv2.FONT_HERSHEY_SIMPLEX,front_scal2,(0, 0, 0),
                        1, cv2.LINE_AA,)


        # Visualize the encode
        if len(sorted_row) > 0:
            first_center = sorted_row[0][0]  # Get the center of the first dot
            x, y = int(first_center[0]), int(first_center[1])
            text = f"{int(sum_decimal)}"
            if Model == 0:
                front_scal = 0.3
                front_thinkness = 1
                front_offset = 50
            elif Model == 1:
                front_scal = 0.8
                front_thinkness = 2
                front_offset = 100
            if reverse_flag:
                cv2.putText(
                    frame,
                    text,
                    (x + front_offset, y),  # Text offset to the right
                    cv2.FONT_HERSHEY_SIMPLEX,
                    front_scal,
                    (0, 0, 255),
                    front_thinkness,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    frame,
                    text,
                    (x - front_offset, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    front_scal,
                    (0, 0, 255),
                    front_thinkness,
                    cv2.LINE_AA,
                )

        return sum_decimal, frame


 
    def encode_full_rows_with_centers(self, frame, full_rows):
        """
        对完整的行进行十进制编码，同时返回每行圆心坐标列表
        :param frame: 输入图像
        :param full_rows: 多行圆信息 [ [((x1, y1), r1), ...], [((x3, y3), r3), ...], ... ]
        :return: [decimal_codes, centers_list]
                decimal_codes: 每行的十进制编码列表 [(行号, 编码), ...]
                centers_list: 每行圆心坐标列表 [[(x1, y1), (x2, y2), ...], [(x3, y3), (x4, y4), ...], ...]

        Encode the complete rows and return the decimal codes along with circle center coordinates.
        :param frame: Input image
        :param full_rows: List of multiple row data [[(x1, y1), r1], [(x3, y3), r3], ...], ...
        :return: [decimal_codes, centers_list]
            decimal_codes: List of decimal codes for each row [(row_index, code), ...]
            centers_list: List of circle center coordinates for each row [[(x1, y1), (x2, y2), ...], [(x3, y3), (x4, y4), ...], ...]
        """
        encoded_rows = []
        global Image_vi, Flag_vis
        if Flag_vis:
            Image_vi = copy.deepcopy(frame)
            # Image_vi = frame.copy()

        for row_idx, row in enumerate(full_rows):
            # rospy.loginfo(f"row_idx: {row_idx}")
            # Convert the row to decimal value
            decimal_code, frame = self.encode_row_to_decimal(frame, row)
            
            # Retrieve the circle centers for the current row
            row_centers = [(center[0], center[1]) for center, r in row]

            encoded_rows.append((decimal_code, row_centers))
        if Flag_vis:
            alpha = 0.4
            cv2.addWeighted(Image_vi, alpha, frame, 1 - alpha, 0, Image_vi)
            output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_visual_filtered_circles.png"
            cv2.imwrite(output_path, Image_vi)

        return frame, encoded_rows
 


    def detect_complete_rows(self, detected_circles, image, row_circle_count=30, raw_tolerance=5.0, raw_tolerance_factor=2.0, colum_tolerance=40.0, colum_tolerance_factor=2.0):
        """
        Group the detected circles into rows, finding complete rows (each row containing `row_circle_count` circles).

        Parameters:
        - detected_circles: list of tuples [(x, y, radius), ...] representing the detected circles
        - row_circle_count: expected number of circles per row
        - tolerance: row detection tolerance

        Returns:
        - full_rows: List of rows, each containing `row_circle_count` circles.
        """
        
        # detected_circles.sort(key=lambda circle: circle[0][1])
        # Sort circles by their y-coordinate, breaking ties with x-coordinate
        detected_circles.sort(key=lambda circle: (circle[0][1], circle[0][0]))
        # for i, circle in enumerate(detected_circles[:50]):
        #     print(f"circles {i+1}: center: ({circle[0][0]}, {circle[0][1]})")

        
        ungrouped_circles = detected_circles.copy()  # 初始化未分组的圆
        

        rows = []  # Store grouped rows
        flag = False
        current_row = []
        while len(ungrouped_circles) > 0.0:  # Iterate until all circles are grouped
            
            # rospy.loginfo(f"******** size of ungrouped_circles: {len(ungrouped_circles)}")
            current_row = [ungrouped_circles[0]]  # Start a new row with the first ungrouped circle
            ungrouped_circles.pop(0)  # Remove the circle from ungrouped list
            # rospy.loginfo(f"******** len of ungrouped_circles: {len(ungrouped_circles)}")

            # Iterate all ungrouped list
            i = 0
            flag = False
            while i < len(ungrouped_circles):
                # circle  belong to the same row
                if (
                    abs(ungrouped_circles[i][0][1] - current_row[-1][0][1]) <= raw_tolerance
                and abs(ungrouped_circles[i][0][0] - current_row[-1][0][0]) <= colum_tolerance
                ):
                    current_row.append(ungrouped_circles[i])
                    ungrouped_circles.pop(i)
                else:
                    i += 1  # Move to the next circle if no match is found

                # Add the current row to rows if it meets the minimum circle count
                if len(current_row) >= 30:
                    rows.append(current_row)
                    current_row = []
                    # rospy.loginfo(f"******** len of current_row(break): {len(current_row)}")
                    break
                if(i >= len(ungrouped_circles)):
                    rows.append(current_row)
                    flag = True
                    current_row = []
                    # rospy.loginfo(f"====== i:{i}, len of ungrouped_circles: {len(ungrouped_circles)}")
                    # rospy.loginfo(f"====== len of current_row: {len(current_row)}")

            

        # Add the last row if it exists
        if current_row and not flag:
            rows.append(current_row)
        # rospy.loginfo(f"******** size of current_row: {len(rows)}")

        # Process each row and sort by x-coordinate
        full_rows = []
        for row in rows:
            # rospy.loginfo(f"******** len of row: {len(row)}")
            if len(row) == row_circle_count:  #Add row only if it meets the required circle count
                row.sort(key=lambda circle: circle[0][0])  # Sort by x-coordinate
                full_rows.append(row)
            else:
                ungrouped_circles.extend(row)
        rospy.loginfo(f"******** size of full_rows: {len(full_rows)}")
        #------------------------------------------
        ungrouped_circles.sort(key=lambda circle: (circle[0][1], circle[0][0]))
        rows = []
        update_raw_tolerance = raw_tolerance*raw_tolerance_factor
        update_colum_tolerance = colum_tolerance*colum_tolerance_factor
        flag = False
        while len(ungrouped_circles) > 0.0:
            
            current_row = [ungrouped_circles[0]]
            ungrouped_circles.pop(0)
            # rospy.loginfo(f"******** New len of ungrouped_circles: {len(ungrouped_circles)}")
            # 遍历剩余的未分组圆
            i = 0
            flag = False
            while i < len(ungrouped_circles):
                # Check if the circle belongs to the current row based on tolerance
                if (
                    abs(ungrouped_circles[i][0][1] - current_row[-1][0][1]) <= update_raw_tolerance
                and abs(ungrouped_circles[i][0][0] - current_row[-1][0][0]) <= update_colum_tolerance
                ):
                    current_row.append(ungrouped_circles[i])
                    ungrouped_circles.pop(i)
                else:
                    i += 1

                # Add the current row to rows if it meets the criteria
                if len(current_row) >= 30:
                    rows.append(current_row)
                    flag = True
                    current_row = []
                    # rospy.loginfo(f"******** New size of ungrouped_circles: {len(current_row)}")
                    break
        if current_row and not flag:
            rows.append(current_row)
        rospy.loginfo(f"******** New len of rows: {len(rows)}")
        for row in rows:
            # rospy.loginfo(f"******** len of row: {len(row)}")
            if len(row) == row_circle_count:
                row.sort(key=lambda circle: circle[0][0])
                full_rows.append(row)
             
        # --------------------

        full_rows.sort(key=lambda row: row[0][0][1])
        rospy.loginfo(f"******** Final size of full_rows: {len(full_rows)}")

        if True:
            # Visualize the complete rows
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
            text_offset = 18
            front_scal = 0.3
            front_thinkness = 1
            if Model == 0:
                text_offset = 18
                front_scal = 0.3
                front_thinkness = 1
            elif Model == 1:
                text_offset = 50
                front_scal = 0.7
                front_thinkness = 2
            for row_idx, row in enumerate(full_rows):
                color = colors[row_idx % len(colors)]
                # cv2.circle(image, (int(row[0][0][0]), int(row[0][0][1])), int(row[0][1]), color, -1)
                
                for circle_idx, (center, r) in enumerate(row):
                    # cv2.circle(image, (int(center[0]), int(center[1])), int(r), color, -1)
                    # The first circle of one row, lable the counter number
                    if circle_idx == 0:
                        position = (int(center[0] - text_offset), int(center[1]))
                        cv2.putText(image, f"{row_idx+1}", position, cv2.FONT_HERSHEY_SIMPLEX, 
                                    front_scal, (0, 255, 0), front_thinkness, cv2.LINE_AA)

            # output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_filtered_circles.png"
            # cv2.imwrite(output_path, Image_vi)

        rospy.loginfo(f"**************complete_rows size {len(full_rows)}")
        return image, full_rows
                    

    def detect_and_visualize_circles(self, image):
        """
        Detect and visualize circles in the given image, and output their size (radius).
        :param image: Input image
        :return: Processed image and detected circle information
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # # Convert to binary image
        # _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # # Detect contours
        # contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # Adjust parameter minDist to set the minimum distance between circles.
        # Use param1 and param2 to adjust edge detection and circle detection thresholds.
        # Set minRadius and maxRadius to restrict the range of circle radii.
        # contours = cv2.HoughCircles(
        #     blurred, 
        #     cv2.HOUGH_GRADIENT, 
        #     dp=1.2, 
        #     minDist=4, 
        #     param1=20, 
        #     param2=5, 
        #     minRadius=6, 
        #     maxRadius=9.0
        # )
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(image, contours, -1, (0,255,0), 2)

        # Initialize list to store detected circles
        detected_circles = []
        
        
        # Iterate through contours
        for contour in contours:
            # Find the minimum enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            center = (x, y)
            # center = (int(x), int(y))
            # radius = int(radius)

            # Calculate perimeter and area
            perimeter = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)

            # Skip contours with zero area to avoid division errors
            if area == 0:
                continue

            # Compute circularity: 4 * pi * area / (perimeter^2), where circularity equals 1 for a perfect circle
            circularity = 4 * np.pi * (area / (perimeter * perimeter))

            
            # Check the color properties of the circle(outlier with white color)
            if self.is_white(image, (x, y)):
                # rospy.logwarn("Get abnormal color circle")
                continue
            

            # cv2.putText(
            #     image,f"{int(radius)}", (int(x), int(y)+25),  
            #     cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 0, 0),
            #     1, cv2.LINE_AA,)

            # Model-specific thresholds for circle detection: perfect circle, the radius
            if Model == 0:
                if 0.001 <= circularity <= 1.8 and 0.05 <= radius <= 8.0:
                    detected_circles.append((center, radius)) # undistorted
            elif Model == 1:
                # if 0.01 <= circularity <= 2.5 and 0.2 <= radius <= 14:
                # if 0.005 <= circularity <= 6.0 and 1.6 <= radius <= 13: # left-up
                if 0.005 <= circularity <= 6.0 and 6.0 <= radius <= 21: # right-down, 8.5 MIN threshod, 2k r=10, camera1 r<20,2-4k
                # if 0.003 <= circularity <= 8.0 and 1.6 <= radius <= 30: # right-down, 8.5 MIN threshod, 2k r=10, camera1 r<20,4k
                # if 0.005 <= circularity <= 6.0 and 8.5 <= radius <= 9.5: # right-down
                    detected_circles.append((center, radius)) # birdview


                # cv2.circle(image, (int(center[0]), int(center[1])), int(radius), (0, 0, 255), 2)
        rospy.loginfo(f"detected circles size {len(detected_circles)}")

        if True:
            method = "distance" # undistorted
            if Model == 0:
                method = "distance" # undistorted
            elif Model == 1:
                method = "dbscan"# birdview

            if method == "distance":
                filtered_circles = tools.filter_circles_by_distance(detected_circles, 25.0)# undistorted
            elif method == "dbscan":
                # clustered_circles = tools.cluster_circles_dbscan(detected_circles, 50, 10)
                clustered_circles = tools.cluster_circles_dbscan(detected_circles, 180, 50) # birdview, left_up
                filtered_circles = [circle for cluster in clustered_circles.values() for circle in cluster]
                # assert len(filtered_circles) == len(set(filtered_circles)), "Duplicate circles found!"

            else:
                raise ValueError("Invalid method. Use 'distance' or 'dbscan'.")
        


        rospy.loginfo(f"=======filtered detected circles size {len(filtered_circles)}")
        # cv2.circle(image, (int(center[0]), int(center[1])), int(radius), (0, 255, 0), 2)
        
        # Call the deduplication function
        if Model == 0:
            unique_circles = self.remove_duplicate_circles(filtered_circles, distance_threshold=5.0)
        elif Model == 1:
            # unique_circles = self.remove_duplicate_circles(filtered_circles, distance_threshold=30.0)# up_left
            unique_circles = self.remove_duplicate_circles(filtered_circles, distance_threshold=30.0)

        if False:
            # Visualize and draw circles
            for (center, radius) in unique_circles:
                x = int(center[0])
                y = int(center[1])
                cv2.circle(image, (x, y), int(radius), (255, 0, 0), -1)
                
                # Display coordinates at the center of the circle
                # center_text = f"({x}, {y})"
                # center_text = f"{x}"
                # center_text = f"{y}"
                # if Model == 0:
                #     cv2.putText(image, center_text, (x - 8, y + 10), 
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 0, 255), 1) # UNDISTORED
                # elif Model == 1:
                #     cv2.putText(image, center_text, (x - 8, y + 10), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 255), thickness=1, lineType=cv2.LINE_AA) # BIRDVIEW
             
                

        rospy.loginfo(f"Filtered circles after removing duplicates: {len(unique_circles)}")

        # self.counter = self.counter + 1
        # output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_filtered_circles.png"
        # cv2.imwrite(output_path, image)
        # cv2.imshow("Filtered Circles", image)
        
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
 

        return image, unique_circles

    # Define a function to calculate circularity
    def calculate_circularity(self, area, perimeter):
        return 4 * np.pi * (area / (perimeter * perimeter))

    # Perform deduplication and optimization for detected circles
    def remove_duplicate_circles(self, filtered_circles, distance_threshold=5.0):
        # Sort the circles by x-coordinate (ascending order)
        filtered_circles.sort(key=lambda circle: circle[0][0])

        # Store unique circles
        unique_circles = []
        visited = set()

        for i, (center1, radius1) in enumerate(filtered_circles):
            if i in visited:
                continue
            x1, y1 = center1
            area1 = np.pi * (radius1 ** 2)
            perimeter1 = 2 * np.pi * radius1
            circularity1 = self.calculate_circularity(area1, perimeter1)

            best_circle = (center1, radius1, area1, circularity1)

            min_index = min(len(filtered_circles), i + 500)
            for j in range(i + 1, min_index):
            # for j in range(i + 1, len(filtered_circles)):
                if j in visited:
                    continue

                center2, radius2 = filtered_circles[j]
                x2, y2 = center2

                # Calculate the distance between two circles
                distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                if distance <= distance_threshold:
                    area2 = np.pi * (radius2 ** 2)
                    perimeter2 = 2 * np.pi * radius2
                    circularity2 = self.calculate_circularity(area2, perimeter2)

                    # Keep the circle with the better attributes
                    # if circularity2 > best_circle[3] or (circularity2 == best_circle[3] and area2 > best_circle[2]):
                    # if circularity2 > best_circle[3] or area2 > best_circle[2]:
                    if area2 > best_circle[2]:
                        best_circle = (center2, radius2, area2, circularity2)
                        # rospy.loginfo(f"&&&&&&&&&&&&&&&&&&&&&& Replace {i} with {j} distance:{distance}")
                    visited.add(j)
            unique_circles.append((best_circle[0], best_circle[1]))

        return unique_circles

    def get_first_row_and_col(self, full_rows, image):
        """
        Extract the first row and column of circles from the given list.

        :param full_rows: A complete list of rows, where each row is a list of circle information [(x, y, radius), ...].
        :return: The first row's circle centers and the first column's circle centers.
        """
        if not full_rows:
            return [], []

        # First row: Sort the first row by x-coordinate
        first_row = sorted(full_rows[0], key=lambda circle: circle[0][0])
        first_row_centers = [center for center, _ in first_row]

        # First column: Extract the first circle in each row, then sort by y-coordinate
        first_col = [row[0] for row in full_rows]
        first_col_centers = sorted([center for center, _ in first_col], key=lambda c: c[1])

        if False:
            # Visualize the first row and first column with index labels
            for idx, (x, y) in enumerate(first_row_centers):
                cv2.circle(image, (int(x), int(y)), 4, (255, 0, 0), -1)
                cv2.putText(image, str(idx), (int(x), int(y) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 0), 1)

            for idx, (x, y) in enumerate(first_col_centers):
                cv2.circle(image, (int(x), int(y)), 4, (0, 0, 255), -1)
                cv2.putText(image, str(idx), (int(x), int(y) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 0, 255), 1)


        return first_row_centers, first_col_centers, image


    def calculate_basis(self, first_row_centers, first_col_centers):
        """
        Calculate the directional basis vectors for the coordinate system using the first row and first column of circle centers.

        :param first_row_centers: List of circle centers in the first row [(x1, y1), (x2, y2), ...]
        :param first_col_centers: List of circle centers in the first column [(x1, y1), (x2, y2), ...]
        :return: Basis vectors basis_x, basis_y
        """
        if len(first_row_centers) < 2 or len(first_col_centers) < 2:
            raise ValueError("The number of circles in the first row or column is insufficient to calculate basis vectors")
        
        # Calculate the X-direction basis vector using the average of differences between each point and the first point
        row_differences = [np.array(point) - np.array(first_row_centers[0]) for point in first_row_centers[1:]]
        basis_x = np.mean(row_differences, axis=0)
        basis_x = basis_x / np.linalg.norm(basis_x)  # Normalize
        
        # Calculate the Y-direction basis vector using the average of differences between each point and the first point
        col_differences = [np.array(point) - np.array(first_col_centers[0]) for point in first_col_centers[1:]]
        basis_y = np.mean(col_differences, axis=0)
        basis_y = basis_y / np.linalg.norm(basis_y)  # Normalize
        
        # Ensure basis vectors are orthogonal (optional, based on requirements)
        dot_product = np.dot(basis_x, basis_y)
        if abs(dot_product) > 1e-6:
            basis_y = basis_y - dot_product * basis_x
            basis_y = basis_y / np.linalg.norm(basis_y)
        
        return basis_x, basis_y


    def project_pixel_to_basis(self, pixel, origin, basis_x, basis_y):
        """
        Calculate the projection of a pixel in the basis coordinate system.

        :param pixel: Pixel coordinates (x, y)
        :param origin: Origin of the coordinate system (x0, y0), usually the center of the first circle in the first row or column
        :param basis_x: Basis vector in the X direction
        :param basis_y: Basis vector in the Y direction
        :return: Projections (u, v)
        """
        # Convert pixel coordinates to a vector relative to the origin
        relative_vector = np.array(pixel) - np.array(origin)

        # Calculate projections
        u = np.dot(relative_vector, basis_x)  # Projection on the X direction
        v = np.dot(relative_vector, basis_y)  # Projection on the Y direction

        return u, v


    def visualize_basis_from_calculation(self, image, origin, basis_x, basis_y, scale=50, color_x=(255, 0, 0), color_y=(0, 255, 0)):
        """
        Visualize the basis vectors returned by calculate_basis on the image.

        :param image: Input image
        :param origin: Origin coordinates (x0, y0)
        :param basis_x: Basis vector in the X direction
        :param basis_y: Basis vector in the Y direction
        :param scale: Scaling factor for the basis vectors
        :param color_x: Color for the X-direction basis vector (B, G, R)
        :param color_y: Color for the Y-direction basis vector (B, G, R)
        :return: Image with the visualized basis vectors
        """
        thickness = 1
        tipLength=0.01
        text_offset = 10
        front_scal = 0.5
        front_thinkness = 2
        end_point_x_scal_factor = 1
        end_point_y_scal_factor = 2
        if Model == 0:
            thickness = 1
            tipLength=0.01
            text_offset = 10
            front_scal = 0.5
            front_thinkness = 2
            end_point_x_scal_factor = 1
            end_point_y_scal_factor = 2
        elif Model == 1:
            thickness = 3
            tipLength=0.06
            text_offset = 30
            front_scal = 1.0
            front_thinkness = 3
            end_point_x_scal_factor = 3
            end_point_y_scal_factor = 3.5

        # Calculate the endpoints for X and Y basis vectors
        end_point_x = (
            int(origin[0] + basis_x[0] * scale*end_point_x_scal_factor), 
            int(origin[1] + basis_x[1] * scale*end_point_x_scal_factor)
        )
        end_point_y = (
            int(origin[0] + basis_y[0] * scale*end_point_y_scal_factor), 
            int(origin[1] + basis_y[1] * scale*end_point_y_scal_factor)
        )

        # Convert origin to integer coordinates
        origin = (int(origin[0]), int(origin[1]))

        # Draw arrows for X and Y basis vectors
        image_with_basis = image.copy()
        cv2.arrowedLine(image_with_basis, origin, end_point_x, color_x, thickness=thickness, tipLength=tipLength)   
        cv2.arrowedLine(image_with_basis, origin, end_point_y, color_y, thickness=thickness, tipLength=tipLength) 

        # Add text labels for X and Y
        cv2.putText(image_with_basis, "X", (end_point_x[0]+text_offset, end_point_x[1]), cv2.FONT_HERSHEY_SIMPLEX, front_scal, color_x, front_thinkness)
        cv2.putText(image_with_basis, "Y", (end_point_y[0]-text_offset, end_point_y[1]+text_offset), cv2.FONT_HERSHEY_SIMPLEX, front_scal, color_y, front_thinkness)

        return image_with_basis


    def image_callback(self, msg):
        """
        Callback function to process the image data from the topic.
        """
        try:
            # Convert the ROS image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Detect and visualize circles
        visualization, circles = self.detect_and_visualize_circles(frame)
        # circle_centers = np.array([center for center, r in circles])
        
        # Automatically detect the raw of circle pattern
        if Model == 0:
            visualization, full_rows = self.detect_complete_rows(circles, visualization, row_circle_count=30, raw_tolerance=6.0, raw_tolerance_factor=1.2, colum_tolerance=150, colum_tolerance_factor=1.2) # camera3-undistored
        elif Model == 1:
            # 40,1000
            # visualization, full_rows = self.detect_complete_rows(circles, visualization, row_circle_count=30, raw_tolerance=12.0, raw_tolerance_factor=1.2, colum_tolerance=1400, colum_tolerance_factor=1.2) # camera3-birdview，old dot(left_up)using,2-4k
            visualization, full_rows = self.detect_complete_rows(circles, visualization, row_circle_count=30, raw_tolerance=18.0, raw_tolerance_factor=1.2, colum_tolerance=1600, colum_tolerance_factor=1.2) # camera1-birdview，(right_down)4k-24
            # visualization, full_rows = self.detect_complete_rows(circles, visualization, row_circle_count=30, raw_tolerance=12.0, raw_tolerance_factor=1.2, colum_tolerance=1100, colum_tolerance_factor=1.2) # camera3-birdview，old dot(right_down)
        
        visualization, encoded_rows = self.encode_full_rows_with_centers(visualization, full_rows)
        if False:
            file_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_encoded_rows.csv"
            tools.save_encoded_rows_to_csv(encoded_rows, file_path)
        
        if True:
            first_row_centers, first_col_centers, visualization = self.get_first_row_and_col(full_rows, visualization)
            # Calculate the basis vectors
            origin = first_row_centers[0]
            basis_x, basis_y = self.calculate_basis(first_row_centers, first_col_centers)

            # Visualize the basis vectors
            visualization = self.visualize_basis_from_calculation(visualization, origin, basis_x, basis_y, scale=500)

        if True:
            output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_filtered_circles.png"
            cv2.imwrite(output_path, visualization)
            
        if True:
            # output_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter)
            # tools.visualize_error(full_rows, save_path=output_path)
            file_path = "/home/holly/catkin_ws_dit669/src/dots_ucs/image/" + str(self.counter) + "_full_rows.csv"
            tools.save_full_rows_to_csv(full_rows, file_path)

        self.counter = self.counter + 1
        '''
        # Calculate the projection of a pixel in the basis coordinate system
        pixel = (175, 125)  # Example pixel coordinates
        u, v = self.project_pixel_to_basis(pixel, origin, basis_x, basis_y)
        rospy.loginfo(f"pixel projection: {pixel} ---> {u, v}")
        '''
        # visualize and anylize the error
        if False:
            try:
                visualization_msg = self.bridge.cv2_to_imgmsg(visualization, encoding="bgr8")
                visualization_msg.header.stamp = rospy.Time.now()
                self.visualization_pub.publish(visualization_msg)

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")


if __name__ == "__main__":
    try:
        node = CircleGridAutoPattern()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
