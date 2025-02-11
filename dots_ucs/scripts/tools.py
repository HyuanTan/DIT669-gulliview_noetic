#!/usr/bin/env python3
import math
from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D
import csv
import os
from collections import defaultdict
from scipy.stats import zscore

def filter_circles_by_distance(circles, distance_threshold=50):
    """
    Filter out clusters of circles that are too close based on a distance threshold.
    
    :param circles: Detected circles [(x, y, radius), ...]
    :param distance_threshold: Distance threshold to filter close clusters
    :return: Retained circle clusters
    """
    filtered_circles = []
    for i, (center1, r1) in enumerate(circles):
        x1, y1 = center1
        for j, (center2, r2) in enumerate(circles):
            x2, y2 = center2
            if i >= j:
                continue
            # distance
            distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
            if distance <= distance_threshold:
                filtered_circles.append(((x1, y1), r1))
                break

    return filtered_circles


def cluster_circles_dbscan(circles, eps=50, min_samples=2):
    """
    Use DBSCAN clustering to filter circle clusters.
    
    :param circles: Detected circles [(x, y, radius), ...]
    :param eps: Maximum distance threshold for DBSCAN
    :param min_samples: Minimum number of samples per cluster
    :return: Dictionary of circles grouped by cluster
    """
    # circle centers
    circle_centers = np.array([center for center, r in circles])

    # DBSCAN clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(circle_centers)
    labels = clustering.labels_

    # Group circles by cluster
    clustered_circles = {}
    for label, (center, r) in zip(labels, circles):
        if label == -1:  # noise
            continue
        if label not in clustered_circles:
            clustered_circles[label] = []
        clustered_circles[label].append((center, r))

    return clustered_circles

def save_encoded_rows_to_csv(encoded_rows, file_path):
    """
    Save encoded_rows to a CSV file.
    
    :param encoded_rows: List of encoded rows [(row index, [(x1, y1), (x2, y2), ...])]
    :param file_path: Output CSV file path
    """
    with open(file_path, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['Decimal Code', 'Circle Centers (x, y)'])
        for decimal_code, row_centers in encoded_rows:
            row_data = [decimal_code] + [f"({x}, {y})" for x, y in row_centers]
            writer.writerow(row_data)


def save_full_rows_to_csv(full_rows, filename):
    """
    Saves the full_rows list to a CSV file with values formatted to 4 decimal places.
    
    :param full_rows: List of rows to save.
    :param filename: Name of the CSV file to save.
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        '''
        for row in full_rows:
            # Convert each tuple in the row to a string
            writer.writerow([str(item) for item in row])
        '''

        for row in full_rows:
            # Convert each tuple to a string, formatting numbers to 4 decimal places
            formatted_row = [
                f"({x[0]:.4f}, {x[1]:.4f}), {r:.4f}" if isinstance(x, tuple) else f"{x:.4f}"
                for (x, r) in row
            ]
            writer.writerow(formatted_row)


def load_full_rows_from_csv(filename):
    """
    Loads the full_rows list from a CSV file.
    
    :param filename: Name of the CSV file to load.
    :return: List of rows as loaded from the file.
    """
    full_rows = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Convert each string back to its original tuple representation
            full_rows.append([eval(item) for item in row])
    return full_rows
    

def visualize_error(full_rows, save_path="output", cameraid=""):
    """
    visuialize the adjacent circles distance error

    :param full_rows: center of circles in format: [[((x1, y1), r1), ((x2, y2), r2), ...], ...]
    :param save_path: save path
    """
    real_distance_dis = 42.0
    real_distance_sam = 0.0
    # distance error in x/y direction
    row_diff_x, row_diff_y, row_euclidean = [], [], []
    col_diff_x, col_diff_y, col_euclidean = [], [], []
    row_x_coords, row_y_coords = [], []
    col_x_coords, col_y_coords = [], []
    # Initialize lists for relative errors (relative to mean)
    row_relative_errors = []
    col_relative_errors = []

    for row in full_rows:
        for i in range(len(row) - 1):
            (x1, y1), _ = row[i]
            (x2, y2), _ = row[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            distance = np.sqrt(dx**2 + dy**2)  # Euclidean Distances
            row_diff_x.append(dx)
            row_diff_y.append(dy)
            row_euclidean.append(distance)
            row_x_coords.append(x1)
            row_y_coords.append(y1)
        row_x_coords.append(row[-1][0][0])
        row_y_coords.append(row[-1][0][1])
    mean_distance = np.mean(row_euclidean)
    row_relative_errors = [(d - mean_distance) / mean_distance for d in row_euclidean]
        

    # transpose full_rows to anylize coulum
    full_cols = list(zip(*full_rows))

    for col in full_cols:
        for i in range(len(col) - 1):
            (x1, y1), _ = col[i]
            (x2, y2), _ = col[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            distance = np.sqrt(dx**2 + dy**2) # Euclidean Distances
            col_diff_x.append(dx)
            col_diff_y.append(dy)
            col_euclidean.append(distance)
            col_x_coords.append(x1)
            col_y_coords.append(y1)
        col_x_coords.append(col[-1][0][0])
        col_y_coords.append(col[-1][0][1])
    mean_distance = np.mean(col_euclidean)
    col_relative_errors = [(d - mean_distance) / mean_distance for d in col_euclidean]

    # normalize
    def normalize(data, real_data):
        data = np.array(data) - real_data
        return data

    def normalize2(data):
        data = np.array(data)
        return (data - np.min(data)) / (np.max(data) - np.min(data))
    '''
    row_diff_x_norm = normalize(row_diff_x, real_distance_dis)
    row_diff_y_norm = normalize(row_diff_y, real_distance_sam)
    row_euclidean_norm = normalize(row_euclidean, real_distance_dis)

    col_diff_x_norm = normalize(col_diff_x, real_distance_sam)
    col_diff_y_norm = normalize(col_diff_y, real_distance_dis)
    col_euclidean_norm = normalize(col_euclidean, real_distance_dis)
    '''
    
    row_diff_x_norm = normalize2(row_diff_x)
    row_diff_y_norm = normalize2(row_diff_y)
    row_euclidean_ori = row_euclidean
    row_euclidean_norm = normalize2(row_euclidean)

    col_diff_x_norm = normalize2(col_diff_x)
    col_diff_y_norm = normalize2(col_diff_y)
    col_euclidean_ori = col_euclidean
    col_euclidean_norm = normalize2(col_euclidean)


    x_ticks_step = 0.5
    ##----------------------- HIS 和 log plot in same picture
    if False:
        bin_num = 20
        start = 36.4
        end = 47.6
        interval = 0.2
        print(f"interval:{interval}")
        bins_list = np.round(np.arange(start, end + interval, interval), 2).tolist()
        
        counts, bins = np.histogram(row_euclidean, bins=bins_list)
        # counts, bins = np.histogram(row_euclidean,  bins='fd')
        
        # counts, bins = np.histogram(row_euclidean,  bins=bin_method)
        # sturges
        # doane
        # sqrt
        # scott
        
       
        # exp_counts = np.exp(counts)   
        log_counts = -np.log(counts+1)
        
        print(f"counts:{counts}\n")
        print(f"sum of counts:{sum(counts)}, size:{len(full_rows)}, sum/29:{sum(counts)*1.0/29}")

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, gridspec_kw={'height_ratios': [3, 1]})

        # blue bars(frequency)
        bars1 = ax1.bar(bins[:-1], counts, width=np.diff(bins), align='edge', color='blue', alpha=0.7, label='Frequency')
        ax1.set_ylabel("Frequency", color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.set_title("Row Neighbors Euclidean Distances (Frequency and Logarithmic Frequency)")

        for x, y, width in zip(bins[:-1], counts, np.diff(bins)):
            if y > 0:
                ax1.text(x + width / 2, y, f"{int(y)}", ha='center', va='bottom', fontsize=6, color='black')
                 
        # green bars(Negative Log(Frequency+1))
        bars2 = ax2.bar(bins[:-1], log_counts, width=np.diff(bins), align='edge', color='green', alpha=0.7, label='Negative Log Frequency')
        ax2.set_xlabel("Euclidean Distance(mm)")
        ax2.set_ylabel("Negative Log(Frequency+1) ", color='green')
        ax2.tick_params(axis='y', labelcolor='green')

        
        for x, y, width in zip(bins[:-1], log_counts, np.diff(bins)):
            if y < 0:
                ax2.text(x + width / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=6, color='black')
                
        for x in bins:
            ax1.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)
            ax2.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)

        
        # ax2.set_xticks(bins)
        # ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=45, ha='right', fontsize=8)

        ax1.set_xticks(bins) 
        ax1.set_xticklabels([f"{b-real_distance_dis:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax1.tick_params(axis='x', labelcolor='blue', bottom=True, labelbottom=True, top=False)
        
        ax2.set_xticks(bins) 
        ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax2.tick_params(axis='x', labelcolor='green', bottom=True, labelbottom=True, top=False)
        
 
        plt.tight_layout()
        plt.savefig(f"{save_path}_ori_log_fre_his_row.png")
        # plt.show()
        plt.close()

        #--------------------------------
        # col_euclidean
        counts, bins = np.histogram(col_euclidean, bins=bins_list)
        # counts, bins = np.histogram(col_euclidean, bins=bin_method)
        
        log_counts = -np.log(counts + 1)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, gridspec_kw={'height_ratios': [3, 1]})

        
        bars1 = ax1.bar(bins[:-1], counts, width=np.diff(bins), align='edge', color='blue', alpha=0.7, label='Frequency')
        ax1.set_ylabel("Frequency", color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.set_title("Column Neighbors Euclidean Distances (Frequency and Logarithmic Frequency)")

        
        for x, y, width in zip(bins[:-1], counts, np.diff(bins)):
            if y > 0:
                ax1.text(x + width / 2, y, f"{int(y)}", ha='center', va='bottom', fontsize=6, color='black')
                
        
        bars2 = ax2.bar(bins[:-1], log_counts, width=np.diff(bins), align='edge', color='green', alpha=0.7, label='Negative Log Frequency')
        ax2.set_xlabel("Euclidean Distance(mm)")
        ax2.set_ylabel("Negative Log(Frequency+1) ", color='green')
        ax2.tick_params(axis='y', labelcolor='green')

        
        for x, y, width in zip(bins[:-1], log_counts, np.diff(bins)):
            if y < 0:
                ax2.text(x + width / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=6, color='black')
               
        for x in bins:
            ax1.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)
            ax2.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)

        
        # ax2.set_xticks(bins)
        # ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=45, ha='right', fontsize=8)

        ax1.set_xticks(bins) 
        ax1.set_xticklabels([f"{b-real_distance_dis:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax1.tick_params(axis='x', labelcolor='blue', bottom=True, labelbottom=True, top=False)
        
        ax2.set_xticks(bins) 
        ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax2.tick_params(axis='x', labelcolor='green', bottom=True, labelbottom=True, top=False)
        
 
        plt.tight_layout()
        plt.savefig(f"{save_path}_ori_log_fre_his_col.png") 
        # plt.show()
        plt.close()

    ##-----------------------
    if False:
         
        row_euclidean_norm_tmp = normalize(row_euclidean_ori, real_distance_dis)
        col_euclidean_norm_tmp = normalize(col_euclidean_ori, real_distance_dis)

        scatter_factor = 150
        print(f"row_size_x:{len(row_x_coords)}, row_size_y:{len(row_y_coords)}, size_err:{len(row_euclidean_norm_tmp)}")
        # print(f"row_x_coords:{row_x_coords}\n")
 

        row_x_coords_trimmed = []
        row_y_coords_trimmed = []
        col_x_coords_trimmed = []
        col_y_coords_trimmed = []

        block_size = 30
        take_size = 29

        #  row_x_coords and row_y_coords
        for i in range(0, len(row_x_coords), block_size):
            row_x_coords_trimmed.extend(row_x_coords[i:i + take_size]) 
            row_y_coords_trimmed.extend(row_y_coords[i:i + take_size]) 

        
        sizes = [len(sublist) for sublist in full_cols]
        # print(f"Sizes of each sublist: {sizes}")
        block_size = sizes[0]
        take_size = sizes[0] - 1

        # col_x_coords and col_y_coords
        for i in range(0, len(col_x_coords), block_size):
            col_x_coords_trimmed.extend(col_x_coords[i:i + take_size]) 
            col_y_coords_trimmed.extend(col_y_coords[i:i + take_size]) 
        row_x_coords = row_x_coords_trimmed
        row_y_coords = row_y_coords_trimmed

        col_x_coords = col_x_coords_trimmed
        col_y_coords = col_y_coords_trimmed

        # print(f"----row_x_coords:{row_x_coords}")
        # print(f"----col_euclidean_norm_tmp:{col_euclidean_norm_tmp}")

        col_euclidean_norm_tmp = abs(col_euclidean_norm_tmp)
        row_euclidean_norm_tmp = abs(row_euclidean_norm_tmp)
        print(f"--row_size_x:{len(row_x_coords)}, row_size_y:{len(row_y_coords)}, size_err:{len(row_euclidean_norm_tmp)}")
        print(f"--col_size_x:{len(col_x_coords)}, col_size_y:{len(col_y_coords)}, size_err:{len(col_euclidean_norm_tmp)}")

        
        plt.figure(figsize=(12, 6))
        # scatter in row
        plt.subplot(1, 2, 1)
        scatter = plt.scatter(
            row_x_coords,
            row_y_coords,
            s=row_euclidean_norm_tmp * scatter_factor,  # 点的大小
            # s=np.log1p(row_euclidean_norm_tmp) * scatter_factor,  # 点的大小
            c=row_euclidean_norm_tmp,       # 点的颜色
            cmap="viridis",
            alpha=0.8
        )

        
        for x, y, value in zip(row_x_coords, row_y_coords, row_euclidean_norm_tmp):
            plt.text(
                x, y,                     # position
                f"{value:.2f}",           # value
                fontsize=5,               # fontsize
                color="black",            # color
                ha="center", va="center"  
            )
        plt.colorbar(scatter, label="Row Distance Error")
        plt.title("Row Neighbors(Distance Error/mm)")
        plt.xlabel("X Pixel Coordinate")
        plt.ylabel("Y Pixel Coordinate")

        # scatter in colum
        plt.subplot(1, 2, 2)
        scatter = plt.scatter(
            col_x_coords,
            col_y_coords,
            s=col_euclidean_norm_tmp * scatter_factor,  # size of scatter
            # s=np.log1p(col_euclidean_norm_tmp) * scatter_factor,
            c=col_euclidean_norm_tmp,       
            cmap="plasma",
            alpha=0.8
        )
        for x, y, value in zip(col_x_coords, col_y_coords, col_euclidean_norm_tmp):
            plt.text(
                x, y,                     
                f"{value:.2f}",          
                fontsize=5,               
                color="black",            
                ha="center", va="center"  
            )
        plt.colorbar(scatter, label="Column Distance Error")
        plt.title("Column Neighbors(Distance Error/mm)")
        plt.xlabel("X Pixel Coordinate")
        plt.ylabel("Y Pixel Coordinate")

        plt.tight_layout()
        fig = plt.gcf()
        fig.set_size_inches(18.5, 10.5)   
        plt.savefig(f"{save_path}_scatter_mapped.png", dpi=300, bbox_inches='tight')
        # plt.savefig(f"{save_path}_scatter_mapped.png")   
        # plt.show()
        plt.close()

    # Z-score to detect and delect outlier
    row_euclidean_tmp = np.array(row_euclidean)
    row_euclidean_tmp = row_euclidean_tmp.flatten()
    row_z_scores = zscore(row_euclidean_tmp)
    # threshold = 3 # more than 99.7% within this are normal data
    threshold = 5
    row_filtered_data = row_euclidean_tmp[abs(row_z_scores) < threshold] # filter outlier

    col_euclidean_tmp = np.array(col_euclidean)
    col_euclidean_tmp = col_euclidean_tmp.flatten()
    col_z_scores = zscore(col_euclidean_tmp)
    col_filtered_data = col_euclidean_tmp[abs(col_z_scores) < threshold] # filter outlier
    if True:
        # Compute statistics
        row_stats = {
            "camera_id": str(cameraid),
            "mean": np.mean(row_euclidean),
            "median": np.median(row_euclidean),
            "variance": np.var(row_euclidean),
            "size": len(row_euclidean),
            # "min": np.min(row_euclidean),
            # "q1": np.percentile(row_euclidean, 25),
            # "median": np.median(row_euclidean),
            # "q3": np.percentile(row_euclidean, 75),
            # "max": np.max(row_euclidean),
            "zscore_mean": np.mean(row_filtered_data),
            "zscore_variance": np.var(row_filtered_data),
            "zscore_size": len(row_filtered_data),
        }
        col_stats = {
            "camera_id": str(cameraid),
            "mean": np.mean(col_euclidean),
            "median": np.median(col_euclidean),
            "variance": np.var(col_euclidean),
            "size": len(col_euclidean),
            # "min": np.min(col_euclidean),
            # "q1": np.percentile(col_euclidean, 25),
            # "median": np.median(col_euclidean),
            # "q3": np.percentile(col_euclidean, 75),
            # "max": np.max(col_euclidean),
            "zscore_mean": np.mean(col_filtered_data),
            "zscore_variance": np.var(col_filtered_data),
            "zscore_size": len(col_filtered_data),
        }

        # Print statistics
        print("Row Statistics:", row_stats)
        print("Column Statistics:", col_stats)
        stats_text = (
            "Row Statistics:\n"
            f"  ID: {row_stats['camera_id']}\n"
            f"  Mean: {row_stats['mean']}\n"
            f"  Median: {row_stats['median']}\n"
            f"  Variance: {row_stats['variance']}\n"
            f"  Size: {row_stats['size']}\n"
            # f"  Min: {row_stats['min']}\n"
            # f"  Q1: {row_stats['q1']}\n"
            # f"  Median: {row_stats['median']}\n"
            # f"  Q3: {row_stats['q3']}\n"
            # f"  Max: {row_stats['max']}\n\n"
            f"  ZscoreMean: {row_stats['zscore_mean']}\n"
            f"  ZscoreVariance: {row_stats['zscore_variance']}\n"
            f"  ZscoreSize: {row_stats['zscore_size']}\n\n"
            "Column Statistics:\n"
            f"  ID: {col_stats['camera_id']}\n"
            f"  Mean: {col_stats['mean']}\n"
            f"  Median: {col_stats['median']}\n"
            f"  Variance: {col_stats['variance']}\n"
            f"  Size: {col_stats['size']}\n"
            # f"  Min: {col_stats['min']}\n"
            # f"  Q1: {col_stats['q1']}\n"
            # f"  Median: {col_stats['median']}\n"
            # f"  Q3: {col_stats['q3']}\n"
            # f"  Max: {col_stats['max']}\n"
            f"  ZscoreMean: {col_stats['zscore_mean']}\n"
            f"  ZscoreVariance: {col_stats['zscore_variance']}\n"
            f"  ZscoreSize: {col_stats['zscore_size']}\n\n"
        )

        # save
        with open(f"{save_path}_statistics.txt", "w") as file:
            file.write(stats_text)
    if True:
        #=========row
        
        bins = np.linspace(min(min(row_euclidean), min(row_filtered_data)), max(max(row_euclidean), max(row_filtered_data)), 30)
        
        counts1, _ = np.histogram(row_euclidean, bins=bins)
        log_counts1 = -np.log(counts1 + 1) 
        counts2, _ = np.histogram(row_filtered_data, bins=bins)
        log_counts2 = -np.log(counts2 + 1)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, gridspec_kw={"height_ratios": [3, 1]})

        bars1 = ax1.bar(bins[:-1], counts1, width=np.diff(bins), align='edge', color='blue', alpha=0.5, label='Original Row Distance Frequency')
        bars2 = ax1.bar(bins[:-1], counts2, width=np.diff(bins), align='edge', color='orange', alpha=0.5, label=f'Filtered(Z-Score-threshold:{threshold}) Row Distance Frequency')
        ax1.set_ylabel("Frequency", color='black')
        ax1.set_title("Frequency and Logarithmic Frequency of Original and Filtered(Z-Score) Row Distance")
        ax1.legend()

        for x, y in zip(bins[:-1], counts1):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='bottom', fontsize=9, color='black')
        for x, y in zip(bins[:-1], counts2):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='top', fontsize=9, color='black')

        bars3 = ax2.bar(bins[:-1], log_counts1, width=np.diff(bins), align='edge', color='green', alpha=0.5, label='Original Row Distance Log Frequency')
        bars4 = ax2.bar(bins[:-1], log_counts2, width=np.diff(bins), align='edge', color='red', alpha=0.5, label=f'Filtered(Z-Score-threshold:{threshold}) Row Distance Log Frequency')
        ax2.set_ylabel("Negative Log Frequency", color='black')
        ax2.set_xlabel("Row Distance/mm")
        ax2.legend()

        for x, y in zip(bins[:-1], log_counts1):
            if y < 0: 
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=8, color='black')
        for x, y in zip(bins[:-1], log_counts2):
            if y < 0:
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='bottom', fontsize=8, color='black')

        for x in bins:
            ax1.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)
            ax2.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)

        # Add legend and grid
        ax2.legend()
        ax2.grid(axis='y', linestyle='--', alpha=0.7)
        ax1.set_xticks(bins) 
        ax1.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax1.tick_params(axis='x', labelcolor='blue', bottom=True, labelbottom=True, top=False)
        
        ax2.set_xticks(bins)  
        ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax2.tick_params(axis='x', labelcolor='green', bottom=True, labelbottom=True, top=False)

        plt.tight_layout()
        plt.savefig(f"{save_path}_row_distance_z-score_fre_his.png", dpi=300, bbox_inches='tight')
        plt.close()

        #=========colum
        bins = np.linspace(min(min(col_euclidean), min(col_filtered_data)), max(max(col_euclidean), max(col_filtered_data)), 30)
        counts1, _ = np.histogram(col_euclidean, bins=bins)
        log_counts1 = -np.log(counts1 + 1) 
        counts2, _ = np.histogram(col_filtered_data, bins=bins)
        log_counts2 = -np.log(counts2 + 1)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, gridspec_kw={"height_ratios": [3, 1]})

        bars1 = ax1.bar(bins[:-1], counts1, width=np.diff(bins), align='edge', color='blue', alpha=0.5, label='Original Column Distance Frequency')
        bars2 = ax1.bar(bins[:-1], counts2, width=np.diff(bins), align='edge', color='orange', alpha=0.5, label=f'Filtered(Z-Score-threshold:{threshold}) Column Distance Frequency')
        ax1.set_ylabel("Frequency", color='black')
        ax1.set_title("Frequency and Logarithmic Frequency of Original and Filtered(Z-Score) Column Distance")
        ax1.legend()

        for x, y in zip(bins[:-1], counts1):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='bottom', fontsize=9, color='black')
        for x, y in zip(bins[:-1], counts2):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='top', fontsize=9, color='black')

        bars3 = ax2.bar(bins[:-1], log_counts1, width=np.diff(bins), align='edge', color='green', alpha=0.5, label='Original Column Distance Log Frequency')
        bars4 = ax2.bar(bins[:-1], log_counts2, width=np.diff(bins), align='edge', color='red', alpha=0.5, label=f'Filtered(Z-Score-threshold:{threshold}) Column Distance Log Frequency')
        ax2.set_ylabel("Negative Log Frequency", color='black')
        ax2.set_xlabel("Column Distance/mm")
        ax2.legend()

        for x, y in zip(bins[:-1], log_counts1):
            if y < 0:
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=8, color='black')
        for x, y in zip(bins[:-1], log_counts2):
            if y < 0:
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='bottom', fontsize=8, color='black')

        for x in bins:
            ax1.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)
            ax2.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)

        # Add legend and grid
        ax2.legend()
        ax2.grid(axis='y', linestyle='--', alpha=0.7)
        ax1.set_xticks(bins)
        ax1.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax1.tick_params(axis='x', labelcolor='blue', bottom=True, labelbottom=True, top=False)
        
        ax2.set_xticks(bins) 
        ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax2.tick_params(axis='x', labelcolor='green', bottom=True, labelbottom=True, top=False)

        plt.tight_layout()
        plt.savefig(f"{save_path}_column_distance_z-score_fre_his.png", dpi=300, bbox_inches='tight')
        plt.close()

def visualize_radius_error(full_rows, save_path="output", cameraid=""):
    # get r1 and the position in the coordinates
    referen_radius = 10.0 # mm
    r1_values = []
    coordinates = []

    for row in full_rows:
        for (x, y), r1 in row:
            r1_values.append(r1)  # r1
            coordinates.append((x, y))  # position
    
    if False:
        # Frequency and Logarithmic Frequency
        bin_num = 20
        # start = 9.0
        # end = 12.0
        # interval = 0.05
        # print(f"interval:{interval}")
        # bins_list = np.round(np.arange(start, end + interval, interval), 2).tolist()
        # counts, bins = np.histogram(r1_values, bins=bins_list)
        counts, bins = np.histogram(r1_values, bins="fd")
        log_counts = -np.log(counts + 1)  # in case log(0)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, gridspec_kw={"height_ratios": [3, 1]})

        bars1 = ax1.bar(bins[:-1], counts, width=np.diff(bins), align='edge', color='blue', alpha=0.7, label='Frequency')
        ax1.set_ylabel("Frequency", color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.set_title("Radius Values Frequency and Logarithmic Frequency")

        for x, y in zip(bins[:-1], counts):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='bottom', fontsize=9, color='black')
        for x, y, width in zip(bins[:-1], log_counts, np.diff(bins)):
            if y < 0:
                ax2.text(x + width / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=6, color='black')

        bars2 = ax2.bar(bins[:-1], log_counts, width=np.diff(bins), align='edge', color='green', alpha=0.7, label='Negative Log Frequency')
        ax2.set_ylabel("Negative Log(Frequency+1)", color='green')
        ax2.tick_params(axis='y', labelcolor='green')
        ax2.set_xlabel("Radius Values/mm")

        for x, y in zip(bins[:-1], log_counts):
            if y > 0:
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=9, color='black')

        for x in bins:
            ax1.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)
            ax2.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)

        ax1.set_xticks(bins)
        ax1.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax1.tick_params(axis='x', labelcolor='blue', bottom=True, labelbottom=True, top=False)
        
        ax2.set_xticks(bins) 
        ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax2.tick_params(axis='x', labelcolor='green', bottom=True, labelbottom=True, top=False)

        plt.tight_layout()
        plt.savefig(f"{save_path}_radius_ori_log_fre_his.png", dpi=300, bbox_inches='tight')
        plt.close()

    # ------------------ scatter
    if False:
        r_diff = [r - referen_radius for r in r1_values]

        # log_r = np.log1p(r1_values)
        # r_diff = [r - referen_radius for r in log_r]
        # slipt coordinates as x 和 y
        x_coords, y_coords = zip(*coordinates)
        factor = 120

        plt.figure(figsize=(12, 10))
        scatter = plt.scatter(
            x_coords, y_coords,
            c=r_diff,  
            cmap="viridis",  
            s=[v * factor for v in r_diff],   
            alpha=0.8 
        )
        
        cbar = plt.colorbar(scatter)
        cbar.set_label("radius-referen_radius error/mm", fontsize=12)

        for x, y, value in zip(x_coords, y_coords, r_diff):
            plt.text(
                x, y,                      
                f"{value:.2f}",           
                fontsize=6,                
                color="black",             
                ha="center", va="center"  
            )

        
        plt.xlabel("X Pixel Coordinate", fontsize=12)
        plt.ylabel("Y Pixel Coordinate", fontsize=12)
        plt.title("radius-referen_radius error/mm", fontsize=14)

         
        plt.tight_layout()
        fig = plt.gcf()
        fig.set_size_inches(18.5, 10.5)   
        plt.savefig(f"{save_path}_radius-referen_radius_error.png", dpi=300, bbox_inches='tight')
        plt.show()
    # Z-score to detect and delect outlier
    r1_values_tmp = np.array(r1_values)
    r1_values_tmp = r1_values_tmp.flatten()
    z_scores = zscore(r1_values_tmp)
    # threshold = 3 # more than 99.7% within this are normal data
    threshold = 5
    filtered_data = r1_values_tmp[abs(z_scores) < threshold] # filter outlier
    if True:

        # Compute statistics
        r_stats = {
            "camera_id": str(cameraid),
            "mean": np.mean(r1_values),
            "median": np.median(r1_values),
            "variance": np.var(r1_values),
            "size": len(r1_values),
            # "min": np.min(r1_values),
            # "q1": np.percentile(r1_values, 25),
            # "median": np.median(r1_values),
            # "q3": np.percentile(r1_values, 75),
            # "max": np.max(r1_values),
            "zscore_mean": np.mean(filtered_data),
            "zscore_variance": np.var(filtered_data),
            "zscore_size": len(filtered_data),
        }
        print("Radius Statistics:", r_stats)
        stats_text = (
            "Radius Statistics:\n"
            f"  ID: {r_stats['camera_id']}\n"
            f"  Mean: {r_stats['mean']}\n"
            f"  Median: {r_stats['median']}\n"
            f"  Variance: {r_stats['variance']}\n"
            f"  Size: {r_stats['size']}\n"
            # f"  Min: {r_stats['min']}\n"
            # f"  Q1: {r_stats['q1']}\n"
            # f"  Median: {r_stats['median']}\n"
            # f"  Q3: {r_stats['q3']}\n"
            # f"  Max: {r_stats['max']}\n"
            f"  ZscoreMean: {r_stats['zscore_mean']}\n"
            f"  ZscoreVariance: {r_stats['zscore_variance']}\n"
            f"  ZscoreSize: {r_stats['zscore_size']}\n\n"
        )
         
        with open(f"{save_path}_radius_statistics.txt", "w") as file:
            file.write(stats_text)

    # --------------------------------------------------------
    if True:
        
        bins = np.linspace(min(min(r1_values), min(filtered_data)), max(max(r1_values), max(filtered_data)), 30)
         
        counts1, _ = np.histogram(r1_values, bins=bins)
        log_counts1 = -np.log(counts1 + 1) 
        counts2, _ = np.histogram(filtered_data, bins=bins)
        log_counts2 = -np.log(counts2 + 1)

        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True, gridspec_kw={"height_ratios": [3, 1]})

        bars1 = ax1.bar(bins[:-1], counts1, width=np.diff(bins), align='edge', color='blue', alpha=0.5, label='Original Radius Frequency')
        bars2 = ax1.bar(bins[:-1], counts2, width=np.diff(bins), align='edge', color='orange', alpha=0.5, label=f'Filtered Radius(Z-Score-threshold:{threshold}) Frequency')
        ax1.set_ylabel("Frequency", color='black')
        ax1.set_title("Frequency and Logarithmic Frequency of Original and Filtered(Z-Score) Radius/mm")
        ax1.legend()

        for x, y in zip(bins[:-1], counts1):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='bottom', fontsize=9, color='black')
        for x, y in zip(bins[:-1], counts2):
            if y > 0:
                ax1.text(x + np.diff(bins)[0] / 2, y, f"{y}", ha='center', va='top', fontsize=9, color='black')

        bars3 = ax2.bar(bins[:-1], log_counts1, width=np.diff(bins), align='edge', color='green', alpha=0.5, label='Original Radius Log Frequency')
        bars4 = ax2.bar(bins[:-1], log_counts2, width=np.diff(bins), align='edge', color='red', alpha=0.5, label=f'Filtered Radius(Z-Score-threshold:{threshold}) Log Frequency')
        ax2.set_ylabel("Negative Log Frequency", color='black')
        ax2.set_xlabel("radius/mm")
        ax2.legend()

        for x, y in zip(bins[:-1], log_counts1):
            if y < 0:   
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='top', fontsize=8, color='black')
        for x, y in zip(bins[:-1], log_counts2):
            if y < 0:   
                ax2.text(x + np.diff(bins)[0] / 2, y, f"{y:.2f}", ha='center', va='bottom', fontsize=8, color='black')

        for x in bins:
            ax1.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)
            ax2.axvline(x=x, color='gray', linestyle='--', linewidth=0.5)

        # Add legend and grid
        ax2.legend()
        ax2.grid(axis='y', linestyle='--', alpha=0.7)
        ax1.set_xticks(bins)   
        ax1.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax1.tick_params(axis='x', labelcolor='blue', bottom=True, labelbottom=True, top=False)
         
        ax2.set_xticks(bins)   
        ax2.set_xticklabels([f"{b:.1f}" for b in bins], rotation=90, ha='right', fontsize=6)
        ax2.tick_params(axis='x', labelcolor='green', bottom=True, labelbottom=True, top=False)

        plt.tight_layout()
        plt.savefig(f"{save_path}_radius_z-score_fre_his.png", dpi=300, bbox_inches='tight')
        plt.close()
