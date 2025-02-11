import numpy as np
import pandas as pd
import cv2
import apriltag
from typing import List, Tuple, Dict
import re
import csv
import tools

class CoordinateProcessor:
    def __init__(self):
        self.H = None
        self.rows_data = {}
        
    def detect_apriltags(self, image_path: str) -> Tuple[np.ndarray, List[int]]:
        """
        Detect AprilTags and obtain pixel points
        """
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Failed to load image: {image_path}")
            
        options = apriltag.DetectorOptions(families="tag25h9")
        detector = apriltag.Detector(options)
        
        detections = detector.detect(image)
        if not detections:
            raise ValueError("No AprilTags detected in the image")
            
        pixel_points = []
        tag_ids = []
        
        for detection in detections:
            pixel_points.append([detection.center[0], detection.center[1]])
            tag_ids.append(detection.tag_id)
                
        return np.array(pixel_points), tag_ids
        
    def load_csv_data(self, csv_path: str):
        """
        Read pixel points from csv file
        """
        try:
            with open(csv_path, 'r') as f:
                csv_reader = csv.reader(f)
                next(csv_reader)  
                
                for row in csv_reader:
                    if not row:
                        continue
                    try:
                        current_row = int(row[0])
                        coordinates = []
                        
                        for coord_str in row[1:]:
                            if not coord_str.strip():
                                continue
                            coord_str = coord_str.strip('"')
                            match = re.match(r'\(([^,]+),\s*([^)]+)\)', coord_str)
                            if match:
                                x = float(match.group(1))
                                y = float(match.group(2))
                                coordinates.append((x, y))
                        
                        if coordinates:
                            self.rows_data[current_row] = coordinates
                            
                    except ValueError as e:
                        print(f"Warning: Error processing row:")
                        continue
                        
            print(f"Successfully loaded {len(self.rows_data)} rows of data")
            
        except Exception as e:
            print(f"Error loading CSV: {str(e)}")
            raise
            
    def calibrate_homography(self, pixel_points: np.ndarray, world_points: np.ndarray):
        """
        obtain homography matrix
        """
        self.H, _ = cv2.findHomography(pixel_points, world_points)
        
    def transform_coordinates(self) -> Tuple[pd.DataFrame, List[List[Tuple]]]:
        """
        Transfrom the coordinates
        """
        if self.H is None:
            raise ValueError("Homography matrix not calibrated")
            
        all_points = []
        evaluation_data = []  
        processed_rows = 0
        
        for row_num in sorted(self.rows_data.keys()):
            row_coords = []  
            
            for x, y in self.rows_data[row_num]:
                world_coord = self.pixel_to_world((x, y))
                world_x = float(world_coord[0])
                world_y = float(world_coord[1])
                
                all_points.append({
                    'row': row_num,
                    'pixel_x': x,
                    'pixel_y': y,
                    'world_x': world_x,
                    'world_y': world_y
                })
                
                row_coords.append(((world_x*1000, world_y*1000), 1.0))
            
            evaluation_data.append(row_coords)
            
            processed_rows += 1
            if processed_rows % 10 == 0:
                print(f"Processed {processed_rows} rows...")
        
        return pd.DataFrame(all_points), evaluation_data
    
    def pixel_to_world(self, pixel_point: Tuple[float, float]) -> np.ndarray:
        """
        Transform pixel points to global coordinates
        """
        pixel_h = np.array([pixel_point[0], pixel_point[1], 1.0])
        world_h = np.dot(self.H, pixel_h)
        world_h /= world_h[2]
        return world_h[:2]
        
    def process_and_evaluate(self, 
                           output_csv: str = 'transformed_coordinates.csv',
                           evaluation_path: str = 'evaluation_results') -> Dict:
        """
        evaluate the result
        """
        df, evaluation_data = self.transform_coordinates()
        df.to_csv(output_csv, index=False)
        print(f"Results saved to {output_csv}")
        
        tools.visualize_error(evaluation_data, evaluation_path)
        return df, evaluation_data
        
    def calculate_statistics(self, df: pd.DataFrame) -> Dict:
        """
        
        """
        row_distances = []
        
        for row_num in df['row'].unique():
            row_data = df[df['row'] == row_num].sort_values('pixel_x')
            for i in range(len(row_data) - 1):
                dx = row_data['world_x'].iloc[i+1] - row_data['world_x'].iloc[i]
                dy = row_data['world_y'].iloc[i+1] - row_data['world_y'].iloc[i]
                dist = np.sqrt(dx**2 + dy**2)
                row_distances.append(dist)
                
        return {
            'row_stats': {
                'mean': np.mean(row_distances),
                'std': np.std(row_distances),
                'max': np.max(row_distances),
                'min': np.min(row_distances)
            },
            'total_points': len(df)
        } 
    
if __name__ == "__main__":
    processor = CoordinateProcessor()
    # Change the path here
    pixel_points, tag_ids = processor.detect_apriltags("../new_layout_dot/camera_3/0_undistore.png")
    
    # world coordinates for the tags
    world_points = np.array([
        [0, 0],     # Tag 0
        [4.28, 0],  # Tag 1
        [0, 2],     # Tag 2
        [4.28, 2],  # Tag 3
        # [0, 4],     # Tag 4
        # [4.28, 4],  # Tag 5
        # [0, 6],     # Tag 6
        # [4.28, 6],  # Tag 7
        # [0, 8],     # Tag 8
        # [4.28, 8],  # Tag 9
    ], dtype=np.float32)
    
    # Change the path here
    processor.load_csv_data("../new_layout_dot/camera_3/0_encoded_rows_reversed.csv")
    
    processor.calibrate_homography(pixel_points, world_points)
    df, evaluation_data = processor.process_and_evaluate(
        output_csv='transformed_coordinates.csv',
        evaluation_path='evaluation_results'
    )
    