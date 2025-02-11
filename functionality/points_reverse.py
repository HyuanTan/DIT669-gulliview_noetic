import numpy as np
import csv
import re
import cv2

class ReversePoints:
    def __init__(self):
        self.inv_H = None
        self.rows_data = {}
        self.transformed_points = {}

    def load_csv_data(self, csv_path: str):
        """
        Read pixel points from csv file
        """
        try:
            with open(csv_path, 'r') as f:
                csv_reader = csv.reader(f)
                next(csv_reader, None)  
                
                for row in csv_reader:
                    if not row:
                        continue
                    
                    try:
                        current_row = row[0]
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
                    
                    except Exception as e:
                        print(f"Warning: Error processing row: {row}")
                        continue
                
                print(f"Successfully loaded {len(self.rows_data)} rows of data")
        
        except Exception as e:
            print(f"Error loading CSV: {str(e)}")
            raise

    def apply_inverse_homography(self, H):
        """
        Apply inverse homography matrix
        """
        self.inv_H = np.linalg.inv(H)
        
        for row, points in self.rows_data.items():
            transformed_row_points = []
            for x, y in points:
                point_h = np.array([x, y, 1])
                transformed_h = self.inv_H @ point_h
                
                transformed = transformed_h[:2] / transformed_h[2]
                transformed_row_points.append(transformed)
            
            self.transformed_points[row] = transformed_row_points

    def save_transformed_csv(self, output_path):
        """
        Store to csv file
        """
        with open(output_path, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            
            csv_writer.writerow(['Row', 'Coordinates'])
            
            for row in sorted(self.transformed_points.keys(), key=int, reverse=True):
                coords = self.transformed_points[row]
                coord_strs = [f"({x:.3f}, {y:.3f})" for x, y in coords]
                csv_writer.writerow([row] + coord_strs)

    def visualize_points(self, image_path, output_path):
        """
        visualize the transformed points in the image
        """
        
        img = cv2.imread(image_path)
        
        if img is None:
            print(f"Failed to read image: {image_path}")
            return
        
        for row, points in self.transformed_points.items():
            for x, y in points:
                px, py = int(x), int(y)
                
                if 0 <= px < img.shape[1] and 0 <= py < img.shape[0]:
                    cv2.circle(img, (px, py), 3, (0, 255, 0), -1)
        
        cv2.imwrite(output_path, img)

    def process_and_save_csv(self, input_path, output_csv_path, output_img_path, image_path, H):
        """
        
        """
        self.load_csv_data(input_path)
        self.apply_inverse_homography(H)
        self.save_transformed_csv(output_csv_path)
        self.visualize_points(image_path, output_img_path)

def main():
    # Homography matrixes from previous transformation
    H_0 = np.array([
        [2.511308666174278059e+00, -3.188707124905909573e-02, -2.249727394035198529e+02],
        [-2.668063229024094096e-01, 2.126552988677924638e+00, 5.703167058869376888e+03],
        [-1.794258762592294940e-05, -6.772662860831485073e-05, 1.000000000000000000e+00]
    ])
    H_1 = np.array([
        [2.965149761630339498e+00, 3.224686427336546091e-01, -5.684003120778002085e+02],
        [-3.861307138632132097e-02, 3.138236435760745735e+00, 1.454390761971641496e+03],
        [3.228781263940040354e-05, 8.665253021051338934e-05, 1.000000000000000000e+00]
    ])
    H_2 = np.array([
        [2.606058813931388496e+00, -9.135263635651602598e-02, -1.552901262363500052e+02],
        [-5.064540589861957209e-02, 2.312689614491223455e+00, 3.676771109204234563e+03],
        [2.518823320110797651e-06, -5.467681126595114877e-05, 1.000000000000000000e+00]
    ])

    H_3 = np.array([
        [2.802096559642733187e+00, 2.449653946648121805e-01, -4.300931561795235893e+02],
        [-4.993725418556726331e-02, 2.918462813575139148e+00, -4.344302478282036191e+02],
        [-3.963657552095839262e-06, 9.492876629333226509e-05, 1.000000000000000000e+00]
    ])



    # paths need to modify
    input_path = "../new_layout_dot/camera_3/0_encoded_rows.csv"
    result_csv_path = "../new_layout_dot/camera_3/0_encoded_rows_reversed.csv"
    result_img_path = "../new_layout_dot/camera_3/0_encoded_rows_marked.jpg"
    original_image_path = "../new_layout_dot/camera_3/0_undistore.png"

    RP = ReversePoints()
    RP.process_and_save_csv(
        input_path, 
        result_csv_path, 
        result_img_path, 
        original_image_path, 
        H_3
    )

if __name__ == "__main__":
    main()