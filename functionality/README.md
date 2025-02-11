### csv-coordinate-processor:
`CoordinateProcessor` has different functions, for one camera, we need
to first obtain its AprilTags' pixel points, and we provide the global coordinates of them. With the `fcalibrate_homography` function and the inputs are coordinates of pixel points and global coordinates, the homography matrix `H` is calculated and stored. We can use `H` and inverse of `H` to realize the transformation between any pixel points and global coordinates.

In this program, it also reads csv file info and obtain the transformed global coordinates for evaluation.

### points_reverse:
Since the detection of the points on the map requires to rectify the image, we apply the inverse of that transform matrix and obtain the actual pixel points of the center of dots for further evaluation.
