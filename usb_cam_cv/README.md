# Disable the camera's automatic focus feature
```
v4l2-ctl -d /dev/videoX --list-ctrls

v4l2-ctl -d /dev/videoX --list-ctrls-menus

v4l2-ctl -d /dev/videoX --set-ctrl=focus_automatic_continuous=0

v4l2-ctl -d /dev/videoX --get-ctrl=focus_automatic_continuous
```

# camera calibration
[camera_calibration](https://wiki.ros.org/camera_calibration)

[Monocular Camera Calibration with ROS](https://www.youtube.com/watch?v=yAYqt3RpT6c)

Reference:
Zhang, Zhengyou. “A Flexible New Technique for Camera Calibration.” IEEE Trans. Pattern Anal. Mach. Intell. 22 (2000): 1330-1334.



# Publish the image from camera
```
gv
zsh
export ROS_MASTER_URI=http://192.168.50.103:11311


# 2k
roslaunch usb_cam_cv gulliview_camera_publisher_2k.launch

# 4k
roslaunch usb_cam_cv gulliview_camera_publisher_4k.launch
```