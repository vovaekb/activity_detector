# activity_detector

ROS package for detecting activity in a 3D scene.
It uses Octree spatial change detector from PCL. The main idea: if a spatial change is registered in the point cloud, then it is considered as an activity. The activity is seen as completed if no change was detected during 15 seconds.

Algorithm:
- Removing NaN points from point cloud
- Detecting spatial changes in point cloud
- Removing outliers from a PointCloud

## Usage:
Run ROS master:
```
roscore
```
Run openni_launch:
```
roslaunch openni_launch openni.launch
```
Run rviz:
```
rosrun rviz rviz
```
Run activity_detector node:
```
rosrun activity_detector detector_node input:=/camera/depth/points
```
You can specify /camera/depth_registered/points topic for the argument input.
There is a set of parameters the node takes:
- fp -              frame rate
- res -             Octree resolution for the Octree change detector (side length of octree voxels)
- noise -           noise filter for the Octree change detector
- rad -             radius for RadiusOutlierRemoval noise filtering
- min_neighbors -   minimum number of neighbors for RadiusOutlierRemoval noise filtering

Add display of type PointCloud2 with topic /activity in rviz.
Here is the result of activity detection:

![ScreenShot](https://raw.github.com/vovaekb/activity_detector/master/screenshots/activity_detect_3.png)
