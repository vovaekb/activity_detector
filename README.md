# activity_detector

ROS package for detecting activity in a 3D scene.
It uses Octree spatial change detector from PCL. The main idea: if a spatial change is registered in the point cloud, then it is considered as an activity. The activity is seen as completed if no change was detected during 15 seconds.
Usage:
```
rosrun activity_detector detector_node input:=/camera/depth/points
```
You can specify /camera/depth_registered/points topic for the argument input.
