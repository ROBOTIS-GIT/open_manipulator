ar_pose Demo
============
These demos require bag files with recorded sensor data to run.
Data for the demos can be download into the workspace directory
(126MB) using the supplied script
  ./setup_demos

demo_single.launch
  Single AR Marker tracking demo
  roslaunch ar_pose demo_single.launch

demo_reverse.launch
  Reversed TF for camera tracking using fixed AR Marker
  roslaunch ar_pose demo_reverse.launch

demo_multi.launch
  Multi-marker tracking
  roslaunch ar_pose demo_multi.launch
