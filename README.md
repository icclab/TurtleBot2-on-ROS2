# turtlebot2 on ROS2 at ICCLab

### Start the base + navigation

	ros2 launch turtlebot2_bringup icclab_tb2-2_bringup.launch.py (or tb2-1, which has a different laser scanner)
	ros2 launch turtlebot2_nav icclab_tb2_nav2.launch.py

### Start the camera streaming

	ros2 launch turtlebot2_bringup astra_icclab.launch.py

### Enable image transport compression

	ros2 run image_transport republish raw compressed --ros-args --remap in:=/camera/color/image_raw --remap out/compressed:=/camera/color/image/compressed
	ros2 run image_transport republish raw compressedDepth --ros-args --remap in:=/camera/depth/image_raw --remap out/compressedDepth:=/camera/depth/image/compressedDepth
