# ECE 6460 Homework #3

The purpose of this assignment is to provide a hands-on example of processing point cloud data from a 3D LIDAR in a typical public road setting. The goal is to detect 3D objects in the vehicle's surroundings.

## Due Date
This assignment is due on Sunday, October 27th at midnight.

## Provided Files
- Initial `homework3.launch` that must be modified to complete the assignment
- Initial `homework3_node.cpp`, `Homework3.cpp` and `Homework3.hpp` which must be modified to implement the required functionality of the `homework3` node.
- Rviz display configuration `homework3.rviz` that is already set up to visualize the inputs and outputs of the `homework3` node.
- ROS bag file `homework3.bag` that the automated test runs to evaluate performance. It is automatically downloaded by the test script, but the direct link to the bag file is here:
[OneDrive Link](https://onedrive.live.com/download?resid=B7FCF91CEE77A2BE%219312&authkey=!ADXklsIuMh3WSw4).

## Requirements
Edit the template source files for the `homework3` node and the `homework3.launch` file to meet the following requirements. The breakdown of the number of grading points is shown in square brackets.

### 1. Set Up ROS Launch System [10 / 100]
- [3 / 10] Modify `homework3.launch` to run an instance of the `homework3` node.
- [4 / 10] Add a `static_transform_publisher` node instance to `homework3.launch` that defines a TF frame transform from `base_footprint` (parent frame) to `ouster` (child frame).
  - The transformation between base_footprint and ouster is defined by:
	  - Translation: `0.6` x, `0.0` y, `1.5` z
	  - Orientation: `0.00` roll, `-0.04` pitch, `-0.02` yaw
- [3 / 10] Configure ROS to use the time stamp from the bag file by setting the `use_sim_time` parameter.

### 2. Process 3D LIDAR Points [40 / 100]
- [4 / 40] Subscribe to the raw point cloud topic `/ouster/points_raw` which is of type `sensor_msgs/PointCloud2`
- [4 / 40] Transform incoming point cloud from the `ouster` frame to `base_footprint`
- [4 / 40] Apply a passthrough filter to transformed cloud that only keeps points within the following region of interest (ROI):
	- `0.0 < x < 100.0`
	- `-10.0 < y < 10.0`
	- `-2.0 < z < 3.0`
- [4 / 40] Apply a down-sampling filter to the ROI cloud with a voxel size of 0.2 m.
- [20 / 40] Remove all points from the down-sampled cloud with normal vectors less than 30 degrees from vertical.
- [4 / 40] Run the Euclidean clustering algorithm on the cloud. Use the following settings:
	- Cluster tolerance: `0.5`
	- Min cluster size: `5`
	- Max cluster size: `5000`

### 3. Improve the Efficiency of the Normal Angle Filter [10 / 100]
- [5 / 10] Implement the normal angle filter condition from the previous requirement without using a square root function inside a `for` loop.
- [5 / 10] Implement the normal angle filter condition without using trigonometric or inverse trigonometric functions inside a `for` loop.

### 4. Populate and Publish Outputs [15 / 100]
- [5 / 15] Merge each individual cluster cloud into a single point cloud, copy the result into a `sensor_msgs/PointCloud2` message and publish it on the `/merged_cluster_cloud` topic.
- [5 / 15] Populate a `avs_lecture_msgs/TrackedObject` for each cluster, add it to a `avs_lecture_msgs/TrackedObjectArray` message, and publish it on the `/objects` topic.
- [5 / 15] Publish a `geometry_msgs/PoseArray` message on the `/normals` topic that contains the normal vectors of the processed cloud.

### 5. Pass Automated Tests [25 / 100]
Open a terminal to the root folder of the ROS workspace and run `tests.bash`. Make sure the automated test script runs and reports that all tests passed. An Internet connection is required the first time the test is run because it will automatically download the test bag `homework3.bag` into the `devel` folder of the ROS workspace. After the first run, the test will work without Internet.

The test script for Homework 3 verifies the following:

- Makes sure ROS is set to use clock from bag file
- Checks merged cluster cloud topic `/merged_cluster_cloud`
  - Messages are published
  - Frame ID is `base_footprint`
  - Appropriate number of points present in each message
- Checks filtered normals array topic `/normals`
  - Messages are published
  - Frame ID is `base_footprint`
  - Appropriate number of poses present in each message
  - None of the poses are less than 30 degrees from vertical (up or down)
- Checks the object bounding box array topic `/objects`
  - Messages are published
  - Frame ID is `base_footprint`
  - Appropriate number of poses present in each message
  - None of the bounding boxes exceed a size of 6 meters in both the x and y axes

## Run the System Manually
In addition to running the automated test, it is recommended to run the system manually and view the results in Rviz like this:
- Start a `roscore`
- Start the ROS nodes: `roslaunch homework3 homework3.launch`
- Download the bag file using the link in the Provided Files section of these instructions
- Open a terminal to the folder where the bag file is stored and play it back: `rosbag play --clock homework3.bag`
