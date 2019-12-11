
# 1. Installation

## A. Prerequisities

- Install ROS by following [our reference](./kimera/docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

- Install system dependencies:
```bash
sudo apt-get install python-wstool python-catkin-tools  protobuf-compiler autoconf
# Change `melodic` below for your own ROS distro
sudo apt-get install ros-melodic-cmake-modules
```

## B. Package Installation Installation

Using [catkin](http://wiki.ros.org/catkin):

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/melodic # Change `melodic` to your ROS distro
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

# Clone repo
cd  (clone anywhere you would like)
git clone https://github.mit.edu/VNAV2019-submissions/TEAM_4.git
cd ~/catkin_ws/src

# Install dependencies from rosinstall file using wstool
wstool init # Use unless wstool is already initialized
wstool merge TEAM_4/final_project/install/vnav_team_4_rosinstall_https.rosinstall

# Download and update all dependencies
wstool update
```
Finally, compile:

```bash
# Compile code
catkin build -j$(nproc)
```
# 2. Launch Files
 All necessary launch files are located under TEAM_4/final_project/launch_files
 It is necessary to copy these files to their appropriate location (Kimera/Kimera-Semantics)
 
 ## A. Kimera
 ```bash
 cd ~/catkin_ws/src/Kimera-VIO-ROS/launch/
 mkdir real_data
 cp TEAM_4/final_project/launch_files/kimera/real_data/kimera_ros_real_data_BACKEND.launch ~/catkin_ws/src/Kimera-VIO-ROS/launch/real_data/
 cp TEAM_4/final_project/launch_files/kimera/real_data/kimera_ros_real_data_RUN.launch ~/catkin_ws/src/Kimera-VIO-ROS/launch/real_data/
 
  cp TEAM_4/final_project/launch_files/kimera/simulation/kimera_ros.launch ~/catkin_ws/src/Kimera-VIO-ROS/launch/simulation/
   cp TEAM_4/final_project/launch_files/kimera/simulation/kimera_ros_dcist.launch ~/catkin_ws/src/Kimera-VIO-ROS/launch/simulation/

```
 ## B. Kimera-Semantics
 ```bash
 cd ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/
 mkdir real_data
 mkdir simulation
 
 cp TEAM_4/final_project/launch_files/kimera_semantics/real_data/kimera_semantics_real_data_cam_2_mask_rcnn.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/real_data 
 cp TEAM_4/final_project/launch_files/kimera_semantics/real_data/kimera_semantics_real_data_cam_2_segmentation_node.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/real_data
 
 cp TEAM_4/final_project/launch_files/kimera_semantics/simulation/kimera_semantics_simulation_segmentation_node.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/simulation
 cp TEAM_4/final_project/launch_files/kimera_semantics/simulation/kimera_semantics_simulation_gt_semantics.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/simulation
 
  cp TEAM_4/final_project/launch_files/kimera_semantics/simulation/kimera_semantics_dcist.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/simulation
 cp TEAM_4/final_project/launch_files/kimera_semantics/simulation/kimera_semantics_dcist_nn.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/simulation
  cp TEAM_4/final_project/launch_files/kimera_semantics/simulation/kimera_semantics_dcist_noised.launch ~/catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/simulation
```
# 3. External Semantic Segmentation Setup:
It is necessary to change the off the shelf semantic segmentation models to work within the pipeline
## A: Semantic Segmentation Node
- Open in the text editor of your choice:
```bash
~/catkin_ws/src/ros_semantic_segmenation/semantic_segmenation/nodes/segmentation_node
```
- If running python 2.7, change the first line of the file to: 
```bash
  #!/usr/bin/env python2.7
```
- In the same file, add the following prior to pub_semantic_color.publish(m) - the last line of the software:
```bash
  m.header.frame_id = "cam_2_depth_optical_frame" #if running the real_data bag
  m.header.frame_id = "husky/depth" #if running the simulation bag
```
- Also change the input topic name:
```bash
  TOPIC_IMAGE = rospy.get_param('~topic_image', '/cam_2/color/image_raw') #if running the real_data bag
  TOPIC_IMAGE = rospy.get_param('~topic_image', '/husky/camera_left/image_raw') #if running the simulation bag
```

- It is also possible to change the semantic segmentaiton rate on line 19 as well as the model uses on line 15

## B: Mask-RCNN
Similar to the semantic segmentation node above, we need to change the mask rcnn.
- Open the following file:
```bash
~/catkin_ws/src/mask_rcnn_ros/examples/example.launch
```
- Change the input topic name
```bash
<remap from="~input" to "/cam_2/color/image_raw" /> #if running the real_data bag
<remap from="~input" to "/husky/camera_left/image_raw" /> #if running the simulation bag
```
- Open the following file:
```bash
~/catkin_ws/src/mask_rcnn_ros/nodes/mask_rcnn_node
```
- Change the first line if running python 2.7:
```bash
#!/usr/bin/env python2.7
```
- In the same file, add the following prior to line 110:
```bash
  image_msg.header = result_msg.header
  image_msg.header.frame_id = "cam_2_depth_optical_frame" #if running the real_data bag
  image_msg.header.frame_id = "husky/depth" #if running the real_data bag
```

# 4. Example Usage
In order to run an example, execute each of the following in a separate terminal, and make sure to source! Note that bags are not provided; please contact the authors for access.
```bash
roscore
```
```bash
rosbag play --info --clock Team_4/final_project/bags/real_data.bag
```
```bash
roslaunch kimera_ros kimera_ros_real_data_RUN.launch
```
```bash
roslaunch kimera_ros_semantics kimera_semantics_real_data_cam_2_segmentation_node.launch
```
```bash
rosrun semantic_segmentation segmentation_node
```
```bash
rviz -d $(rospack find kimera_ros)/rviz/kimera_vio_euroc.rviz
```
```bash
rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_gt.rviz
```

# 5. Analysis in Simulation
We provide launch files to reproduce analyses of mesh quality as a function of pose estimate quality and semantic segmentation rate.

To run the groundtruth pipeline, run the groundtruth semantics launch file, play the bag, and run the rviz visualizer. Note that the DCIST bag is not provided; pleas contact the authors for access.
```bash
roslaunch kimera_ros_semantics kimera_semantics_dcist.launch
```
```bash
rosbag play Team_4/final_project/bags/dcist_bag.bag
```
```bash
rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_gt.rviz
```

To test mesh quality vs. semantic segmentation, run any semantic segmentation pipeline, then run the semantic segmentation launch file:
```bash
roslaunch kimera_ros_semantics kimera_semantics_dcist_nn.launch
```
```bash
rosbag play Team_4/final_project/bags/dcist_bag.bag
```
```bash
rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_gt.rviz
```

To test mesh quality vs. pose estimation accuracy, run the data corruption script, then run the noised data launch file:
```bash
python Team_4/final_project/scripts/generate_noisy_transform.py
```
```bash
roslaunch kimera_ros_semantics kimera_semantics_dcist_noised.launch
```
```bash
rosbag play Team_4/final_project/bags/dcist_bag.bag
```
```bash
rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_gt.rviz
```

# 6. Real-World Data Collection
For our analysis, we utilized two cameras, one to feed to Kimera-VIO for pose estimation and one to generate ground-truth poses. 
 - For the pose estimation data, we used the Intel RealSense d435i depth camera with built-in IMU: https://www.intelrealsense.com/depth-camera-d435i/
 - For the ground truth poses, we collected data with the Intel RealSense T265 tracking camera with built-in IMU: https://www.intelrealsense.com/tracking-camera-t265/

# 7. Trajectory Analysis (evo)
To quantify the quality of pose estimates, we analyzed the error in the trajectory given by the Kimera pose estimates to the trajectory created by the ground-truth poses. To do so, we used the evo tool, which can be installed by cloning the following github and following the instructions provided: https://github.com/MichaelGrupp/evo
