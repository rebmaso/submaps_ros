# Submaps submaps_ros

This is a ROS navigation stack based on OKVIS 2 and Supereight 2 submaps. The implicit global map is made of multiple local occupancy submaps. The node takes in sensor measurements (IMU, stereo, depth) as ROS topics. The user can plan trajectories on the fly by sending the 3D coordinates of a goal position. The planner is RRT connect, with submaps-compatible collision function. Can easily edit the code and switch to other planners by using the same collision function.

![This is an image](/imgs/cool_pic.png)

## Build

You need to install OMPL to use the planner. Follow [this](https://ompl.kavrakilab.org/installation.html) guide.

You also need all the dependencies listed in Okvis 2 and Supereight 2:

`` sudo apt-get install git g++ cmake libeigen3-dev libopencv-dev freeglut3-dev libopenni2-dev make libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev libboost-dev libboost-filesystem-dev``

Additional stuff that my project needs:

`` sudo apt-get install ros-noetic-desktop-full ros-noetic-cv-bridge ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-image-transport libpcl-dev``

This project contains submodules (a fork of Okvis 2 and Supereight 2). After you clone it, you should run this command:

`` git submodule update --init --recursive ``

Source the ros setup:

`` source /opt/ros/noetic/setup.bash ``

Build the packages: (need the last two flags to build okvis w/o librealsense & libtorch)

`` catkin build -DCMAKE_BUILD_TYPE=Release -DHAVE_LIBREALSENSE=Off -DUSE_NN=Off ``

You can also do this using catkin profiles:

```
catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release -DHAVE_LIBREALSENSE=Off -DUSE_NN=Off
catkin build --profile release
```

Now copy and paste the "utils" folder in the outer directory of your workspace (must be 2 directories before the ros_submapping folder). it contains the DDoW vocabulary that Okvis needs, plus an empty foldr where submap meshes will be generated for visualization (old ones are deleted at each new run of the app).

## Set the config file

The app needs 2 config files to work. One is related to the IMU + stereo sensor and is needed by Okvis. The other is needed by the mapping part. The latter also contains additional stuff like the distance threshold to generate new submaps (if new keyframe + distant_enough -> generate new map) and the planner bounds. You'll find some ready configs that work with the uHumans dataset and with a Realsense D455.

## To run on the uHumans2 dataset

This app has been tested on the uHumans2 dataset. I chose it because it's one of the few that provides both RGB-D and stereo data. Just download one of the bags (preferably one with no humans: the ones that end with _00h) following the guide [here](http://web.mit.edu/sparklab/datasets/uHumans2/). Play the bag, then launch the pipeline using the default args:

`` roslaunch ros_submapping ros_submapping.launch ``

## To run on a bag that follows the ASL format topic names

Just remember to always use the correct configs for depth, stereo & IMU, and to scale the depth images so that you have float values on 32 bits and 1m depth = 1.0. Make sure to also set the planner bounds properly.
You can find a script that converts an ASL dataset into a rosbag in the utils folder. Please feel free to contact me if you need ready-to-use rosbags.

`` roslaunch ros_submapping ros_submapping.launch config_okvis:="config_realsense_D455_Tommaso.yaml" config_s8:="config_realsense_D455_depth_Tommaso.yaml" imu_topic:="/imu0" cam0_topic:="/cam0" cam1_topic:="/cam1" depth_topic:="/depth0" ``

## Using the planner

Just publish a goal on the /navgoal topic like this.
This goal is a point in the odom frame (it's the okvis world reference frame).

`` rostopic pub -r 0.5 /navgoal geometry_msgs/Point  '{x: 1.0, y: 0.0, z: 0.0}' ``

## Visualization

Starting the pipeline automatically launches Rviz wih a default config. If you open the panel, you'll find a list of extra ROS topics you might want to visualize (stereo and depth frames, Okvis trajectory, ...).
If you don't want Rviz, just launch with `` rviz:=false ``.
 

## Cool features I want to add / problems to fix

Please feel free to add ideas here.

- Smarter map generation policy: don't create new maps when re-visiting places. Maybe use the spatial hashing thing to check if there's already an old, local map we might want to update, instead of creating a new one. This is the problem of relying on OKVIS' pose graph. I also don't think this issue has been tackled before: looks like voxgraph builds new maps following a purely temporal policy, and integrates into last one.

- Implement own planner. A thing that suits submaps is building local graphs (no collision checking) as you create submaps, and then explore them together + check collision at planning time, linking them. A sort of extended PRM (Voxplan already did it with ESDF maps). Or maybe just write an A* (not in OMPL, might be faster) that uses my submaps-enabled collision function.

- Add some pose-graph-based path verification to avoid cutting through walls when drift distorts map and creates artificial free space.

- Use GPS-enabled OKVIS 2?

- Migrate to ROS 2.







