# Submaps planner

![This is an image](/imgs/cool_pic.png)


Dependencies:

You need to install OMPL to use the planner. follow [this](https://ompl.kavrakilab.org/installation.html) guide.

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

## To run on a Gazebo simulation

Spawn the drone and run the RotorS controller (do this before you launch the processing pipeline! early sensor data are always crappy in Gazebo: best to discard the very first).
You can find that this launch also publishes a fixed odom frame with the start location of the robot, so that we can visualize state estimate and maps correctly.

`` roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch <world_name:=garching_kitchen> ``

Worlds that have been tested: garching_kitchen, maze.

Run the localization & submapping pipeline

`` roslaunch ros_submapping ros_submapping.launch config_okvis:="config_visensor_Tommaso.yaml" config_s8:="config_visensor_depth_Tommaso.yaml" imu_topic:="/firefly/vi_sensor/imu" cam0_topic:="/firefly/vi_sensor/left/image_raw" cam1_topic:="/firefly/vi_sensor/right/image_raw" depth_topic:="/firefly/vi_sensor/camera_depth/depth/disparity" ``

To publish a waypoint:

`` rosrun rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>] ``

If you want to publish a whole trajectory:

`` rosrun ros_submapping waypoint_publisher ``


## To record a bag from Gazebo

If you want to test the pipeline on a simulation, but don't have much compute power, you can run it on a gazebo bag.
Launch the drone as before, and fly it around as you wish:

`` roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch ``

`` rosrun rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>] ``

Record a rosbag of the right topics (some for the pipeline, some for rviz):

`` rosbag record /tf /firefly/vi_sensor/left/image_raw /firefly/vi_sensor/right/image_raw /firefly/vi_sensor/camera_depth/depth/disparity /firefly/vi_sensor/imu -O simulation.bag ``

Launch the ros node:

`` roslaunch ros_submapping ros_submapping.launch ``

Now you can play the bag. The pipeline will start tracking & mapping:

`` rosbag play simulation.bag ``

## To run on the uHumans2 dataset

This project has also been tested on the uHumans2 dataset. I chose it because it's one of the few that provides both RGB-D and stereo data. Just download one of the bags (preferably one with no humans: the ones that end with _00h) following the guide [here](http://web.mit.edu/sparklab/datasets/uHumans2/). Play the bag, then launch the pipeline using the default args:

`` roslaunch ros_submapping ros_submapping.launch ``

## To run on a bag that follows the ASL format topic names

Just remember to always use the correct configs for depth, stereo & IMU, and to scale the depth images so that you have float values on 32 bits and 1m depth = 1.0.

`` roslaunch ros_submapping ros_submapping.launch config_okvis:="config_realsense_D455_Tommaso.yaml" config_s8:="config_realsense_D455_depth_Tommaso.yaml" imu_topic:="/imu0" cam0_topic:="/cam0" cam1_topic:="/cam1" depth_topic:="/depth0" ``

## Using the planner

Just publish a goal on the /navgoal topic like this.
This goal is a point in the odom frame (origin is where Okvis starts).

`` rostopic pub -r 0.5 /navgoal geometry_msgs/Point  '{x: 1.0, y: 0.0, z: 0.0}' ``

## Cool features you might want to add / problems to fix

Please feel free to add ideas here.

- Smarter map generation policy: don't create new maps when re-visiting places. Maybe use the spatial hashing thing to check if there's already an old, local map we might want to update, instead of creating a new one. This is the problem of relying on OKVIS' pose graph.

- Implement own planner. A thing that suits submaps is building local graphs as you create submaps, and then explore them with Dijkstra at planning time, linking them. A sort of extended PRM (Voxplan already did it with ESDF maps). Or maybe just write an A* (not in OMPL) that uses my submaps-enabled collision function.

- Use GPS-enabled OKVIS 2?

- Migrate to ROS 2.







