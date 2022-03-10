# Submaps planner

![This is an image](/imgs/cool_pic.png)

============== TODO ==============

Dependencies:

ompl, pcl, stuff that okvis & se need, ...

Things you may need to tune:

queue sizes inside supereightinterface & threadedslam (if youre using a slow setup, this gives you more time before everything crashes or gets stuck), change config files (stereo + depth cam) according to your setup, change scaling of depth images inside supereightinterface (e.g if you work with tum dataset, must scale by 5000 pixel values), depending on how slow your computer is you should change the gazebo simtime rate in the world file youre using.

Remember to:

module load ompl

============== ==============

This project contains submodules. After you clone it, you should run this command:

`` git submodule update --init --recursive ``

Source the ros setup:

`` source /opt/ros/noetic/setup.bash ``

Build the packages: (need the last two flags to build okvis w/o librealsense & libtorch)

`` catkin build -DCMAKE_BUILD_TYPE=Release -DHAVE_LIBREALSENSE=Off -DUSE_NN=Off ``

If you want to debug with gdb instead:

`` catkin build -DCMAKE_BUILD_TYPE=Debug -DHAVE_LIBREALSENSE=Off -DUSE_NN=Off ``

You can do both using catkin profiles:

```
catkin config --profile debug -x _debug --cmake-args -DCMAKE_BUILD_TYPE=Debug -DHAVE_LIBREALSENSE=Off -DUSE_NN=Off
catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release -DHAVE_LIBREALSENSE=Off -DUSE_NN=Off
catkin build --profile debug
catkin build --profile release
```

Now create a folder named "in_out" in the top directory of your workspace and add dbow, traj and "meshes" folder

## To run on a Gazebo simulation

Spawn the drone and run the RotorS controller:

`` roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch ``

Nice worlds you can try out: garching_kitchen, kitchen ...

Run the localization & submapping pipeline

`` roslaunch ros_submapping ros_submapping.launch ``

To publish a waypoint:

`` rosrun rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>] ``

If you want to publish a whole trajectory:

`` rosrun ros_submapping waypoint_publisher ``

To run rviz with custom config:

`` rosrun rviz rviz -d ~/catkin_ws/src/rviz_rotors/rviz/config_1.rviz ``


## To run offline, on a dataset recorded in gazebo (faster)

Launch the drone as before, and fly it around as you wish:

`` roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch ``

`` rosrun rotors_gazebo waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>] ``

Record a rosbag of the right topics (some for the pipeline, some for rviz):

`` rosbag record /tf /firefly/vi_sensor/left/image_raw /firefly/vi_sensor/right/image_raw /firefly/vi_sensor/camera_depth/depth/disparity /firefly/vi_sensor/imu -O simulation.bag ``

Launch the ros node:

`` roslaunch ros_submapping ros_submapping.launch ``

Now you can play the bag. The pipeline will start tracking & mapping:

`` rosbag play simulation.bag ``

## To run on a dataset

This project has also been tested on the uHumans2 dataset. I chose it because it's one of the few that provides both RGB-D and stereo data. Just download one of the bags (preferably one with no humans: the ones that end with _00h) following the guide [here](http://web.mit.edu/sparklab/datasets/uHumans2/). Play the bag, then launch the pipeline as usual:

`` roslaunch ros_submapping ros_submapping.launch ``

## Using the planner

Just publish a goal on the /navgoal topic like this:

`` rostopic pub -1 /navgoal geometry_msgs/Pose  '{position:  {x: 0.0, y: 0.0, z: 1.0}, orientation: {x: 0.0,y: 0.0,z: 0.0,w: 1.0}}' ``





