cmake_minimum_required(VERSION 3.10)
project(ros_submapping)

# if you have trouble with some package, remember to try HINTS option in find_package or set the cmake path

# C++ standard
set(CMAKE_CXX_STANDARD 17)
# WITHOUT WE GET CONFLICTING EIGEN SETTINGS INSIDE OKVIS THAT LEAD TO ALIGNMENT ISSUES
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC") # -march=native

find_package(catkin REQUIRED COMPONENTS 
gazebo_msgs 
geometry_msgs 
mav_msgs 
roscpp 
sensor_msgs 
message_filters 
cv_bridge 
image_transport
pcl_conversions
pcl_ros
dynamic_reconfigure)

catkin_package(
  # CATKIN_DEPENDS
  #   gazebo_msgs
  #   geometry_msgs
  #   mav_msgs
  #   roscpp
  #   sensor_msgs
  #   message_filters
  #   image_transport
  #   cv_bridge
  # pcl_conversions
  # pcl_ros
  #   dynamic_reconfigure
)

# # Add ompl
# find_package(OMPL HINTS /usr/local) # ompl or OMPL?
# message(FATAL_ERROR 
# "OMPL FOUND: ${OMPL_FOUND} 
# \nOMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}
# \nOMPL_LIBRARIES: ${OMPL_LIBRARIES}
# \nOMPLAPP_LIBRARIES: ${OMPLAPP_LIBRARIES}")
# include_directories(${OMPL_INCLUDE_DIRS})
# link_directories(${OMPL_LIBRARIES})
# link_directories(${OMPLAPP_LIBRARIES})

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# Add PCL
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Boost filesystem
find_package(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Add supereight
add_subdirectory(supereight-2-srl)

# OpenCV 
find_package(OpenCV 4.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen 
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

# alias for Supereight 2
set(SUPEREIGHT_LIB "supereight")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Compile OKVIS
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(okvis2)

# Include OKVIS header files
include_directories(
  okvis2/okvis_util/include
  okvis2/okvis_kinematics/include
  okvis2/okvis_time/include
  okvis2/okvis_cv/include
  okvis2/okvis_common/include
  okvis2/okvis_ceres/include
  okvis2/okvis_timing/include
  okvis2/okvis_frontend/include 
  okvis2/okvis_multisensor_processing/include
  okvis2/okvis_apps/include
)

include_directories(include)
include_directories(
  ${CMAKE_BINARY_DIR}/include
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(main src/main.cpp src/SupereightInterface.cpp src/Publisher.cpp src/Planner.cpp)
target_link_libraries(main PRIVATE 
okvis_util okvis_kinematics okvis_time okvis_cv okvis_common okvis_ceres okvis_timing okvis_frontend okvis_multisensor_processing okvis_apps pthread ${SUPEREIGHT_LIB} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher ${catkin_EXPORTED_TARGETS})