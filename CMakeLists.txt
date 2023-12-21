cmake_minimum_required(VERSION 3.22)
project(ov2slam)

if(POLICY CMP0022)
  cmake_policy(SET CMP0022 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
#######
# NOTE!
#######
# You might need to remove the -march=native if you experience
# exceptions related to ceres or opengv.
# opengv comes natively with the march=native option while
# ceres requires to specify it at build time.
# If you experience exceptions you can either re-compile
# opengv and ov2slam without -march=native or re-compile
# ceres with it
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -march=native -g")

# iBOW-LCD
# First check if the related thridparty libs have been
# built. Only set the use of iBoW-LCD to true if built.
set(WITH_IBOW_LCD ON)
if( EXISTS "${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/build/liblcdetector.so" )
  add_definitions(-DIBOW_LCD)
  message(STATUS "iBoW-LCD found!  Going to use Loop Closer!")
else ( )
  set(WITH_IBOW_LCD OFF)
  message(STATUS "iBoW-LCD NOT found!  Loop Closer will not be enabled!")
endif ( )

# OpenCV
set(WITH_OPENCV_CONTRIB ON)
find_package(OpenCV REQUIRED)

if (OpenCV_FOUND)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_INCLUDE_DIRS}")
  if(WITH_OPENCV_CONTRIB)
    message(STATUS "Going to use OpenCV contrib!")
    add_definitions(-DOPENCV_CONTRIB)
  endif()
endif (OpenCV_FOUND)

#Eigen3
find_package(Eigen3 REQUIRED)

if (Eigen3_FOUND)
  message(STATUS "Found Eigen3 lib : ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif (Eigen3_FOUND)

#OpenGV
find_package(opengv REQUIRED)

if (opengv_FOUND)
  message(STATUS "Found Opengv lib : ${opengv_INCLUDE_DIR} - ${opengv_DIR}")
  add_definitions(-DUSE_OPENGV)
endif (opengv_FOUND)

# Ceres
find_package(Ceres REQUIRED)

if (Ceres_FOUND)
  message(STATUS "Found Ceres lib : ${CERES_VERSION} - ${Ceres_DIR}")
endif (Ceres_FOUND)

#Sophus
find_package(Sophus REQUIRED)

if (Sophus_FOUND)
  message(STATUS "Found Sophus lib : ${Sophus_VERSION} - ${Sophus_DIR}")
endif (Sophus_FOUND)

###################
## Find packages ##
###################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

###########
## Build ##
###########

## add backward
add_subdirectory(thirdparty/backward-cpp)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/ov2slam.cpp
  src/camera_visualizer.cpp
  src/visual_front_end.cpp
  src/frame.cpp
  src/slam_params.cpp
  src/camera_calibration.cpp
  src/feature_extractor.cpp
  src/feature_tracker.cpp
  src/map_manager.cpp
  src/map_point.cpp
  src/mapper.cpp
  src/multi_view_geometry.cpp
  src/ceres_parametrization.cpp
  src/optimizer.cpp
  src/estimator.cpp
  src/loop_closer.cpp
)

# Add mandatory dependencies
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    /opt/ros/humble/include/cv_bridge
    ${OpenCV_INCLUDE_DIRS}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/ceres_parametrization/
    ${PROJECT_SOURCE_DIR}/thirdparty/obindex2/lib/include/
    ${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/include/
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    Eigen3::Eigen
    Sophus::Sophus
    Ceres::ceres
    opengv
    ${OpenCV_LIBRARIES}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/build/liblcdetector.so
)

install(FILES ${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/build/liblcdetector.so DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ov2slam_node.cpp ${BACKWARD_ENABLE})
add_backward(${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  image_transport
  std_msgs
  sensor_msgs
  cv_bridge
)

## Add mandatory dependencies
target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/ceres_parametrization/
    ${PROJECT_SOURCE_DIR}/thirdparty/obindex2/lib/include/
    ${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/include/
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/build/liblcdetector.so
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
