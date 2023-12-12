cmake_minimum_required(VERSION 3.5)
project(ov2slam)

if(POLICY CMP0022)
  cmake_policy(SET CMP0022 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

#######
# NOTE!
#######
# Set to OFF if you did not compile OpenCV w. opencv_contrib
set(WITH_OPENCV_CONTRIB ON)

if(WITH_OPENCV_CONTRIB)
    message("\nGoing to use OpenCV contrib! (WITH_OPENCV_CONTRIB is set to : ${WITH_OPENCV_CONTRIB}.
Set it to OFF is OpenCV was not compiled with the opencv_contrib modules)\n")
    add_definitions(-DOPENCV_CONTRIB)
endif()

# iBOW-LCD
# First check if the related thridparty libs have been
# built. Only set the use of iBoW-LCD to true if built.
set(WITH_IBOW_LCD OFF)

if( EXISTS "${PROJECT_SOURCE_DIR}/thirdparty/ibow_lcd/build/liblcdetector.so" )
  add_definitions(-DIBOW_LCD)
  set(WITH_IBOW_LCD ON)
  message(STATUS "iBoW-LCD found!  Going to use Loop Closer!")
else ( )
  message(STATUS "iBoW-LCD NOT found!  Loop Closer will not be enabled!")
endif ( )

# OpenCV
find_package(OpenCV REQUIRED)

if (OpenCV_FOUND)
  message(STATUS "Found OpenCV lib : ${OpenCV_VERSION} - ${OpenCV_INCLUDE_DIRS}")
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

find_package(eigen3_cmake_module PATHS /usr/local/share/eigen3_cmake_module/cmake NO_DEFAULT_PATH)

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
find_package(pcl_ros REQUIRED)


###########
## Build ##
###########

include_directories(
  include
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ov2slam_node.cpp ${BACKWARD_ENABLE})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  image_transport
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2_ros
  pcl_ros
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
