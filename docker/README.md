OV²SLAM can be build for the use in docker. The goal of docker build is to create a reproducible experimental environment.


## Implementation notes

- OpenCV version must match the version used in ROS. Otherwise one has to rebuild standard packages like ros-${DISTRO}-image-transport.

- OpenCV is build from sources there to enable opencv-non-free features for feature matching.


## Building

For ROS Noetic:

```shell
cd ov2slam

# Building SLAM image with dependencies...
docker build . -f docker/Dockerfile -t ov2slam-noetic --build-arg ROS_DISTRO=noetic --build-arg OPENCV_VERSION=4.2.0
```
    

## How to run 

Below are some sample commands for ROS Noetic.

```shell
docker run -it --rm  ov2slam-noetic bash

# inside docker container:
source /ws/devel/setup.bash; 

roslaunch ov2slam ov2slam.launch bag:=/ws/src/ov2slam/kitti_2011_09_26_drive_0002_synced.bag config:=/ws/src/ov2slam/parameters_files/accurate/kitti/kitti_00-02.yaml stream_rate:=1.0 delay:=5
```

Running with X forwarding (there are security concerns, see [http://wiki.ros.org/docker/Tutorials/GUI](http://wiki.ros.org/docker/Tutorials/GUI) for details)
```shell
xhost +local:docker
docker run -it --rm --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ov2slam-noetic bash
```

For Nvidia cards:
```shell
xhost +local:docker
docker run -it --rm --privileged --net=host --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 ov2slam-noetic bash
```
