OV²SLAM can be build for the use in docker. The goal of docker build is to create a reproducible experimental environment.


## Implementation notes

- OpenCV version must match the version used in ROS. Otherwise one has to rebuild standard packages like ros-${DISTRO}-image-transport.

- OpenCV is build from sources there to enable opencv-non-free features for feature matching.


## Building

For ROS Humble:

```shell
cd ov2slam

# Building SLAM image with dependencies...
docker build . -f docker/Dockerfile -t ov2slam-humble --build-arg ROS_DISTRO=humble --build-arg ARCHITECTURE=arm64v8
```
    

## How to run 

Below are some sample commands for ROS Humble.

```shell
docker run -it --rm  ov2slam-humble bash

# inside docker container:
source /ws/devel/setup.bash; 

ros2 launch ov2slam ov2slam.xml bag:=/ws/src/ov2slam/rosbags/kitti_2011_09_26_drive_0002_synced config:=/ws/src/ov2slam/parameters_files/accurate/kitti/kitti_00-02.yaml stream_rate:=1.0 delay:=5
```

Running with X forwarding (there are security concerns, see [http://wiki.ros.org/docker/Tutorials/GUI](http://wiki.ros.org/docker/Tutorials/GUI) for details)
```shell
xhost +local:docker
docker run -it --rm --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ov2slam-humble bash
```

For Nvidia cards:
```shell
xhost +local:docker
docker run -it --rm --privileged --net=host --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -e NVIDIA_VISIBLE_DEVICES=0 ov2slam-humble bash
```
