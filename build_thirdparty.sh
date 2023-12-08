#!/bin/sh

##############
### OpenCV ###
##############
echo -e "\nBuilding OpenCV lib...\n"

cd thirdparty/opencv
mkdir build install && cd build
cmake .. -D CMAKE_BUILD_TYPE=RELEASE \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH="../../opencv_contrib/modules" \
    -D BUILD_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_IPP=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_opencv_adas=OFF \
    -D BUILD_opencv_bgsegm=OFF \
    -D BUILD_opencv_bioinspired=OFF \
    -D BUILD_opencv_ccalib=OFF \
    -D BUILD_opencv_datasets=ON \
    -D BUILD_opencv_datasettools=OFF \
    -D BUILD_opencv_face=OFF \
    -D BUILD_opencv_latentsvm=OFF \
    -D BUILD_opencv_line_descriptor=OFF \
    -D BUILD_opencv_matlab=OFF \
    -D BUILD_opencv_optflow=ON \
    -D BUILD_opencv_reg=OFF \
    -D BUILD_opencv_saliency=OFF \
    -D BUILD_opencv_surface_matching=OFF \
    -D BUILD_opencv_text=OFF \
    -D BUILD_opencv_tracking=ON \
    -D BUILD_opencv_xobjdetect=OFF \
    -D BUILD_opencv_xphoto=OFF \
    -D BUILD_opencv_stereo=OFF \
    -D BUILD_opencv_hdf=OFF \
    -D BUILD_opencv_cvv=OFF \
    -D BUILD_opencv_fuzzy=OFF \
    -D BUILD_opencv_dnn=OFF \
    -D BUILD_opencv_dnn_objdetect=OFF \
    -D BUILD_opencv_dnn_superres=OFF \
    -D BUILD_opencv_dpm=OFF \
    -D BUILD_opencv_quality=OFF \
    -D BUILD_opencv_rapid=OFF \
    -D BUILD_opencv_rgbd=OFF \
    -D BUILD_opencv_sfm=OFF \
    -D BUILD_opencv_shape=ON \
    -D BUILD_opencv_stitching=OFF \
    -D BUILD_opencv_structured_light=OFF \
    -D BUILD_opencv_alphamat=OFF \
    -D BUILD_opencv_aruco=OFF \
    -D BUILD_opencv_phase_unwrapping=OFF \
    -D BUILD_opencv_photo=OFF \
    -D BUILD_opencv_gapi=OFF \
    -D BUILD_opencv_video=ON \
    -D BUILD_opencv_ml=ONN \
    -D BUILD_opencv_python2=OFF \
    -D WITH_GSTREAMER=OFF \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D CMAKE_INSTALL_PREFIX="../install/"
make -j`nproc` install
cd ../../..

##############
### OpenGV ###
##############
echo -e "\nBuilding OpenGV lib...\n"

cd thirdparty/opengv
mkdir build && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc` && make install
cd ../../..

################
### OBIndex2 ###
################
echo -e "\nBuilding OBIndex2 lib...\n"

cd thirdparty/obindex2
mkdir build && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
cd ../../..

################
### iBoW-LCD ###
################
echo -e "\nBuilding iBoW-LCD lib...\n"

cd thirdparty/ibow_lcd
mkdir build && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
cd ../../..

##############
### Sophus ###
##############
echo -e "\nBuilding Sophus lib...\n"

cd thirdparty/Sophus
mkdir build install && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../install/"
make -j`nproc` install
cd ../../..

####################
### Ceres Solver ###
####################
echo -e "\nBuilding Ceres Solver lib...\n"

cd thirdparty/ceres-solver
mkdir build install && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../install/" -DBUILD_EXAMPLES=OFF
make -j`nproc` install
cd ../../..
