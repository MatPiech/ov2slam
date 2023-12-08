#!/bin/sh

echo ""
echo "Building Obindex2 lib!"
echo ""

cd thirdparty/obindex2

mkdir build && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
cd ../../..

echo ""
echo "Building iBoW-LCD lib!"
echo ""

cd thirdparty/ibow_lcd

mkdir build && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j`nproc`
cd ../../..

echo ""
echo "Building Sophus lib!"
echo ""

cd thirdparty/Sophus

mkdir build install && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="../install/"
make -j`nproc` install
cd ../../..

echo ""
echo "Building Ceres lib!"
echo ""

cd thirdparty/ceres-solver

mkdir build install && cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../install/" -DBUILD_EXAMPLES=OFF
make -j`nproc` install
cd ../../..
