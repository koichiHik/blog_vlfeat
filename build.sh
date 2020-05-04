#!/bin/bash

ADDITIONAL_CMAKE_MODULE_DIR=./cmake

# GLOGS
GLOG_INCLUDE_DIRS=/usr/include
GLOG_LIBRARY_DIRS=/usr/lib/x86_64-linux-gnu/
GLOG_LIBRARIES=glog

# GFLAGS
GFLAGS_INCLUDE_DIRS=/usr/include
GFLAGS_LIBRARY_DIRS=/usr/lib/x86_64-linux-gnu/
GFLAGS_LIBRARIES=gflags

# Configure Vlfeat Library.
if [ ! -e ./extern/lib/vlfeat ]; then
  mkdir -p extern/lib/
  mkdir -p extern/lib/include
fi

cd extern/vlfeat
make
cd ../../

cp -r extern/vlfeat/bin/glnxa64/libvl.so extern/lib/
cp -r extern/vlfeat/vl extern/lib/include/

# Configure Main Repository.
if [ ! -e ./build ]; then
  mkdir build
fi
cd build

cmake \
  -D ADDITIONAL_CMAKE_MODULE_DIR=${ADDITIONAL_CMAKE_MODULE_DIR} \
  -D CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
  ../

make

cd ../