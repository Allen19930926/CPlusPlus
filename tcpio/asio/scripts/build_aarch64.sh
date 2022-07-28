#!/bin/bash
SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
PLATFORM=aarch64
PROJECT_DIR=$SCRIPT_DIR/..
CMAKE_DIR=$PROJECT_DIR/cmake
BUILD_DIR=$PROJECT_DIR/BrownSugar/${PLATFORM}
mkdir -p $BUILD_DIR
cd $BUILD_DIR
cp $SCRIPT_DIR/build.properties.${PLATFORM} $BUILD_DIR/build.properties.local
cmake -DCMAKE_TOOLCHAIN_FILE=$CMAKE_DIR/aarch64.toolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$BUILD_DIR/output/ -DCMAKE_BUILD_TYPE=Debug $PROJECT_DIR
cmake --build . -v
make install