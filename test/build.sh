#!/bin/bash
reset; clear;
SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
PROJECT_DIR=$SCRIPT_DIR
BUILD_DIR=$PROJECT_DIR/build/
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4