PLAT=("$@")
reset; clear;

if [ ! -n "$PLAT" ]; then
    PLATFORM=ubuntu
else
    PLATFORM=${PLAT[-1]}
fi
echo ${PLATFORM}

SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
PROJECT_DIR=$SCRIPT_DIR/..
BUILD_DIR=$PROJECT_DIR/build/${PLATFORM}

if [[ ${PLATFORM} =~ "arm" ]]; then
    CMAKE_DIR=$SCRIPT_DIR/cmake/aarch64.toolchain.cmake
else
    CMAKE_DIR=$SCRIPT_DIR/cmake/linux_x86.toolchain.cmake
fi

echo ${CMAKE_DIR}

# rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
cmake -DCMAKE_TOOLCHAIN_FILE=$CMAKE_DIR -DCMAKE_BUILD_TYPE=Debug  ../..
cmake --build .