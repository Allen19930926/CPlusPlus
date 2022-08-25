SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
PLATFORM=aarch64
PROJECT_DIR=$SCRIPT_DIR/..
CMAKE_DIR=$PROJECT_DIR/cmake
BUILD_DIR=$PROJECT_DIR/build/${PLATFORM}
rm -rf $BUILD_DIR/CMakeFiles CmakeCache.txt
mkdir -p $BUILD_DIR
cd $BUILD_DIR
cp $SCRIPT_DIR/build.properties.${PLATFORM} $BUILD_DIR/build.properties.local
cmake -DCMAKE_TOOLCHAIN_FILE=$CMAKE_DIR/aarch64.toolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$BUILD_DIR/output/ -DCMAKE_BUILD_TYPE=Debug ../..
# cmake --build . -v -j4
cmake --build . -j4
make install
