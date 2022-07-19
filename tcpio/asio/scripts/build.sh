#reset; clear;
#cur_dir=$(cd "$(dirname "$0")"; pwd)
#echo "current dir path is: ${cur_dir}"
#rm -rf ${cur_dir}/brownsugar
#mkdir brownsugar
#cd brownsugar
#cmake ..
#make -j4


reset; clear;
SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
PLATFORM=ubuntu
PROJECT_DIR=$SCRIPT_DIR/..
CMAKE_DIR=$PROJECT_DIR/cmake
BUILD_DIR=$PROJECT_DIR/BrownSugar/${PLATFORM}
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR
cp $SCRIPT_DIR/build.properties.${PLATFORM} $BUILD_DIR/build.properties.local
cmake -DCMAKE_INSTALL_PREFIX:PATH=$BUILD_DIR/output/ $PROJECT_DIR 
cmake --build .