reset; clear;
cur_dir=$(cd "$(dirname "$0")"; pwd)
echo "current dir path is: ${cur_dir}"
rm -rf ${cur_dir}/brownsugar
mkdir brownsugar
cd brownsugar
cmake ..
make -j4