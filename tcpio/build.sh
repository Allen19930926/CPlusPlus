reset; clear;
cur_dir=$(cd "$(dirname "$0")"; pwd)
echo "current dir path is: ${cur_dir}"
rm -rf ${cur_dir}/demo
mkdir demo
cd demo
cmake ..
make -j2