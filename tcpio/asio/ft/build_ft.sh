reset; clear;
cur_dir=$(cd "$(dirname "$0")"; pwd)
echo "current dir path is: ${cur_dir}"
rm -rf ${cur_dir}/fusion_ft
mkdir fusion_ft
cd fusion_ft
cmake ..
make -j2
./fusion_ft