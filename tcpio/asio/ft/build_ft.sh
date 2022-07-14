reset; clear;
cur_dir=$(cd "$(dirname "$0")"; pwd)
echo "current dir path is: ${cur_dir}"
rm -rf ${cur_dir}/build
mkdir build
cd build
cmake ..  -DENABLE_GCOV=1
make -j2
./ft

################## for lcov ##################
mkdir coverage_result
cd coverage_result
lcov --rc lcov_branch_coverage=1 -q -c -d ${cur_dir}/build -o app_cov.info
lcov --rc lcov_branch_coverage=1 -e app_cov.info "*/app/*" -o app_cov.info
genhtml --rc lcov_branch_coverage=1 app_cov.info
firefox index.html