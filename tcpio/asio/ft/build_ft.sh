reset; clear;
SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
rm -rf ${SCRIPT_DIR}/build
cd ${SCRIPT_DIR}
mkdir build
cd build
cmake ${SCRIPT_DIR}  -DFT_TEST=1
make -j2
./ft

################## for lcov ##################
mkdir coverage_result
cd coverage_result
lcov --rc lcov_branch_coverage=1 -q -c -d ${SCRIPT_DIR}/build -o app_cov.info
lcov --rc lcov_branch_coverage=1 -q -e app_cov.info "*/app/*" -o app_cov.info
#lcov --rc lcov_branch_coverage=1 -q -e app_cov.info "*.cpp" -o app_cov.info
genhtml -q --rc lcov_branch_coverage=1 app_cov.info
#firefox index.html