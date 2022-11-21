ARGS=("$@")

reset; clear;
SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
PROJECT_DIR=${SCRIPT_DIR}/..
BUILD_DIR=${PROJECT_DIR}/build/unit_test
# rm -rf ${BUILD_DIR}
mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}
rm fusion_unit_test
cmake ${PROJECT_DIR}/unit_test  -DFT_TEST=2
make -j2
./fusion_unit_test

################## for lcov ##################
if [[ ${ARGS} =~ "cov" ]]; then
    rm -rf coverage_result
    mkdir coverage_result
    cd coverage_result
    echo "start calc branch coverage, please wait..."
    lcov --rc lcov_branch_coverage=1 -q -c -d .. -o app_cov.info
    lcov --rc lcov_branch_coverage=1 -q -e app_cov.info "*/app/*" -o app_cov.info
    lcov --rc lcov_branch_coverage=1 -q -e app_cov.info "*.cpp" -o app_cov.info
    genhtml -q --prefix='pwd' --rc lcov_branch_coverage=1 app_cov.info
    # firefox index.html
fi
