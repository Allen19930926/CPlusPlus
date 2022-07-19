
cmake_minimum_required(VERSION 3.5)

set(TURBOJPEG_LIBRARIES turbojpeg)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TurboJPEG DEFAULT_MSG TURBOJPEG_LIBRARIES)

