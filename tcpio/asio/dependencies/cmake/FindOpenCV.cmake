cmake_minimum_required(VERSION 3.5)

if (MSVC)
    set(OPENCV_LIBRARIES_POSTFIX "345")
    set(OPENCV_LIBRARIES 
                     opencv_highgui${OPENCV_LIBRARIES_POSTFIX}
                     opencv_calib3d${OPENCV_LIBRARIES_POSTFIX}
                     opencv_flann${OPENCV_LIBRARIES_POSTFIX}
                     opencv_features2d${OPENCV_LIBRARIES_POSTFIX}
                     opencv_imgcodecs${OPENCV_LIBRARIES_POSTFIX}
                     opencv_imgproc${OPENCV_LIBRARIES_POSTFIX}
                     opencv_core${OPENCV_LIBRARIES_POSTFIX}
                     )
else()
    set(OPENCV_LIBRARIES 
        opencv_highgui 
        opencv_calib3d
        opencv_flann
        opencv_features2d
        opencv_imgcodecs
        opencv_imgproc
        opencv_core
        libpng zlib
        )
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenCV DEFAULT_MSG OPENCV_LIBRARIES)
