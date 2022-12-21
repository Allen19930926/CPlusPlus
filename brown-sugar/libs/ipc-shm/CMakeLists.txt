cmake_minimum_required(VERSION 3.15)

ADD_DEFINITIONS(-DIPC_UIO_MODULE_PATH="/lib/modules/5.10.41-rt42+g98b692708f63/extra/ipc-shm-uio.ko")
ADD_DEFINITIONS(-DIPC_UIO_MODULE_NAME="ipc_shm_uio")

add_library(ipc-shm OBJECT
    src/common/ipc-queue.c
    src/common/ipc-shm.c
    src/os/ipc-os.c)
target_include_directories(ipc-shm PUBLIC include)
