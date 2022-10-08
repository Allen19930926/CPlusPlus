#include "gtest/gtest.h"
#include "glog/logging.h"

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    google::ShutdownGoogleLogging();
    return ret;
}