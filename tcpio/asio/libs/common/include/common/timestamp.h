//
// Copyright 2016 Horizon Robotics.
//

#ifndef SRC_COMMON_TIMESTAMP_H_
#define SRC_COMMON_TIMESTAMP_H_

#include <string>

namespace HobotADAS {

typedef int64_t TimeStamp;
using TimeStampUs = int64_t;  // time stamp in microsecond

// get time stamp, in millisecond
TimeStamp GetTimeStamp();
TimeStamp GetClockRawTimeStamp();
// get time stamp, in microsecond
TimeStamp GetTimeStampUs();

class TimeRegister {
 protected:
  TimeStamp toc_;
  std::string info_;
  const char* file;
  int line;
  int verbose_;

 public:
  TimeRegister(const std::string info, int verbose, const char* file, int line);
  ~TimeRegister();
};

#define TIME_REGISTER_VERBOSE(info, verbose) \
  TimeRegister timer(info, verbose, __FILE__, __LINE__)
#define TIME_REGISTER(info) TIME_REGISTER_VERBOSE(info, LOG_VERBOSE_LOW)

}  // end of namespace HobotADAS

#endif  // SRC_COMMON_TIMESTAMP_H_
