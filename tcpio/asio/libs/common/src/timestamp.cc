//
// Copyright 2016 Horizon Robotics.
//

#include "common/timestamp.h"

#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include <stdio.h>
#include <time.h>

#if 0 && defined(ADAS_FPGA)
#include <engine/fpga-engine/FPGAInterface.h>
#endif

// #include <hobot-adas/utils/logging/logging.h>

namespace HobotADAS {
#ifdef _WIN32
TimeStamp GetTimeStamp() {
#else
TimeStamp __attribute__((weak)) GetTimeStamp() {
#endif
#ifdef _WIN32
  // LARGE_INTEGER freq, curr_time;
  // QueryPerformanceFrequency(&freq);
  // QueryPerformanceCounter(&curr_time);
  // return curr_time.QuadPart * 1000 / freq.QuadPart;
  {
    time_t tt = time(NULL);
    SYSTEMTIME systime;
    GetLocalTime(&systime);

    struct tm *tm_time;
    tm_time = localtime(&tt);

    return tt * 1000LL + systime.wMilliseconds;
  }
#elif defined(ADAS_FPGA) && !defined(ADAS_ALTERA)
  {
    VLOG(LOG_VERBOSE_HIGHEST)
        << "should link GetTimeStamp() function in libhobot-adas-fpga.a";
  }
#elif defined(__linux__) || defined(__ANDROID__) || defined(LINUX)
{
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  return (static_cast<int64_t>(curr_time.tv_sec) * 1000LL +
          static_cast<int64_t>(curr_time.tv_usec) / 1000LL);
}
#elif defined(__MACH__)
#include <chrono>
TimeStamp microsecs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
return microsecs;
#elif defined(__QNX__)
{
  struct timespec tmv;
  clock_gettime(CLOCK_MONOTONIC, &tmv);
  return (tmv.tv_sec * 1000LL + tmv.tv_nsec / 1000000LL);
}
#else
REPORT_ERROR_POSITION;
#endif
}

#ifdef _WIN32
TimeStamp GetTimeStampUs() {
#else
TimeStamp __attribute__((weak)) GetTimeStampUs() {
#endif
#ifdef _WIN32
  // LARGE_INTEGER freq, curr_time;
  // QueryPerformanceFrequency(&freq);
  // QueryPerformanceCounter(&curr_time);
  // return curr_time.QuadPart * 1000 / freq.QuadPart;
  {
    time_t tt = time(NULL);
    SYSTEMTIME systime;
    GetLocalTime(&systime);

    struct tm *tm_time;
    tm_time = localtime(&tt);

    return tt * 1000000LL + systime.wMilliseconds;
  }
#elif defined(ADAS_FPGA) && !defined(ADAS_ALTERA)
  {
    VLOG(LOG_VERBOSE_HIGHEST)
        << "should link GetTimeStampUs() function in libhobot-adas-fpga.a";
  }
#elif defined(__linux__) || defined(__ANDROID__) || defined(LINUX)
{
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  return (static_cast<int64_t>(curr_time.tv_sec) * 1000000LL +
          static_cast<int64_t>(curr_time.tv_usec));
}
#elif defined(__MACH__)
#include <chrono>
return std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::high_resolution_clock::now().time_since_epoch())
    .count();
#elif defined(__QNX__)
{
  struct timespec tmv;
  clock_gettime(CLOCK_MONOTONIC, &tmv);
  return (tmv.tv_sec * 1000000LL + tmv.tv_nsec / 1000LL);
}
#else
REPORT_ERROR_POSITION;
#endif
}

TimeRegister::TimeRegister(const std::string info, int verbose,
                           const char *file, int line) {
  info_ = info;
  verbose_ = verbose;
  toc_ = GetTimeStamp();
  // struct timeval curr_time;
  // gettimeofday(&curr_time, NULL);
  // toc_ = curr_time.tv_sec * 1000000 + curr_time.tv_usec;
  this->line = line;
  this->file = file;
}

TimeRegister::~TimeRegister() {
#if CODE_LEVEL > LITE_CODE_LEVEL
  toc_ = GetTimeStamp() - toc_;
  // struct timeval curr_time;
  // gettimeofday(&curr_time, NULL);
  // toc_ = curr_time.tv_sec * 1000000 + curr_time.tv_usec - toc_;
  // VLOG(LOG_VERBOSE_HIGHEST) << toc_ << " " << info_;
  // char buf[128];
  // sprintf(buf, "#%06lldms: %s", toc_, info_.c_str());
  // VLOG(LOG_VERBOSE_HIGHEST) << file << " " << line << " " << buf;
#if (GOOGLE_STRIP_LOG == 0)
  char buf[1024];
#ifdef ADAS_J2
  snprintf(buf, sizeof(buf), "#%06ldms: %s", toc_, info_.c_str());
#else
  snprintf(buf, sizeof(buf), "#%06lldms: %s", toc_, info_.c_str());
#endif
  if (VLOG_IS_ON(verbose_)) {
#ifdef __ANDROID__
    ALOGI("%s\t%d\t%s", file, line, buf);
#endif
    google::LogMessage(file, line, google::GLOG_INFO).stream() << buf;
  }
  // printf("%s\n", buf);
#endif
#endif
}

#ifndef _WIN32
TimeStamp __attribute__((weak)) GetClockRawTimeStamp() {
  struct timespec ts = {0, 0};
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return (ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL);
}
#endif

}  // end of namespace HobotADAS
