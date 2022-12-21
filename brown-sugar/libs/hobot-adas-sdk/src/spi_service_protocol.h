// Copyright 2022 Horizon Robotics.
#pragma once
#include <dirent.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "zmq.h"
class Pub {
 public:
  Pub(std::string filepath) {
    if (filepath == "") {
      std::cout << "no zmq ipc file input" << std::endl;
    }
    file_path = filepath;
    if ((context = zmq_ctx_new()) == NULL) {
      std::cout << "pub context init failed" << std::endl;
    }
    InitPub();
  }
  ~Pub() {
    zmq_close(puber);
    zmq_ctx_destroy(context);
  }
  int InitPub() {
    if (access(file_path.c_str(), F_OK) == -1) {
      if (creat(file_path.c_str(), 0755) < 0) {
        std::cout << "create file failed" << std::endl;
        return -1;
      }
    }
    std::string addr = "ipc://" + file_path;
    if ((puber = zmq_socket(context, ZMQ_PUB)) == NULL) {
      std::cout << " pub socket init failed" << std::endl;
      return -1;
    }
    int hwm = 100;
    int rc = zmq_setsockopt(puber, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    if (rc < 0) {
      std::cout << "set sndhwm failed" << std::endl;
      return -1;
    }

    int linger = 1000;
    rc = zmq_setsockopt(puber, ZMQ_LINGER, &linger, sizeof(linger));
    if (rc < 0) {
      std::cout << "set linger failed" << std::endl;
      return -1;
    }

    int sndbuf = 16 * 1024;
    rc = zmq_setsockopt(puber, ZMQ_SNDBUF, &sndbuf, sizeof(sndbuf));
    if (rc < 0) {
      std::cout << "set sndbuf failed" << std::endl;
      return -1;
    }

    if (zmq_bind(puber, addr.c_str()) < 0) {
      std::cout << "pub bind failed: " << zmq_strerror(errno) << std::endl;
      return -1;
    }

    pub_inite = true;
    usleep(150000);
    return 0;
  }

  std::string file_path;
  bool pub_inite = false;
  void *context;
  void *puber;
};

class PubCANInput: public Pub {
 public:
  PubCANInput(std::string filepath):Pub(filepath) {}
  ~PubCANInput(void) {}
    int IpcCANInputPub(uint8_t *header_data, int header_length, uint8_t *frame_data, int frame_length) {
    if (!pub_inite) {
      if (InitPub() == -1) {
        std::cout << "pub init failed!" << std::endl;
        return 0;
      }
    }

    zmq_msg_t msg;
    int rc = 0;
    rc = zmq_msg_init_size(&msg, frame_length + header_length);
    if (rc != 0) {
      return -1;
    }
    memcpy(zmq_msg_data(&msg), header_data, header_length);
    memcpy(zmq_msg_data(&msg) + header_length, frame_data, frame_length);
    // std::cout << "send message size is: " << sizeof(long) *
    // zmq_msg_size(&msg) <<std::endl;
    if (zmq_msg_send(&msg, puber, 0) < 0) {
      std::cout << "send message to J2 failed" << __FUNCTION__;
    }
    zmq_msg_close(&msg);
    return 0;
  }

};
