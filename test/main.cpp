//
// chat_client.cpp
// ~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2022 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include "asio.hpp"
#include <chrono>
#include "asio/steady_timer.hpp"
#include <string>
#include <functional>

using asio::ip::tcp;
using namespace std::chrono;
using namespace std::placeholders;

class chat_client
{
public:
  chat_client(asio::io_context& io_context,
      std::string ipaddr, std::string port)
    : io_context_(io_context),
      socket_(io_context),
      timer(io_context),
      ip(ipaddr), port_(port),is_connect(false)
  {
    do_connect();

    keepAlive();
  }

  void write(const std::string msg)
  {
    asio::post(io_context_,
        [this, msg]()
        {
            do_write(msg);
        });
  }

  void close()
  {
    asio::post(io_context_, [this]() { socket_.close(); });
  }

private:
  void do_connect()
  {

    tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(ip, port_);
    asio::async_connect(socket_, endpoints,
        [this](std::error_code ec, tcp::endpoint)
        {
          std::cout << "connected state: " << ec.message() << std::endl;
          if (!ec)
          {
            do_read_header();
            is_connect = true;
          }
        });
  }

  void keepAlive()
  {
    if (is_connect)
    {
        std::string msg("keep alive");
        write(msg);
    }
    timer.expires_after(std::chrono::milliseconds(1000));
    timer.async_wait(std::bind(&chat_client::keepAlive, this));
  }

  void do_read_header()
  {
    socket_.async_read_some(asio::buffer(readbuf, 512), std::bind(&chat_client::onread, this, _1, _2));
  }

  void onread(std::error_code ec, std::size_t /*length*/)
    {
        if (!ec)
        {
            std::cout << "read message: " << &readbuf[0] << std::endl;
            do_read_header();
        }
    };

  void do_write(const std::string msg)
  {

    std::cout << "write msg:" << msg;
    asio::async_write(socket_,
        asio::buffer(msg.data(),
          msg.length()+1),
        [this](std::error_code ec, std::size_t /*length*/)
        {
          if (!ec)
          {
              std::cout << " success" << std::endl;
          }
          else
          {
            is_connect = false;
            std::cout << " failed" << std::endl;
            std::cout << "write fail reason: " << ec.message() << std::endl;
            socket_.close();
            do_connect();
          }
        });
  }

private:
  asio::io_context& io_context_;
  tcp::socket socket_;
  char readbuf[512];
  char writebuf[512];
  asio::steady_timer timer;
  std::string ip;
  std::string port_;
  bool is_connect;
};

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: chat_client <host> <port>\n";
      return 1;
    }

    asio::io_context io_context;

    chat_client c(io_context, argv[1], argv[2]);

    std::thread t([&io_context](){ io_context.run(); });

    t.detach();

    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    c.close();
  }

  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}