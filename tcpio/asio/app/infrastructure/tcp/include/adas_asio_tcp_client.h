#ifndef B24BA3E3_14EC_47C8_95B7_EC70F30AAA60
#define B24BA3E3_14EC_47C8_95B7_EC70F30AAA60

#include "asio.hpp"
#include "asio/steady_timer.hpp"

using asio::ip::tcp;

class AdasAsioTcpClient
{
public:
  AdasAsioTcpClient(asio::io_context& io_context, const tcp::resolver::results_type& endpoints);
  void close();
  bool IsConnected() {return isConnected_;}

private:
  void do_connect(const tcp::resolver::results_type& endpoints);
  void do_read();
  void on_read(const std::error_code & ec, size_t bytes);

private:
  bool isConnected_;
  asio::io_context& io_context_;
  tcp::socket socket_;
  enum { max_length = 1024 };
  char data_[max_length];
  asio::steady_timer deadline_;
  tcp::resolver::results_type endpoints_;
};


#endif /* B24BA3E3_14EC_47C8_95B7_EC70F30AAA60 */
