#ifndef COM_TCP_SERVER_H_
#define COM_TCP_SERVER_H_

#include <atomic>
#include <deque>

#include "asio/thread.hpp"
#include "com_tcp_message.h"
#include "asio.hpp"
#include "event_msg.h"
#include "event_queue.h"
#include <iostream>
#include <vector>
#include <glog/logging.h>

using asio::ip::tcp;

class ComTcpSession : public std::enable_shared_from_this<ComTcpSession> {
public:
	ComTcpSession(tcp::socket socket) : socket_(std::move(socket)), write_in_progress(false) {}

	void Start() {
		doReadHeader();
	}

	void Write(const ComTcpMessage& msg) {
		if (writeMsgs.size() >= 50) {
			LOG(WARNING) << "com tcp session: discard message";
			return;
		}
		if (!write_in_progress) {
			writeMsgs.emplace_back(msg);
			doWrite();
		}
	}

private:
	void doReadHeader() {
		auto self(shared_from_this());
		asio::async_read(socket_, asio::buffer(readBuffer, ComTcpMessage::HeaderLength),
			[this, self](std::error_code ec, std::size_t /*length*/) {
			if (!ec && ComTcpMessage::DecodeHeader(readBuffer, &readFrameBodyLen, &readFrameType)) {
				doReadBody();
			}
			else {
			}
		});
	}

	void doReadBody() {
		auto self(shared_from_this());
		asio::async_read(socket_, asio::buffer(readBuffer + ComTcpMessage::HeaderLength, readFrameBodyLen),
			[this, self](std::error_code ec, std::size_t /*length*/) {
			if (!ec) {
				// for (int i = 0; i < readMsg.BodyLength(); i++) 
    			// 	std::cout << std::hex << std::setfill('0') << std::setw(2) << readMsg.Body()[i] << " ";
				CDD_FUSION_EVENT_QUEUE.push({ readFrameType, readBuffer + ComTcpMessage::HeaderLength, static_cast<uint16_t>(readFrameBodyLen) });
				doReadHeader();
			}
			else {
			}
		});
	}

	void doWrite() {
		auto self(shared_from_this());
		if (writeMsgs.empty()) {
			write_in_progress = false;
			return;
		}
		write_in_progress = true;
		std::vector<char> data;
		while (!writeMsgs.empty()) {
			auto msg = writeMsgs.front();
			data.insert(data.end(), msg.Data(), msg.Data() + msg.Length());
			writeMsgs.pop_front();
		}
		asio::async_write(socket_, asio::buffer(data, data.size()),
			[this, self](std::error_code ec, std::size_t /*length*/) {
			write_in_progress = false;
			if (ec) {
				LOG(ERROR) << "com error: " << ec.message();
			}
		});
	}
	tcp::socket socket_;
	std::deque<ComTcpMessage> writeMsgs;
	int readFrameBodyLen;
	MsgType readFrameType;
	char readBuffer[4096*10];
	std::atomic_bool write_in_progress;

};

class ComTcpServer {
public:
	ComTcpServer(asio::io_context& io_context, short port)
		: acceptor_(io_context, tcp::endpoint(tcp::v4(), port)) {}

	void Start() {
		do_accept();
	}

	std::shared_ptr<ComTcpSession> GetSession() { return session_; }

	void Write(const ComTcpMessage & msg) {
		if (!session_) {
			return;
		}
		session_->Write(msg);
	}

private:
	void do_accept() {
		acceptor_.async_accept([this](std::error_code ec, tcp::socket socket) {
			if (!ec) {
				session_ = std::make_shared<ComTcpSession>(std::move(socket));
				session_->Start();
			}
			else {
				/* only one session */
			}
			do_accept();
		});
	}
	tcp::acceptor acceptor_;
	std::shared_ptr<ComTcpSession> session_ = NULL;
};

#endif
