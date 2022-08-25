#ifndef COM_TCP_SERVER_H_
#define COM_TCP_SERVER_H_

#include <deque>

#include "com_tcp_message.h"
#include "asio.hpp"
#include "event_queue.h"
#include <iostream>

using asio::ip::tcp;

class ComTcpSession : public std::enable_shared_from_this<ComTcpSession> {
public:
	ComTcpSession(tcp::socket socket) : socket_(std::move(socket)) {}

	void Start() {
		doReadHeader();
	}

	void Write(const ComTcpMessage& msg) {
		bool write_in_progress = !writeMsgs.empty();
		writeMsgs.push_back(msg);
		if (!write_in_progress) {
			doWrite();
		}
	}

private:
	void doReadHeader() {
		auto self(shared_from_this());
		asio::async_read(socket_, asio::buffer(readMsg.Data(), ComTcpMessage::HeaderLength),
			[this, self](std::error_code ec, std::size_t /*length*/) {
			if (!ec && readMsg.DecodeHeader()) {
				doReadBody();
			}
			else {
			}
		});
	}

	void doReadBody() {
		auto self(shared_from_this());
		asio::async_read(socket_, asio::buffer(readMsg.Body(), readMsg.BodyLength()),
			[this, self](std::error_code ec, std::size_t /*length*/) {
			if (!ec) {
				// for (int i = 0; i < readMsg.BodyLength(); i++) 
    			// 	std::cout << std::hex << std::setfill('0') << std::setw(2) << readMsg.Body()[i] << " ";
				CDD_FUSION_EVENT_QUEUE.push({ readMsg.FrameType, readMsg.Body(), static_cast<uint16_t>(readMsg.BodyLength()) });
				doReadHeader();
			}
			else {
			}
		});
	}

	void doWrite() {
		auto self(shared_from_this());
		asio::async_write(socket_, asio::buffer(writeMsgs.front().Data(), writeMsgs.front().Length()),
			[this, self](std::error_code ec, std::size_t /*length*/) {
			if (!ec) {
				writeMsgs.pop_front();
				if (!writeMsgs.empty()) {
					doWrite();
				}
			}
			else {
			}
		});
	}
	tcp::socket socket_;
	std::deque<ComTcpMessage> writeMsgs;
	ComTcpMessage readMsg;

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
