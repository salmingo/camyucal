/*!
 * @file udp_asio.cpp 基于boost::asio封装UDP通信
 * @version 0.1
 * @date May 23, 2017
 */
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <string.h>
#include <string>
#include "udpasio.h"

using std::string;
using namespace boost::asio;

//////////////////////////////////////////////////////////////////////////////
UdpPtr makeudp_session(uint16_t port) {
	return boost::make_shared<UDPSession>(port);
}

UDPSession::UDPSession(const uint16_t portLoc) {
	bufrcv_.reset(new char[UDP_PACK_SIZE]);
	bytercv_ = 0;
	connected_ = false;
	if (!portLoc) sock_.reset(new udp::socket(keep_.get_service(), udp::v4()));
	else {
		sock_.reset(new udp::socket(keep_.get_service(), udp::endpoint(udp::v4(), portLoc)));
		start_read();
	}
}

UDPSession::~UDPSession() {
	Close();
}

void UDPSession::Connect(const char *ip, const uint16_t port) {
	udp::resolver resolver(keep_.get_service());
	udp::resolver::query query(ip, boost::lexical_cast<string>(port));
	udp::endpoint remote = *(resolver.resolve(query));
	sock_->async_connect(remote, boost::bind(&UDPSession::handle_connect, this, placeholders::error));
}

void UDPSession::Close() {
	if (sock_.unique()) {
		if (sock_->is_open()) sock_->close();
		sock_.reset();
		connected_ = false;
	}
}

bool UDPSession::IsOpen() {
	return (sock_.unique() && sock_->is_open());
}

void UDPSession::RegisterConnect(const CBSlot &slot) {
	if (!cbconn_.empty()) cbconn_.disconnect_all_slots();
	cbconn_.connect(slot);
}

void UDPSession::RegisterRead(const CBSlot &slot) {
	mutex_lock lck(mtxrcv_);
	if (!cbrcv_.empty()) cbrcv_.disconnect_all_slots();
	cbrcv_.connect(slot);
}

void UDPSession::RegisterWrite(const CBSlot &slot) {
	mutex_lock lck(mtxsnd_);
	if (!cbsnd_.empty()) cbsnd_.disconnect_all_slots();
	cbsnd_.connect(slot);
}

const char *UDPSession::Read(int &n) {
	mutex_lock lck(mtxrcv_);
	n = bytercv_;
	return n == 0 ? NULL : bufrcv_.get();
}

const char *UDPSession::BlockRead(int &n) {
	mutex_lock lck(mtxrcv_);
	boost::posix_time::milliseconds t(500);

	cvread_.timed_wait(lck, t);
	n = bytercv_;
	return n == 0 ? NULL : bufrcv_.get();
}

void UDPSession::Write(const void *data, const int n) {
	mutex_lock lck(mtxsnd_);

	if (connected_) {
		sock_->async_send(buffer(data, n),
				boost::bind(&UDPSession::handle_write, this,
						placeholders::error, placeholders::bytes_transferred));
	}
	else {
		sock_->async_send_to(buffer(data, n), remote_,
				boost::bind(&UDPSession::handle_write, this,
						placeholders::error, placeholders::bytes_transferred));
	}
}

void UDPSession::handle_connect(const error_code& ec) {
	connected_ = !ec;
	if (!cbconn_.empty()) cbconn_((const long) this, ec.value());
	if (!ec) start_read();
}

void UDPSession::handle_read(const error_code& ec, const int n) {
	if (!ec || ec == error::message_size) {
		bytercv_ = n;
		cvread_.notify_one();
		if (!cbrcv_.empty()) cbrcv_((const long) this, n);
		start_read();
	}
}

void UDPSession::handle_write(const error_code& ec, const int n) {
	if (!ec && !cbsnd_.empty()) cbsnd_((const long) this, 0);
}

void UDPSession::start_read() {
	if (connected_) {
		sock_->async_receive(buffer(bufrcv_.get(), UDP_PACK_SIZE),
				boost::bind(&UDPSession::handle_read, this,
						placeholders::error, placeholders::bytes_transferred));
	}
	else {
		sock_->async_receive_from(buffer(bufrcv_.get(), UDP_PACK_SIZE), remote_,
				boost::bind(&UDPSession::handle_read, this,
						placeholders::error, placeholders::bytes_transferred));
	}
}
