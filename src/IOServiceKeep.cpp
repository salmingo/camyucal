/*
 * @file ioservice_keep.cpp 封装boost::asio::io_service, 维持run()在生命周期内的有效性
 * @date 2017-01-27
 * @version 0.1
 * @author Xiaomeng Lu
  */

#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include "IOServiceKeep.h"

IOServiceKeep::IOServiceKeep() {
	work_.reset(new work(ios_));
	thrd_keep_.reset(new boost::thread(boost::bind(&io_service::run, &ios_)));
}

IOServiceKeep::~IOServiceKeep() {
	work_.reset();
	ios_.stop();
	thrd_keep_->join();
}

io_service& IOServiceKeep::get_service() {
	return ios_;
}
