/*
 * @file CameraGYCCD.cpp GangYu相机封装接口定义文件
 * @version 0.2
 * @date 2017-12-20
 **/
#include <math.h>
#include <netinet/in.h>
#include "CameraGYCCD.h"

CameraGYCCD::CameraGYCCD(const char *camIP) {
	camip_ = camIP;
	gain_  = 0xFFFF;
	shtr_  = 0xFFFF;
	expdur_= 0xFFFF;
	idmsg_ = 0;

	byteimg_ = 0;
	bytercvd_ = 0;
	packtot_ = 0;
	packlen_ = 0;
	headlen_ = 0;
	idfrm_ = 0;
	idpack_ = 0;
}

CameraGYCCD::~CameraGYCCD() {
}

bool CameraGYCCD::Reboot() {
	try {
		write(0x20054, 0x12AB3C4D);
		nfcam_->errcode = CAMERR_SUCCESS;
		return true;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->errcode = CAMERR_REBOOT;
		return false;
	}
}

bool CameraGYCCD::connect() {
	try {

		return true;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->errcode = CAMERR_CONNECT;
		return false;
	}
}

void CameraGYCCD::disconnect() {
	try {

	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->errcode = CAMERR_DISCONNECT;
	}
}

bool CameraGYCCD::start_expose(double duration, bool light) {
	try {

		nfcam_->errcode = CAMERR_SUCCESS;
		return true;
	}
	catch(std::runtime_error &ex) {

		return false;
	}
}

void CameraGYCCD::stop_expose() {
	try {

		nfcam_->errcode = CAMERR_SUCCESS;
	}
	catch(std::runtime_error &ex) {

	}
}

int CameraGYCCD::check_state() {
	return 0;
}

int CameraGYCCD::readout_image() {
	return 0;
}

double CameraGYCCD::sensor_temperature() {
	return nfcam_->coolget;
}

void CameraGYCCD::update_cooler(double &coolset, bool onoff) {

}

uint32_t CameraGYCCD::update_readport(uint32_t index) {
	try {

		nfcam_->errcode = CAMERR_SUCCESS;
		return index;
	}
	catch(std::runtime_error &ex) {

		return 0;
	}
}

uint32_t CameraGYCCD::update_readrate(uint32_t index) {
	try {

		nfcam_->errcode = CAMERR_SUCCESS;
		return index;
	}
	catch(std::runtime_error &ex) {

		return 0;
	}
}

uint32_t CameraGYCCD::update_gain(uint32_t index) {
	try {

		nfcam_->errcode = CAMERR_SUCCESS;
		return index;
	}
	catch(std::runtime_error &ex) {

		return 0;
	}
}

void CameraGYCCD::update_roi(int &xstart, int &ystart, int &width, int &height, int &xbin, int &ybin) {

}

int CameraGYCCD::update_adcoffset(uint16_t offset, FILE *output) {
	/* 1 - 读取当前设置 */

	/* 2 - 采集本底, 统计overscan区背景 */

	/* 3 - 计算期望与实际偏差, 若小于阈值(10)则结束 */

	/* 4 - 各通道RGB各加10, 采集本底, 并统计背景, 计算增益 */

	/* 5 - 计算期望与实际偏差, 设置各通道RGB偏置 */

	return 0;
}

void CameraGYCCD::read(uint32_t addr, uint32_t &value) {
	mutex_lock lck(mtxreg_);
	boost::array<uint8_t, 16> towrite = {0x42, 0x01, 0x00, 0x80, 0x00, 0x04};
	const uint8_t *rcvd;
	int n;

	((uint16_t*) &towrite)[3] = htons(msgid());
	((uint32_t*) &towrite)[2] = htonl(addr);
	udpcmd_->Write(towrite.c_array(), towrite.size());
	rcvd = udpcmd_->BlockRead(n);
	if (n == 12) value = ntohl(((uint32_t*)&rcvd)[2]);
	else {
		char txt[100];
		int m = sprintf(txt, "read register<%0X> get <%d> bytes reply: ", addr, n);
		for (int i = 0; i < n; ++i) m += sprintf(txt + m, "%02X ", rcvd[i]);
		throw std::runtime_error(txt);
	}
}

void CameraGYCCD::write(uint32_t addr, uint32_t value) {
	mutex_lock lck(mtxreg_);
	boost::array<uint8_t, 16> towrite = {0x42, 0x01, 0x00, 0x82, 0x00, 0x08};
	const uint8_t *rcvd;
	int n;

	((uint16_t*) &towrite)[3] = htons(msgid());
	((uint32_t*) &towrite)[2] = htonl(addr);
	((uint32_t*) &towrite)[3] = htonl(value);
	udpcmd_->Write(towrite.c_array(), towrite.size());
	rcvd = udpcmd_->BlockRead(n);

	if (n != 12 || rcvd[11] != 0x1) {
		char txt[100];
		int m = sprintf(txt, "write register<%0X> get <%d> bytes reply: ", addr, n);
		for (int i = 0; i < n; ++i) m += sprintf(txt + m, "%02X ", rcvd[i]);
		throw std::runtime_error(txt);
	}
}

uint16_t CameraGYCCD::msgid() {
	if (++idmsg_ == 0) idmsg_ = 1;
	return idmsg_;
}

void CameraGYCCD::re_transmit() {

}

void CameraGYCCD::re_transmit(uint32_t first, uint32_t last) {

}

void CameraGYCCD::stat_zone(ChannelZone *zone, double &mean, double &rms) {
	int x1(zone->x1), y1(zone->y1), x2(zone->x2), y2(zone->y2);
	int n((x2 - x1) * (y2 - y1));
	int w(nfcam_->wsensor);
	int x, y;
	double sum(0.0), sq(0.0), t;
	uint16_t *data = (uint16_t*) nfcam_->data.get();

	for (y = y1, data += y1 * w; y < y2; data += w) {
		for (x = x1; x < x2; ++x) {
			sum += (t = data[x]);
			sq += (t * t);
		}
	}
	mean = sum / n;
	rms = sqrt((sq - mean * sum) / n);
}

void CameraGYCCD::thread_heartbeat() {
	try {
		uint32_t value;
		boost::chrono::seconds period;
		read(0x938, value);
		period = boost::chrono::seconds(uint32_t(value * 7E-4 + 0.5));

		while(1) {
			boost::this_thread::sleep_for(period);
			read(0x938, value);
		}
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->state = CAMSTAT_ERROR;
		nfcam_->errcode = CAMERR_HEARTBEAT;
		cbexp_(0, 0.0, nfcam_->state);
	}
}

void thread_readout() {
	boost::chrono::milliseconds period(100);
	boost::mutex mtxdummy;
}
