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
	msgid_ = 0;
}

CameraGYCCD::~CameraGYCCD() {
}

bool CameraGYCCD::connect() {
	return false;
}

void CameraGYCCD::disconnect() {

}

bool CameraGYCCD::start_expose(double duration, bool light) {
	return false;
}

void CameraGYCCD::stop_expose() {

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
	return 0;
}

uint32_t CameraGYCCD::update_readrate(uint32_t index) {
	return 0;
}

uint32_t CameraGYCCD::update_gain(uint32_t index) {
	return 0;
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
	if (++msgid_ == 0) msgid_ = 1;
	return msgid_;
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

}
