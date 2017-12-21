/*
 * @file CameraGYCCD.cpp GangYu相机封装接口定义文件
 * @version 0.2
 * @date 2017-12-20
 **/
#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
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
		/* 创建UDP连接 */
		const UDPSession::CBSlot &slot = boost::bind(&CameraGYCCD::ReceiveImageData, this, _1, _2);
		udpdata_ = makeudp_session(PORT_DATA);
		udpdata_->RegisterRead(slot);
		udpcmd_ = makeudp_session();
		udpcmd_->Connect(camip_.c_str(), PORT_CAMERA);

		/* 尝试连接相机 */
		uint32_t addrHost = get_hostaddr();
		if (!addrHost) throw std::runtime_error("no matched host IP address");

		boost::array<uint8_t, 8> towrite = {0x42, 0x01, 0x00, 0x02, 0x00, 0x00};
		const uint8_t *rcvd;
		int bytercvd, trycnt(0);
		uint32_t value;

		idmsg_ = 0;
		do {
			((uint16_t*)&towrite)[3] = htons(msgid());
			udpcmd_->Write(towrite.c_array(), towrite.size());
			rcvd = udpcmd_->BlockRead(bytercvd);
		} while(bytercvd < 48 && ++trycnt <= 3);
		if (bytercvd < 48) throw std::runtime_error("failed to communicate with camera");

		/* 相机初始化 */
		write(0x0938,   0x3A98);		// 网络连接最长保持时间, 量纲: 毫秒
		write(0x0A00,   0x03);		// 网络连接特性. 0x02: 共享; 0x03: 独占
		write(0x0D00,   PORT_DATA);	// 数据端口
		write(0x0D04,   1500);		// 数据包大小
		write(0x0D08,   0);			// 数据包间延时
		write(0x0D18,   addrHost);	// 主机地址
		write(0xA000,   0x01);		// 启用图像采集

		read(0x20008,  gain_);		// 增益
		read(0x2000C,  shtr_);		// 快门模式
		read(0x20010,  expdur_);		// 曝光时间, 量纲: 微秒
		read(0xA004,   value);		// 图像宽度
		nfcam_->wsensor = int(value);
		read(0xA008,   value);		// 图像高度
		nfcam_->hsensor = int(value);
		byteimg_ = nfcam_->wsensor * nfcam_->hsensor * 2;
		read(0x0D04,   packlen_);	// 数据包大小
		headlen_ = 8;
		packlen_ -= (20 + 8 + headlen_); // 20: IP Header; 8: UDP Header; headnlen_: Customized Header
		packtot_ = int(ceil(double(byteimg_ + 64) / packlen_));
		packflag_.reset(new uint8_t[packtot_ + 1]);

		/* 线程 */
		thrdhb_.reset(new boost::thread(&boost::bind(&CameraGYCCD::thread_heartbeat, this)));
		thrdread_.reset(new boost::thread(&boost::bind(&CameraGYCCD::thread_readout, this)));

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
	int first, last;

	for (first = 1; first <= packtot_ && packflag_[first]; ++first);
	for (last = first; last <= packtot_ && !packflag_[last]; ++last);
	re_transmit(first, --last);
}

void CameraGYCCD::re_transmit(uint32_t first, uint32_t last) {
	mutex_lock lck(mtxreg_);
	boost::array<uint8_t, 20> towrite = {0x42, 0x00, 0x00, 0x40, 0x00, 0x0C};
	((uint16_t*)&towrite)[3] = htons(msgid());
	((uint32_t*)&towrite)[2] = htonl(idfrm_);
	((uint32_t*)&towrite)[3] = htonl(first);
	((uint32_t*)&towrite)[4] = htonl(last);
	udpcmd_->Write(towrite.c_array(), towrite.size());
}

void CameraGYCCD::ReceiveImageData(const long udp, const long len) {

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
		period = boost::chrono::seconds(uint32_t(ceil(value * 3E-4 + 0.5)));

		while(1) {
			boost::this_thread::sleep_for(period);
			if (nfcam_->state <= CAMSTAT_EXPOSE) read(0x938, value);
		}
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->state = CAMSTAT_ERROR;
		nfcam_->errcode = CAMERR_HEARTBEAT;
		cbexp_(nfcam_->state, 0, 0.0);
	}
}

void CameraGYCCD::thread_readout() {
	boost::chrono::milliseconds period(100);
	boost::mutex mtxdummy;
}

uint32_t CameraGYCCD::get_hostaddr() {
	ifaddrs *ifaddr, *ifa;
	uint32_t addr(0), addrcam, mask;
	bool found(false);

	inet_pton(AF_INET, camip_.c_str(), &addrcam);
	if (!getifaddrs(&ifaddr)) {
		for (ifa = ifaddr; ifa != NULL && !found; ifa = ifa->ifa_next) {// 只采用IPv4地址
			if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
			addr = ((struct sockaddr_in*) ifa->ifa_addr)->sin_addr.s_addr;
			mask = ((struct sockaddr_in*) ifa->ifa_netmask)->sin_addr.s_addr;
			found = (addr & mask) == (addrcam & mask); // 与相机IP在同一网段
		}
		freeifaddrs(ifaddr);
	}

	return found ? ntohl(addr) : 0;
}
