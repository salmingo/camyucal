/*
 * @file CameraGYCCD.cpp GangYu相机封装接口定义文件
 * @version 0.2
 * @date 2017-12-20
 **/

#include <math.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include "CameraGYCCD.h"

using namespace boost::posix_time;

CameraGYCCD::CameraGYCCD(const char *camIP) {
	camip_  = camIP;
	gain_   = 0xFFFF;
	shtr_   = 0xFFFF;
	expdur_ = 0xFFFF;
	idmsg_  = 0;

	byteimg_  = 0;
	bytercvd_ = 0;
	packtot_  = 0;
	packlen_  = 0;
	headlen_  = 0;
	idfrm_    = 0;
	idpck_    = 0;
}

CameraGYCCD::~CameraGYCCD() {
}

bool CameraGYCCD::Reboot() {
	try {
		write(0x20054, 0x12AB3C4D);
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
			rcvd = (const uint8_t*) udpcmd_->BlockRead(bytercvd);
		} while(bytercvd < 48 && ++trycnt <= 3);
		if (bytercvd < 48) throw std::runtime_error("failed to communicate with camera");

		/* 相机初始化 */
		write(0x0A00,   0x03);		// 网络连接特性. 0x02: 共享; 0x03: 独占
		write(0x0D00,   PORT_DATA);	// 数据端口
		write(0x0D04,   1500);		// 数据包大小
		write(0x0D08,   0);			// 数据包间延时
		write(0x0D18,   addrHost);	// 主机地址
		write(0xA000,   0x01);		// 启用图像采集
		write(0x0938,   0x3A98);		// 网络连接最长保持时间, 量纲: 毫秒. 0x3A98=15000

		read(0x20008,  gain_);		// 增益
		read(0x2000C,  shtr_);		// 快门模式
		read(0x20010,  expdur_);		// 曝光时间, 量纲: 微秒
		read(0x0D04,   packlen_);	// 数据包大小
		read(0xA004,   value); nfcam_->wsensor = int(value);		// 图像宽度
		read(0xA008,   value); nfcam_->hsensor = int(value);		// 图像高度
		byteimg_ = nfcam_->wsensor * nfcam_->hsensor * 2;
		headlen_ = 8;
		packlen_ -= (20 + 8 + headlen_); // 20: IP Header; 8: UDP Header; headnlen_: Customized Header
		packtot_ = int(ceil(double(byteimg_ + 64) / packlen_));
		packflag_.reset(new uint8_t[packtot_ + 1]);

		/* 线程 */
		thrdhb_.reset(new boost::thread(boost::bind(&CameraGYCCD::thread_heartbeat, this)));
		thrdread_.reset(new boost::thread(boost::bind(&CameraGYCCD::thread_readout, this)));

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
		interrupt_thread(thrdread_);
		interrupt_thread(thrdhb_);
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->errcode = CAMERR_DISCONNECT;
	}
}

bool CameraGYCCD::start_expose(double duration, bool light) {
	try {
		uint32_t value;
		// 曝光前初始化
		bytercvd_ = 0;
		idfrm_    = 0;
		idpck_    = 0;
		memset(packflag_.get(), 0, packtot_ + 1);
		// 开始曝光
		if (shtr_ != (value = light ? 0 : 2)) {
			write(0x2000C, value);
			read(0x2000C,  shtr_);
			boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
		}
		if (expdur_ != (value = uint32_t(duration * 1E6))) {
			write(0x20010, value);
			read(0x20010,  expdur_);
		}
		write(0x20000, 0x01);
		waitread_.notify_one();

		return true;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg  = ex.what();
		nfcam_->errcode = CAMERR_EXPOSE;
		return false;
	}
}

void CameraGYCCD::stop_expose() {
	if ((nfcam_->state == CAMSTAT_EXPOSE || nfcam_->state == CAMSTAT_READOUT)) {
		try {
			write(0x20050, 0x1);
			if (nfcam_->state == CAMSTAT_READOUT) imgrdy_.notify_one();
		}
		catch(std::runtime_error &ex) {
			nfcam_->errmsg  = ex.what();
			nfcam_->errcode = CAMERR_ABTEXPOSE;
		}
	}
}

int CameraGYCCD::check_state() {
	return nfcam_->state;
}

int CameraGYCCD::readout_image() {
	boost::mutex dummy;
	mutex_lock lck(dummy);

	imgrdy_.wait(lck);
	return (byteimg_ == bytercvd_ ? CAMSTAT_IMGRDY : CAMSTAT_IDLE);
}

double CameraGYCCD::sensor_temperature() {
	return nfcam_->coolget;
}

void CameraGYCCD::update_cooler(double &coolset, bool onoff) {
	// ...物理上不支持该功能
}

uint32_t CameraGYCCD::update_readport(uint32_t index) {
	// ...物理上不支持该功能
	return 0;
}

uint32_t CameraGYCCD::update_readrate(uint32_t index) {
	// ...物理上不支持该功能
	return 0;
}

uint32_t CameraGYCCD::update_gain(uint32_t index) {
	try {
		if (gain_ != index && index <= 2) {
			write(0x20008, index);
			read(0x20008, gain_);
		}
		return gain_;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg  = ex.what();
		nfcam_->errcode = CAMERR_REGISTER;
		return 0;
	}
}

void CameraGYCCD::update_roi(int &xstart, int &ystart, int &width, int &height, int &xbin, int &ybin) {
	// ...物理上不支持该功能
}

int CameraGYCCD::update_adcoffset(uint16_t offset, FILE *output) {
	boost::chrono::seconds wait(1);
	GYChannel channel;
	int i, delta, total, res;
	int &state = nfcam_->state;

	/* 1 - 读取当前设置 */
	if (!load_preamp_offset(channel.offset)) return 2;
	if (output) {
		fprintf(output, "Initial Offset:\n");
		output_offset(output, channel.offset);
	}
	/* 2 - 采集本底, 统计overscan区背景 */
	if (output) {
		fprintf(output, "take one frame bias image, to evaluate mean value of over-scan zone\n");
	}
	if (!Expose(0.0, false)) {
		if (output) fprintf(output, "failed to take bias image\n");
		return 2;
	}
	do {
		if (output) fprintf(output, "patience...\n");
		boost::this_thread::sleep_for(wait);
	} while(state != CAMSTAT_ERROR && state != CAMSTAT_IMGRDY && state != CAMSTAT_IDLE);
	if (state == CAMSTAT_ERROR) {
		if (output) 	fprintf(output, "failed to take bias image\n");
		return 2;
	}
	stat_overscan(&channel);
	/* 3 - 计算期望与实际偏差, 若小于阈值(10)则结束 */
	double vmin(1E30), vmax(-1E30);
	for (i = 0; i < 4; ++i) {
		if (channel.mean[i] < vmin) vmin = channel.mean[i];
		if (channel.mean[i] > vmax) vmax = channel.mean[i];
	}
	if ((vmax - vmin) < 10.0) return 1;
	else if (output) {
		fprintf(output, "statistics result:\n");
		output_statistics(output, &channel);
	}
	/* 4 - 各通道RGB各加10, 采集本底, 并统计背景, 计算增益 */
	double mean[4];
	memcpy(mean, channel.mean, 4 * sizeof(double));

	for (i = 0; i < 4; ++i) {
		channel.offset[i].r += 10;
		channel.offset[i].g += 10;
		channel.offset[i].b += 10;
	}
	if (!apply_preamp_offset(channel.offset)) {
		if (output) fprintf(output, "failed to modify preamp offset");
		return 2;
	}

	if (output) {
		fprintf(output, "take one frame bias image, to evaluate preamp gain\n");
	}
	if (!Expose(0.0, false)) {
		if (output) fprintf(output, "failed to take bias image\n");
		return 2;
	}
	do {
		if (output) fprintf(output, "patience...\n");
		boost::this_thread::sleep_for(wait);
	} while(state != CAMSTAT_ERROR && state != CAMSTAT_IMGRDY && state != CAMSTAT_IDLE);
	if (state == CAMSTAT_ERROR) {
		if (output) 	fprintf(output, "failed to take bias image\n");
		return 2;
	}
	stat_overscan(&channel);
	if (output) {
		fprintf(output, "statistics result:\n");
		output_statistics(output, &channel);
	}

	for (i = 0; i < 4; ++i) {
		channel.gain[i] = (channel.mean[i] - mean[i]) / 30;
	}
	if (output) {
		for (i = 0; i < 4; ++i) {
			fprintf(output, "Channel#%d gain = %.1f\n", i + 1, channel.gain[i]);
		}
		fflush(output);
	}
	/* 5 - 计算期望与实际偏差 */
	for (i = 0; i < 4; ++i) {
		delta = int((channel.mean[i] - offset) / channel.gain[i]);
		total = channel.offset[i].r + channel.offset[i].g + channel.offset[i].b - delta;
		res = total % 3;
		total /= 3;

		channel.offset[i].r = total;
		channel.offset[i].g = total;
		channel.offset[i].b = total;

		if (res) { ++channel.offset[i].r; --res; }
		if (res) { ++channel.offset[i].g; --res; }
	}
	if (!apply_preamp_offset(channel.offset)) {
		if (output) fprintf(output, "failed to modify preamp offset\n");
		return 2;
	}
	/* 6 - 采集本底, 评估修正结果 */
	if (output) {
		fprintf(output, "take one frame bias image, to evaluate preamp tuning result\n");
	}
	if (!Expose(0.0, false)) {
		if (output) fprintf(output, "failed to take bias image\n");
		return 2;
	}
	do {
		if (output) fprintf(output, "patience...\n");
		boost::this_thread::sleep_for(wait);
	} while(state != CAMSTAT_ERROR && state != CAMSTAT_IMGRDY && state != CAMSTAT_IDLE);
	if (state == CAMSTAT_ERROR) {
		if (output) 	fprintf(output, "failed to take bias image\n");
		return 2;
	}
	stat_overscan(&channel);
	if (output) {
		fprintf(output, "statistics result:\n");
		output_statistics(output, &channel);
	}

	/* 7 - 保存参数 */
	if (output) {
		fprintf(output, "fine tuned offset:\n");
		output_offset(output, channel.offset);
	}
	save_preamp_offset(channel.offset);

	return 0;
}

int CameraGYCCD::update_network(int option, const char *value) {
	try {
		uint32_t addr;
		uint32_t vold, vnew;

		addr = option == 1 ? 0x64C : (option == 2 ? 0x65C : 0x66C);
		read(addr, vold);
		inet_pton(AF_INET, value, &vnew);
		vnew = ntohl(vnew);
		if (vold != vnew) {
			write(addr, vnew);
			return 0;
		}

		return 1;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errmsg = ex.what();
		nfcam_->errcode = CAMERR_REGISTER;
		return 2;
	}
}

void CameraGYCCD::read(uint32_t addr, uint32_t &value) {
	mutex_lock lck(mtxreg_);
	boost::array<uint8_t, 16> towrite = {0x42, 0x01, 0x00, 0x80, 0x00, 0x04};
	const char *rcvd;
	int n;

	((uint16_t*) &towrite)[3] = htons(msgid());
	((uint32_t*) &towrite)[2] = htonl(addr);
	udpcmd_->Write(towrite.c_array(), towrite.size());
	rcvd = udpcmd_->BlockRead(n);
	if (n == 12) value = ntohl(((uint32_t*)rcvd)[2]);
	else {
		char txt[100];
		int m = sprintf(txt, "read register<%0X> get <%d> bytes reply: ", addr, n);
		for (int i = 0; i < n; ++i) m += sprintf(txt + m, "%02X ", rcvd[i] & 0xFF);
		throw std::runtime_error(txt);
	}
}

void CameraGYCCD::write(uint32_t addr, uint32_t value) {
	mutex_lock lck(mtxreg_);
	boost::array<uint8_t, 16> towrite = {0x42, 0x01, 0x00, 0x82, 0x00, 0x08};
	const char *rcvd;
	int n;

	((uint16_t*) &towrite)[3] = htons(msgid());
	((uint32_t*) &towrite)[2] = htonl(addr);
	((uint32_t*) &towrite)[3] = htonl(value);
	udpcmd_->Write(towrite.c_array(), towrite.size());
	rcvd = udpcmd_->BlockRead(n);

	if (n != 12 || rcvd[11] != 0x1) {
		char txt[100];
		int m = sprintf(txt, "write register<%0X> get <%d> bytes reply: ", addr, n);
		for (int i = 0; i < n; ++i) m += sprintf(txt + m, "%02X ", rcvd[i] & 0xFF);
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
	if (bytercvd_ == byteimg_ // 已完成图像读出
			|| nfcam_->state < CAMSTAT_EXPOSE    // 在EXPOSE或READOUT状态, 处理收到的图像数据
			|| nfcam_->state > CAMSTAT_READOUT)
		return;

	int bytes;
	const uint8_t *pack;
	uint8_t type;
	uint16_t idfrm;
	uint32_t idpck, pcklen;

	lastdata_ = microsec_clock::universal_time();
	if (nfcam_->state == CAMSTAT_EXPOSE) nfcam_->state = CAMSTAT_READOUT;

	pack = (const uint8_t *) udpdata_->Read(bytes);
	type  = pack[4];     // 数据包类型
	idpck = (pack[5] << 16) | (pack[6] << 8) | pack[7];  // 数据包编号
	if (idfrm_ != (idfrm = (pack[2] << 8) | pack[3])) idfrm_ = idfrm;   // 图像帧编号

	if (type == PACK_PAYLOAD) {// 图像数据包
		if (!packflag_[idpck]) {// 避免重复接收
			// 计算有效数据大小
			pcklen = idpck != packtot_ ? len - headlen_ : len - headlen_ - 64;
			if (idpck == packtot_ || pcklen == packlen_) {// 存储一包图像数据
				memcpy(nfcam_->data.get() + (idpck - 1) * packlen_, pack + headlen_, pcklen);
				bytercvd_ += pcklen;
				packflag_[idpck] = 1; // 置接收标志
				if (bytercvd_ == byteimg_) imgrdy_.notify_one();
				else if (idpck != (idpck_ + 1)) re_transmit();
			}
		}
	}
	else if (type == PACK_TRAILER) {// 附加数据包
		re_transmit();
	}
	idpck_ = idpck;
}

void CameraGYCCD::stat_overscan(GYChannel *ch) {
	int x1, y1, x2, y2;
	int n;
	int w(nfcam_->wsensor);
	int x, y;
	double sum, sq, t, mean;
	uint16_t *data = (uint16_t*) nfcam_->data.get();
	uint16_t *ptr;

	for (int i = 0; i < 4; ++i) {
		x1 = ch->overscan[i].x1;
		y1 = ch->overscan[i].y1;
		x2 = ch->overscan[i].x2;
		y2 = ch->overscan[i].y2;
		n  = (x2 - x1) * (y2 - y1);

		for (y = y1, ptr = data + y1 * w, sum = sq = 0.0; y < y2; ++y, ptr += w) {
			for (x = x1; x < x2; ++x) {
				sum += (t = ptr[x]);
				sq += (t * t);
			}
		}
		ch->mean[i] = mean = sum / n;
		ch->rms[i] = sqrt((sq - mean * sum) / n);
	}
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
		nfcam_->state = CAMSTAT_ERROR;
		nfcam_->errcode = CAMERR_HEARTBEAT;
		nfcam_->errmsg = ex.what();
		cbexp_(nfcam_->state, 0, 0.0);
	}
}

void CameraGYCCD::thread_readout() {
	boost::chrono::milliseconds period(100);
	boost::mutex dummy;
	mutex_lock lck(dummy);
	ptime now;
	double error_readout(10.0);
	int &state = nfcam_->state;

	while(state != CAMSTAT_ERROR) {
		waitread_.wait(lck);

		do {
			boost::this_thread::sleep_for(period);
			now = microsec_clock::universal_time();

			if (((now - nfcam_->tmobs).total_seconds() - nfcam_->expdur) > error_readout) {// 长时间无响应
				nfcam_->state   = CAMSTAT_ERROR;
				nfcam_->errcode = CAMERR_READOUT;
				nfcam_->errmsg  = "long time no data reply";
				cbexp_(nfcam_->state, 0.0, 0.0);
			}
			else if (state == CAMSTAT_READOUT
					&& (now - lastdata_).total_milliseconds() > 10) {// 读出态, 检查是否读出中断
				re_transmit();
			}
		} while(state == CAMSTAT_EXPOSE || state == CAMSTAT_READOUT);
	}
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

bool CameraGYCCD::load_preamp_offset(ChannelOffset *offset) {// 从控制器加载参数
	try {
		uint32_t val;

		write(0x10010, 1);
		read(0x11000, val);   offset[0].r = int(val);
		read(0x11004, val);   offset[0].g = int(val);
		read(0x11008, val);   offset[0].b = int(val);
		read(0x1100C, val);   offset[1].r = int(val);
		read(0x11010, val);   offset[1].g = int(val);
		read(0x11014, val);   offset[1].b = int(val);

		write(0x10010, 2);
		read(0x11000, val);   offset[2].r = int(val);
		read(0x11004, val);   offset[2].g = int(val);
		read(0x11008, val);   offset[2].b = int(val);
		read(0x1100C, val);   offset[3].r = int(val);
		read(0x11010, val);   offset[3].g = int(val);
		read(0x11014, val);   offset[3].b = int(val);
		return true;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errcode = CAMERR_REGISTER;
		nfcam_->errmsg  = ex.what();
		return false;
	}
}

bool CameraGYCCD::apply_preamp_offset(ChannelOffset *offset) {// 临时应用参数
	for (int i = 0; i < 4; ++i) {
		if (!apply_preamp_offset(i, 0, offset[i].r)) return false;
		if (!apply_preamp_offset(i, 1, offset[i].g)) return false;
		if (!apply_preamp_offset(i, 2, offset[i].b)) return false;
	}

	return true;
}

bool CameraGYCCD::apply_preamp_offset(int channel, int color, int offset) {
	try {
		uint32_t val, val0, addr;

		val0 = color == 0 ? 0x05 : (color == 1 ? 0x06 : 0x07);
		val  = offset >= 0 ? offset : 0x0100 - offset;
		val += (val0 << 12);
		write(0x00010010, channel < 2 ? 1 : 2); // 切换通道
		write(0x00010000, (channel == 0 || channel == 2) ? 0x0010 : 0x0011);
		write(0x00010004, val); // 更新偏置
		return true;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errcode = CAMERR_REGISTER;
		nfcam_->errmsg  = ex.what();
		return false;
	}
}

bool CameraGYCCD::save_preamp_offset(ChannelOffset *offset) {// 永久存储参数
	try {
		uint32_t addr;

		write(0x0001FFF0, 0x1A3C57C9); // begin

		// 写入各通道偏置值
		write(0x00010010, 1); // 上半区域
		addr = 0x11000; write(addr, offset[0].r);
		addr += 4;      write(addr, offset[0].g);
		addr += 4;      write(addr, offset[0].b);
		addr += 4;      write(addr, offset[1].r);
		addr += 4;      write(addr, offset[1].g);
		addr += 4;      write(addr, offset[1].b);

		write(0x00010010, 2); // 下半区域
		addr = 0x11000; write(addr, offset[2].r);
		addr += 4;      write(addr, offset[2].g);
		addr += 4;      write(addr, offset[2].b);
		addr += 4;      write(addr, offset[3].r);
		addr += 4;      write(addr, offset[3].g);
		addr += 4;      write(addr, offset[3].b);

		write(0x00013000, 0x1); // end
		write(0x0001FFF0, 0x0);

		return true;
	}
	catch(std::runtime_error &ex) {
		nfcam_->errcode = CAMERR_REGISTER;
		nfcam_->errmsg  = ex.what();
		return false;
	}
}

void CameraGYCCD::output_offset(FILE *output, ChannelOffset *offset) {
	for (int i = 0; i < 4; ++i) {
		fprintf(output, "Channel#%d Offset:  %+4d  %+4d  %+4d\n", i + 1,
				offset[i].r, offset[i].g, offset[i].b);
	}
	fflush(output);
}

void CameraGYCCD::output_statistics(FILE *output, GYChannel *channel) {
	for (int i = 0; i < 4; ++i) {
		fprintf(output, "Channel#%d:  %7.1f   %6.2f\n", i + 1, channel->mean[i], channel->rms[i]);
	}
	fflush(output);
}
