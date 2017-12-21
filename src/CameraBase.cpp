/*
 * @file CameraBase.cpp 定义文件, 通用相机控制接口
 * @version 0.1
 * @date 2017-11-17
 */
#include <boost/make_shared.hpp>
#include "CameraBase.h"

CameraBase::CameraBase() {
	nfcam_ = boost::make_shared<Information>();
}

CameraBase::~CameraBase() {
	Disconnect();
}

bool CameraBase::Connect() {
	if (!nfcam_->connected && connect()) {
		nfcam_->init();
		thrdCycle_.reset(new boost::thread(boost::bind(&CameraBase::thread_cycle, this)));
		thrdExpose_.reset(new boost::thread(boost::bind(&CameraBase::thread_expose, this)));
	}

	return nfcam_->connected;
}

void CameraBase::Disconnect() {
	if (nfcam_->connected) {
		if (nfcam_->state == CAMSTAT_EXPOSE) stop_expose();
		boost::chrono::milliseconds wait(100);
		while (nfcam_->state > CAMSTAT_IDLE) boost::this_thread::sleep_for(wait);
		interrupt_thread(thrdExpose_);
		interrupt_thread(thrdCycle_);
		disconnect();
		nfcam_->uninit();
	}
}

bool CameraBase::Reboot() {
	return false;
}

bool CameraBase::Expose(double duration, bool light) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE) {
		if (!start_expose(duration, light)) return false;
		nfcam_->begin_expose(duration);
		nfcam_->format_tmobs();
		cvexp_.notify_one();

		return true;
	}
	return false;
}

void CameraBase::AbortExpose() {
	if (nfcam_->state >= CAMSTAT_EXPOSE) {
		nfcam_->aborted = true;
		stop_expose();
	}
}

void CameraBase::SetCooler(double coolset, bool onoff) {
	if (nfcam_->connected) update_cooler(coolset, onoff);
}

void CameraBase::SetReadPort(uint32_t index) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE)
		nfcam_->readport = update_readport(index);
}

void CameraBase::SetReadRate(uint32_t index) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE)
		nfcam_->readrate = update_readrate(index);
}

void CameraBase::SetGain(uint32_t index) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE)
		nfcam_->gain = update_gain(index);
}

void CameraBase::SetROI(int xstart, int ystart, int width, int height, int xbin, int ybin) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE) {
		ROI &roi = nfcam_->roi;
		roi.set_roi(xstart, ystart, width, height, xbin, ybin); // 检验/校正ROI设置
		xstart = roi.xstart, ystart = roi.ystart;
		width  = roi.width,  height = roi.height;
		xbin   = roi.xbin,   ybin   = roi.ybin;
		update_roi(xstart, ystart, width, height, xbin, ybin);
		roi.set_roi(xstart, ystart, width, height, xbin, ybin); // 由相机反馈更新设置
	}
}

int CameraBase::SetADCOffset(uint16_t offset, FILE *output) {
	int rslt(3);
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE) {
		nfcam_->mode = CAMMOD_CALIBRATE;
		rslt = update_adcoffset(offset, output);
		nfcam_->mode = CAMMOD_NORMAL;
	}
	return rslt;
}

int CameraBase::SetIP(const char *ip) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE) {
		return update_network(1, ip);
	}
	return 3;
}

int CameraBase::SetNetmask(const char *mask) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE) {
		return update_network(2, mask);
	}
	return 3;
}

int CameraBase::SetGateway(const char *gw) {
	if (nfcam_->connected && nfcam_->mode == CAMMOD_NORMAL && nfcam_->state == CAMSTAT_IDLE) {
		return update_network(3, gw);
	}
	return 3;
}

void CameraBase::RegisterExposeProcess(const ExpProcSlot &slot) {
	if (nfcam_->state <= CAMSTAT_IDLE) {
		if (!cbexp_.empty()) cbexp_.disconnect_all_slots();
		cbexp_.connect(slot);
	}
}

void CameraBase::thread_cycle() {
	boost::chrono::seconds period(10);
	double coolget;

	boost::this_thread::sleep_for(boost::chrono::seconds(1));
	while(1) {
		coolget = sensor_temperature();
		if (nfcam_->coolget != coolget) nfcam_->coolget = coolget;
		boost::this_thread::sleep_for(period);
	}
}

void CameraBase::thread_expose() {
	boost::mutex dummy;
	mutex_lock lck(dummy);
	boost::chrono::milliseconds period;
	double left, percent;
	int &state = nfcam_->state;

	while(1) {
		cvexp_.wait(lck);
		while ((state = check_state()) == CAMSTAT_EXPOSE) {// 曝光过程
			nfcam_->check_expose(left, percent);
			period = boost::chrono::milliseconds(left > 0.1 ? 100 : int(left * 1000));
			if (!cbexp_.empty()) cbexp_(state, left, percent);
			if (left > 0.0) boost::this_thread::sleep_for(period);
		}
		if (state == CAMSTAT_READOUT) {// 读出过程
			nfcam_->end_expose();
			if (!cbexp_.empty()) cbexp_(state, 0.0, 100.0);
			state = readout_image();
		}
		/*
		 * 曝光过程结束时, 可能出现三种状态:
		 * 1) CAMERA_IMGRDY: 正常结束, 图像已进入内存
		 * 2) CMAERA_IDLE  : 异常结束, 设备无错误
		 * 3) CAMERA_ERROR : 异常结束, 设备错误
		 */
		if (state == CAMSTAT_IMGRDY && nfcam_->aborted) state = CAMSTAT_IDLE;
		if (!cbexp_.empty()) cbexp_(state, 0.0, 100.0);
		if (state == CAMSTAT_IMGRDY) state = CAMSTAT_IDLE;
	}
}

void CameraBase::interrupt_thread(threadptr &thrd) {
	if (thrd.unique()) {
		thrd->interrupt();
		thrd->join();
		thrd.reset();
	}
}
