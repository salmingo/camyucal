/*
 * @file CameraBase.h 声明文件, 通用相机控制接口
 * @version 0.1
 * @date 2017-11-17
 */

#ifndef CAMERABASE_H_
#define CAMERABASE_H_

#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>

using std::string;
using boost::posix_time::ptime;

//////////////////////////////////////////////////////////////////////////////
enum CAMERA_STATUS {// 相机工作状态
	CAMSTAT_ERROR,		// 错误
	CAMSTAT_IDLE,		// 空闲
	CAMSTAT_EXPOSE,		// 曝光过程中
	CAMSTAT_READOUT,		// 读出过程中
	CAMSTAT_IMGRDY		// 已完成曝光, 可以读出数据进入内存
};

enum CAMERA_MODE {// 相机工作模式
	CAMMOD_NORMAL,		// 常规模式
	CAMMOD_CALIBRATE		// 定标模式. 用于校准偏置电压
};

/// 相机错误代码
enum CAMERR_CODE {// 相机故障代码
	CAMERR_SUCCESS,		//< 正确
	CAMERR_CONNECT,		//< 连接
	CAMERR_DISCONNECT,	//< 断开连接
	CAMERR_EXPOSE,		//< 开始曝光
	CAMERR_ABTEXPOSE,	//< 中止曝光
	CAMERR_HEARTBEAT,	//< 心跳
	CAMERR_READOUT,		//< 读出
	CAMERR_REBOOT,		//< 软重启相机
	CAMERR_REGISTER,		//< 寄存器访问
	CAMERR_LAST
};

//////////////////////////////////////////////////////////////////////////////

class CameraBase {
public:
	CameraBase();
	virtual ~CameraBase();

public:
	/* 数据结构 */
	typedef boost::shared_array<uint8_t> ucharray;	//< 无符号字节型数组
	typedef boost::unique_lock<boost::mutex> mutex_lock;
	typedef boost::shared_ptr<boost::thread> threadptr;

	struct ROI {// ROI区, XY坐标起始点为(1,1)
		int wfull, hfull;	//< 全帧尺寸, 量纲: 像素
		int xstart, ystart;	// ROI区左上角在全幅中的位置
		int width, height;	// ROI区在全副中的宽度和高度
		int xbin, ybin;		// ROI区X、Y方向合并因子

	public:
		ROI() {
			wfull = hfull = -1;
			xstart = ystart = 1;
			width = height = -1;
			xbin = ybin = 1;
		}

		/*!
		 * @brief 设置探测器尺寸
		 * @param w 宽度
		 * @param h 高度
		 */
		void set_sensor(int w, int h) {
			wfull = width = w;
			hfull = height = h;
		}

		/*!
		 * @brief ROI区合并后像素数
		 * @param w 宽度
		 * @param h 高度
		 * @return
		 * 总像素数
		 */
		int get_dimension(int &w, int &h) {
			w = width / xbin;
			h = height / ybin;
			return (w * h);
		}

		/*!
		 * @brief 查看ROI区设置
		 * @param xs X轴起始位置
		 * @param ys Y轴起始位置
		 * @param w  宽度
		 * @param h  高度
		 * @param xb X轴合并因子
		 * @param yb Y轴合并因子
		 */
		void get_roi(int &xs, int &ys, int &w, int &h, int &xb, int &yb) {
			xs = xstart;
			ys = ystart;
			w = width;
			h = height;
			xb = xbin;
			yb = ybin;
		}

		void reset() {// 重置ROI区
			xstart = ystart = 1;
			xbin = ybin = 1;
			width = wfull, height = hfull;
		}

		/*!
		 * @brief 设置ROI区
		 * @param xs
		 * @param ys
		 * @param w
		 * @param h
		 * @param xb
		 * @param yb
		 */
		void set_roi(int xs, int ys, int w, int h, int xb = 1, int yb = 1) {
			int res;
			if (w <= 0 || w > wfull) w = wfull;
			if (xb < 1 || xb > w / 2) xb = 1;
			if ((res = w % xb)) w = w > res ? w - res : w + xb - res;
			if ((res = xs + w - wfull) > 1) xs -= (res - 1);
			xstart = xs, width = w, xbin = xb;

			if (h <= 0 || h > hfull) h = hfull;
			if (yb < 1 || yb > h / 2) yb = 1;
			if ((res = h % yb)) h = h > res ? h - res : h + yb - res;
			if ((res = ys + h - hfull) > 1) ys -= (res - 1);
			ystart = ys, height = h, ybin = yb;
		}
	};

	struct Information {// 相机工作参数
		string	model;		//< 型号
		int		wsensor;		//< 探测器宽度, 像素
		int		hsensor;		//< 探测器高度, 像素
		uint32_t	readport;	//< 读出端口档位
		uint32_t	readrate;	//< 读出速度档位
		uint32_t	gain;		//< 增益档位
		double coolset;		//< 制冷温度

		bool	connected;		//< 连接标志
		int		mode;		//< 工作模式
		int		state;		//< 工作状态. CAMERA_STATUS枚举值
		int		errcode;		//< 错误代码. 此时state == CAMERA_ERROR ?
		string	errmsg;		//< 错误描述
		double	coolget;		//< 芯片温度
		ROI		roi;			//< 感兴趣区
		ucharray data;		//< 图像数据存储区

		bool		aborted;		//< 抛弃当前积分数据
		double	expdur;		//< 积分时间, 量纲: 秒
		ptime	tmobs;		//< 曝光起始时间, 用于记录和监测曝光进度
		ptime	tmend;		//< 曝光结束时间
		double	jd;			//< 曝光起始时间对应儒略日

		string	datestr;		//< 曝光起始日期, 仅日期信息, 格式: YYMMDD
		string	timestr;		//< 缩略曝光起始时间, 格式: YYMMDDThhmmssss

	public:
		/*!
		 * @brief 相机连接后初始化
		 */
		void init() {
			int w, h, n;
			roi.set_sensor(wsensor, hsensor);
			n = (roi.get_dimension(w, h) * 2 + 15) & ~15; // 内存对齐16字节
			data.reset(new uint8_t[n]);
			connected = true;
			mode  = CAMMOD_NORMAL;
			state = CAMSTAT_IDLE;
			errcode = CAMERR_SUCCESS;
		}

		/*!
		 * @brief 相机断开连接后取消初始化
		 */
		void uninit() {
			connected = false;
		}

		/*!
		 * @brief 记录曝光起始状态
		 * @param t 曝光时间
		 */
		void begin_expose(double t) {
			tmobs = boost::posix_time::microsec_clock::universal_time();
			expdur = t;
			aborted = false;
			state = CAMSTAT_EXPOSE;
		}

		/*!
		 * @brief 由曝光起始时间构建相关时间信息
		 */
		void format_tmobs() {
			ptime::date_type date;
			ptime::time_duration_type tdt;
			boost::format fmtdate("%02d%02d%02d");
			boost::format fmttime("%sT%02d%02d%04d");

			date = tmobs.date();
			tdt = tmobs.time_of_day();
			fmtdate % (date.year() - 2000) % date.month() % date.day();
			datestr = fmtdate.str();
			fmttime % datestr.c_str() % tdt.hours() % tdt.minutes()
					% (tdt.seconds() + tdt.fractional_seconds() / 10000);
			timestr = fmttime.str();
			jd = date.julian_day() + tdt.fractional_seconds() * 1E-6 / 86400.0 - 0.5;
		}

		/*!
		 * @brief 记录曝光结束状态
		 */
		void end_expose() {
			tmend = boost::posix_time::microsec_clock::universal_time();
		}

		/*!
		 * @brief 检查曝光进度
		 * @param left    剩余积分时间, 量纲: 秒
		 * @param percent 曝光进度, 百分比
		 * @return
		 * 曝光未完成返回true, 否则返回false
		 */
		void check_expose(double &left, double &percent) {
			namespace pt = boost::posix_time;
			pt::time_duration td = pt::microsec_clock::universal_time() - tmobs;

			double dt = td.total_microseconds() * 1E-6;
			if ((left = expdur - dt) <= 0.0) {
				left = 0.0;
				percent = 100.000001;
			}
			else percent = dt * 100.000001 / expdur;
		}

		/*!
		 * @brief 上下午判断
		 * @return
		 * true: 上午; false: 下午
		 */
		bool ampm() {
			ptime now = boost::posix_time::second_clock::local_time();
			return now.time_of_day().hours() < 12;
		}
	};
	typedef boost::shared_ptr<Information> InfoPtr;
	/*!
	 * @brief 声明曝光进度回调函数
	 * @param _1 相机工作状态
	 * @param _2 曝光进度剩余时间
	 * @param _3 曝光进度, 量纲: 百分比
	 */
	typedef boost::signals2::signal<void (int, double, double)> ExposeProcess;
	typedef ExposeProcess::slot_type ExpProcSlot;

protected:
	/* 成员变量 */
	InfoPtr nfcam_;	//< 相机工作参数
	threadptr thrdCycle_;	//< 线程句柄, 执行周期任务
	threadptr thrdExpose_;	//< 线程句柄, 监测曝光过程
	ExposeProcess cbexp_;	//< 曝光进度回调函数
	boost::condition_variable cvexp_;	//< 条件变量, 开始曝光

public:
	/* 接口 */
	/*!
	 * @brief 查看相机工作状态
	 * @return
	 * 工作状态
	 */
	InfoPtr GetInformation() { return nfcam_; }
	/*!
	 * @brief 注册曝光进度回调函数
	 * @param slot 函数插槽
	 */
	void RegisterExposeProcess(const ExpProcSlot &slot);
	/*!
	 * @brief 尝试连接相机
	 * @return
	 * 相机连接结果
	 */
	bool Connect();
	/*!
	 * @brief 断开与相机之间的连接
	 */
	void Disconnect();
	/*!
	 * @brief 检查相机连接状态
	 * @return
	 */
	bool IsConnected() { return nfcam_->connected; }
	/*!
	 * @brief 软重启相机
	 * @return
	 * 相机重启操作结果
	 */
	virtual bool Reboot();
	/*!
	 * @brief 尝试启动曝光流程
	 * @param duration  曝光周期, 量纲: 秒
	 * @param light     是否需要外界光源
	 * @return
	 * 曝光启动结果
	 */
	bool Expose(double duration, bool light = true);
	/*!
	 * @brief 中止当前曝光过程
	 */
	void AbortExpose();
	/*!
	 * @brief 设置制冷温度与制冷模式
	 * @param coolset 制冷温度
	 * @param onoff   是否启动制冷. true: 启动; false: 停止
	 */
	void SetCooler(double coolset, bool onoff = true);
	/*!
	 * @brief 设置读出端口
	 * @param index 读出端口档位
	 */
	void SetReadPort(uint32_t index);
	/*!
	 * @brief 设置读出速度
	 * @param index 读出速度档位
	 */
	void SetReadRate(uint32_t index);
	/*!
	 * @brief 设置增益
	 * @param index 增益档位
	 */
	void SetGain(uint32_t index);
	/*!
	 * @brief 设置ROI区域
	 * @param xstart X轴起始位置
	 * @param ystart Y轴起始位置
	 * @param width  宽度
	 * @param height 高度
	 * @param xbin   X轴合并因子
	 * @param ybin   Y轴合并因子
	 */
	void SetROI(int xstart = 1, int ystart = 1, int width = -1, int height = -1, int xbin = 1, int ybin = 1);
	/*!
	 * @brief 设置本底基准值
	 * @param offset 基准值
	 * @param output 调制过程输出文件
	 * @return
	 * 0 -- 调制成功
	 * 1 -- 不需要调制
	 * 2 -- 调制失败
	 * 3 -- 不满足条件
	 */
	int SetADCOffset(uint16_t offset, FILE *output = NULL);
	/*!
	 * @brief 变更IP地址
	 * @param ip IPv4地址
	 * @return
	 * IP地址修改结果
	 * 0: 修改成功
	 * 1: 不需要更新
	 * 2: 更新失败
	 * 3: 不满足更新条件
	 */
	int SetIP(const char *ip);
	/*!
	 * @brief 变更子网掩码
	 * @param mask 子网掩码
	 * @return
	 * 子网掩码修改结果
	 * 0: 修改成功
	 * 1: 不需要更新
	 * 2: 更新失败
	 * 3: 不满足更新条件
	 */
	int SetNetmask(const char *mask);
	/*!
	 * @brief 变更网关地址
	 * @param gw 网关地址
	 * @return
	 * 网关修改结果
	 * 0: 修改成功
	 * 1: 不需要更新
	 * 2: 更新失败
	 * 3: 不满足更新条件
	 */
	int SetGateway(const char *gw);

protected:
	/* 功能 */
	/*!
	 * @brief 执行相机连接操作
	 * @return
	 * 连接结果
	 */
	virtual bool connect() = 0;
	/*!
	 * @brief 执行相机断开操作
	 */
	virtual void disconnect() = 0;
	/*!
	 * @brief 启动相机曝光流程
	 * @param duration 积分时间, 量纲: 秒
	 * @param light    快门控制模式. true: 自动或常开; false: 常关
	 * @return
	 * 曝光流程启动结果
	 */
	virtual bool start_expose(double duration, bool light) = 0;
	/*!
	 * @brief 中止曝光流程
	 */
	virtual void stop_expose() = 0;
	/*!
	 * @brief 检查相机工作状态
	 * @return
	 * 相机工作状态
	 */
	virtual int check_state() = 0;
	/*!
	 * @brief 从控制器读出图像数据进入内存
	 * @return
	 * 相机工作状态
	 */
	virtual int readout_image() = 0;
	/*!
	 * @brief 采集探测器温度
	 * @return
	 * 探测器温度
	 */
	virtual double sensor_temperature() = 0;
	/*!
	 * @brief 执行相机制冷操作
	 * @param coolset 制冷温度
	 * @param onoff   制冷模式
	 */
	virtual void update_cooler(double &coolset, bool onoff) = 0;
	/*!
	 * @brief 设置读出端口
	 * @param index 档位
	 * @return
	 * 改变后档位
	 */
	virtual uint32_t update_readport(uint32_t index) = 0;
	/*!
	 * @brief 设置读出速度
	 * @param index 档位
	 * @return
	 * 改变后档位
	 */
	virtual uint32_t update_readrate(uint32_t index) = 0;
	/*!
	 * @brief 设置增益
	 * @param index 档位
	 * @return
	 * 改变后档位
	 */
	virtual uint32_t update_gain(uint32_t index) = 0;
	/*!
	 * @brief 设置ROI区域
	 * @param xstart X轴起始位置
	 * @param ystart Y轴起始位置
	 * @param width  宽度
	 * @param height 高度
	 * @param xbin   X轴合并因子
	 * @param ybin   Y轴合并因子
	 */
	virtual void update_roi(int &xstart, int &ystart, int &width, int &height, int &xbin, int &ybin) = 0;
	/*!
	 * @brief 设置偏置电压
	 * @param offset 基准值
	 * @param output 调制过程输出文件
	 * @return
	 * 0 -- 调制成功
	 * 1 -- 不需要调制
	 * 2 -- 调制失败
	 */
	virtual int update_adcoffset(uint16_t offset, FILE *output) = 0;
	/*!
	 * @brief 更新网络配置参数
	 * @param option 参数选项. 1: IP; 2: Netmask; 3: Gateway
	 * @param value  参数
	 * @return
	 * 0: 成功更新
	 * 1: 不需要更新
	 * 2: 更新失败
	 */
	virtual int update_network(int option, const char *value) = 0;

protected:
	/* 线程 */
	/*!
	 * @brief 周期线程, 监测温度等
	 */
	void thread_cycle();
	/*!
	 * @brief 线程, 监测曝光过程
	 */
	void thread_expose();
	/*!
	 * @brief 中断线程
	 * @param thrd 线程句柄
	 */
	void interrupt_thread(threadptr &thrd);
};

#endif /* CAMERABASE_H_ */
