/*
 * @file CameraGYCCD.h GangYu相机封装接口声明文件
 * @version 0.2
 * @date 2017-12-20
 * @author Xiaomeng Lu
 * - 基于UDP向相机发送控制指令
 * - 基于UDP从相机读取图像数据
 */

#ifndef SRC_CAMERAGYCCD_H_
#define SRC_CAMERAGYCCD_H_

#include "CameraBase.h"
#include "udpasio.h"

//////////////////////////////////////////////////////////////////////////////
/// 定义: 相机相关常数
#define PORT_CAMERA		3956		//< 相机指令端口
#define PORT_DATA		49152	//< 本地数据端口

/// 定义: 感光区和overscan区
struct ChannelZone {//< 单区域坐标
	int x1, x2;		//< X坐标最小最大值
	int y1, y2;		//< Y坐标最小最大值
};

struct ChannelOffset {//< 单通道偏置
	int r, g, b;		//< RGB偏置
};

struct GYZONE {//< GY相机区域
	ChannelZone   light[4];		//< 感光区
	ChannelZone   overscan[4];	//< 过扫区
	ChannelOffset offset[4];		//< 偏置值

public:
	GYZONE() {
		/* 感光区 */
		light[0].x1 = 20;
		light[0].y1 = 20;
		light[0].x2 = 2068;
		light[0].y2 = 2068;

		light[1].x1 = 2068;
		light[1].y1 = 20;
		light[1].x2 = 4116;
		light[1].y2 = 2068;

		light[2].x1 = 2068;
		light[2].y1 = 2068;
		light[2].x2 = 4116;
		light[2].y2 = 4116;

		light[3].x1 = 20;
		light[3].y1 = 2068;
		light[3].x2 = 2068;
		light[3].y2 = 4116;

		/* 过扫区 */
		overscan[0].x1 = 4136;
		overscan[0].y1 = 20;
		overscan[0].x2 = 4166;
		overscan[0].y2 = 2068;

		overscan[1].x1 = 4166;
		overscan[1].y1 = 20;
		overscan[1].x2 = 4196;
		overscan[1].y2 = 2068;

		overscan[2].x1 = 4166;
		overscan[2].y1 = 2068;
		overscan[2].x2 = 4196;
		overscan[2].y2 = 4116;

		overscan[3].x1 = 4136;
		overscan[3].y1 = 2068;
		overscan[3].x2 = 4166;
		overscan[3].y2 = 4116;
	}
};
//////////////////////////////////////////////////////////////////////////////

class CameraGYCCD: public CameraBase {
public:
	CameraGYCCD(const char *camIP);
	virtual ~CameraGYCCD();

protected:
	/* 成员变量 */
	string	camip_;		//< 相机IP地址
	UdpPtr	udpcmd_;		//< 网络连接: 指令
	UdpPtr	udpdata_;	//< 网络连接: 数据
	boost::mutex mtxreg_;	//< 互斥锁: 寄存器
	uint32_t gain_;			//< 增益档位
	uint32_t shtr_;			//< 快门模式
	uint32_t expdur_;		//< 曝光时间, 量纲: 微秒
	uint16_t msgid_;			//< 信息编号

protected:
	/* 功能 */
	/*!
	 * @brief 执行相机连接操作
	 * @return
	 * 连接结果
	 */
	virtual bool connect();
	/*!
	 * @brief 执行相机断开操作
	 */
	virtual void disconnect();
	/*!
	 * @brief 启动相机曝光流程
	 * @param duration 积分时间, 量纲: 秒
	 * @param light    快门控制模式. true: 自动或常开; false: 常关
	 * @return
	 * 曝光流程启动结果
	 */
	virtual bool start_expose(double duration, bool light);
	/*!
	 * @brief 中止曝光流程
	 */
	virtual void stop_expose();
	/*!
	 * @brief 检查相机工作状态
	 * @return
	 * 相机工作状态
	 */
	virtual int check_state();
	/*!
	 * @brief 从控制器读出图像数据进入内存
	 * @return
	 * 相机工作状态
	 */
	virtual int readout_image();
	/*!
	 * @brief 采集探测器温度
	 * @return
	 * 探测器温度
	 */
	virtual double sensor_temperature();
	/*!
	 * @brief 执行相机制冷操作
	 * @param coolset 制冷温度
	 * @param onoff   制冷模式
	 */
	virtual void update_cooler(double &coolset, bool onoff);
	/*!
	 * @brief 设置读出端口
	 * @param index 档位
	 * @return
	 * 改变后档位
	 */
	virtual uint32_t update_readport(uint32_t index);
	/*!
	 * @brief 设置读出速度
	 * @param index 档位
	 * @return
	 * 改变后档位
	 */
	virtual uint32_t update_readrate(uint32_t index);
	/*!
	 * @brief 设置增益
	 * @param index 档位
	 * @return
	 * 改变后档位
	 */
	virtual uint32_t update_gain(uint32_t index);
	/*!
	 * @brief 设置ROI区域
	 * @param xstart X轴起始位置
	 * @param ystart Y轴起始位置
	 * @param width  宽度
	 * @param height 高度
	 * @param xbin   X轴合并因子
	 * @param ybin   Y轴合并因子
	 */
	virtual void update_roi(int &xstart, int &ystart, int &width, int &height, int &xbin, int &ybin);
	/*!
	 * @brief 设置偏置电压
	 * @param offset 基准值
	 * @param output 调制过程输出文件
	 * @return
	 * 0 -- 调制成功
	 * 1 -- 不需要调制
	 * 2 -- 调制失败
	 */
	virtual int update_adcoffset(uint16_t offset, FILE *output);

protected:
	/*!
	 * @brief 读取寄存器存储数值
	 * @param addr   寄存器地址
	 * @param value  host顺序数值
	 * @note
	 * 当操作失败时抛出异常
	 */
	void read(uint32_t addr, uint32_t &value);
	/*!
	 * @brief 将数值写入寄存器
	 * @param addr   寄存器地址
	 * @param value  host顺序数值
	 * @note
	 * 当操作失败时抛出异常
	 */
	void write(uint32_t addr, uint32_t value);
	/*!
	 * @brief 逐一递增计算信息编号
	 * @return
	 * 信息编号
	 */
	uint16_t msgid();
	/*!
	 * @brief 区域统计
	 * @param zone  区域坐标
	 * @param mean  平均值
	 * @param rms   标准均方差
	 */
	void stat_zone(ChannelZone *zone, double &mean, double &rms);
	/*!
	 * @brief 周期线程: 心跳机制
	 */
	void thread_heartbeat();
};

#endif /* SRC_CAMERAGYCCD_H_ */
