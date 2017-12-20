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
	uint16_t idmsg_;			//< 信息编号
	threadptr thrdhb_;		//< 心跳线程
	threadptr thrdread_;		//< 读出线程
	boost::condition_variable waitread_;	//< 等待开始读出图像数据
	boost::condition_variable imgrdy_;	//< 等待完成或中止图像读出
	/* 图像数据分包接收 */
	ptime lastdata_;			//< 最后一次收到数据包UTC时间
	uint32_t byteimg_;		//< 图像大小, 量纲: 字节
	uint32_t bytercvd_;		//< 已接收图像大小, 量纲: 字节
	uint32_t packtot_;		//< 图像数据分包总数
    /*!
     * @brief packlen_ 单元包有效数据标准大小
     * - Read(0x0D04, packlen_)获得网络支持的UDP包容量, 该值扣除20字节IP头+8字节UDP头+8字节相机定制头
     * @brief headlen_ 单元包帧头大小
     * @brief packflag_ 标注数据包是否已接收
     * - 第一个单元不使用
     * @note
     * - 相机发送的分包数据, 最后一个多64字节填充数据
     * - 接收单元包数据直接写入相机图像数据存储区: nfcam_->data
     */
    uint32_t packlen_;		//< 数据包大小
    uint32_t headlen_;		//< 定制数据头大小
    boost::shared_array<uint8_t> packflag_;	//< 数据包已接收标记
    uint16_t idfrm_;		//< 图像帧编号
    uint32_t idpack_;	//< 一帧图像中的包编号

public:
	/*!
	 * @brief 软重启相机
	 * @return
	 * 相机重启操作结果
	 */
	bool Reboot();

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
	 * @brief 检查并申请重传丢失数据包
	 */
	void re_transmit();
	/*!
	 * @brief 申请重传指定范围数据包
	 * @param first 数据包起始编号
	 * @param last  数据包结束编号
	 */
	void re_transmit(uint32_t first, uint32_t last);
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
	/*!
	 * @brief 监测线程: 监测图像读出
	 * @note
	 * - 当短时间无读出时, 请求重新读出
	 * - 当长时间无读出时, 触发错误
	 */
	void thread_readout();
};

#endif /* SRC_CAMERAGYCCD_H_ */
