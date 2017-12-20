/*!
 * @file udp_asio.h 基于boost::asio封装UDP通信
 * @version 0.1
 * @date May 23, 2017
 * @note
 * - 基于boost::asio封装实现UDP通信服务器和客户端
 */

#ifndef UDPASIO_H_
#define UDPASIO_H_

#include <boost/signals2.hpp>
#include "IOServiceKeep.h"

//////////////////////////////////////////////////////////////////////////////
#define UDP_PACK_SIZE	1500

using boost::asio::ip::udp;
using boost::system::error_code;

class UDPSession {
public:
	/*!
	 * @brief 构造函数
	 * @param portLoc 本地UDP端口. 0代表由系统分配
	 */
	UDPSession(const uint16_t portLoc = 0);
	virtual ~UDPSession();

public:
	// 数据类型
	typedef boost::signals2::signal<void (const long, const long)> CallbackFunc;	//< 回调函数
	typedef CallbackFunc::slot_type CBSlot;		//< 回调函数插槽

protected:
	// 数据类型
	typedef boost::shared_ptr<udp::socket> sockptr;
	typedef boost::unique_lock<boost::mutex> mutex_lock;
	typedef boost::shared_array<char> carray;	//< 字符型数组

protected:
	// 成员变量
	IOServiceKeep keep_;		//< 维持boost::asio::io_service对象有效
	sockptr sock_;			//< UDP套接口
	bool connected_;			//< 是否面向连接
	udp::endpoint remote_;	//< 对应远程端点地址
	boost::condition_variable cvread_;	//< 阻塞读出时的条件变量

	int bytercv_;			//< 收到的字节数
	CallbackFunc cbconn_;	//< 连接回调函数
	CallbackFunc cbrcv_;		//< 接收回调函数
	CallbackFunc cbsnd_;		//< 发送回调函数
	carray bufrcv_;			//< 接收缓存区
	boost::mutex mtxrcv_;	//< 接收互斥锁
	boost::mutex mtxsnd_;	//< 发送互斥锁

public:
	/*!
	 * @brief 设置远程主机
	 * @param ip     远程主机IP地址
	 * @param port   远程主机UDP端口
	 * @return
	 * 若套接口上已建立连接则返回false， 否则返回true
	 * @note
	 * 后续通信发往远程主机
	 */
	void Connect(const char *ip, const uint16_t port);
	/*!
	 * @brief 关闭套接口
	 */
	void Close();
	/*!
	 * @brief 检查套接口是否已经打开
	 * @return
	 * 套接口打开标识
	 */
	bool IsOpen();
	/*!
	 * @brief 读取已接收数据
	 * @param n 数据长度, 量纲: 字节
	 * @return
	 * 存储数据缓冲区地址
	 */
	const char *Read(int &n);
	/*!
	 * @brief 延时读取已接收数据
	 * @param n 数据长度, 量纲: 字节
	 * @return
	 * 存储数据缓冲区地址
	 */
	const char *BlockRead(int &n);
	/*!
	 * @brief 将数据写入套接口
	 * @param data 待发送数据
	 * @param n    待发送数据长度, 量纲: 字节
	 * @return
	 * 操作结果
	 */
	void Write(const void *data, const int n);
	/*!
	 * @brief 注册异步连接回调函数
	 * @param slot 插槽
	 */
	void RegisterConnect(const CBSlot &slot);
	/*!
	 * @brief 注册异步接收回调函数
	 * @param slot 插槽
	 */
	void RegisterRead(const CBSlot &slot);
	/*!
	 * @brief 注册异步发送回调函数
	 * @param slot 插槽
	 */
	void RegisterWrite(const CBSlot &slot);

protected:
	/*!
	 * @brief 启动异步信息接收
	 */
	void start_read();
	/*!
	 * @breif 处理连接结果
	 * @param ec 错误代码
	 */
	void handle_connect(const error_code& ec);
	/*!
	 * @brief 处理收到的网络信息
	 * @param ec 错误代码
	 * @param n  接收数据长度, 量纲: 字节
	 */
	void handle_read(const error_code& ec, const int n);
	/*!
	 * @brief 处理异步网络信息发送结果
	 * @param ec 错误代码
	 * @param n  发送数据长度, 量纲: 字节
	 */
	void handle_write(const error_code& ec, const int n);
};
typedef boost::shared_ptr<UDPSession> UdpPtr;
/*!
 * @brief 工厂函数, 创建UDP客户端指针
 * @return
 * 基于UDPClient的指针
 */
extern UdpPtr makeudp_session(uint16_t port = 0);

#endif
