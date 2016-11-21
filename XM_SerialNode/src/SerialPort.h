/*
 * SerialPort.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <inttypes.h>
#include <vector>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include "XM_msgs/XM_Datagram.h"
#include "XM_msgs/UInt8Array.h"

using namespace std;
using namespace boost;
using namespace boost::asio;

namespace XM_SerialNode {

// 串口通信选项
class SerialParams {
public:
	string serialPort; 					// 串口的设备文件
	unsigned int baudRate; 				// 波特率
	unsigned int flowControl; 			// 流控
	unsigned int parity; 				// 校验位
	unsigned int stopBits; 				// 停止位
	SerialParams() :
			serialPort(), baudRate(115200), flowControl(0), parity(0), stopBits(0)
	{
	}
	SerialParams(
			string _serialPort,
			unsigned int _baudRate,
			unsigned int _flowControl,
			unsigned int _parity,
			unsigned int _stopBits
			) :
			serialPort(_serialPort),
			baudRate(_baudRate),
			flowControl(_flowControl),
			parity(_parity),
			stopBits(_stopBits)
	{
	}
};

typedef vector<uint8_t> ByteVector;
typedef shared_ptr<ByteVector> pByteVector;

class SerialPort {
private:
	shared_ptr<deadline_timer>	m_ptimer;		// 超时定时器
	shared_ptr<io_service> 	m_pios;				// io_service对象
	shared_ptr<serial_port>	m_pSerial;			// 串口对象的指针
	mutex 					m_serialMutex;		// 串口对象的互斥锁. 按照boost官方文档, serial_port对象不是线程安全的. 故需要此锁

	enum {HEADER_LEN = 4};

	enum STATE {
		WAITING_FF, WAITING_FF2, READING_HEAD, READING_DATA, READING_CHECKSUM
	} m_state;								// 程序工作状态

	SerialParams	m_serialParams; 		// 串口的配置数据
	int				m_timeOut; 				// 数据报超时时间

	ByteVector		m_tempBuf;				// 数据读取的临时缓冲区

	ByteVector 		m_currentHeader;		// 正在读取的报头(4字节)
	size_t 			m_HeaderBytesRead;		// 报头已经读取的字节数

	ByteVector 		m_currentData;			// 正在读取的报文数据
	size_t 			m_DataBytesRead;		// 数据已经读取的字节数

	queue<pByteVector>	m_writeQueue;		// 待发送数据的队列
	mutex									m_writeQueueMutex;	// 队列的互斥锁

	function<void(XM_msgs::XM_DatagramPtr)> m_dataCallbackFunc;		// 数据回调函数
	function<void()> m_errorCallbackFunc;	// 错误回调函数

	// 跑io_service::run()的线程
	boost::thread m_thread;

	// 线程的主过程, 主要是在跑io_service::run()
	void mainRun();

	// 为了方便写的函数
	void start_a_read();
	void start_a_write();

	// async_read_some的Handler
	void readHandler(const system::error_code &ec, size_t bytesTransferred);
	// async_write_some的Handler
	void writeHandler(const system::error_code &ec);
	// 超时定时器的Handler
	void timeoutHandler(const system::error_code &ec);

public:
	SerialPort();
	virtual ~SerialPort();

	void setSerialParams(const SerialParams &params);		// 设置串口参数

	void setTimeOut(int timeout);							// 设置超时时间

	bool startThread();										// 启动线程
	bool stopThread();										// 停止线程

	// 设置收到数据之后的回调函数
	void setCallbackFunc(const function<void(XM_msgs::XM_DatagramPtr)> &func);

	// 向串口中发送一个数据报文
	bool writeDatagram(const XM_msgs::XM_Datagram &datagram);
	// 向串口中直接写入一串数据
	bool writeRaw(const ByteVector &rawData);
};

} /* namespace XM_SerialNode */

#endif /* SERIALPORT_H_ */
