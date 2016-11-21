/*
 * SerialNode.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#ifndef SERIALNODE_H_
#define SERIALNODE_H_

#include <ros/ros.h>
#include <map>
#include "SerialPort.h"
#include "XM_msgs/XM_Datagram.h"
#include "XM_msgs/UInt8Array.h"

namespace XM_SerialNode {

class SerialNode {
private:
	SerialParams 		m_serialParams; 		// 串口的配置数据
	int 				m_timeOut; 				// 数据报超时时间

	shared_ptr<SerialPort>	m_pSerialPort;

	ros::NodeHandle 	m_node, m_privateNode;	// 当前NodeHandle与私有NodeHandle

	// Note: 按照ROS的文档, ROS提供的对象统统是thread-safe的, 所以不用管互斥了
	ros::Subscriber		m_sub_xmmsg;			// SendSerialData的ROS订阅对象
	ros::Subscriber		m_sub_raw;				// SendSerialData_raw的ROS订阅对象
	ros::Publisher		m_pub_Allrecv;			// AllRecvData的ROS发布对象
	std::map<uint8_t, ros::Publisher>	m_pubsByRecvID;		// 各receiveID对应的ROS发布对象

	void loadParams();

	void xm_Datagram_Callback(const XM_msgs::XM_Datagram::ConstPtr &msg);
	void xm_RawData_Callback(const XM_msgs::UInt8Array::ConstPtr &msg);

	void serialCallback(XM_msgs::XM_DatagramPtr pDatagram);

public:
    SerialNode(const ros::NodeHandle &node, const ros::NodeHandle &prinode,const char* port_name);
	virtual ~SerialNode();
};

} /* namespace XM_SerialNode */
#endif /* SERIALNODE_H_ */
