/*
 * main.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */
#include "ros/ros.h"
#include "SerialNode.h"
#include "SerialPort.h"

using namespace std;
using namespace XM_SerialNode;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SerialNode");

    SerialNode serialnode_base(ros::NodeHandle(), ros::NodeHandle("~"),"/dev/ttyUSB0");
	
	ros::spin();

	return 0;
}


