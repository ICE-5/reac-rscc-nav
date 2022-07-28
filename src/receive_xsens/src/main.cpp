/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include "xcommunication/legacydatapacket.h"
#include "xcommunication/int_xsdatapacket.h"
#include "xcommunication/enumerateusbdevices.h"

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "conio.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "udp.h"
#include "timeoutserial.h"

void makeRotMatrix(cv::Mat* mat, double roll, double pitch, double yaw)
{
	mat->at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
	mat->at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
	mat->at<double>(0, 2) = cos(pitch)*sin(yaw);
	mat->at<double>(1, 0) = cos(pitch)*sin(roll);
	mat->at<double>(1, 1) = cos(pitch)*cos(roll);
	mat->at<double>(1, 2) = -sin(pitch);
	mat->at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
	mat->at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
	mat->at<double>(2, 2) = cos(pitch)*cos(yaw);
}

void makeTimeString(int computerSec, int dayOffset, char* buffer, char* nmea)
{
	int day = int(computerSec / 86400) - dayOffset + 1;
	int hour = int(computerSec / 3600) % 24;
	int minute = int(computerSec / 60) % 60;
	int second = computerSec % 60;

	char checksum = 0;
	int length = sprintf (buffer, "GPRMC,%02i%02i%02i,A,0000,00,N,00000,00,W,000.0,000.0,%02i0115,000.0,E", hour, minute, second, day);
	for (int i = 0; i < length; i++){
		checksum = char(checksum^buffer[i]);
	}
	sprintf (nmea, "$%s*%02x\r\n", buffer, checksum);

	//ROS_INFO ("%d %d %d %d %d", computerSec, day, hour, minute, second);
}

//--------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "receive_xsens");
	ros::NodeHandle nh;
  	ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  	ros::Publisher publisher = nh.advertise<sensor_msgs::Imu> ("/imu/data", 1);

	sensor_msgs::Imu imuData;
	imuData.header.frame_id = "imu";

  	std::string imu_port;
  	nhPrivate.param<std::string>("imu_port", imu_port, "/dev/ttyUSB0");
  	int imu_baudrate;
  	nhPrivate.param<int>("imu_baudrate", imu_baudrate, 115200);
  	bool use_ethernet;
  	nhPrivate.param<bool>("use_ethernet", use_ethernet, "true");
  	std::string velodyne_ip;
  	nhPrivate.param<std::string>("velodyne_ip", velodyne_ip, "192.168.1.201");
  	int velodyne_port;
  	nhPrivate.param<int>("velodyne_port", velodyne_port, 10110);
	int nmea_baudrate;
  	nhPrivate.param<int>("nmea_baudrate", nmea_baudrate, 9600);
  	std::string nmea_port;
  	nhPrivate.param<std::string>("nmea_port", nmea_port, "/dev/ttyUSB1");

        double laser_wrt_imu_yaw;
  	nhPrivate.param<double>("laser_wrt_imu_yaw", laser_wrt_imu_yaw, 0);
        double laser_wrt_imu_pitch;
  	nhPrivate.param<double>("laser_wrt_imu_pitch", laser_wrt_imu_pitch, 0);
        double laser_wrt_imu_roll;
  	nhPrivate.param<double>("laser_wrt_imu_roll", laser_wrt_imu_roll, 0);

        cv::Mat mat_calib(3, 3, CV_64F, cv::Scalar::all(0));
        cv::Mat mat_ori(3, 3, CV_64F, cv::Scalar::all(0));
        cv::Mat mat_gyro(1, 3, CV_64F, cv::Scalar::all(0));
        cv::Mat mat_acc(1, 3, CV_64F, cv::Scalar::all(0));

        bool not_use_calib = true;
	if (laser_wrt_imu_roll != 0 || laser_wrt_imu_pitch != 0 || laser_wrt_imu_yaw != 0) {
		makeRotMatrix(&mat_calib, laser_wrt_imu_roll, laser_wrt_imu_pitch, laser_wrt_imu_yaw);
		not_use_calib = false;
	}

	bool imuInited = false;
	double timeDiff = 0;
	int dayOffset = 0;
	char buffer[66], nmea[66];

  	TimeoutSerial* serial;
	UDPSend* udp;

	if (use_ethernet) {
		udp = new UDPSend(velodyne_ip, velodyne_port);
	} else {
  		serial = new TimeoutSerial(nmea_port, nmea_baudrate);
		serial->setTimeout(boost::posix_time::seconds(5));  	
	}

	DeviceClass device;

	try
	{
		// Scan for connected USB devices
		XsPortInfo mtPort(imu_port, XsBaud::numericToRate(imu_baudrate));

		// Open the port with the detected device
		if (!device.openPort(mtPort))
			throw std::runtime_error("Could not open port. Aborting.");

		// Put the device in configuration mode
		if (!device.gotoConfig())
		{
			throw std::runtime_error("Could not put device into configuration mode. Aborting.");
		}

		// Request the device Id to check the device type
		mtPort.setDeviceId(device.getDeviceId());

		// Check if we have an MTi / MTx / MTmk4 device
		if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isLegacyMtig() && !mtPort.deviceId().isMtMk4() && !mtPort.deviceId().isFmt_X000())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

		// Print information about detected MTi / MTx / MTmk4 device
		std::cout << "Device: " << device.getProductCode().toStdString() << " opened." << std::endl;

		// Reset the device orientation
		if (!device.resetOrientation())
		{
			throw std::runtime_error("Could not reset device orientation. Aborting.");
		}

		// Put the device in measurement mode
		if (!device.gotoMeasurement())
		{
			throw std::runtime_error("Could not put device into measurement mode. Aborting.");
		}

		try
		{
			XsByteArray data;
			XsMessageArray msgs;
			while (ros::ok())
			{
				device.readDataToBuffer(data);
				device.processBufferedData(data, msgs);
				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
				{
					// Retrieve a packet
					XsDataPacket packet;
					if ((*it).getMessageId() == XMID_MtData) {
						LegacyDataPacket lpacket(1, false);
						lpacket.setMessage((*it));
						lpacket.setXbusSystem(false);
						lpacket.setDeviceId(mtPort.deviceId(), 0);
						lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);	//lint !e534
						XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
					}
					else if ((*it).getMessageId() == XMID_MtData2) {
						packet.setMessage((*it));
						packet.setDeviceId(mtPort.deviceId());
					}

					if (packet.containsStatus()) {
						if (packet.containsSampleTime64() && (packet.status() & 4194304) != 0) {
							double computerTime = ros::Time::now().toSec();
							if (!imuInited) {
                                                                int computerSecInit = int(computerTime);
                                                                timeDiff = computerSecInit - packet.sampleTime64() / 10000.0;
                                				dayOffset = int(computerSecInit / 86400);
								imuInited = true;
                            				}

							int computerSec = int(timeDiff + packet.sampleTime64() / 10000.0 + 0.5);

					            	makeTimeString(computerSec, dayOffset, buffer, nmea);

							if (use_ethernet) {
					            		udp->send(std::string(nmea));
							} else {
                                				serial->writeString(std::string(nmea));
                            				}
                        			}

						if (packet.containsOrientation()) {
							XsQuaternion quaternion = packet.orientationQuaternion();

							if (not_use_calib) {
								imuData.orientation.x = quaternion.x();
								imuData.orientation.y = quaternion.y();
								imuData.orientation.z = quaternion.z();
								imuData.orientation.w = quaternion.w();
							} else {
								double roll, pitch, yaw;
								tf::Matrix3x3(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w())).getRPY(roll, pitch, yaw);
        							makeRotMatrix(&mat_ori, roll, pitch, yaw);
								cv::Mat mat_ori_rot = mat_ori * mat_calib;

								pitch = -asin(mat_ori_rot.at<double>(1, 2));
								roll = atan2(mat_ori_rot.at<double>(1, 0) / cos(pitch), mat_ori_rot.at<double>(1, 1) / cos(pitch));
								yaw = atan2(mat_ori_rot.at<double>(0, 2) / cos(pitch), mat_ori_rot.at<double>(2, 2) / cos(pitch));

								geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

								imuData.orientation.x = quat.x;
								imuData.orientation.y = quat.y;
								imuData.orientation.z = quat.z;
								imuData.orientation.w = quat.w;
							}
						}

						if (packet.containsCalibratedGyroscopeData()) {
							XsVector gyroscope = packet.calibratedGyroscopeData();

							if (not_use_calib) {
								imuData.angular_velocity.x = gyroscope.at(0);
								imuData.angular_velocity.y = gyroscope.at(1);
								imuData.angular_velocity.z = gyroscope.at(2);
							} else {
								mat_gyro.at<double>(0, 0) = gyroscope.at(1);
								mat_gyro.at<double>(0, 1) = gyroscope.at(2);
								mat_gyro.at<double>(0, 2) = gyroscope.at(0);
								cv::Mat mat_gyro_rot = mat_gyro * mat_calib;

								imuData.angular_velocity.x = mat_gyro_rot.at<double>(0, 2);
								imuData.angular_velocity.y = mat_gyro_rot.at<double>(0, 0);
								imuData.angular_velocity.z = mat_gyro_rot.at<double>(0, 1);
							}
						}

						if (packet.containsCalibratedAcceleration()) {
							XsVector acceleration = packet.calibratedAcceleration();

							if (not_use_calib) {
								imuData.linear_acceleration.x = acceleration.at(0);
								imuData.linear_acceleration.y = acceleration.at(1);
								imuData.linear_acceleration.z = acceleration.at(2);
							} else {
								mat_acc.at<double>(0, 0) = acceleration.at(1);
								mat_acc.at<double>(0, 1) = acceleration.at(2);
								mat_acc.at<double>(0, 2) = acceleration.at(0);
								cv::Mat mat_acc_rot = mat_acc * mat_calib;

								imuData.linear_acceleration.x = mat_acc_rot.at<double>(0, 2);
								imuData.linear_acceleration.y = mat_acc_rot.at<double>(0, 0);
								imuData.linear_acceleration.z = mat_acc_rot.at<double>(0, 1);
							}
						}

						if (packet.containsSampleTime64() && imuInited) {
							imuData.header.stamp = ros::Time().fromSec(timeDiff + packet.sampleTime64() / 10000.0);
							publisher.publish(imuData);
						}
					}
				}

    				ros::spinOnce();
				msgs.clear();
				XsTime::msleep(1.0);
			}
		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}

		// Close port
		std::cout << "Closing port..." << std::endl;
		device.close();
	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}

	std::cout << "Successful exit." << std::endl;

	return 0;
}
