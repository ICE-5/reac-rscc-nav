#include "udp.h"

using namespace boost::asio;

UDPSend::UDPSend(std::string remote_ip, int port) : remote_port(port)
{
	remote_endpoint = udp::endpoint(ip::address::from_string(remote_ip), remote_port);
	try {
		sock = new udp::socket(io_service, udp::endpoint(udp::v4(), 4000));
	} catch (std::exception& e) {
		ROS_ERROR_STREAM(e.what());
	}
}

void UDPSend::send(std::string str) 
{
	boost::shared_ptr<std::string> message(new std::string(str));
	ROS_DEBUG_STREAM("message:" << str);
    	sock->send_to(boost::asio::buffer(str), remote_endpoint, 0, error);
    	ROS_DEBUG_STREAM("Error code: " << error);
}

