/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#include <ros/ros.h>


#include "advanced_navigation_driver/ntrip_client.h"


NTRIP_Client::NTRIP_Client()
{

}

NTRIP_Client::~NTRIP_Client()
{

}

void NTRIP_Client::run()
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param("server_url", server_url, std::string(""));
	pnh.param("port", port, std::string("2101"));
	pnh.param("mountpoint", mountpoint, std::string(""));

	//pnh.param("mode", mode, "AUTO"); //HTTP, RTSP, NTRIP1, AUTO, UDP

	pnh.param("username", username, std::string(""));
	pnh.param("password", password, std::string(""));

	rtcm_pub = nh.advertise<mavros_msgs::RTCM>("geonav_odom", 10);
	ros::Subscriber nmea_sub = nh.subscribe("NMEA", 1, &NTRIP_Client::nmeaCallback, this); 

	ntrip_args.server = server_url.c_str();
	ntrip_args.port = port.c_str();
	ntrip_args.user = username.c_str();
	ntrip_args.password = password.c_str();
	ntrip_args.nmea = 0;
	ntrip_args.data = 0;
	ntrip_args.bitrate = 0;
	ntrip_args.proxyhost = 0;
	ntrip_args.proxyport = port.c_str();
	ntrip_args.mode = 4; //AUTO
	ntrip_args.initudp = 0;
	ntrip_args.udpport = 0;
	ntrip_args.baud = 0;
	ntrip_args.serdevice = 0;

	int err = ntrip_initialise(&ntrip_args, buf);
	if (err)
	{
		ROS_ERROR("Error init NTRIP Client");
	}


	ros::spin();

}

void NTRIP_Client::nmeaCallback(const nmea_msgs::SentenceConstPtr& msg)
{
	int numbytes = 0;

	ntrip_args.nmea = msg->sentence.c_str();

	int err = ntrip(&ntrip_args, buf, &numbytes);
	if (err)
	{
		ROS_ERROR("Couldnt get RTCM data.");
	}

	mavros_msgs::RTCM rtcm_msg;
	rtcm_msg.header.stamp = ros::Time::now();
	rtcm_msg.data.insert(rtcm_msg.data.end(), &buf[0], &buf[numbytes]);

	
	rtcm_pub.publish(rtcm_msg);

}



int main(int argc, char *argv[]) {
	
	// Set up ROS node //
	ros::init(argc, argv, "ntrip_client_node");

	NTRIP_Client ntrip_client;
	ntrip_client.run();
   
	return 0;
}