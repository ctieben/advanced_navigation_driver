#ifndef AN_NTRIP_CLIENT_H
#define AN_NTRIP_CLIENT_H

#include <ros/ros.h>

#include <nmea_msgs/Sentence.h>
#include <mavros_msgs/RTCM.h>

#include "NTRIP_Client/NTRIP/ntripclient.h"


class NTRIP_Client 
{
private:
    ros::Publisher rtcm_pub;

    void nmeaCallback(const nmea_msgs::SentenceConstPtr& msg);

    std::string server_url;
    std::string port;
    std::string mountpoint;
    std::string username;
    std::string password;
    //std::string mode;

    char buf[1000];

    Args ntrip_args;


public:
    NTRIP_Client();

    ~NTRIP_Client();

    void run();
};


/*

       -s server_url    URL of the NTRIP server
       -m mountpoint    Name of the mountpoint
       -u username      Username of your NTRIP account
       -p password      Password of your NTRIP account 

       GGA
*/

#endif  // AN_NTRIP_CLIENT_H