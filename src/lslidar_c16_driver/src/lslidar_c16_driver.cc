/*
 * This file is part of lslidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <lslidar_c16_driver/lslidar_c16_driver.h>

#define PAUSE printf("Press Enter key to continue..."); fgetc(stdin);

namespace lslidar_c16_driver {

LslidarC16Driver::LslidarC16Driver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    socket_id(-1){
    return;
}

LslidarC16Driver::~LslidarC16Driver() {
    (void) close(socket_id);
    return;
}

bool LslidarC16Driver::loadParameters() {

    //pnh.param("frame_id", frame_id, std::string("lslidar"));
    pnh.param("start_time", start_time, 0.0);
    pnh.param("last_time", last_time, 86400.0);
    std::cout<<"start_time : " << start_time<<" , end_time = "<<last_time<<std::endl;
    pnh.param("pcap_file_path", pcap_file_path, std::string("/home/lyh/lab/data/iking0307/laser2020-3-7-15-31-12.pcap"));
    std::cout<<"pcap_file_path : "<<pcap_file_path<<std::endl;

    pnh.param("lidar_ip", lidar_ip_string, std::string("192.168.1.222"));
    pnh.param<int>("device_port", UDP_PORT_NUMBER,2368);
    pnh.param<bool>("add_multicast", add_multicast, false);
    pnh.param("group_ip", group_ip_string, std::string("234.2.3.2"));
    inet_aton(lidar_ip_string.c_str(), &lidar_ip);
    ROS_INFO_STREAM("Opening UDP socket: address " << lidar_ip_string);
    if(add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    return true;
}

bool LslidarC16Driver::createRosIO() {

    // ROS diagnostics
    diagnostics.setHardwareID("Lslidar_C16");
    // c16 publishs 20*16 thousands points per second.
    // Each packet contains 12 blocks. And each block
    // contains 32 points. Together provides the
    // packet rate.
    const double diag_freq = 16*20000.0 / (12*32);
    diag_max_freq = diag_freq;
    diag_min_freq = diag_freq;
    ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic.reset(new TopicDiagnostic(
                         "lslidar_packets", diagnostics,
                         FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
                         TimeStampStatusParam()));

    // Output
    packet_pub = nh.advertise<lslidar_c16_msgs::LslidarC16Packet>(
                "lslidar_packet", 10000000);
    return true;
}

bool LslidarC16Driver::openUDPPort() {
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) {
        perror("socket");
        return false;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(UDP_PORT_NUMBER);      // short, in network byte order
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: ROS_ERROR errno
        return false;
    }
    //add multicast
    if(add_multicast){
        ip_mreq groupcast;
        groupcast.imr_interface.s_addr=INADDR_ANY;
        groupcast.imr_multiaddr.s_addr=inet_addr(group_ip_string.c_str());

        if(setsockopt(socket_id,IPPROTO_IP,IP_ADD_MEMBERSHIP,(char*)&groupcast,sizeof(groupcast))<0) {
            perror("set multicast error");
            close(socket_id);
            return false;
        }
    }
    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block");
        return false;
    }

    return true;
}

bool LslidarC16Driver::openSavedPcap()
{
    char errbuf[100];
    this->pfile = pcap_open_offline(pcap_file_path.c_str(), errbuf);
    printf("pcap_file = %s",pcap_file_path.c_str());

    if (NULL == pfile)
    {
        printf("%s\n", errbuf);
        std::cout<<"pcap_file_path : "<<pcap_file_path<<std::endl;
        return false;
    }
    return true;
}

bool LslidarC16Driver::initialize() {

    this->initTimeStamp();

    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create all ROS IO...");
        return false;
    }

    if (!openUDPPort()) {
        ROS_ERROR("Cannot open UDP port...");
        return false;
    }

    if(!openSavedPcap())
    {
        ROS_ERROR("Cannot open Saved pcap file...");
        return false;
    }

    ROS_INFO("Initialised lslidar c16 without error");
    return true;


}

int LslidarC16Driver::getPacket(lslidar_c16_msgs::LslidarC16PacketPtr& packet)
{
    double time1 = ros::Time::now().toSec();
//    double start_time = 0;
//    double last_time = 86400;

    bool not_start = true;
    bool ended = false;

    //fill the packet with 1206 byte lidar raw data
    pcap_pkthdr *pkthdr = 0;
    const u_char *pktdata = 0;

    //play speed
    ros::Rate r(100000);
    if (pcap_next_ex(pfile, &pkthdr, &pktdata) != 1)
    {
//        ROS_WARN("Reach the end of pcap file");
        return 1;
    }

//    if(this->timeStamp.sec > start_time && this->timeStamp.sec < last_time)
//    {
//        not_start = false;
//        r.sleep();
//    }
//    else
//    {
//        not_start = true;
//    }

    if(this->timeStamp.sec > start_time)
    {
        not_start = false;
        r.sleep();
        if(this->timeStamp.sec > last_time)
        {
            ended = true;
        }
    }


    memcpy(&packet->data[0],&pktdata[42],1206);

    this->getFPGA_GPSTimeStamp(packet);

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred.
    double time2 = ros::Time::now().toSec();
    //    packet->stamp = ros::Time((time2 + time1) / 2.0);
    packet->stamp = this->timeStamp;

    if(ended)
        return -1;
    if(not_start)
        return 1;
    return 0;
}

bool LslidarC16Driver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lslidar_c16_msgs::LslidarC16PacketPtr packet(
                new lslidar_c16_msgs::LslidarC16Packet());

    // Since the lslidar delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    //for (int i = 0; i < config_.npackets; ++i)
    //  {
    //    while (true)
    //      {
    //        // keep reading until full packet received
    //        int rc = input_->getPacket(&scan->packets[i]);
    //        if (rc == 0) break;       // got a full packet?
    //        if (rc < 0) return false; // end of file reached?
    //      }
    //  }
    int rc = 0;
    while (true)
    {
        // keep reading until full packet received
        rc = getPacket(packet);
        if (rc == 0) break;       // got a full packet?

        if (rc < 0)
        {
            PAUSE;
            return false;
        } // end of file reached?
    }

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full lslidar scan.");
    packet_pub.publish(*packet);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic->tick(packet->stamp);
    diagnostics.update();

    return true;
}

void LslidarC16Driver::initTimeStamp(void)
{
    int i;

    for(i = 0;i < 10;i ++)
    {
        this->packetTimeStamp[i] = 0;
    }
    this->pointcloudTimeStamp = 0;

    this->timeStamp = ros::Time(0.0);
}

void LslidarC16Driver::getFPGA_GPSTimeStamp(lslidar_c16_msgs::LslidarC16PacketPtr &packet)
{
    unsigned char head2[] = {packet->data[0],packet->data[1],packet->data[2],packet->data[3]};

    if(head2[0] == 0xA5 && head2[1] == 0xFF)
    {
        if(head2[2] == 0x00 && head2[3] == 0x5A)
        {
            this->packetTimeStamp[4] = packet->data[41];
            this->packetTimeStamp[5] = packet->data[40];
            this->packetTimeStamp[6] = packet->data[39];
            this->packetTimeStamp[7] = packet->data[38];
            this->packetTimeStamp[8] = packet->data[37];
            this->packetTimeStamp[9] = packet->data[36];

            cur_time.tm_sec = this->packetTimeStamp[4];
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8]-1;
            cur_time.tm_year = this->packetTimeStamp[9]+2000-1900;
            this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time));
            this->timeStamp.sec = (cur_time.tm_hour*60+cur_time.tm_min)*60+cur_time.tm_sec;

            if (GPSCountingTS != this->pointcloudTimeStamp)
            {
                cnt_gps_ts = 0;
                GPSCountingTS = this->pointcloudTimeStamp;
            }
            else if (cnt_gps_ts == 3)
            {
                GPSStableTS = GPSCountingTS;
            }
            else
            {
                cnt_gps_ts ++;
            }
            //            ROS_DEBUG("GPS: y:%d m:%d d:%d h:%d m:%d s:%d",
            //                      cur_time.tm_year,cur_time.tm_mon,cur_time.tm_mday,cur_time.tm_hour,cur_time.tm_min,cur_time.tm_sec);
        }
    }
    else if(head2[0] == 0xFF && head2[1] == 0xEE)
    {
        uint64_t packet_timestamp;
        packet_timestamp = (packet->data[1200]  +
                packet->data[1201] * pow(2, 8) +
                packet->data[1202] * pow(2, 16) +
                packet->data[1203] * pow(2, 24)) * 1e3;


        if ((last_FPGA_ts - packet_timestamp) > 0)
        {
            GPS_ts = GPSStableTS;

            // ROS_DEBUG("This is step time, using new GPS ts %lu", GPS_ts);
        }

        last_FPGA_ts = packet_timestamp;
        // timeStamp = ros::Time(this->pointcloudTimeStamp+total_us/10e5);

        timeStamp.nsec = packet_timestamp;
        //        timeStamp = ros::Time(GPS_ts, packet_timestamp);
        //        ROS_DEBUG("ROS TS: %f, GPS: y:%d m:%d d:%d h:%d m:%d s:%d; FPGA: us:%lu",
        //                  timeStamp.toSec(), GPS_ts, packet_timestamp);

    }
}

} // namespace lslidar_driver
