/**
  ******************************************************************************
  * @file    udp_client.hpp
  * @author  Alex Liu 
  * @version V1.0.1
  * @date    2020/08/13
  * @brief   sample udp library tib_k331
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef UDP_CLIENT_HPP
#define UDP_CLIENT_HPP

#include <iostream>
#include <string>
#include <boost/asio.hpp>

using namespace std;
using namespace boost::asio;

class udpSendHandler
{
public:
        static string dst_ip_addr;
        static string src_ip_addr;
        static unsigned short dst_port;
        static unsigned short src_port;
#pragma pack(push, 1)
        struct target_pos
        {
                int16_t _joint1;
                int16_t _joint2;
                int16_t _joint3;
                int16_t _joint4;
        } target_pos0;
#pragma pack(pop)

public:
        explicit udpSendHandler()
        {
                udp_socket = new ip::udp::socket(ios);
                dst_addr = new ip::udp::endpoint(ip::address::from_string(dst_ip_addr), dst_port);
                src_addr = new ip::udp::endpoint(ip::address::from_string(src_ip_addr), src_port);
                udp_socket->open(dst_addr->protocol());
                udp_socket->bind(*src_addr); //绑定起来
                size_t frame_length = 13;    //最长的长度
                send_buf = new unsigned char[frame_length];
                send_buf[0] = 0x55;
                send_buf[1] = 0xaa;                         //包头
                for (uint8_t i = 2; i <= frame_length; i++) //初始化
                {
                        send_buf[i] = 0x00;
                }
                frame_length_ = frame_length;
        }
        ~udpSendHandler()
        {
                udp_socket->close();
                delete[] send_buf;
                delete dst_addr;
                delete udp_socket;
        }
        void udp_transmit(int16_t joint1, int16_t joint2, int16_t joint3, int16_t joint4)
        {
                target_pos0._joint1 = joint1; //要初始化吗
                target_pos0._joint2 = joint2;
                target_pos0._joint3 = joint3;
                target_pos0._joint4 = joint4;
                send_buf[2] = 0x09;
                send_buf[3] = 0x01;
                memcpy(&send_buf[4], &target_pos0, 8);

                char check = 0;
                for (int i = 3; i < 12; i++)
                {
                        check += send_buf[i]; //这可能有点问题，得看看到底怎么加的，char有正负
                }
                send_buf[12] = check;

                udp_socket->send_to(buffer(send_buf, frame_length_), *dst_addr, 0, ignore_error);
        }

private:
        io_service ios;
        ip::udp::socket *udp_socket;
        ip::udp::endpoint *dst_addr, *src_addr;
        boost::system::error_code ignore_error;

        unsigned char *send_buf;
        size_t frame_length_;
};

#endif