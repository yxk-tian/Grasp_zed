#include "Arm_ikine.h"
#include "camera.h"
#include "udp_client.hpp"
#include "mydetector.hpp"

string udpSendHandler::dst_ip_addr = "192.168.0.113";
string udpSendHandler::src_ip_addr = "192.168.0.111";
unsigned short udpSendHandler::dst_port = 8080;
unsigned short udpSendHandler::src_port = 8081;

int main()
{
        PanTiltDetector target_detector;

        target_detector.FrameProduce();
        target_detector.DetectRealize();
        double yaw_ik, pitch_ik, depth_ik;
        target_detector.getangle(yaw_ik, pitch_ik, depth_ik);
        yaw_ik = -yaw_ik;
        depth_ik = depth_ik * 1000;
        // cout << yaw_ik << " " << pitch_ik << " " << depth_ik << endl;
        Arm_ikine arm_zero(new camera(yaw_ik, pitch_ik, depth_ik)); //输入两个角度和一个深度值in mm

        cout << "ikine:" << arm_zero.return_ikine().joint1 << ' ' << arm_zero.return_ikine().joint2 << ' '
             << arm_zero.return_ikine().joint3 << ' ' << arm_zero.return_ikine().joint4 << endl;
        // udpSendHandler target;
        // target.udp_transmit(arm_zero.return_ikine().joint1, arm_zero.return_ikine().joint2, arm_zero.return_ikine().joint3, arm_zero.return_ikine().joint4);
        // system("pause");
        return 0;
}