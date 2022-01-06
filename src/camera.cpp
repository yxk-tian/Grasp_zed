#include "camera.h"
Eigen::Matrix4d kine(vector<double> degtheta);
camera::camera(double a, double b, double c)
{
        //输入的是两个偏角，和一个深度值，1是左右偏角，2是上下，3是深度值
        vector<double> P{a, b, c};
        this->_theta1 = P[0];
        this->_theta2 = P[1];
        this->_depth = P[2];
        Eigen::Matrix4d T04, T45, T05;
        const float pi = 3.14159265359;
        T45 << 1, 0, 0, 50.5,
            0, 1, 0, -31.5,
            0, 0, 1, 15.25,
            0, 0, 0, 1; //TODO 改一下xyz坐标，摄像机相对爪子坐标系的变换矩阵，这个除非是标定给改了，否则不随角度变化
        vector<double> q0 = {180, 160, -70, 0};
        T04 = kine(q0);
        T05 = T45 * T04;
        double theta1, theta2;
        theta1 = this->_theta1 * pi / 180; //转换成弧度值
        theta2 = this->_theta2 * pi / 180;
        this->cz = sqrt(1 / (1 + tan(theta1) * tan(theta1) + tan(theta2) * tan(theta2))) * this->_depth;
        this->cx = this->cz * tan(theta2);
        this->cy = -this->cz * tan(theta1);
        Eigen::Vector4d U(this->cx, this->cy, this->cz, 1);
        Eigen::Vector4d C;
        C = T05 * U;
        this->x = C(0);
        this->y = C(1);
        this->z = C(2);
}
void camera::print_base_Coordinate()
{
        cout << "Base coordinate "
             << "x=" << this->x << "y=" << this->y << "z=" << this->z << endl;
}
void camera::print_camera_Coordinate()
{
        cout << "camera coordinate "
             << "cx=" << this->cx << "cy=" << this->cy << "cz=" << this->cz;
}

Eigen::Vector3d camera::returnBaseCoor()
{
        Eigen::Vector3d BaseCoor(this->x, this->y, this->z);
        return BaseCoor;
}

Eigen::Matrix4d kine(vector<double> degtheta)
{
        //输入的是角度值
        const float pi = 3.14159265359;
        //杆1的dh参数
        float alpha1, a1, d1, theta1;
        alpha1 = 90; // in degrees
        a1 = 0;      // in mm
        d1 = 317;    // in mm
        //杆2的dh参数
        float alpha2, a2, d2, theta2;
        alpha2 = 0; // in degrees
        a2 = 400;   // in mm //TODO 要重新规划一下末端坐标所在位置
        d2 = 0;     // in mm

        //杆3的dh参数
        float alpha3, a3, d3, theta3;
        alpha3 = 90; // in degrees
        a3 = 0;      // in mm
        d3 = 0;      // in mm

        //杆4的dh参数
        float alpha4, a4, d4, theta4;
        alpha4 = 0; // in degrees
        a4 = 0;     // in mm
        d4 = 460;   //TODO减去了60 in mm
        //转换成弧度制
        theta1 = degtheta[0] * pi / 180;
        theta2 = degtheta[1] * pi / 180;
        theta3 = degtheta[2] * pi / 180;
        theta4 = degtheta[3] * pi / 180;
        //转换成弧度制
        alpha1 = alpha1 * pi / 180;
        alpha2 = alpha2 * pi / 180;
        alpha3 = alpha3 * pi / 180;
        alpha4 = alpha4 * pi / 180;

        Eigen::Matrix4d T, T01, T12, T23, T34;
        T01 << cos(theta1), -sin(theta1) * cos(alpha1), sin(theta1) * sin(alpha1), a1 * cos(theta1),
            sin(theta1), cos(theta1) * cos(alpha1), -cos(theta1) * sin(alpha1), a1 * sin(theta1),
            0, sin(alpha1), cos(alpha1), d1,
            0, 0, 0, 1;
        T12 << cos(theta2), -sin(theta2) * cos(alpha2), sin(theta2) * sin(alpha2), a2 * cos(theta2),
            sin(theta2), cos(theta2) * cos(alpha2), -cos(theta2) * sin(alpha2), a2 * sin(theta2),
            0, sin(alpha2), cos(alpha2), d2,
            0, 0, 0, 1;
        T23 << cos(theta3), -sin(theta3) * cos(alpha3), sin(theta3) * sin(alpha3), a3 * cos(theta3),
            sin(theta3), cos(theta3) * cos(alpha3), -cos(theta3) * sin(alpha3), a3 * sin(theta3),
            0, sin(alpha3), cos(alpha3), d3,
            0, 0, 0, 1;
        T34 << cos(theta4), -sin(theta4) * cos(alpha4), sin(theta4) * sin(alpha4), a4 * cos(theta4),
            sin(theta4), cos(theta4) * cos(alpha4), -cos(theta4) * sin(alpha4), a4 * sin(theta4),
            0, sin(alpha4), cos(alpha4), d4,
            0, 0, 0, 1;
        T = T01 * T12 * T23 * T34;
        return T;
}