#include "Arm_ikine.h"

Arm_ikine::Arm_ikine(camera *Camera)
{
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
        d4 = 460;   // TODO 得减去60  in mm
        this->_ptr_camera = Camera;
        //换算坐标，因为底座到地面还有一段距离，这段距离要减掉
        float Px, Py, Pz;
        Eigen::Vector3d P;
        P = this->_ptr_camera->returnBaseCoor();
        Px = P(0);
        Py = P(1);
        Pz = P(2) - d1; //减掉基座到地面距离

        //关节三的解析解
        double c3, s3;
        c3 = (Px * Px + Py * Py + Pz * Pz - a2 * a2 - d4 * d4) / (2 * a2 * d4);
        s3 = sqrt(1 - c3 * c3);

        vector<double> q3(2);
        q3[0] = atan2(s3, c3);
        q3[1] = atan2(-s3, c3);

        //关节二的解析解
        //s3前为正号
        vector<double> q2(4);
        q2[0] = atan2(Pz * (a2 + d4 * c3) - d4 * s3 * sqrt(Px * Px + Py * Py), (a2 + d4 * c3) * sqrt(Px * Px + Py * Py) + Pz * d4 * s3);
        q2[1] = atan2(Pz * (a2 + d4 * c3) + d4 * s3 * sqrt(Px * Px + Py * Py), -(a2 + d4 * c3) * sqrt(Px * Px + Py * Py) + Pz * d4 * s3);
        //s3前为负号
        s3 = -s3;
        q2[2] = atan2(Pz * (a2 + d4 * c3) - d4 * s3 * sqrt(Px * Px + Py * Py), (a2 + d4 * c3) * sqrt(Px * Px + Py * Py) + Pz * d4 * s3);
        q2[3] = atan2(Pz * (a2 + d4 * c3) + d4 * s3 * sqrt(Px * Px + Py * Py), -(a2 + d4 * c3) * sqrt(Px * Px + Py * Py) + Pz * d4 * s3);

        //关节一的解析解,atan2是从-180到180,所以要把小于0的转换成另一个象限
        vector<double> q1(2);
        q1[0] = atan2(Py, Px);
        q1[1] = atan2(-Py, -Px);
        for (int i = 0; i < 2; i++)
        {
                if (q1[i] < 0)
                {
                        q1[i] = q1[i] + 2 * pi;
                }
        }
        //输出求解角度，这个是根据文章来的
        Eigen::Matrix<double, 4, 3> q_out;

        q_out << q1[0], q2[0], q3[0],
            q1[0], q2[2], q3[1],
            q1[1], q2[1], q3[0],
            q1[1], q2[3], q3[1]; //弧度制

        //换算成关节角度，这是根据关节坐标系得出的
        Eigen::Matrix<double, 4, 3> q_all;
        double Rad_to_deg = 45.0 / atan(1.0);
        double c = pi / 2;
        Eigen::Matrix<double, 4, 3> q_c; //转换矩阵
        q_c << 0, 0, c,
            0, 0, c,
            0, 0, c,
            0, 0, c;
        q_all = q_out + q_c;
        q_all = q_all * Rad_to_deg; //角度制

        //取小数点后两位
        for (int i = 0; i < q_all.rows(); i++)
        {
                for (int j = 0; j < q_all.cols(); j++)
                {
                        q_all(i, j) = round((q_all(i, j) * 100)) / 100.0;
                }
        }

        //筛选结果
        std::vector<int> ret1, ret2, ret3;

        for (int i = 0; i < 4; i++)
        {
                if (q_all(i, 0) >= 50 && q_all(i, 0) <= 310)
                {
                        ret1.push_back(i);
                }
        }

        for (int i = 0; i < 4; i++)
        {
                if (q_all(i, 1) >= 0 && q_all(i, 1) <= 160) //TODO 得确定最终的角度范围
                {
                        ret2.push_back(i);
                }
        }

        for (int i = 0; i < 4; i++)
        {
                if (q_all(i, 2) >= -70 && q_all(i, 2) <= 90) //TODO 得确定最终的角度范围
                {
                        ret3.push_back(i);
                }
        }

        std::sort(ret1.begin(), ret1.end());
        std::sort(ret2.begin(), ret2.end());
        std::sort(ret3.begin(), ret3.end());
        std::vector<int> nfin, fin;
        std::set_intersection(ret1.begin(), ret1.end(),
                              ret2.begin(), ret2.end(),
                              std::back_inserter(nfin));

        std::sort(nfin.begin(), nfin.end());
        std::set_intersection(nfin.begin(), nfin.end(),
                              ret3.begin(), ret3.end(),
                              std::back_inserter(fin));
        if (fin.size() == 0)
        {
                cout << "This position can't be catched\n"
                     << endl; //也有可能是没有坐标传进来
                this->_ifget = 0;
        }
        else
        {
                this->_q = q_all.row(fin[0]); //角度值
                this->_ifget = 1;
        }
}
bool Arm_ikine::if_get()
{
        return this->_ifget;
}

Arm_ikine::target_pos Arm_ikine::return_ikine()
{
        if (this->_ifget == 1)
        {
                target_pos0.joint1 = _q(0) * 100;
                target_pos0.joint2 = _q(1) * 100;
                target_pos0.joint3 = _q(2) * 100;
                target_pos0.joint4 = 0;
                // cout << _q(0) << " " << _q(1) << " " << _q(2) << endl;
                return target_pos0;
        }
        else
        {
                cout << "This position can't be catched\n"
                     << endl;
                this->_q << 180, 160, -70; //这是初始位置的值
                target_pos0.joint1 = _q(0) * 100;
                target_pos0.joint2 = _q(1) * 100;
                target_pos0.joint3 = _q(2) * 100;
                target_pos0.joint4 = 0;
                return target_pos0;
        }
}

camera *Arm_ikine::return_ptr()
{
        return this->_ptr_camera;
}

Arm_ikine::~Arm_ikine()
{
        delete this->_ptr_camera;
}