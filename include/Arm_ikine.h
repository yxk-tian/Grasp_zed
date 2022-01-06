#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "camera.h"
using namespace std;

class Arm_ikine
{
private:
        Eigen::RowVector3d _q; //逆解角度
        bool _ifget;           //1是能到，0是没得到
        camera *_ptr_camera;
        struct target_pos
        {
                int16_t joint1;
                int16_t joint2;
                int16_t joint3;
                int16_t joint4;
        } target_pos0;

public:
        Arm_ikine(camera *camera);
        target_pos return_ikine();
        camera *return_ptr();
        bool if_get();
        ~Arm_ikine();
};
