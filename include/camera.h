#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
using namespace std;

class camera
{
private:
        double _theta1, _theta2, _depth; //相机得到的参数,前两个是角度值，后一个是mm
        double x, y, z;                  // 基座坐标系的坐标
        double cx, cy, cz;               //相机坐标系的坐标

public:
        camera(double a, double b, double c);
        void print_base_Coordinate();
        void print_camera_Coordinate();
        Eigen::Vector3d returnBaseCoor();
};
