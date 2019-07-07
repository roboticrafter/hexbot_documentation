#ifndef PVECTOR_H
#define PVECTOR_H

#include "Arduino.h"


/*************函数详细说明请参照CPP文件*************/

class PVector {

  public:

    double x, y, z;                                                              //  Global Variables  全局变量（通常表示机体坐标）

    PVector();
    PVector(double _x, double _y, double _z);
    
    void set_xyz(double _x, double _y, double _z);
    void add(PVector p);
    PVector sub(PVector p);
    void normalize();
    double dot(PVector p);
    double dist(PVector p);
};

#endif


/*********PVector作用：为坐标运算提供一个容器，简化运算*******/

