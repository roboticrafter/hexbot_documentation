#ifndef ARM_H
#define ARM_H

//#define START_PROTECT

#include <Arduino.h>
#include <Serial_arm.h>
#include <HardwareSerial.h>
#include <Steer.h>
#include <EEPROM.h>
#include <PVector.h>
//#include <TimerOne.h>
#include "Arm_protect.h"

#define USB_SER                     &Serial                                            //USB串口设置


#define MINDIS 135
#define MAXDIS 325
#define MIN2ANGLE 0.48
#define MAX2ANGLE 2.88
#define MAXMAINANGLE 78
#define DEBUG


//mechanical constraints        机械臂尺寸，详细尺寸说明见说明书

/*************函数详细说明请参照CPP文件*************/

class Arm{

private:
//const double a = 120, b = 40, c = 188, d = 24.28, e = 79, f = 21.6, g = 12, h = 34.8;
	const double a = 120, b = 40, c = 159.30, d = 16.92, e = 51.75, f = 51.50, g = 8, h = 28.80;

    HardwareSerial *comSer;
    
    int *pos_goal;                                                                             //存储目标位置
    
    byte Steer_Detect();
    void Para_Init();
    
    short Rad2Pos(double rad);
    short Angle2Pos(double angle);
    double Pos2Angle(double pos);
    double Angle2Rad(double angle);
	
	void copy_des_to_current_xyz();
	double cur_xyz[3] = {0.0};
	
	
public:
    //word clc_Times_Total;                                                              //舵机从当前位置到达目标位置的总耗时

    byte Steer_Num;
    short *offPos;
    double *theta;

    Steer *steer[7] = {NULL};
    
    void begin(HardwareSerial *desireSer);
    
    void position_init(void);
    
    void inverse_movement(double x_ , double y_, double z_);
    void inverse_movement(PVector pt);
    
    void move_to_position( double x_ , double y_, double z_ , word runtime);
    void move_to_position( PVector pt , word runtime);
    void move_to_position( word pos0 , word pos1, word pos2 , word runtime);

    void offset_by_pos(byte id, short offset);
    void offset_by_angle(byte id, double angle);
    
    void Set_Arm_Torque_On(void);
    void Set_Arm_Torque_Off(void);
    
    void get_Arm_angle(double angle[]);
    void get_Arm_pos(word pos[]);
    
    boolean turn_steer_345_to_positon(word pos3 , word pos4, word pos5 , word runtime);
    boolean turn_steer_345_to_positon(double angle3 , double angle4, double angle5 , word runtime);
	
	boolean turn_steer6_to_positon(word pos6, word runtime);
	boolean turn_steer6_to_positon(double angle6, word runtime);
	
	boolean turn_steer_to_positon(byte id , word pos, word runtime);
	boolean turn_steer_to_positon(byte id, double angle , word runtime);
    
    void Get_Offset();
    double Rad2Angle(double rad);
    double Pos2Rad(word pos);
	
	boolean judgeMust2angle();
	boolean judgeTheta();
	
	double 	des_xyz[3] = {0.0};
	void current_xyz_init();
	void line_to_desxyz_zwc();
};

extern Arm MyArm;

double mapFloat(double val, double in_min, double in_max, double out_min, double out_max);

#endif
