#ifndef ARM_LEARN_H
#define ARM_LEARN_H

#include <Arduino.h>
#include <Serial_arm.h>
#include <HardwareSerial.h>
#include <Steer.h>
#include <EEPROM.h>
#include <PVector.h>
#include <Arm.h>

#define BTN0_PIN                   2                                                //按键0：学习开始与结束按键                                               
#define BTN1_PIN                   3                                                //按键1：位点记录与位点重现按钮
#define BUZZER_PIN               37                                              //蜂鸣器引脚

#define RUN_VELOCITY           0.3                                             //运行速度
#define RUN_TIME                   2000                                         //运行时间设定，修改该值可以改变机械臂的速度


/*****class里面的函数具体说明请参考相应的源文件****/

class Arm_learn{

private:
        
public:
    void Arm_learn_(void);
    
    int button0_time_cnt;                              //按键0的按下时长计数
    int button1_time_cnt;                              //按键1的按下时长计数（里面的数字代表经过了多少个10毫秒）
    int rom_offset;                                         //记录EEPROM内的地址偏移
    word record_point_num;                          //记录位点的数目
    
    boolean start_learn_flag;                          //开始学习标志位
    boolean record_flag;                                //位点记录标志位
    
    void button_learn_detect(void);
    void record_one_point(void);
    void start_learn(void);
    void reappear_learn(void);
    word get_steer_positon(byte i);
    
    word get_point_dis();    
    void set_reappear_velocity();
};

extern Arm_learn MyArm_learn;

#endif
