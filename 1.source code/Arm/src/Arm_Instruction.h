#ifndef _INSTRUCTION_H
#define _INSTRUCTION_H

#include "Arduino.h"
#include "Arm.h"

#ifndef ARMPORT
  #define ARMPORT Serial
#endif

#define PI 3.14

#define SERVO_NUM 7

#define SPEEED_K 3.3

const double thetaxMin[SERVO_NUM] = { 0,  0, -1.134464,  0.17453292,  0,  0, 0};
const double thetaxMax[SERVO_NUM] = {PI, PI, 2.0071287, 2.9670596, PI, PI, PI/2};


class Arm_Instruction {
	
  private:	
	//const double a = 120, b = 40, c = 188, d = 24.28, e = 79, f = 21.6, g = 12, h = 34.8;
	const double a = 120, b = 40, c = 159.30, d = 16.92, e = 51.75, f = 51.50, g = 8, h = 28.80;

	double thetax[SERVO_NUM];
	PVector joint[9];
	PVector j5, j6, vec56, vec67;
	
	word posdata[7] = {0};
	word prepos[7] = {2047, 2047, 2047};
	
  public:
	int IK3(PVector pt);
    int IK5(PVector j6, PVector vec56_d);
    int IK6(PVector j6, PVector vec56_d, PVector vec67_d);
	void calcJoints();
	PVector zAxiRotate(PVector point, double _angle);
	PVector arbitraryRotate(PVector point, PVector pointA, PVector pointB, double _angle);
	PVector calcProjectionPt(PVector pt0, PVector pt1, PVector nVec);

    // UART
    void receiveCom();
	void receivef5();
	void receivef9();
	void receivefa();
	void receivefb();
	void receivefc();
	void allRad2post(int num);
	
	word getTime();
	
};


#endif