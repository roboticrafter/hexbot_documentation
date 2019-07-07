#include<Arm_Instruction.h>

void Arm_Instruction::receiveCom() {
	while (ARMPORT.available() > 0) {
		int rxBuf = ARMPORT.parseInt();
		Serial.println(rxBuf);
		if (rxBuf == 0xfe) {
			byte instruction = ARMPORT.parseInt();
					Serial.println(instruction);
			if(instruction > 0xf0 && instruction < 0xfd) {
				switch(instruction) {
					case 0xf5: receivef5(); break;
					case 0xf9: receivef9(); break;
					case 0xfa: receivefa(); break;
					case 0xfb: receivefb(); break;
					case 0xfc: receivefc(); break;
				}
				
			}else {
				break;
			}
		}
	}
}

word Arm_Instruction::getTime() {
	
	word tmp = 0;
	word det = 0;
	
	for(int i = 0; i < 7; i++) {
		
		if (posdata[i]>prepos[i]) {det = posdata[i]-prepos[i];}
		else {det = prepos[i]- posdata[i];}
		if(tmp < det ) {
			tmp = det;
		}
	}
	
	for (int i = 0; i < 7; i++) {
		prepos[i] = posdata[i];
	}
	tmp = (word)(SPEEED_K*(double)tmp);
	return tmp;
}

void Arm_Instruction::allRad2post(int num) {
	for(int i = 0; i < num; i++) {
		posdata[i] = (word)(mapFloat(thetax[i], 0, PI, 1024, 3071));
		Serial.println(posdata[i]);
	}
}

void Arm_Instruction::receivef5() {
	byte tmp = 0;
	int i = 0;
	byte data[10];
	while (ARMPORT.available() > 0) {
		tmp = ARMPORT.parseInt();
		data[i++] = tmp;
		
		if(i > 2) {break;} 
		if(tmp == 0xff) {
			switch(data[0]) {
				case 0: Serial.println(0);MyArm.Set_Arm_Torque_Off(); break;
				case 1: Serial.println(1);MyArm.Set_Arm_Torque_On(); break;
				case 2: Serial.println(2); break;
			}
			break;
		}
	}
}

void Arm_Instruction::receivef9() {
	byte tmp = 0;
	int i = 0;
	byte data[20];
	while (ARMPORT.available() > 0) {
		tmp = ARMPORT.parseInt();
		data[i++] = tmp;
		
		if(i > 15) {break;} 
		else if(tmp == 0xff) {
			  int n = i;
			  for (int i = 0; i < n/2; i++) {
                double posCode = data[i * 2] * 128 + data[i * 2 + 1];
                thetax[i] = posCode * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
				Serial.println(thetax[i]);
              }
			  Serial.println("pos: ");
				
			for(int i = 0; i < SERVO_NUM; i++) {
				posdata[i] = (word)(mapFloat(thetax[i], 0, 180, 1024, 3071));
				Serial.println(posdata[i]);	
			}
			  Serial.println("time: ");
			  word tmpTime = getTime();
			  Serial.println(tmpTime);
			  //移动
			  MyArm.move_to_position(posdata[0], posdata[1], posdata[2], tmpTime);
			  MyArm.turn_steer_345_to_positon(posdata[3],posdata[4],posdata[5],tmpTime);
			  if(MyArm.Steer_Num > 6) {MyArm.turn_steer6_to_positon(posdata[6], tmpTime);}
			  break;
		}
	}
}
//254 250 0 0 1 80 1 100 0 0 0 21 0 0 0 0 1 12 0 0 0 100 255 
//254 251 0 0 1 80 1 100 0 0 0 21 0 0 0 0 0 100 255 

void Arm_Instruction::receivefa() {
	byte tmp = 0;
	int i = 0;
	byte data_t[30];
	byte data[10];
	while (ARMPORT.available() > 0) {
		tmp = ARMPORT.parseInt();
		data_t[i++] = tmp;
		
		if(i > 22) {break;} 
		else if(tmp == 0xff) {
			  int n = i;
			if(n == 21) {  
			  for (int i = 0; i < n/2; i++) {
                data[i] = data_t[i * 2] * 128 + data_t[i * 2 + 1];
				Serial.println(data[i]);
              }
			  int mul;
              mul = 1; if (data[0] > 1024) mul = -1; j6.x = data[0] % 1024 * mul;
              mul = 1; if (data[1] > 1024) mul = -1; j6.y = data[1] % 1024 * mul;
              mul = 1; if (data[2] > 1024) mul = -1; j6.z = data[2] % 1024 * mul;
              //
              mul = 1; if (data[3] > 1024) mul = -1; vec56.x = data[3] % 1024 * mul;
              mul = 1; if (data[4] > 1024) mul = -1; vec56.y = data[4] % 1024 * mul;
              mul = 1; if (data[5] > 1024) mul = -1; vec56.z = data[5] % 1024 * mul;
              //
              mul = 1; if (data[6] > 1024) mul = -1; vec67.x = data[6] % 1024 * mul;
              mul = 1; if (data[7] > 1024) mul = -1; vec67.y = data[7] % 1024 * mul;
              mul = 1; if (data[8] > 1024) mul = -1; vec67.z = data[8] % 1024 * mul;
              //
              thetax[6] = ((double)(data[9])) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
			  IK6(j6, vec56, vec67);
			  //IK3(j6);
			  for(int j = 0 ; j < 6; j++) {
				  if(!(thetax[j] < 3.14 && thetax[j] > 0) ) {thetax[j] = PI/2;}
				  Serial.println(thetax[j]);  //执行前六个角度的舵机旋转
			  }
			  
			  Serial.println("pos: ");
			  allRad2post(6);
			  posdata[6] = (word)(mapFloat(thetax[6], 0, 180, 1024, 3071));
			  Serial.println(posdata[6]);
			  Serial.println("time: ");
			  word tmpTime = getTime();
			  Serial.println(tmpTime);			  
			  //移动
			  MyArm.move_to_position(posdata[0], posdata[1], posdata[2], tmpTime);
			  MyArm.turn_steer_345_to_positon(posdata[3],posdata[4],posdata[5],tmpTime);
			  if(MyArm.Steer_Num > 6) {MyArm.turn_steer6_to_positon(posdata[6], tmpTime);}
			  break;
			} else {
				break;
			}  
		}
	}
}

void Arm_Instruction::receivefb() {
	byte tmp = 0;
	int i = 0;
	byte data_t[30];
	byte data[10];
	while (ARMPORT.available() > 0) {
		tmp = ARMPORT.parseInt();
		data_t[i++] = tmp;
		
		if(i > 22) {break;} 
		else if(tmp == 0xff) {
			  int n = i;
			if(n == 17) {  
			  for (int i = 0; i < n/2; i++) {
                data[i] = data_t[i * 2] * 128 + data_t[i * 2 + 1];
				Serial.println(data[i]);
              }
			  
			  int mul;
              mul = 1; if (data[0] > 1024) mul = -1; j6.x = data[0] % 1024 * mul;
              mul = 1; if (data[1] > 1024) mul = -1; j6.y = data[1] % 1024 * mul;
              mul = 1; if (data[2] > 1024) mul = -1; j6.z = data[2] % 1024 * mul;
              //
              mul = 1; if (data[3] > 1024) mul = -1; vec56.x = data[3] % 1024 * mul;
              mul = 1; if (data[4] > 1024) mul = -1; vec56.y = data[4] % 1024 * mul;
              mul = 1; if (data[5] > 1024) mul = -1; vec56.z = data[5] % 1024 * mul;
              //
              thetax[5] = ((double)(data[6])) * 9 / 50;
              thetax[6] = ((double)(data[7])) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
			  
			  IK5(j6, vec56);
			  for(int j = 0 ; j < 5; j++) {
				  if(!(thetax[j] < 3.14 && thetax[j] > 0) ) {thetax[j] = PI/2;}
			  }

			  //执行第67个
			   Serial.println("pos: ");
			  allRad2post(5);
			  posdata[5] = (word)(mapFloat(thetax[5], 0, 180, 1024, 3071));
			  posdata[6] = (word)(mapFloat(thetax[6], 0, 180, 1024, 3071));
			  Serial.println(posdata[5]);
			  Serial.println(posdata[6]);
			  Serial.println("time: ");
			  word tmpTime = getTime();
			  Serial.println(tmpTime);
			   //移动
			  MyArm.move_to_position(posdata[0], posdata[1], posdata[2], tmpTime);
			  MyArm.turn_steer_345_to_positon(posdata[3],posdata[4],posdata[5],tmpTime);
			  if(MyArm.Steer_Num > 6) {MyArm.turn_steer6_to_positon(posdata[6], tmpTime);}
			  break;
			  
			} else {
				break;
			}  
		}
	}
}

void Arm_Instruction::receivefc() {
	byte tmp = 0;
	int i = 0;
	byte data_t[30];
	byte data[10];
	while (ARMPORT.available() > 0) {
		tmp = ARMPORT.parseInt();
		data_t[i++] = tmp;
		
		if(i > 22) {break;} 
		else if(tmp == 0xff) {
			  int n = i;
			if(n == 15) {  
			  for (int i = 0; i < n/2; i++) {
                data[i] = data_t[i * 2] * 128 + data_t[i * 2 + 1];
				Serial.println(data[i]);
              }
			  
			 int mul;
              mul = 1; if (data[0] > 1024) mul = -1; j5.x = data[0] % 1024 * mul;
              mul = 1; if (data[1] > 1024) mul = -1; j5.y = data[1] % 1024 * mul;
              mul = 1; if (data[2] > 1024) mul = -1; j5.z = data[2] % 1024 * mul;
              //
              thetax[3] = ((double)(data[3])) * 9 / 50;
              thetax[4] = ((double)(data[4])) * 9 / 50;
              thetax[5] = ((double)(data[5])) * 9 / 50;
              thetax[6] = ((double)(data[6])) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18degree)
			  
			  IK3(j5);
			  for(int j = 0 ; j < 3; j++) {
				  if(!(thetax[j] < 3.14 && thetax[j] > 0) ) {thetax[j] = PI/2;}
			  }
			  
			  Serial.println("pos: ");
			  allRad2post(3);
			  posdata[3] = (word)(mapFloat(thetax[3], 0, 180, 1024, 3071));
			  posdata[4] = (word)(mapFloat(thetax[4], 0, 180, 1024, 3071));
			  posdata[5] = (word)(mapFloat(thetax[5], 0, 180, 1024, 3071));
			  posdata[6] = (word)(mapFloat(thetax[6], 0, 180, 1024, 3071));
			  Serial.println(posdata[3]);
			  Serial.println(posdata[4]);
			  Serial.println(posdata[5]);
			  Serial.println(posdata[6]);
			  Serial.println("time: ");
			  word tmpTime = getTime();
			  Serial.println(tmpTime);
			   //移动
			  MyArm.move_to_position(posdata[0], posdata[1], posdata[2], tmpTime);
			  MyArm.turn_steer_345_to_positon(posdata[3],posdata[4],posdata[5],tmpTime);
			  if(MyArm.Steer_Num > 6) {MyArm.turn_steer6_to_positon(posdata[6], tmpTime);}
			  break;
			  
			} else {
				break;
			}  
		}
	}
}

int Arm_Instruction::IK3(PVector pt) {
  double x = pt.x, y = pt.y, z = pt.z;
  int status = 1;
  thetax[0] = atan(y / x);
  if (thetax[0] < 0) thetax[0] = PI + thetax[0]; 
  x -= d * cos(thetax[0]);
  y -= d * sin(thetax[0]);
  z -= e;
  double lengthA = sqrt(x * x + y * y + z * z); 
  double lengthC = sqrt(h * h + c * c); 
  double offsetAngle = atan(h / c);
  double angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
  double angleB = atan( z / sqrt(x * x + y * y) );
  double angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
  thetax[1] = angleB + angleC;
  thetax[2] = PI - angleA - angleB - angleC + offsetAngle;
  thetax[2] += 1.134464;

  // range check
  if (thetax[1] > thetaxMin[1] && thetax[1] < thetaxMax[1] &&
      thetax[2] > thetaxMin[2] && thetax[2] < thetaxMax[2] &&
      thetax[2] - 0.8203047 + thetax[1] < PI && thetax[2] + thetax[1] > 1.44862327) {
    status = 0;
  }

  if (status != 0) {
    // ARMPORT.print("IK3 Status for point ("); ARMPORT.print(pt.x); ARMPORT.print(","); ARMPORT.print(pt.y); ARMPORT.print(","); ARMPORT.print(pt.z);
    // ARMPORT.print("): "); ARMPORT.println(status);
  }
  return status;
}

// Function: IK, input joint[6] & Vector(joint[5] to joint[6] direction), calculate theta[0]~[4]
// return: [0]no error; [1]IK3 out of valid range; [2]IK5-theta4 out of range
int Arm_Instruction::IK5(PVector j6, PVector vec56_d) {
  int status = -1;
  PVector vec56_u =  PVector(vec56_d.x, vec56_d.y, vec56_d.z); 
  vec56_u.normalize();
  PVector j5 =  PVector(j6.x - f * vec56_u.x, j6.y - f * vec56_u.y, j6.z - f * vec56_u.z);
  PVector vec56 =  PVector(j6.x - j5.x, j6.y - j5.y, j6.z - j5.z);
  int IK3_status = IK3(j5);
  //println("IK3_status: ", IK3_status);
  if (IK3_status != 0) return IK3_status;
  joint[5] = j5;
  thetax[3] = 0.; thetax[4] = 0.;
  calcJoints();
  PVector j6_0 = joint[6];
  PVector vec56_0 =  PVector(j6_0.x - j5.x, j6_0.y - j5.y, j6_0.z - j5.z);
  PVector vec45 =  PVector(joint[5].x - joint[4].x, joint[5].y - joint[4].y, joint[5].z - joint[4].z);
  PVector j6p = calcProjectionPt(j6, j5, vec45);
  PVector vec56p =  PVector(j6p.x - j5.x, j6p.y - j5.y, j6p.z - j5.z);
  //ARMPORT.print("vec56p= "); ARMPORT.print( vec56p.x ); ARMPORT.print(" ");ARMPORT.print( vec56p.y ); ARMPORT.print(" ");ARMPORT.println( vec56p.z );
  thetax[3] = acos( vec56_0.dot(vec56p) / (j5.dist(j6_0) * j5.dist(j6p)) );
  thetax[4] = acos( vec56.dot(vec56p) / (j5.dist(j6) * j5.dist(j6p)) );
  calcJoints();
  double dist = j6.dist(joint[6]);
  if (dist < 1) { 
    return 0;
  }
  thetax[3] = PI - thetax[3]; 
  thetax[4] = PI - thetax[4]; 
  calcJoints();
  dist = j6.dist(joint[6]);

  if (dist >= 1) {
    ARMPORT.print("IK5 Status: ");
    ARMPORT.println(dist);
  }

  if (dist < 1) { 
    return 0;
  }
  else {
    return 2;
  }
}

// Function: IK, input joint[6], Vector(joint[5] to joint[6] direction) & Vector(joint[6] to joint[7]), calculate thetax[0]~[5]
// return: [0]no error; [1]IK3 out of valid range; [2]IK5-theta4 out of range;
int Arm_Instruction::IK6(PVector j6, PVector vec56_d, PVector vec67_d) {
  int status = -1;
  int IK5_status = IK5(j6, vec56_d);
  if (IK5_status != 0) return IK5_status;
  PVector vec67_u =  PVector(vec67_d.x, vec67_d.y, vec67_d.z);
  vec67_u.normalize();
  PVector j7 =  PVector(j6.x + g * vec67_u.x, j6.y + g * vec67_u.y, j6.z + g * vec67_u.z);
  PVector j7p = calcProjectionPt(j7, j6, vec56_d);
  thetax[5] = 0; calcJoints();
  PVector j7_0 = joint[7];
  PVector vec67_0 =  PVector(j7_0.x - j6.x, j7_0.y - j6.y, j7_0.z - j6.z);
  PVector vec67p =  PVector(j7p.x - j6.x, j7p.y - j6.y, j7p.z - j6.z);
  //(3)- calculate thetax[5]
  double thetaTmp5 = acos( vec67_0.dot(vec67p) / (j6.dist(j7_0) * j6.dist(j7p)) );
  thetax[5] = -thetaTmp5;
  if (vec67_d.x < 0) thetax[5] = -thetax[5];
  if (thetax[5] < 0) thetax[5] = PI + thetax[5]; 
  calcJoints();
  return 0;
}

// Calculate model joints
void Arm_Instruction::calcJoints() {
  joint[1] = PVector(joint[0].x, joint[0].y + d, joint[0].z + e);
  joint[2] = PVector(0, -b * cos(thetax[2] - 1.134464), b * sin(thetax[2] - 1.134464));    joint[2].add(joint[1]);
  joint[3] = PVector(0, a * cos(thetax[1]), a * sin(thetax[1]));     joint[3].add(joint[1]);
  joint[4] = PVector(0, h * sin(thetax[2] - 1.134464), h * cos(thetax[2] - 1.134464));     joint[4].add(joint[3]);
  joint[5] = PVector(0, c * cos(thetax[2] - 1.134464), -c * sin(thetax[2] - 1.134464));    joint[5].add(joint[4]);
  joint[6] = PVector(0, f * sin(thetax[2] - 1.134464 + thetax[4]), f * cos(thetax[2] - 1.134464 + thetax[4]));    joint[6].add(joint[5]);
  joint[7] = PVector(0, -g * cos(thetax[2] - 1.134464 + thetax[4]), g * sin(thetax[2] - 1.134464 + thetax[4]));   joint[7].add(joint[6]);
  joint[7] = arbitraryRotate(joint[7], joint[6], joint[5], thetax[5]); 
  joint[6] = arbitraryRotate(joint[6], joint[5], joint[4], thetax[3] - HALF_PI); 
  joint[7] = arbitraryRotate(joint[7], joint[5], joint[4], thetax[3] - HALF_PI); 
  joint[8] = PVector(2 * joint[6].x - joint[7].x, 2 * joint[6].y - joint[7].y, 2 * joint[6].z - joint[7].z);
  for (int i = 1; i < 9; i++) {
    joint[i] = zAxiRotate(joint[i], thetax[0] - HALF_PI);
  }
}

PVector Arm_Instruction::zAxiRotate(PVector point, double _angle) {
  PVector pt;
  pt = PVector( cos(_angle) * point.x - sin(_angle) * point.y, sin(_angle) * point.x + cos(_angle) * point.y, point.z );
  return pt;
}

PVector Arm_Instruction::arbitraryRotate(PVector point, PVector pointA, PVector pointB, double _angle) {
  PVector pt = PVector(0, 0, 0);
  double x = point.x, y = point.y, z = point.z;
  double u = pointB.x - pointA.x, v = pointB.y - pointA.y, w = pointB.z - pointA.z;
  double l = sqrt(u * u + v * v + w * w);
  u /= l; v /= l; w /= l;
  double a = pointA.x, b = pointA.y, c = pointA.z;
  double u2 = u * u, v2 = v * v, w2 = w * w;
  double au = a * u, av = a * v, aw = a * w;
  double bu = b * u, bv = b * v, bw = b * w;
  double cu = c * u, cv = c * v, cw = c * w;
  double ux = u * x, uy = u * y, uz = u * z;
  double vx = v * x, vy = v * y, vz = v * z;
  double wx = w * x, wy = w * y, wz = w * z;
  pt.x = (a * (v2 + w2) - u * (bv + cw - ux - vy - wz)) * (1 - cos(_angle)) + x * cos(_angle) + (-cv + bw - wy + vz) * sin(_angle);
  pt.y = (b * (u2 + w2) - v * (au + cw - ux - vy - wz)) * (1 - cos(_angle)) + y * cos(_angle) + (cu - aw + wx - uz) * sin(_angle);
  pt.z = (c * (u2 + v2) - w * (au + bv - ux - vy - wz)) * (1 - cos(_angle)) + z * cos(_angle) + (-bu + av - vx + uy) * sin(_angle);
  return pt;
}


PVector Arm_Instruction::calcProjectionPt(PVector pt0, PVector pt1, PVector nVec) {
  PVector n = PVector(nVec.x, nVec.y, nVec.z);
  n.normalize();
  PVector vec10 = PVector(pt0.x - pt1.x, pt0.y - pt1.y, pt0.z - pt1.z);
  double dot = vec10.dot(n);
  PVector projectionPt = PVector(pt0.x - dot * n.x, pt0.y - dot * n.y, pt0.z - dot * n.z);
  return projectionPt;
}

