#include <Arm_blutooth.h>

Arm_pump *arm_pump = new Arm_pump();

boolean bFlag = false;
boolean dFlag = false;
boolean fFlag = false;
boolean hFlag = false;
boolean jFlag = false;
boolean lFlag = false;
boolean tlFlag = false;
boolean trFlag = false;
boolean fourlFlag = false;
boolean fourrFlag = false;
boolean fivelFlag = false;
boolean fiverFlag = false;
boolean flagDir = true;


void Arm_blutooth::begin(HardwareSerial *desireSer) {
    comSer = desireSer;
    comSer->begin(9600);
    comSer->setTimeout(5);
	
	MyArm.position_init();
	delay(2100);
	MyArm.move_to_position(0, (double)200,232,500);
	delay(500);
	MyArm.turn_steer_345_to_positon((word)2047, 2047, 2047, 500);
	delay(500);
	
	basestep = 1;
	step = 1;
	addAmplitude = basestep*step;
	
	y = 0;
	x = 200;
	z = 232;
	pos0 = 2047;
	pos1 = 2047;
	pos2 = 2047;
	pos3 = 2047;
	pos4 = 2047;
	pos5 = 2047;
	pos6 = 2047;
	
	runTime = 10000;
	perTime = 1;
}

void Arm_blutooth::flagExecute() {
	if (bFlag) {z += addAmplitude;}
	else if (dFlag) {z -= addAmplitude;}
	else if (fFlag) {y += addAmplitude;}
	else if (hFlag) {y -= addAmplitude;}
	else if (jFlag) {x += addAmplitude;}
	else if (lFlag) {x -= addAmplitude;}
	else if (tlFlag) {pos3 -= addAmplitude*6;}
	else if (trFlag) {pos3 += addAmplitude*6;}
	else if (fourlFlag) {pos4 -= addAmplitude*6;}
	else if (fourrFlag) {pos4 += addAmplitude*6;}
	else if (fivelFlag) {pos5 -= addAmplitude*6;}
	else if (fiverFlag) {pos5 += addAmplitude*6;}
	
}

void Arm_blutooth::allJugdeExecute() {
	
		while(comSer->available()) {
			byte actionJudge = comSer->read();
			if (actionJudge != (byte)32) {
				switch(actionJudge) {
				  case (byte)'a': aExecute(); break;
				  case (byte)'b': bExecute(); break;
				  case (byte)'c': cExecute(); break;
				  case (byte)'d': dExecute(); break;
				  case (byte)'e': eExecute(); break;
				  case (byte)'f': fExecute(); break;
				  case (byte)'g': gExecute(); break;
				  case (byte)'h': hExecute(); break;
				  case (byte)'i': iExecute(); break;
				  case (byte)'j': jExecute(); break;
				  case (byte)'k': kExecute(); break;
				  case (byte)'l': lExecute(); break;
				  case (byte)'m': mExecute(); break;
				  case (byte)'n': nExecute(); break;
				  case (byte)'o': oExecute(); break;
				  case (byte)'p': pExecute(); break;
				  case (byte)'q': qExecute(); break;
				  case (byte)'r': rExecute(); break;
				  case (byte)'s': sExecute(); break;
				  case (byte)'t': tExecute(); break;
				  case (byte)'u': uExecute(); break;
				  case (byte)'v': vExecute(); break;
				  case (byte)'w': wExecute(); break;
				  case (byte)'x': xExecute(); break;
				  case (byte)'A': AExecute(); break;
				  case (byte)'B': BExecute(); break;
				  case (byte)'C': CExecute(); break;
				  case (byte)'D': DExecute(); break;
				  case (byte)'E': EExecute(); break;
				  case (byte)'F': FExecute(); break;
				  case (byte)'G': GExecute(); break;
				  case (byte)'H': HExecute(); break;
				  case (byte)'I': IExecute(); break;
				  case (byte)'J': JExecute(); break;
				  case (byte)'K': KExecute(); break;
				  case (byte)'L': LExecute(); break;
				  case (byte)'M': MExecute(); break;
				  case (byte)'N': NExecute(); break;
				  case (byte)'O': OExecute(); break;
				  case (byte)'P': PExecute(); break;
				  case (byte)'Q': QExecute(); break;
				  case (byte)'R': RExecute(); break;
				  default: comSer->println("false command!!!"); break;
				}
			}	
		}
		flagExecute();	
		if (flagDir) {
			MyArm.move_to_position(y, x, z, 10);
			delay(10);
			MyArm.turn_steer_345_to_positon(pos3,pos4,pos5,10);
			delay(10);
		} else {
			MyArm.move_to_position(pos0, pos1, pos2, 10);
			delay(10);
			MyArm.turn_steer_345_to_positon(pos3,pos4,pos5,10);
			delay(10);
		}
}

 /**
 * @brief	                  当接收到A时的执行函数
 */
void Arm_blutooth::AExecute() {
	arm_pump->pump_off();
}

 /**
 * @brief	                  当接收到B时的执行函数
 */
void Arm_blutooth::BExecute() {
	arm_pump->pump_on();
}


 /**
 * @brief	                  当接收到C时的执行函数
 */
void Arm_blutooth::CExecute() {
	arm_pump->pump_off();
}

 /**
 * @brief	                  当接收到D时的执行函数
 */
void Arm_blutooth::DExecute() {
	arm_pump->pump_on();
}


 /**
 * @brief	                  当接收到E时的执行函数
 */
void Arm_blutooth::EExecute() {
	step = 1;
	addAmplitude = basestep*step;
}

 /**
 * @brief	                  当接收到F时的执行函数
 */
void Arm_blutooth::FExecute() {
	step = 2;
	addAmplitude = basestep*step;
}

 /**
 * @brief	                  当接收到G时的执行函数
 */
void Arm_blutooth::GExecute() {
	step = 3;
	addAmplitude = basestep*step;
}

 /**
 * @brief	                  当接收到H时的执行函数
 */
void Arm_blutooth::HExecute() {
	step = 4;
	addAmplitude = basestep*step;
}

 /**
 * @brief	                  当接收到I时的执行函数
 */
void Arm_blutooth::IExecute() {
	word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos0) {tmp2 = tmp - pos0;} else {tmp2 = pos0 - tmp;}
	
	runTime = tmp2*perTime;
	pos0 = tmp;
	MyArm.move_to_position(pos0,pos1,pos2,runTime);
	delay(runTime);}


 /**
 * @brief	                  当接收到J时的执行函数
 */
void Arm_blutooth::JExecute() {
	word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos1) {tmp2 = tmp - pos1;} else {tmp2 = pos1 - tmp;}
	
	runTime = tmp2*perTime;
	pos1 = tmp;
	MyArm.move_to_position(pos0,pos1,pos2,runTime);
	delay(runTime);
}


 /**
 * @brief	                  当接收到K时的执行函数
 */
void Arm_blutooth::KExecute() {
	
	word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos2) {tmp2 = tmp - pos2;} else {tmp2 = pos2 - tmp;}
	
	runTime = tmp2*perTime;
	pos2 = tmp;
	MyArm.move_to_position(pos0,pos1,pos2,runTime);
	delay(runTime);}


 /**
 * @brief	                  当接收到L时的执行函数
 */
void Arm_blutooth::LExecute() {
	
word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos3) {tmp2 = tmp - pos3;} else {tmp2 = pos3 - tmp;}
	
	runTime = tmp2*perTime;
	pos3 = tmp;
	MyArm.turn_steer_345_to_positon(pos3 , pos4, pos5 , runTime);
	delay(runTime);
}


 /**
 * @brief	                  当接收到M时的执行函数
 */
void Arm_blutooth::MExecute() {
	
	word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos4) {tmp2 = tmp - pos4;} else {tmp2 = pos4 - tmp;}
	
	runTime = tmp2*perTime;
	pos4 = tmp;
	MyArm.turn_steer_345_to_positon(pos3 , pos4, pos5 , runTime);
	delay(runTime);
}


 /**
 * @brief	                  当接收到N时的执行函数
 */
void Arm_blutooth::NExecute() {
	
	word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos5) {tmp2 = tmp - pos5;} else {tmp2 = pos5 - tmp;}
	
	runTime = tmp2*perTime;
	pos5 = tmp;
	MyArm.turn_steer_345_to_positon(pos3 , pos4, pos5 , runTime);
	delay(runTime);
}

 /**
 * @brief	                  当接收到O时的执行函数
 */
void Arm_blutooth::OExecute() {
	
	word tmp = (word)(Serial2.parseInt()+1024);
	word tmp2;
	
	if (tmp >= pos6) {tmp2 = tmp - pos6;} else {tmp2 = pos6 - tmp;}
	
	runTime = tmp2*perTime;
	pos6 = tmp;
	MyArm.turn_steer6_to_positon(pos6 , runTime);
	delay(runTime);
}


 /**
 * @brief	                  当接收到P时的执行函数
 */
void Arm_blutooth::PExecute() {
	//Serial.println(word(Serial2.parseInt()));
	//Serial.println(word(Serial2.parseInt()));
	int tmp1 = Serial2.parseInt();
	word tmp2 = Serial2.parseInt();
	word tmp;
	switch (tmp1) {
		case 0: 	
			if (tmp2 >= pos0) {tmp = tmp2 - pos0;} else {tmp = pos0 - tmp2;}
			runTime = tmp*perTime;
			pos0 = tmp2;
			MyArm.move_to_position(pos0,pos1,pos2,runTime);
			break;
		case 1: 	
			if (tmp2 >= pos1) {tmp = tmp2 - pos1;} else {tmp = pos1 - tmp2;}
			runTime = tmp*perTime;
			pos1 = tmp2;
			MyArm.move_to_position(pos0,pos1,pos2,runTime);
			break;
		case 2: 	
			if (tmp2 >= pos2) {tmp = tmp2 - pos2;} else {tmp = pos2 - tmp2;}
			runTime = tmp*perTime;
			pos2 = tmp2;
			MyArm.move_to_position(pos0,pos1,pos2,runTime);
			break;
		case 3: 	
			if (tmp2 >= pos3) {tmp = tmp2 - pos3;} else {tmp = pos3 - tmp2;}
			runTime = tmp*perTime;
			pos3 = tmp2;
			MyArm.turn_steer_345_to_positon(pos3 , pos4, pos5 , runTime);
			break;
		case 4: 	
			if (tmp2 >= pos4) {tmp = tmp2 - pos4;} else {tmp = pos4 - tmp2;}
			runTime = tmp*perTime;
			pos4 = tmp2;
			MyArm.turn_steer_345_to_positon(pos3 , pos4, pos5 , runTime);
			break;
		case 5: 	
			if (tmp2 >= pos5) {tmp = tmp2 - pos5;} else {tmp = pos5 - tmp2;}
			runTime = tmp*perTime;
			pos5 = tmp2;
			MyArm.turn_steer_345_to_positon(pos3 , pos4, pos5 , runTime);
			break;
		case 6: 	
			if (tmp2 >= pos6) {tmp = tmp2 - pos6;} else {tmp = pos6 - tmp2;}
			runTime = tmp*perTime;
			pos6 = tmp2;
			MyArm.turn_steer6_to_positon(pos6 , runTime);
			break;
		
	}
}

/**
 * @brief	                  当接收到Q时的执行函数
 */
void Arm_blutooth::QExecute() {
	y = 0;
	x = 200;
	z = 232;
	pos0 = 2047;
	pos1 = 2047;
	pos2 = 2047;
	pos3 = 2047;
	pos4 = 2047;
	pos5 = 2047;
	flagDir = false;
	MyArm.position_init();
	delay(2100);
	MyArm.move_to_position(pos0,pos1,pos2,500);
	delay(500);
	MyArm.turn_steer_345_to_positon((word)pos3, pos4, pos5, 500);
	delay(500);
}

/**
 * @brief	                  当接收到R时的执行函数
 */
void Arm_blutooth::RExecute() {
	y = 0;
	x = 200;
	z = 232;
	pos3 = 2047;
	pos4 = 2047;
	pos5 = 2047;
	flagDir = true;
	MyArm.position_init();
	delay(2100);
	MyArm.move_to_position(y, (double)x,z,500);
	delay(500);
	MyArm.turn_steer_345_to_positon((word)pos3, pos4, pos5, 500);
	delay(500);
}

 /**
 * @brief	                  当接收到a时的执行函数
 */
void Arm_blutooth::aExecute() {
	bFlag = false;
}

 /**
 * @brief	                  当接收到b时的执行函数
 */
void Arm_blutooth::bExecute() {
	bFlag = true;
}


 /**
 * @brief	                  当接收到c时的执行函数
 */
void Arm_blutooth::cExecute() {
	dFlag = false;
}

/**
 * @brief	                  当接收到d时的执行函数
 */
void Arm_blutooth::dExecute() {
	dFlag = true;
}

 /**
 * @brief	                  当接收到e时的执行函数
 */
void Arm_blutooth::eExecute() {
	fFlag = false;
}

 /**
 * @brief	                  当接收到f时的执行函数
 */
void Arm_blutooth::fExecute() {
	fFlag = true;
}


 /**
 * @brief	                  当接收到g时的执行函数
 */
void Arm_blutooth::gExecute() {
	hFlag = false;
}


 /**
 * @brief	                  当接收到h时的执行函数
 */
void Arm_blutooth::hExecute() {
	hFlag = true;
}


 /**
 * @brief	                  当接收到i时的执行函数
 */
void Arm_blutooth::iExecute() {
	jFlag = false;
}


 /**
 * @brief	                  当接收到j时的执行函数
 */
void Arm_blutooth::jExecute() {
	jFlag = true;
}

 /**
 * @brief	                  当接收到k时的执行函数
 */
void Arm_blutooth::kExecute() {
	lFlag = false;
}

 /**
 * @brief	                  当接收到l时的执行函数
 */
void Arm_blutooth::lExecute() {
	lFlag = true;
}

 /**
 * @brief	                  当接收到m时的执行函数
 */
void Arm_blutooth::mExecute() {
	tlFlag = false;
}

 /**
 * @brief	                  当接收到n时的执行函数
 */
void Arm_blutooth::nExecute() {
	tlFlag = true;
}

 /**
 * @brief	                  当接收到o时的执行函数
 */
void Arm_blutooth::oExecute() {
	trFlag = false;
}

 /**
 * @brief	                  当接收到p时的执行函数
 */
void Arm_blutooth::pExecute() {
	trFlag = true;
}

 /**
 * @brief	                  当接收到q时的执行函数
 */
void Arm_blutooth::qExecute() {
	fourlFlag = false;
}

 /**
 * @brief	                  当接收到r时的执行函数
 */
void Arm_blutooth::rExecute() {
	fourlFlag = true;
}

 /**
 * @brief	                  当接收到s时的执行函数
 */
void Arm_blutooth::sExecute() {
	fourrFlag = false;
}

 /**
 * @brief	                  当接收到t时的执行函数
 */
void Arm_blutooth::tExecute() {
	fourrFlag = true;
}

 /**
 * @brief	                  当接收到u时的执行函数
 */
void Arm_blutooth::uExecute() {
	fivelFlag = false;
}

 /**
 * @brief	                  当接收到v时的执行函数
 */
void Arm_blutooth::vExecute() {
	fivelFlag = true;
}

 /**
 * @brief	                  当接收到w时的执行函数
 */
void Arm_blutooth::wExecute() {
	fiverFlag = false;
}

 /**
 * @brief	                  当接收到x时的执行函数
 */
void Arm_blutooth::xExecute() {
	fiverFlag = true;
}


