#include<Arm_Instruction.h>

Arm_Instruction a;

void setup() {
  // put your setup code here, to run once:
  MyArm.begin(USB_SER);
  Serial.println("Arm_four_move_test");
  MyArm.position_init();
  delay(3000);
  Serial.println(PI);
  Serial.println(a.getTime());
}

void loop() {
  // put your main code here, to run repeatedly:
  a.receiveCom();
}
