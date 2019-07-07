#include<Arm_pump.h>



Arm_pump::Arm_pump()
{
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
}

 boolean Arm_pump::pump_on()
 {
  digitalWrite(PUMP_PIN, HIGH);
 }
 
 boolean Arm_pump::pump_off()
 {
   digitalWrite(PUMP_PIN, LOW);
 }
