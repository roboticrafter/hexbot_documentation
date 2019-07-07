#ifndef ARM_BLUTOOTH_H
#define ARM_BLUTOOTH_H

#include <Arm.h>
#include <Arm_pump.h>

//定义接受常量

class Arm_blutooth{

  //总常量执行函数
  //各个常量执行函数
  
  private:
    HardwareSerial *comSer;                //串口指针
    word addAmplitude;                     //实际增加步伐
    word basestep;                         //基础步伐
    word step;                             //接收蓝牙的步伐
    word currentPos[7];                    //当前位置（从磁编码获取）
    word targetPos[7];                     //目标位置
	double y,x,z;                          //y,x,z坐标轴
	word pos0,pos1,pos2,pos3,pos4,pos5,pos6;
	word runTime;
	word perTime;
    
  public:
  void begin(HardwareSerial *desireSer);
  void allJugdeExecute();
  
  void flagExecute();
  
  void aExecute();
  void bExecute();
  void cExecute();
  void dExecute();
  void eExecute();
  void fExecute();
  void gExecute();
  void hExecute();
  
  void iExecute();
  void jExecute();
  void kExecute();
  void lExecute();
  void mExecute();
  void nExecute();
  void oExecute();
  void pExecute();
  
  void qExecute();
  void rExecute();
  void sExecute();
  void tExecute();
  void uExecute();
  void vExecute();
  void wExecute();
  void xExecute();
  
  void AExecute();
  void BExecute();
  void CExecute();
  void DExecute();
  void EExecute();
  void FExecute();
  void GExecute();
  void HExecute();
  void IExecute();
  void JExecute();
  void KExecute();
  void LExecute();
  void MExecute();
  void NExecute();
  void OExecute();   
  void PExecute();   
  void QExecute();
  void RExecute();
};


#endif
