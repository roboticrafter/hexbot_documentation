#ifndef ARM_BLUTOOTH_H
#define ARM_BLUTOOTH_H

#include <Arm.h>
#include <Arm_pump.h>

//������ܳ���

class Arm_blutooth{

  //�ܳ���ִ�к���
  //��������ִ�к���
  
  private:
    HardwareSerial *comSer;                //����ָ��
    word addAmplitude;                     //ʵ�����Ӳ���
    word basestep;                         //��������
    word step;                             //���������Ĳ���
    word currentPos[7];                    //��ǰλ�ã��Ӵű����ȡ��
    word targetPos[7];                     //Ŀ��λ��
	double y,x,z;                          //y,x,z������
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
