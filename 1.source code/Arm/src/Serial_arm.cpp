#include "Serial_arm.h"


 /**
 * @brief   		                  从串口获取浮点型数据
 * @code                        示例
 *      				                      
 * @endcode
 * @param[in]                 desireSer              
  * @return                     offset2        
 */
 
    double Serial_arm::parsedouble(HardwareSerial *desireSer)
     {
        double offset = desireSer->parseInt();
         double offset2 = 0;
        if(desireSer -> peek() == '.')
       offset2 = desireSer->parseInt();
        
         for(int i = 0; (i<20)&&(offset2 > 1); i++)
        {
          offset2 /= 10; 
        }
          offset2 += offset;
          
          return offset2;
     }
     
   int Serial_arm::parseint(HardwareSerial *desireSer)
   {
          int offset = desireSer->parseInt();
          return offset;
   }