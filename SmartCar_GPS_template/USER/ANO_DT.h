
#ifndef ANO_DT_H_
#define ANO_DT_H_
#include "sys.h"
void Data_Send(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
#endif /* ANO_DT_H_ */
