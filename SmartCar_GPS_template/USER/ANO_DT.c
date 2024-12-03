#include "ANO_DT.h"
#include "delay.h"
#include "usart.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)      ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

uint8_t data_to_send[50];   //发送数据缓存
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
    USART_SendArray(USART1, dataToSend, length);                    //调试串口发送
}
void Data_Send(float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8)
{
    u8 _cnt=0,sum = 0,i;
    float _temp;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xF1;
    data_to_send[_cnt++]=0;
    _temp = data1;//RadtoDeg
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data2;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data3;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data4;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data5;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data6;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data7;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = data8;
    data_to_send[_cnt++]=BYTE3(_temp);
    data_to_send[_cnt++]=BYTE2(_temp);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);


    data_to_send[3] = _cnt-4;//数据帧长度赋值

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i]; //数据校验求解

    data_to_send[_cnt++] = sum;//数据校验赋值
   // uart_write_buffer(UART_3, data_to_send, _cnt);//帧发送
    ANO_DT_Send_Data(data_to_send, _cnt);
}
//ANO_DT_Send_Status(eulerAngle.roll,-eulerAngle.pitch,eulerAngle.yaw,0,0,0);//姿态展示
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
    u8 _cnt=0;
    vs16 _temp;
    vs32 _temp2 = alt;
    u8 sum = 0;
    u8 i = 0; 
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x01;
    data_to_send[_cnt++]=0;

    _temp = (int)(angle_rol*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_pit*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = (int)(angle_yaw*100);
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[_cnt++]=BYTE3(_temp2);
    data_to_send[_cnt++]=BYTE2(_temp2);
    data_to_send[_cnt++]=BYTE1(_temp2);
    data_to_send[_cnt++]=BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[3] = _cnt-4;

    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
