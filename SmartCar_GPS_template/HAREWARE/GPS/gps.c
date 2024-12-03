#include "gps.h" 
#include "led.h" 
#include "delay.h" 								   
#include "usart2.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "mymath.h"
#include "math.h"

#define ANGLE_TO_RAD(x)     ((x) * PI / 180.0)                                  // 角度转弧度
#define RAD_TO_ANGLE(x)     ((x) * 180.0 / PI)                                  // 弧度转角度
#define PI                  (3.1415926535898)


const u32 BAUD_id[9]={4800,9600,19200,38400,57600,115200,230400,460800,921600};//模块支持波特率数组
//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号							  
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{	 		    
	u8 *p=buf;
	while(cx)
	{		 
		if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
		if(*buf==',')cx--;
		buf++;
	}
	return buf-p;	 
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(u8 *buf,u8*dx)
{
	u8 *p=buf;
	u32 ires=0,fres=0;
	u8 ilen=0,flen=0,i;
	u8 mask=0;
	int res;
	while(1) //得到整数和小数的长度
	{
		if(*p=='-'){mask|=0X02;p++;}//是负数
		if(*p==','||(*p=='*'))break;//遇到结束了
		if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
		else if(*p>'9'||(*p<'0'))	//有非法字符
		{	
			ilen=0;
			flen=0;
			break;
		}	
		if(mask&0X01)flen++;
		else ilen++;
		p++;
	}
	if(mask&0X02)buf++;	//去掉负号
	for(i=0;i<ilen;i++)	//得到整数部分数据
	{  
		ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
	}
	if(flen>5)flen=5;	//最多取5位小数
	*dx=flen;	 		//小数点位数
	for(i=0;i<flen;i++)	//得到小数部分数据
	{  
		fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
	} 
	res=ires*NMEA_Pow(10,flen)+fres;
	if(mask&0X02)res=-res;		   
	return res;
}	  							 
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;   	 
	p=buf;
	p1=(u8*)strstr((const char *)p,"$GPGSV");
	len=p1[7]-'0';								//得到GPGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
	if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(u8*)strstr((const char *)p,"$GPGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个GPGSV信息
	}   
}
//分析BDGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_BDGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p,*p1,dx;
	u8 len,i,j,slx=0;
	u8 posx;   	 
	p=buf;
	p1=(u8*)strstr((const char *)p,"$BDGSV");
	len=p1[7]-'0';								//得到BDGSV的条数
	posx=NMEA_Comma_Pos(p1,3); 					//得到可见北斗卫星总数
	if(posx!=0XFF)gpsx->beidou_svnum=NMEA_Str2num(p1+posx,&dx);
	for(i=0;i<len;i++)
	{	 
		p1=(u8*)strstr((const char *)p,"$BDGSV");  
		for(j=0;j<4;j++)
		{	  
			posx=NMEA_Comma_Pos(p1,4+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
			else break; 
			posx=NMEA_Comma_Pos(p1,5+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角 
			else break;
			posx=NMEA_Comma_Pos(p1,6+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
			else break; 
			posx=NMEA_Comma_Pos(p1,7+j*4);
			if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
			else break;
			slx++;	   
		}   
 		p=p1+1;//切换到下一个BDGSV信息
	}   
}
//分析GNGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNGGA");
	posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
	if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);	
	posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
	if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx); 
	// posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
	// if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);  
}
//分析GNGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx; 
	u8 i;   
	p1=(u8*)strstr((const char *)buf,"$GNGSA");
	posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
	if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);	
	for(i=0;i<12;i++)										//得到定位卫星编号
	{
		posx=NMEA_Comma_Pos(p1,3+i);					 
		if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
		else break; 
	}				  
	posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
	if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
	if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);  
	posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
	if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);  
}
//分析GNRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;     
	u32 temp;	   
	float rs;  
	p1=(u8*)strstr((const char *)buf,"RMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.···········//这里改过
	posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
		gpsx->utc.hour=temp/10000;
		gpsx->utc.min=(temp/100)%100;
		gpsx->utc.sec=temp%100;	 	 
	}	
	posx=NMEA_Comma_Pos(p1,3);								//得到纬度
	if(posx!=0XFF)
	{
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
//	posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬 
//	if(posx!=0XFF)gpsx->nshemi=*(p1+posx);					 
 	posx=NMEA_Comma_Pos(p1,5);								//得到经度
	if(posx!=0XFF)
	{												  
		temp=NMEA_Str2num(p1+posx,&dx);		 	 
		gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
		rs=temp%NMEA_Pow(10,dx+2);				//得到'		 
		gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为° 
	}
//	posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
	// if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);		 
	// posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
	// if(posx!=0XFF)
	// {
	// 	temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
	// 	gpsx->utc.date=temp/10000;
	// 	gpsx->utc.month=(temp/100)%100;
	// 	gpsx->utc.year=2000+temp%100;	 	 
	// } 
}
//分析GNVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
	u8 *p1,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNVTG");							 
	posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
	if(posx!=0XFF)
	{
		gpsx->speed=NMEA_Str2num(p1+posx,&dx);
		if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
	}
}  
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
	//NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
	//NMEA_BDGSV_Analysis(gpsx,buf);	//BDGSV解析
	//NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析 	
	//NMEA_GNGSA_Analysis(gpsx,buf);	//GPNSA解析
	NMEA_GNRMC_Analysis(gpsx,buf);	//GPNMC解析
	//NMEA_GNVTG_Analysis(gpsx,buf);	//GPNTG解析
}

///////////////////////////////////////////UBLOX 配置代码/////////////////////////////////////
////检查CFG配置执行情况
////返回值:0,ACK成功
////       1,接收超时错误
////       2,没有找到同步字符
////       3,接收到NACK应答
u8 SkyTra_Cfg_Ack_Check(void)
{			 
	u16 len=0,i;
	u8 rval=0;
	while((USART2_RX_STA&0X8000)==0 && len<100)//等待接收到应答   
	{
		len++;
		delay_ms(5);
	}		 
	if(len<100)   	//超时错误.
	{
		len=USART2_RX_STA&0X7FFF;	//此次接收到的数据长度 
		for(i=0;i<len;i++)
		{
			if(USART2_RX_BUF[i]==0X83)break;
			else if(USART2_RX_BUF[i]==0X84)
			{
				rval=3;
				break;
			}
		}
		if(i==len)rval=2;						//没有找到同步字符
	}else rval=1;								//接收超时错误
    USART2_RX_STA=0;							//清除接收
	return rval;  
}
//配置SkyTra_GPS/北斗模块波特率
//baud_id:0~8，对应波特率,4800/9600/19200/38400/57600/115200/230400/460800/921600	  
//返回值:0,执行成功;其他,执行失败(这里不会返回0了)
u8 SkyTra_Cfg_Prt(u32 baud_id)
{
	SkyTra_baudrate *cfg_prt=(SkyTra_baudrate *)USART2_TX_BUF;
	cfg_prt->sos=0XA1A0;		//引导序列(小端模式)
	cfg_prt->PL=0X0400;			//有效数据长度(小端模式)
	cfg_prt->id=0X05;		    //配置波特率的ID 
	cfg_prt->com_port=0X00;			//操作串口1
	cfg_prt->Baud_id=baud_id;	 	////波特率对应编号
	cfg_prt->Attributes=1; 		  //保存到SRAM&FLASH
	cfg_prt->CS=cfg_prt->id^cfg_prt->com_port^cfg_prt->Baud_id^cfg_prt->Attributes;
	cfg_prt->end=0X0A0D;        //发送结束符(小端模式)
	SkyTra_Send_Date((u8*)cfg_prt,sizeof(SkyTra_baudrate));//发送数据给SkyTra   
	delay_ms(200);				//等待发送完成 
	USART2_Init(BAUD_id[baud_id]);	//重新初始化串口2  
	return SkyTra_Cfg_Ack_Check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
} 
//配置SkyTra_GPS模块的时钟脉冲宽度
//width:脉冲宽度1~100000(us)
//返回值:0,发送成功;其他,发送失败.
u8 SkyTra_Cfg_Tp(u32 width)
{
	u32 temp=width;
	SkyTra_pps_width *cfg_tp=(SkyTra_pps_width *)USART2_TX_BUF;
	temp=(width>>24)|((width>>8)&0X0000FF00)|((width<<8)&0X00FF0000)|((width<<24)&0XFF000000);//小端模式
	cfg_tp->sos=0XA1A0;		    //cfg header(小端模式)
	cfg_tp->PL=0X0700;        //有效数据长度(小端模式)
	cfg_tp->id=0X65	;			    //cfg tp id
	cfg_tp->Sub_ID=0X01;			//数据区长度为20个字节.
	cfg_tp->width=temp;		  //脉冲宽度,us
	cfg_tp->Attributes=0X01;  //保存到SRAM&FLASH	
	cfg_tp->CS=cfg_tp->id^cfg_tp->Sub_ID^(cfg_tp->width>>24)^(cfg_tp->width>>16)&0XFF^(cfg_tp->width>>8)&0XFF^cfg_tp->width&0XFF^cfg_tp->Attributes;    	//用户延时为0ns
	cfg_tp->end=0X0A0D;       //发送结束符(小端模式)
	SkyTra_Send_Date((u8*)cfg_tp,sizeof(SkyTra_pps_width));//发送数据给NEO-6M  
	return SkyTra_Cfg_Ack_Check();
}
//配置SkyTraF8-BD的更新速率	 	    
//Frep:（取值范围:1,2,4,5,8,10,20,25,40,50）测量时间间隔，单位为Hz，最大不能大于50Hz
//返回值:0,发送成功;其他,发送失败.
u8 SkyTra_Cfg_Rate(u8 Frep)
{
	SkyTra_PosRate *cfg_rate=(SkyTra_PosRate *)USART2_TX_BUF;
 	cfg_rate->sos=0XA1A0;	    //cfg header(小端模式)
	cfg_rate->PL=0X0300;			//有效数据长度(小端模式)
	cfg_rate->id=0X0E;	      //cfg rate id
	cfg_rate->rate=Frep;	 	  //更新速率
	cfg_rate->Attributes=0X01;	   	//保存到SRAM&FLASH	.
	cfg_rate->CS=cfg_rate->id^cfg_rate->rate^cfg_rate->Attributes;//脉冲间隔,us
	cfg_rate->end=0X0A0D;       //发送结束符(小端模式)
	SkyTra_Send_Date((u8*)cfg_rate,sizeof(SkyTra_PosRate));//发送数据给NEO-6M 
	return SkyTra_Cfg_Ack_Check();
}


//????????????SkyTraF8-BD?????????????2????
//dbuf?????????????
//len?????????????
void SkyTra_Send_Date(u8* dbuf,u16 len)
{
	u16 j;
	for(j=0;j<len;j++)//???????????
	{
		while((USART2->SR&0X40)==0);//???????,??????????   
		USART2->DR=dbuf[j];  
	}	
}

float my_sqrt(float number)
{
   long i;
   float x, y;
   const float f = 1.5F;
   x = number * 0.5F;
   y = number;
   i = * ( long * ) &y;
   i = 0x5f3759df - ( i >> 1 );

   y = * ( float * ) &i;
   y = y * ( f - ( x * y * y ) );
   y = y * ( f - ( x * y * y ) );
   return number * y;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的距离
// 参数说明     latitude1       第一个点的纬度
// 参数说明     longitude1      第一个点的经度
// 参数说明     latitude2       第二个点的纬度
// 参数说明     longitude2      第二个点的经度
// 返回参数     double          返回两点距离
// 使用示例     get_two_points_distance(latitude1_1, longitude1, latitude2, longitude2);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2)
{  
 	const double EARTH_RADIUS = 6378137;                                        // 地球半径(单位：m)
    double rad_latitude1;
    double rad_latitude2;
    double rad_longitude1;
    double rad_longitude2;
    double distance;
    double a;
    double b;

    rad_latitude1 = ANGLE_TO_RAD(latitude1);                                    // 根据角度计算弧度
    rad_latitude2 = ANGLE_TO_RAD(latitude2);
    rad_longitude1 = ANGLE_TO_RAD(longitude1);
    rad_longitude2 = ANGLE_TO_RAD(longitude2);

    a = rad_latitude1 - rad_latitude2;
    b = rad_longitude1 - rad_longitude2;

    distance = 2 * asin(my_sqrt(pow(my_sin(a / 2), 2) + my_cos(rad_latitude1) * my_cos(rad_latitude2) * pow(my_sin(b / 2), 2)));   // google maps 里面实现的算法
    distance = distance * EARTH_RADIUS;  

    return distance;  
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的方位角
// 参数说明     latitude1       第一个点的纬度
// 参数说明     longitude1      第一个点的经度
// 参数说明     latitude2       第二个点的纬度
// 参数说明     longitude2      第二个点的经度
// 返回参数     double          返回方位角（0至360）
// 使用示例     get_two_points_azimuth(latitude1_1, longitude1, latitude2, longitude2);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2)
{
	double x,y,angle;
    latitude1 = ANGLE_TO_RAD(latitude1);
    latitude2 = ANGLE_TO_RAD(latitude2);
    longitude1 = ANGLE_TO_RAD(longitude1);
    longitude2 = ANGLE_TO_RAD(longitude2);

    x = my_sin(longitude2 - longitude1) * my_cos(latitude2);
    y = my_cos(latitude1) * my_sin(latitude2) - my_sin(latitude1) * my_cos(latitude2) * my_cos(longitude2 - longitude1);
    angle = RAD_TO_ANGLE(fast_atan2(x, y));
    return ((angle > 0) ? angle : (angle + 360));
}






