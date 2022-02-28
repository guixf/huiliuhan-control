#include "stc8g.h"
#include "intrins.h"
#include "stdio.h"
//#include "mem.h"


#define FOSC 11059200UL
#define BRT (65536 - FOSC / 115200 / 4)

//#define ADC_low 870;
//#define ADC_high 800;
sbit MAX6675_SO=P3^2;   
sbit MAX6675_SCK=P5^4;   
sbit MAX6675_CS=P3^3;


typedef     unsigned char   u8;
typedef     unsigned int    u16;
typedef     unsigned long   u32;

#define     Tmp_Length          40      //读写EEPROM缓冲长度

#define     UART1_BUF_LENGTH    (Tmp_Length+6)  //串口缓冲长度

#define PWM2_NORMAL() PCA_PWM2 &=~3

#define PWM2_OUT_0() PCA_PWM2 |=3

#define PWM2_OUT_1() PCA_PWM2 &=~3,CCAP2H = 0


u8  RX1_TimeOut;
u8  TX1_Cnt;    //发送计数
u8  RX1_Cnt;    //接收计数
bit B_TX1_Busy; //发送忙标志

u8  BCC_State;    //异或结果

u8  RX1_Buffer[UART1_BUF_LENGTH]; //接收缓冲
u8  tmp[Tmp_Length];        //EEPROM操作缓冲


bit busy;
			
			u16 h1,h2,h3;//三个温度拐点
			u16 t1,t2,t3;//三个计时器时间节点
			u16 Calc_TMP_Value;			
			u16 SEC_Count;//秒计数
			u8 i = 0;
			u8 Begin_Flag = 0;
			u8 Set_PWM; 

char *ID="test\r\n";

/***********************  延时 *********************************/
void delay_nms(unsigned int n)      
{
    unsigned int i=0,j=0;
    for (i=n;i>0;i--)
     for (j=0;j<1140;j++);  
}
/********************************************************************/

void IapIdle()
{
    IAP_CONTR = 0;                              //关闭IAP功能
    IAP_CMD = 0;                                //清除命令寄存器
    IAP_TRIG = 0;                               //清除触发寄存器
    IAP_ADDRH = 0x80;                           //将地址设置到非IAP区域
    IAP_ADDRL = 0;
}

char IapRead(int addr)
{
    char dat;

    IAP_CONTR = 0x80;                           //使能IAP
    IAP_TPS = 12;                               //设置等待参数12MHz
    IAP_CMD = 1;                                //设置IAP读命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();
    dat = IAP_DATA;                             //读IAP数据
    IapIdle();                                  //关闭IAP功能

    return dat;
}

void IapProgram(int addr, char dat)
{
    IAP_CONTR = 0x80;                           //使能IAP
    IAP_TPS = 12;                               //设置等待参数12MHz
    IAP_CMD = 2;                                //设置IAP写命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_DATA = dat;                             //写IAP数据
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();
    IapIdle();                                  //关闭IAP功能
}

void IapErase(int addr)
{
    IAP_CONTR = 0x80;                           //使能IAP
    IAP_TPS = 12;                               //设置等待参数12MHz
    IAP_CMD = 3;                                //设置IAP擦除命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();                                    //
    IapIdle();                                  //关闭IAP功能
}

unsigned int ReadIapADC(int addr)
{
	u16 Itmp = 0 ;
			Itmp = IapRead(addr);
			addr++;
			Itmp += IapRead(addr)<<8;
	return  Itmp;
}

void UartIsr() interrupt 4 
{
 if (TI)
 {
  TI = 0;
  busy = 0;
 }
 if (RI)
 {
        RI = 0;
        if(RX1_Cnt >= UART1_BUF_LENGTH) RX1_Cnt = 0;
        RX1_Buffer[RX1_Cnt] = SBUF;
        RX1_Cnt++;
        RX1_TimeOut = 5;
  }
}


//在数据传输或者数据下载过程中;通常要保证数据的可靠性和安全性;所以
//发送方和接收方要约定共同的协议;而这个协议中常常会出现校验和的运算。
 unsigned char calc_nmea_checksum(const char *setence)
{
     unsigned char checksum = 0;
			u8 n;
	     n=sizeof(*setence);
           while(*setence)
                {
                checksum ^=(unsigned char)*setence++;
                }

     return  checksum;
}


void UartInit(void)		//115200bps@11.0592MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR &= 0xBF;		//定时器时钟12T模式
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0xFE;		//设置定时初始值
	TH1 = 0xFF;		//设置定时初始值
	ET1 = 0;		//禁止定时器%d中断
	TR1 = 1;		//定时器1开始计时
	
	  B_TX1_Busy = 0;
    TX1_Cnt = 0;
    RX1_Cnt = 0;
    RX1_TimeOut = 0;
}

void UartSend(char dat)
{
 while (busy);
 busy = 1;
 SBUF = dat;
}
void SendString(   char *s)
{
 
    while (*s)                   //检测字符串结束标志
    {
        UartSend(*s++);         //发送当前字符
    }
		
}


unsigned char string[20];
unsigned int  pwm=1000;
unsigned int  pwm_count=0;
unsigned int 	key=0; 
unsigned int  time_50ms_ok=0;
unsigned int  key_time=0;
unsigned int  start_state=0;

void Timer0Init(void)		//50000微秒@11.0592MHz
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0x00;		//设置定时初始值
	TH0 = 0x4C;		//设置定时初始值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
}

/********************************************************************/
/***********************从MAX6675读取温度*********************************************/
unsigned int ReadMAX6675()   
{
u16 count,Value;
//MAX6675_CS=1; //关闭MAX6675
  _nop_();
  _nop_();
MAX6675_CS=0;//置低，使能MAX6675
_nop_();

MAX6675_SCK=0;
Value=0;
//_nop_();
//_nop_();
for(count=16;count>0;count--) //获取16位MSB
{
  MAX6675_SCK=1;  //sck置低
  Value=Value<<1;     //左移1位
    if(MAX6675_SO==1){ //取当前值
			Value|=0x0001;
//			P55 = 1;
		}else{
			Value|=0x0000;
//			P55 = 0;
		}
  MAX6675_SCK=0;
//_nop_();
  //_nop_();
  //_nop_();
//_nop_();
}
MAX6675_CS=1;  //关闭MAX6675
return Value;
}
/***************************************************************************************/
/************************************************
			 实时温度读取函数
从MAX6675中读取实时温度
返回值放大10倍便于PID计算
*************************************************/
unsigned char CalcTemp(void)
{
static u32 	TempValue;
static unsigned int xiaoshu;	
	 TempValue=ReadMAX6675();//读取MAX6675 转换后的温度数值；   
   TempValue=TempValue<<1;         //去掉第15位
   TempValue=TempValue>>4;//去掉第0~2位
   xiaoshu=(TempValue*25+50)/100;
//   TempValue=TempValue*10/4;             //MAX6675最大数值为1023。75，而AD精度为12位，即2的12次方为4096，转换对应数，故要除4；
//   xiaoshu=xiaoshu/4;                        //与上述同理
 //       if(TempValue>=1024)
 //       {TempValue=1024;}
				//TempValue = TempValue*10+xiaoshu;
//   return TempValue;				
   return xiaoshu;				
}

/*********************************************************** 
             PID温度控制做动函数
***********************************************************/ 
void compare_temper(u16 set_temper,u16 temper) 		//PID温度控制输出函数
{ 
//  unsigned char i; 
 
  if(set_temper>temper) 
   { 
    if(set_temper-temper>8)//如果控制目标温度温与实时温度差大于8度，（放大10倍）是50
     { 
        Set_PWM = 0xff;//PWM 输出高电平占空比最大。即全速加温

	      //pwm = 10; 
     }
	else 
    { 
						if(set_temper-temper>1){
											   Set_PWM = 0x8f;//t1保持温度缓慢增加
//													pwm = 600;
						}else{
											Set_PWM = 0x4f;//t2保持温度稳定
//													pwm = 700;
						}
	  
    } 
   } 
			else //目标温度小于实时温度pwm输出低电平最高占空比，关闭加热。
		{ 
					Set_PWM = 0; 
					//pwm = 1000;
		} 
		
//				PCA_PWM2=0x30&(Set_PWM>>2);//高两位XCCAPnH[1:0]
//				CCAP2H =Set_PWM;//低8位CCAPnH[7:0]
	 
}
/***************************************************************************************/
//根据温度和时间设置计算斜率上的温度数值
unsigned char CalcTempValue(int it)
{

	static u8 tmp;
	static u8 Flag=0;
		switch(it){
				case 1:
					      //rin=CalcTemp();//PID输入室温温度采样值
								//xiedu=(h1*10-h0)/t1;//计算斜率
								//if((h1*10-h0)/t1 >30) xiedu=10;//1秒1度
								if(!Flag){
									key_time = 0;
									Calc_TMP_Value=20;//室温设置为20度

									Flag=1;
								}
								if(key_time==20){//1秒1度
									SEC_Count++;
									Calc_TMP_Value =Calc_TMP_Value+1;
									key_time = 0;
								}else{

								}
								//return (h0+xiedu*key_time);
									return (Calc_TMP_Value);
								break;
				case 2:
								if(!Flag){
									key_time = 0;
									Calc_TMP_Value=h1+5;//室温设置为20度

									Flag=1;
								}
								if(key_time==1800){//前60秒H1+5度
									SEC_Count =SEC_Count +60 ;
								}else{//随后开始升温度
									Calc_TMP_Value=h2;//设置为H2
								}
								//return (h0+xiedu*key_time);
									return (Calc_TMP_Value);
								break;
				case 3:
								return t3;
								break;								

		}
}
/***************************************************************************************/
/*PWM初始化*/
void PWM_Init(void)
{
	P_SW1 &= ~0x10;
	P_SW1 |= 0x20;
	
  CCON = 0x00;
  CMOD = (CMOD &~0xe0)|0x08;//PCA时钟为系统时钟
  CL = 0x00;  //PCA计数器初始值低8位
  CH = 0x00;  //PCA计数器初始值高8位
  CCAPM2 = 0x42; //PCA模块0为PWM工作模式
	
  PCA_PWM2 = (PCA_PWM2 &~0xc0)|0x00;//PCA模块0输出8位PWM
	
  CCAP2L = 0x00;
  CCAP2H = 0x00;//PCA模块用在PWM 模式中时,用来控制输出的占空比。
  CR = 1; //启动PCA计时器
}
/*调节PWM占空比*/
void Pwm_outset(u16 grad)
{
//			CR = 0;//先关闭计数
	
//	CCAP2H=CCAP2L= grad;//设置占空比，有 0-255 阶 
//			CCAP2H=
	
//			CR = 1;	//计数打开
if(grad == 0) PWM2_OUT_0();
	else CCAP2H = (u8)(256 - grad), PWM2_NORMAL();

}

/////////////////////////////

//接收处理串口命令
void RX_Command()
	{
		//检查uart1收到的数据
        if(RX1_TimeOut > 0)     //超时计数
        {
					//for(i=0; i<RX1_Cnt; i++)    UartSend(RX1_Buffer[i]);//原样传回，用于测试

				 if(--RX1_TimeOut == 0 && RX1_Cnt == 10)
					 {
							//tmp = 0x00;
							for(i=0; i<9; i++){
								tmp[i]=RX1_Buffer[i];//取最后一个以外的数据，用于校验
//								UartSend(tmp[i]);//原样传回，用于测试
							}
							BCC_State = RX1_Buffer[9];

            if((tmp[0] == 0x55)&&(tmp[1] == 0x02)&&(BCC_State == calc_nmea_checksum(tmp)))//判断包头(0x11)同时判断是否有数据是否4个
            {
							switch(tmp[2])
							{
        				case  0x11://将接收的值设置成
										IapErase(0x0000);
										IapProgram(0x0000, tmp[3]);//写入第一点温度	
										IapProgram(0x0001, tmp[4]);//写入第一点时间		
										IapProgram(0x0002, tmp[5]);//写入第二点温度	
										IapProgram(0x0003, tmp[6]);//写入第二点时间
										IapProgram(0x0004, tmp[7]);//写入第三点温度	
										IapProgram(0x0005, tmp[8]);//写入第三点时间
										h1 = ReadIapADC(0x0000);
										t1 = ReadIapADC(0x0001);
										h2 = ReadIapADC(0x0002);
										t2 = ReadIapADC(0x0003);
										h3 = ReadIapADC(0x0004);
										t3 = ReadIapADC(0x0005);
									sprintf(string,"HuiLiuHan is set T1=%d,S1=%d,T2=%d,S2=%d,T3=%d,S3=%d!\r\n",h1,t1,h2,t2,h3,t3);
								  SendString(string);				
									break;
								case 0x12://接收到55 02 12 00 00 00 00 00 00 45后开始加热
								  sprintf(string,"HuiLiuHan is start %d!\r\n",Begin_Flag);
								  SendString(string);	
										Begin_Flag = 1;//设置开始加热
									break;
  							default:
								  SendString("Command is wrong!\r\nPlease Tx 0x55 0x02 0x11--0x12 value BCC_code\r\n");									  
							}
							

						}else{
									SendString("Wrong Command or Address or CRC errror!\r\nPlease Tx 0x55 0x02 0x11--0x12 value BCC_code\r\n0x11 is T1 S1 T2 S2 T3 S3 BCC_code\r\nSend 55 02 12 00 00 00 00 00 00 45 to Start.\r\n");
						}
							RX1_Cnt = 0;  //							  // 
					}	
				}
}//end

void ReadEPRom()//读取存储的内容
{	
										h1 = ReadIapADC(0x0000);
										t1 = ReadIapADC(0x0001);
										h2 = ReadIapADC(0x0002);
										t2 = ReadIapADC(0x0003);
										h3 = ReadIapADC(0x0004);
										t3 = ReadIapADC(0x0005);
										if (h1 > 0xc8||h1 < 0x64) h1=0x96;
										if (t1 > 0x64||t1 < 0x32) t1=0x5a;
										if (h2 > 0xfa||h2 < 0x96) h2=0xc8;
										if (t2 > 0x96||t2 < 0x32) t2=0x64;
										if (h3 > 0xff||h3 < 0xc8) h3=0xf0;
										if (t3 > 0x96||t3 < 0x14) t3=0x32;
}
/***************************************************************************************/
void main()
{
	
	    u8  flag=0;

			u16 TempValue,set_temper;
//	    u8 Begin_Flag =0;
			SEC_Count = 0;
	
			P5M0 = 0x20;                                //设置P5.5为推挽输出模式
			P5M1 = 0x00;
			P3M0 = 0x00;                                //设置P3为推挽输出模式
			P3M1 = 0x04;
MAX6675_CS=1;	
	
			Timer0Init();	 //	启动定时器0
			UartInit();    //启动uart
			PWM_Init();     //启动PWM
      ES = 1;
      WDT_CONTR = 0x27; //启动看门狗，8s重启


			P55 = 0;  

			ET0 = 1;    //Timer0 interrupt enable
			TR0 = 1;    //Tiner0 run
	
//			ET1 = 1;    //Timer0 interrupt enable
//			TR1 = 1;    //Tiner0 run
			
			  EA = 1;     //打开总中断
		


				   ReadEPRom();
//									sprintf(string,"HuiLiuHan is set T1=%d,S1=%d,T2=%d,S2=%d,T3=%d,S3=%d!\r\n",h1,t1,h2,t2,h3,t3);
//								  SendString(string);		

//				Pwm_outset(256);
			
		while (1)
		{

							RX_Command();

    					WDT_CONTR = 0x37;  //看门狗复位							

Begin_Flag=1;//调试模式
				
      if (time_50ms_ok > 20&&Begin_Flag)            //每1秒执行一次，  
        {  
             time_50ms_ok =0;  
					
					   SEC_Count++;

							TempValue = CalcTemp();			//读取MAX6675，获取温度信息					

//	        delay_nms(220);    
//							TempValue = ReadMAX6675();
//					        sprintf(string,"%d\r\n",TempValue);//用于统计温度曲线变化
//								  SendString(string);					

					
//pid控制部分	
																	if(!flag) {//重设计时器
																		key_time = 0;
																		flag=1;
																	}
					    set_temper=	CalcTempValue(1)				;//获取当前时间应该到达的温度
//							time_key = key_time;
					        sprintf(string,"%d,%d\r\n",TempValue,SEC_Count);//用于统计温度曲线变化
								  SendString(string);	

    					if(CalcTemp<h1){
															compare_temper(set_temper,TempValue);
							}else if(CalcTemp>h1 && CalcTemp<h2){
													    set_temper=	CalcTempValue(2)				;//获取当前时间应该到达的温度
															compare_temper(set_temper,TempValue);
												}else if(CalcTemp>h2){
															 if(!flag) {//重设计时器
																		key_time = 0;
																		flag=1;
																	}
													    set_temper=	CalcTempValue(3)				;//获取当前时间应该到达的温度
															while(t3 > key_time*20)  compare_temper(set_temper,TempValue);
														}else if(key_time*20>t3){
																		Set_PWM=0;
	//																	P55 = 0;//关闭输出
	//																		PCA_PWM2=0;//高两位XCCAPnH[1:0]
	//																		CCAP2H =0;//低8位CCAPnH[7:0]
																	}


				}
//				pwm++;
//				if(pwm > 255) pwm =0; 
//				delay_nms(10);
				Pwm_outset(Set_PWM);
	}
}


void timer0() interrupt 1     //定时器T0中断函数入口,检测按键输入情况
{
     TH0=0X4C;             //初值重载
     TL0=0X00;           //定时50ms=50000us; 50000/2=25000
     time_50ms_ok ++;
     key_time++;
}

void Timer1(void)   interrupt 2 		//1微秒@11.0592MHz
{
	static unsigned char k;
	
	
	AUXR |= 0x80;		//定时器时钟1T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0xF5;		//设置定时初始值
	TH0 = 0xFF;		//设置定时初始值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	
	//模拟pwm输出
	k++;
 //       if(k > pwm)P55=1;//输出高电位
 //       else P55=0;//输出低电位
	
}

