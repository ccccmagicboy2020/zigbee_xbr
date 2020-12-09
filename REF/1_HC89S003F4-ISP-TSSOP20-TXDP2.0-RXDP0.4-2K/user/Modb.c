#include "include.h"


#define  Version_Order_Internal_Length   12
#define  Version_Order_OutSide_Length    10
#define  MCU_ID_Length                   10


unsigned char  HandShake_Count=0;
bit            Read_Flag=0;


unsigned char code ISP_Version_Internal_Order[]={       
														Version_Order_Internal_Length,
														Version,

														Get_Version_OutSide_Order, 
														Get_Version_Internal_Order,	
														Get_ID,

														Erase_Flash_ALL, 
														Erase_Option,	

														Write_Memory,        
														Write_Option,

														Read_Memory,
														Read_Option,

														Go_APP,
														Rst_Read_Option
																		   };
unsigned char  code ISP_Version_OutSide_Order[]={       
	                          Version_Order_OutSide_Length,
														Version,
		
														Get_Version_OutSide_Order,  
														Get_ID,
		
														Erase_Flash_ALL, 
														Erase_Option,
		
														Write_Memory,        
														Write_Option,
		
														Read_Option,
	
														Go_APP,
														Rst_Read_Option
																   };


unsigned char code MCU_ID[]={MCU_ID_Length,0x48,0x43,0x38,0x39,0x53,0x30,0x30,0x33,0x46,0x34};										 											 

void HandShake(void)
{
	unsigned char Data;
/**********************************自适应波特率串口初始化**************************************/	
	TR1=1;//开定时器1
  EA=1;	
	do{	
		if(HandShake_Count>=20)//上电20ms没有检测到数据
		{
			 IAR_Soft_Rst_No_Option();//引脚没有检测到数据，进入APP			  
		}
	}
//	while(P0_4);
//	TR1=0;//关定时器T1
//  EA=0;	
//	P2M0 = 0x38;				      //P20设置为推挽输出		
//	TXD_MAP = 0x20;						//TXD映射P20
//	RXD_MAP = 0x04;						//RXD映射P04

	while(P0_3);
	TR1=0;//关定时器T1
  EA=0;	
	P2M0 = 0x83;				      //P21设置为推挽输出
	TXD_MAP = 0x21;						//TXD映射P21
	RXD_MAP = 0x03;						//RXD映射P03	
	
	T4CON = 0x06;						//T4工作模式：UART1波特率发生器
  TH4 = 0xFF;
	TL4 = 0xF8;							//波特率250000
	SCON2 = 0x02;						//8位UART，波特率可变
	SCON = 0x10;						//允许串行接收
  
	Uart_SendByte(ACK);                 //发送OX79
	if(Uart_RecvByte(&Data,HandShark_TIMEOUT)!=SUCCESS) IAR_Soft_Rst_No_Option();//长时间没有接收到数据，进入APP	
	if(Data!=0x7f)                                      IAR_Soft_Rst_No_Option();//数据不是OX7F，进入APP
		
	Uart_SendByte(0x1f);//第一次发送0x1f
					
	Data=0;
	if(Uart_RecvByte(&Data,HandShark_TIMEOUT)!=SUCCESS) IAR_Soft_Rst_No_Option();//长时间没有接收到数据，进入APP	
	if(Data!=0x7f)                                      IAR_Soft_Rst_No_Option();//数据不是OX7F，进入APP
/*****************************************************************************/	

	
	BORC  =0xC1;			  //BOR:2.0V
	BORDBC=0x14;			  //BOR消抖  消抖时间 = BORDBC[7:0] * 8Tcpu +2 Tcpu	
	
	LVDC = 0xA2;						//LVD设置2.4V,禁止中断
	LVDDBC = 0xFF;				  //设置消抖时间16uS
	
	Uart_SendByte(0x1f);   //第二次发送0x1f	
  
}

unsigned char Receive_Packet (unsigned char *Data)
{
	unsigned char i;
	unsigned int  TempCRC;

	if (Uart_RecvByte(Data,  Command_TIMEOUT)!= SUCCESS)   return NACK_TIME;//长时间没有接收到数据，进入APP
	if (Uart_RecvByte(Data+1,Command_TIMEOUT)!= SUCCESS)   return NACK_TIME;//长时间没有接收到数据，进入APP	        
	if (Data[0]!=((Data[1]^0xff)&0xff))                    return ERROR;	  //验证反码
	
	switch(Data[0])
	{
//		case Get_Version_Internal_Order://获取当前版本以及允许使用的命令
//				     Uart_SendByte(ACK);//接收头码，反码成功
//             Uart_SendByte(Version_Order_Internal_Length);//发送有效数据长度		
//			       Uart_SendPacket(ISP_Version_Internal_Order+1, Version_Order_Internal_Length);//发送有效数据	
//             TempCRC=CRC_CalcCRC_Process(ISP_Version_Internal_Order,Version_Order_Internal_Length+1,Data,0);//获取CRC校验数据	
//             Uart_SendByte(TempCRC >> 8);//发送高八位
//             Uart_SendByte(TempCRC & 0xFF);//发送低八位
//             Read_Flag=0;		
//             return SUCCESS;				
//			                                        break;			
		
		case Get_Version_OutSide_Order://获取当前版本以及允许使用的命令
						 Uart_SendByte(ACK);//接收头码，反码成功
             Uart_SendByte(Version_Order_OutSide_Length);//发送有效数据长度		
			       Uart_SendPacket(ISP_Version_OutSide_Order+1, Version_Order_OutSide_Length);//发送有效数据	
             TempCRC=CRC_CalcCRC_Process(ISP_Version_OutSide_Order,Version_Order_OutSide_Length+1,Data,0);//获取CRC校验数据	
             Uart_SendByte(TempCRC >> 8);//发送高八位
             Uart_SendByte(TempCRC & 0xFF);//发送低八位	
             Read_Flag=1;		     
             return SUCCESS;				
			                                        break;
		
		case Get_ID:          //读取芯片型号
						 Uart_SendByte(ACK);//接收头码，反码成功
			       if(Read_ID()!=SUCCESS)  return ERROR;	//芯片型号错误		
             Uart_SendByte(MCU_ID_Length);//发送有效数据长度			 
			       Uart_SendPacket(MCU_ID+1, MCU_ID_Length);//发送有效数据	
             TempCRC=CRC_CalcCRC_Process(MCU_ID,MCU_ID_Length+1,Data,0);//获取CRC校验数据			
             Uart_SendByte(TempCRC >> 8);//发送高八位
             Uart_SendByte(TempCRC & 0xFF);//发送低八位
             return SUCCESS;
			                                        break;

		case Erase_Flash_ALL:    //擦除 Flash 数据
			Uart_SendByte(ACK);//接收头码，反码成功
			for (i = 0; i < 2; i ++)//保存要擦除的扇区
			{
				if (Uart_RecvByte(Data + 2 + i, NAK_TIMEOUT) != SUCCESS) return NACK_TIME;//长时间没有接收到数据，进入APP
			}
      if(Data[2]==0XFF&&Data[3]==0X00)//判断是否全扇区擦除	
			{
						if (XOR_FLASH_BLANK(0x0000,0x2FFF)==0x00) return SUCCESS;//全扇区插空，擦除成功	
            if (LVD_Check(LVD_TIMEOUT)!=SUCCESS)      return ERROR;//电压不正常						
						if (Earse_Flash())                        return SUCCESS;//全扇区擦除成功
						else                                      return ERROR;//全扇区擦除失败
			}	
			else
					   return ERROR;			
			                                        break;

		case Write_Memory:  //写 Flash 数据	
				Uart_SendByte(ACK);//接收头码，反码成功			
				for (i=0; i<6; i++)//接受起始地址和CRC校验，存入Data[i+2]
				{
					if (Uart_RecvByte(Data+2+i, NAK_TIMEOUT) != SUCCESS)  return NACK_TIME;//长时间没有接收到数据，进入APP
				}
				if (CRC_CalcCRC_Process(Data+2,4,Data+6,1)!=SUCCESS )return ERROR;//CRC校验错误
				
	           Uart_SendByte(ACK);					

				if (Uart_RecvByte(Data+8, NAK_TIMEOUT) !=SUCCESS)       return NACK_TIME;//开始接收数据长度，从Data[8]开始存起

					for (i=0; i<(Data[8]+2); i++)//接收数据和CRC的值
					{
						if (Uart_RecvByte(Data+9+i, NAK_TIMEOUT) != SUCCESS)      return NACK_TIME;//长时间没有接收到数据，进入APP
					}				
       if (CRC_CalcCRC_Process(Data+8,Data[8]+1,Data+Data[8]+9,1)!=SUCCESS)  return ERROR;//CRC校验错误	
       if(LVD_Check(LVD_TIMEOUT)!=SUCCESS)  return ERROR;//电压不正常
				 IND_ADDR=0x0f;
				 IND_DATA=0x00;
				 IAR_Write_arrang((unsigned int)(Data[4]<<8)|(unsigned int)Data[5],Data+9,Data[8]);
				 IAR_Read        ((unsigned int)(Data[4]<<8)|(unsigned int)Data[5],Data+9,Data[8]);														  								 

      if (CRC_CalcCRC_Process(Data+8,Data[8]+1,Data+Data[8]+9,1)!=SUCCESS) 
			{
				        IAR_Clear((unsigned int)(Data[4]<<8)|(unsigned int)Data[5]);//擦除错误扇区
								return ERROR;//CRC校验错误					
			}
		  else                                                                
    				return SUCCESS;	
			                                        break;	

		case Go_APP:       //退出 Bootloader 返回 APP 程序
			Uart_SendByte(ACK);//接收头码，反码成功
			 IAR_Soft_Rst_No_Option();//引脚没有检测到数据，进入APP
					    return SUCCESS;			
			                                        break;
		case Rst_Read_Option:       //复位重读Option	
			 Uart_SendByte(ACK);//接收头码，反码成功			
			 IAR_Soft_Rst_Option();			

					    return SUCCESS;			

		default:		 
						    return ERROR;//发送NACK	
		
			break;
	
	}
}

/*************************************************************
  Function   : CRC_CalcCRC_Compare
  Description: 写入需要校验的数据,计较并返回比较结果
  Input      : *fucp_CheckArr - CRC校验数据首地址   fui_CheckLen - CRC校验数据长度 
               *CRC_Data      - 比较的CRC首地址     CRC_Flag     - 0不比较 1比较
  return     : none    
*************************************************************/
unsigned int CRC_CalcCRC_Process(unsigned char *fucp_CheckArr,unsigned int fui_CheckLen,unsigned char *CRC_Data,bit CRC_Flag)
{

 	CRCC = 0x01;//CRC复位，MSB first，复位初值为0x0000   1021		
	while(fui_CheckLen--)CRCL = *(fucp_CheckArr++);
	if(CRC_Flag==1)
	{
		if((unsigned char)(CRCR>>8) ==*(CRC_Data )&& (unsigned char)(CRCR & 0xFF)==*(CRC_Data+1))//CRC校验高八位
		{
				return SUCCESS; //接收写入地址，与CRC校验数据成功							
		}
		else
				return ERROR;//CRC校验失败		
	}
	else
		return CRCR;
	
}
/***************************************************************************************
  * @说明  	LVD查询函数
  *	@参数	无
  * @返回值 无
  * @注		无
***************************************************************************************/
bit LVD_Check(unsigned long TimeOut)
{
		WDTC |= 0x10;                   //清狗			
	while(TimeOut-- > 0)
	{
		LVDC &=~ 0x08;					        //清除LVD中断标志位 									
		if((LVDC&0x08)==0)						  //判断LVD中断标志位
		{	
			return SUCCESS;//电压正常
		}    
	}
		return ERROR;
}

/***************************************************************************************
  * @说明  	T1中断服务函数
  *	@参数	无
  * @返回值 无
  * @注		无
***************************************************************************************/
void TIMER1_Rpt(void) interrupt TIMER1_VECTOR
{
     HandShake_Count++;
}

