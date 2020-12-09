#define ALLOCATE_EXTERN
#include "include.h"
unsigned char xdata Uart_Buf[200];//数据缓存数组
//与上位机的时序调整
//Earse_Flash()函数修改，增加LVD检测，每擦除前判断LVD电压
//Receive_Packet()增加LVD检测，程序结构修改，烧写错误则全擦，查空修改为查空前128个字节
//HandShake()握手时间间隔修改，看门狗打开，TXD引脚初始化靠后，RXD为施密特输入上拉（必须带上拉）BOR 2.0V，LVD 2.4V
int main(void)
{
	SystemInit();//初始化
  HandShake();//握手
	while(1)
	{
		 WDTC |= 0x10;                   //清狗
		switch(Receive_Packet(Uart_Buf))//接收判断
		{
			case SUCCESS://成功发ACK
					      Uart_SendByte(ACK);		
														break;

			case ERROR://失败发NACK
					      Uart_SendByte(NACK);
														break;
			
			case NACK_TIME://超时跳转APP
				          IAR_Soft_Rst_No_Option();
														break;			
			
		  default://其他不操作
                            break;			
		}		
	}
}

