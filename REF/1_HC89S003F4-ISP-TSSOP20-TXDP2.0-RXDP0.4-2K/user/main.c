#define ALLOCATE_EXTERN
#include "include.h"
unsigned char xdata Uart_Buf[200];//���ݻ�������
//����λ����ʱ�����
//Earse_Flash()�����޸ģ�����LVD��⣬ÿ����ǰ�ж�LVD��ѹ
//Receive_Packet()����LVD��⣬����ṹ�޸ģ���д������ȫ��������޸�Ϊ���ǰ128���ֽ�
//HandShake()����ʱ�����޸ģ����Ź��򿪣�TXD���ų�ʼ������RXDΪʩ�������������������������BOR 2.0V��LVD 2.4V
int main(void)
{
	SystemInit();//��ʼ��
  HandShake();//����
	while(1)
	{
		 WDTC |= 0x10;                   //�幷
		switch(Receive_Packet(Uart_Buf))//�����ж�
		{
			case SUCCESS://�ɹ���ACK
					      Uart_SendByte(ACK);		
														break;

			case ERROR://ʧ�ܷ�NACK
					      Uart_SendByte(NACK);
														break;
			
			case NACK_TIME://��ʱ��תAPP
				          IAR_Soft_Rst_No_Option();
														break;			
			
		  default://����������
                            break;			
		}		
	}
}

