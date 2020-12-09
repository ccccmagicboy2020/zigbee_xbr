#include "include.h"

/*************************************************************
  Function   : IAP_SerialSendByte 
  Description: ����һ���ֽ�
  Input      : guc_Uartbuf-���͵��ֽ�      
  return     : none    
*************************************************************/
void Uart_SendByte(unsigned char guc_Uartbuf)
{  
			SCON &=~0x10;				//UART1����ʹ��	
			SBUF = guc_Uartbuf;         //����8λ��������
			while(!(SCON & 0x02));
			SCON &=~ 0x02;			    //��������жϱ�־λ
			SCON |= 0x10;				//UART1����ʹ��
}

/*************************************************************
  Function   : Uart_SendPacket 
  Description: ����ָ�����ȵ����ݰ�
  Input      : guc_Uartbuf-���ݰ���ַ   Length-�ֽڳ���     
  return     : none    
*************************************************************/
void Uart_SendPacket(unsigned char *guc_Uartbuf, unsigned char Length)
{
    unsigned char i;
    i = 0;
    while (i < Length)
    {
        Uart_SendByte(guc_Uartbuf[i]);
        i++;
    }
}
/*************************************************************
  Function   : IAP_SerialGetByte 
  Description: ����һ���ֽ�
  Input      : guc_Uartbuf-��Ž��յ����ֽ�      
  return     : none     
*************************************************************/
bit Uart_GetByte(unsigned char *guc_Uartbuf)
{
	if(SCON & 0x01)						//�жϽ����жϱ�־λ
	{
		SCON&=~0x01;   				      //�յ����ݺ�������ձ�־			
		*guc_Uartbuf = SBUF;              //ת��8λ���ڽ�������
		return SUCCESS;		
	}
	    return ERROR;	
}

/*************************************************************
  Function   : Uart_RecvByte vv   
  Description: ��ʱ����һ���ֽ�
  Input      : guc_Uartbuf-��Ž��յ����ֽ� TimeOut-��ʱʱ��         
  return     : none    
*************************************************************/
bit Uart_RecvByte(unsigned char *guc_Uartbuf, unsigned long TimeOut)
{
		WDTC |= 0x10;                   //�幷		
	while(TimeOut-- > 0)
	{
		if(Uart_GetByte(guc_Uartbuf) == SUCCESS)
		{
			return SUCCESS;
		}
	}
	return ERROR;
}
