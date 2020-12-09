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
/**********************************����Ӧ�����ʴ��ڳ�ʼ��**************************************/	
	TR1=1;//����ʱ��1
  EA=1;	
	do{	
		if(HandShake_Count>=20)//�ϵ�20msû�м�⵽����
		{
			 IAR_Soft_Rst_No_Option();//����û�м�⵽���ݣ�����APP			  
		}
	}
//	while(P0_4);
//	TR1=0;//�ض�ʱ��T1
//  EA=0;	
//	P2M0 = 0x38;				      //P20����Ϊ�������		
//	TXD_MAP = 0x20;						//TXDӳ��P20
//	RXD_MAP = 0x04;						//RXDӳ��P04

	while(P0_3);
	TR1=0;//�ض�ʱ��T1
  EA=0;	
	P2M0 = 0x83;				      //P21����Ϊ�������
	TXD_MAP = 0x21;						//TXDӳ��P21
	RXD_MAP = 0x03;						//RXDӳ��P03	
	
	T4CON = 0x06;						//T4����ģʽ��UART1�����ʷ�����
  TH4 = 0xFF;
	TL4 = 0xF8;							//������250000
	SCON2 = 0x02;						//8λUART�������ʿɱ�
	SCON = 0x10;						//�����н���
  
	Uart_SendByte(ACK);                 //����OX79
	if(Uart_RecvByte(&Data,HandShark_TIMEOUT)!=SUCCESS) IAR_Soft_Rst_No_Option();//��ʱ��û�н��յ����ݣ�����APP	
	if(Data!=0x7f)                                      IAR_Soft_Rst_No_Option();//���ݲ���OX7F������APP
		
	Uart_SendByte(0x1f);//��һ�η���0x1f
					
	Data=0;
	if(Uart_RecvByte(&Data,HandShark_TIMEOUT)!=SUCCESS) IAR_Soft_Rst_No_Option();//��ʱ��û�н��յ����ݣ�����APP	
	if(Data!=0x7f)                                      IAR_Soft_Rst_No_Option();//���ݲ���OX7F������APP
/*****************************************************************************/	

	
	BORC  =0xC1;			  //BOR:2.0V
	BORDBC=0x14;			  //BOR����  ����ʱ�� = BORDBC[7:0] * 8Tcpu +2 Tcpu	
	
	LVDC = 0xA2;						//LVD����2.4V,��ֹ�ж�
	LVDDBC = 0xFF;				  //��������ʱ��16uS
	
	Uart_SendByte(0x1f);   //�ڶ��η���0x1f	
  
}

unsigned char Receive_Packet (unsigned char *Data)
{
	unsigned char i;
	unsigned int  TempCRC;

	if (Uart_RecvByte(Data,  Command_TIMEOUT)!= SUCCESS)   return NACK_TIME;//��ʱ��û�н��յ����ݣ�����APP
	if (Uart_RecvByte(Data+1,Command_TIMEOUT)!= SUCCESS)   return NACK_TIME;//��ʱ��û�н��յ����ݣ�����APP	        
	if (Data[0]!=((Data[1]^0xff)&0xff))                    return ERROR;	  //��֤����
	
	switch(Data[0])
	{
//		case Get_Version_Internal_Order://��ȡ��ǰ�汾�Լ�����ʹ�õ�����
//				     Uart_SendByte(ACK);//����ͷ�룬����ɹ�
//             Uart_SendByte(Version_Order_Internal_Length);//������Ч���ݳ���		
//			       Uart_SendPacket(ISP_Version_Internal_Order+1, Version_Order_Internal_Length);//������Ч����	
//             TempCRC=CRC_CalcCRC_Process(ISP_Version_Internal_Order,Version_Order_Internal_Length+1,Data,0);//��ȡCRCУ������	
//             Uart_SendByte(TempCRC >> 8);//���͸߰�λ
//             Uart_SendByte(TempCRC & 0xFF);//���͵Ͱ�λ
//             Read_Flag=0;		
//             return SUCCESS;				
//			                                        break;			
		
		case Get_Version_OutSide_Order://��ȡ��ǰ�汾�Լ�����ʹ�õ�����
						 Uart_SendByte(ACK);//����ͷ�룬����ɹ�
             Uart_SendByte(Version_Order_OutSide_Length);//������Ч���ݳ���		
			       Uart_SendPacket(ISP_Version_OutSide_Order+1, Version_Order_OutSide_Length);//������Ч����	
             TempCRC=CRC_CalcCRC_Process(ISP_Version_OutSide_Order,Version_Order_OutSide_Length+1,Data,0);//��ȡCRCУ������	
             Uart_SendByte(TempCRC >> 8);//���͸߰�λ
             Uart_SendByte(TempCRC & 0xFF);//���͵Ͱ�λ	
             Read_Flag=1;		     
             return SUCCESS;				
			                                        break;
		
		case Get_ID:          //��ȡоƬ�ͺ�
						 Uart_SendByte(ACK);//����ͷ�룬����ɹ�
			       if(Read_ID()!=SUCCESS)  return ERROR;	//оƬ�ͺŴ���		
             Uart_SendByte(MCU_ID_Length);//������Ч���ݳ���			 
			       Uart_SendPacket(MCU_ID+1, MCU_ID_Length);//������Ч����	
             TempCRC=CRC_CalcCRC_Process(MCU_ID,MCU_ID_Length+1,Data,0);//��ȡCRCУ������			
             Uart_SendByte(TempCRC >> 8);//���͸߰�λ
             Uart_SendByte(TempCRC & 0xFF);//���͵Ͱ�λ
             return SUCCESS;
			                                        break;

		case Erase_Flash_ALL:    //���� Flash ����
			Uart_SendByte(ACK);//����ͷ�룬����ɹ�
			for (i = 0; i < 2; i ++)//����Ҫ����������
			{
				if (Uart_RecvByte(Data + 2 + i, NAK_TIMEOUT) != SUCCESS) return NACK_TIME;//��ʱ��û�н��յ����ݣ�����APP
			}
      if(Data[2]==0XFF&&Data[3]==0X00)//�ж��Ƿ�ȫ��������	
			{
						if (XOR_FLASH_BLANK(0x0000,0x2FFF)==0x00) return SUCCESS;//ȫ������գ������ɹ�	
            if (LVD_Check(LVD_TIMEOUT)!=SUCCESS)      return ERROR;//��ѹ������						
						if (Earse_Flash())                        return SUCCESS;//ȫ���������ɹ�
						else                                      return ERROR;//ȫ��������ʧ��
			}	
			else
					   return ERROR;			
			                                        break;

		case Write_Memory:  //д Flash ����	
				Uart_SendByte(ACK);//����ͷ�룬����ɹ�			
				for (i=0; i<6; i++)//������ʼ��ַ��CRCУ�飬����Data[i+2]
				{
					if (Uart_RecvByte(Data+2+i, NAK_TIMEOUT) != SUCCESS)  return NACK_TIME;//��ʱ��û�н��յ����ݣ�����APP
				}
				if (CRC_CalcCRC_Process(Data+2,4,Data+6,1)!=SUCCESS )return ERROR;//CRCУ�����
				
	           Uart_SendByte(ACK);					

				if (Uart_RecvByte(Data+8, NAK_TIMEOUT) !=SUCCESS)       return NACK_TIME;//��ʼ�������ݳ��ȣ���Data[8]��ʼ����

					for (i=0; i<(Data[8]+2); i++)//�������ݺ�CRC��ֵ
					{
						if (Uart_RecvByte(Data+9+i, NAK_TIMEOUT) != SUCCESS)      return NACK_TIME;//��ʱ��û�н��յ����ݣ�����APP
					}				
       if (CRC_CalcCRC_Process(Data+8,Data[8]+1,Data+Data[8]+9,1)!=SUCCESS)  return ERROR;//CRCУ�����	
       if(LVD_Check(LVD_TIMEOUT)!=SUCCESS)  return ERROR;//��ѹ������
				 IND_ADDR=0x0f;
				 IND_DATA=0x00;
				 IAR_Write_arrang((unsigned int)(Data[4]<<8)|(unsigned int)Data[5],Data+9,Data[8]);
				 IAR_Read        ((unsigned int)(Data[4]<<8)|(unsigned int)Data[5],Data+9,Data[8]);														  								 

      if (CRC_CalcCRC_Process(Data+8,Data[8]+1,Data+Data[8]+9,1)!=SUCCESS) 
			{
				        IAR_Clear((unsigned int)(Data[4]<<8)|(unsigned int)Data[5]);//������������
								return ERROR;//CRCУ�����					
			}
		  else                                                                
    				return SUCCESS;	
			                                        break;	

		case Go_APP:       //�˳� Bootloader ���� APP ����
			Uart_SendByte(ACK);//����ͷ�룬����ɹ�
			 IAR_Soft_Rst_No_Option();//����û�м�⵽���ݣ�����APP
					    return SUCCESS;			
			                                        break;
		case Rst_Read_Option:       //��λ�ض�Option	
			 Uart_SendByte(ACK);//����ͷ�룬����ɹ�			
			 IAR_Soft_Rst_Option();			

					    return SUCCESS;			

		default:		 
						    return ERROR;//����NACK	
		
			break;
	
	}
}

/*************************************************************
  Function   : CRC_CalcCRC_Compare
  Description: д����ҪУ�������,�ƽϲ����رȽϽ��
  Input      : *fucp_CheckArr - CRCУ�������׵�ַ   fui_CheckLen - CRCУ�����ݳ��� 
               *CRC_Data      - �Ƚϵ�CRC�׵�ַ     CRC_Flag     - 0���Ƚ� 1�Ƚ�
  return     : none    
*************************************************************/
unsigned int CRC_CalcCRC_Process(unsigned char *fucp_CheckArr,unsigned int fui_CheckLen,unsigned char *CRC_Data,bit CRC_Flag)
{

 	CRCC = 0x01;//CRC��λ��MSB first����λ��ֵΪ0x0000   1021		
	while(fui_CheckLen--)CRCL = *(fucp_CheckArr++);
	if(CRC_Flag==1)
	{
		if((unsigned char)(CRCR>>8) ==*(CRC_Data )&& (unsigned char)(CRCR & 0xFF)==*(CRC_Data+1))//CRCУ��߰�λ
		{
				return SUCCESS; //����д���ַ����CRCУ�����ݳɹ�							
		}
		else
				return ERROR;//CRCУ��ʧ��		
	}
	else
		return CRCR;
	
}
/***************************************************************************************
  * @˵��  	LVD��ѯ����
  *	@����	��
  * @����ֵ ��
  * @ע		��
***************************************************************************************/
bit LVD_Check(unsigned long TimeOut)
{
		WDTC |= 0x10;                   //�幷			
	while(TimeOut-- > 0)
	{
		LVDC &=~ 0x08;					        //���LVD�жϱ�־λ 									
		if((LVDC&0x08)==0)						  //�ж�LVD�жϱ�־λ
		{	
			return SUCCESS;//��ѹ����
		}    
	}
		return ERROR;
}

/***************************************************************************************
  * @˵��  	T1�жϷ�����
  *	@����	��
  * @����ֵ ��
  * @ע		��
***************************************************************************************/
void TIMER1_Rpt(void) interrupt TIMER1_VECTOR
{
     HandShake_Count++;
}

