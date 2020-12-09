#ifndef __UART_H__
#define __UART_H__

#define NAK_TIMEOUT             400000 //�������ݳ�ʱ2S
#define Command_TIMEOUT         400000 //�����������ݳ�ʱ2S
#define HandShark_TIMEOUT       4000   //�����������ݳ�ʱ20MS
#define LVD_TIMEOUT             400000 //�ж�LVD��ʱ200MS

#define ACK                     0X79
#define NACK                    0X1F
#define SUCCESS                 0X01
#define ERROR                   0X00
#define NACK_TIME               0X02
#define ACK_READ                0X03


void Uart_SendByte(unsigned char guc_Uartbuf);
void Uart_SendPacket(unsigned char *guc_Uartbuf, unsigned char Length);
bit Uart_GetByte(unsigned char *guc_Uartbuf);
bit Uart_RecvByte(unsigned char *guc_Uartbuf, unsigned long TimeOut);

#endif