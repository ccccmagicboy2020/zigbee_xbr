#ifndef __MODB_H__
#define __MODB_H__

#define    Version             0x03//版本号

#define    Get_Version_Internal_Order 0x00
#define    Get_Version_OutSide_Order  0x01
#define    Get_ID                     0x02

#define    Erase_Flash                0x11
#define    Erase_Option               0x12
#define    Erase_Flash_ALL            0x13

#define    Write_Memory               0x21
#define    Write_Option               0x22

#define    Read_Memory                0x31
#define    Read_Option                0x32

#define    Go_APP                     0x91
#define    Rst_Read_Option            0x92

void          HandShake(void);//握手
unsigned char Receive_Packet (unsigned char *Data);//接收处理命令
unsigned int  CRC_CalcCRC_Process(unsigned char *fucp_CheckArr,unsigned int fui_CheckLen,unsigned char *Data,bit CRC_Flag);//CRC校验
bit           LVD_Check(unsigned long TimeOut);
#endif