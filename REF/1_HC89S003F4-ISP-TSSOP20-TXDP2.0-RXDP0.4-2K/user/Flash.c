#include "include.h"

unsigned char  Read_F_dat  =   0;
unsigned char  xdata  ID_DAT[2]={0};
/*************************************************************
  Function   : CRC_FLASH_BLANK 
  Description: add 地址  Size_Flash 长度
  Input      : guc_Uartbuf-发送的字节      
  return     : 0  !0    
*************************************************************/
unsigned char  XOR_FLASH_BLANK(unsigned int Add,unsigned int Size_Flash)
{
	unsigned int  i=0;
	WDTC |= 0x10;                   //清狗
	while(Size_Flash--)
	if(*(unsigned char code *)(Add++)) return 1;
	return 0;
}
/*********************************
	函数名称：IAP清除扇区函数
	函数功能：清除一个扇区的数据
			  输入扇区地址为当前扇区内任意地址										   
*********************************/
void IAR_Clear(unsigned int clradd)
{
	if(clradd>=0x3800) 
	return;	
	IAP_CMDH = 0xF0;
	IAP_CMDL = 0x0F;
	IAP_ADDR = clradd;
	IAP_CMDH = 0xD2; //选择操作方式， 扇区擦除
	IAP_CMDL = 0x2D;
	IAP_CMDH = 0xE1; //触发
	IAP_CMDL = 0x1E; //触发后 IAP_ADDRL&IAP_ADDRH 指向 0xFF，同时自动锁定
}
/*********************************
	函数名称：IAP清除扇区函数
	函数功能：全扇区擦除										   
*********************************/
void IAR_Clear_arrang(unsigned int clradd,unsigned int len)
{
	unsigned int i;
	WDTC |= 0x10;                   //清狗	
	for(i=0;i<=len;i++)
	{
		if(clradd>=0x3800) 			return;	
		if(XOR_FLASH_BLANK(clradd,128))		IAR_Clear(clradd);
		clradd=clradd+0x80;
	}
}
/*********************************
	函数名称： Earse_Flash(void)函数	
	函数功能： 擦除Flash 前12K						   
*********************************/
unsigned char Earse_Flash(void)
{//003 16K 0000~3fff (擦除前12K)
	IAR_Clear_arrang(0x0000,32);	
	IAR_Clear_arrang(0x1000,32);
	IAR_Clear_arrang(0x2000,32);
	IAR_Clear_arrang(0x3000,16);	
	if(XOR_FLASH_BLANK(0x0000,0x2FFF)!=0x00)  return 0;
	else  return 1;									
}	
/*********************************
	函数名称：FLASH写一个字节
	函数功能：在FLASH任意位置写入一个字节的数据										   
*********************************/
void IAR_Write_Byte(unsigned int add,unsigned char datt)
{
	IAP_DATA = datt; //待编程数据，写入数据寄存器必须放在解锁之前
	/*命令寄存器---解锁*/
	IAP_CMD=0xF00F;
	/*地址寄存器---写地址*/
	IAP_ADDR=add;
	/*命令寄存器---字节编程*/
	IAP_CMD=0xB44B;
	/*命令寄存器---触发一次*/
	IAP_CMD=0xE11E;
}

/*********************************
	函数名称：FLASH写多个字节
	函数功能：FLASH任意位置开始，写任意长度的数据										   
*********************************/
void IAR_Write_arrang(unsigned int add,unsigned char *datt,unsigned int len)
{
	unsigned int i;
	for(i=0;i<len;i++)
	{	
		if(add>=0x3800) return; 	
		IAR_Write_Byte(add,datt[i]);
		add++;
	}
}

/*******************************************************
	函数名称：FLASH读任意个字节
	函数功能：FLASH任意位置开始，读任意长度的数据										   
*******************************************************/
void IAR_Read(unsigned int add,unsigned char *buf,unsigned int len)
{
	while(len--)
	*(buf++)=*((unsigned char code *)(add++));//读取数据
}

/*******************************************************
	函数名称：FLASH软件复位
	函数功能：不重读Option									   
*******************************************************/
void IAR_Soft_Rst_No_Option(void)
{
	IAP_CMD = 0xF00F;
	IAP_CMD = 0x8778;
}
/*******************************************************
	函数名称：FLASH软件复位
	函数功能：重读Option									   
*******************************************************/
void IAR_Soft_Rst_Option(void)
{
	IAP_CMD = 0xF00F;
	IAP_CMD = 0x7887;
}

/*********************************
	函数名称： Read_ID(void)函数	
	函数功能： 读取芯片型号								   
*********************************/
bit Read_ID(void)
{
 	INSCON=0x10;
	IAR_Read(0x0120,ID_DAT,1);
	INSCON=0x00;

	if((ID_DAT[0])==Chip_ID)
	{   
        return SUCCESS;
	}
	else
	{	  
        return ERROR;
	}
}
