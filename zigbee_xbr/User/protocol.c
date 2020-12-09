/**
* @file  protocol.c
* @brief this file contains protocol analysis and construct response function when received zigbee module send message
* @author luchao
* @date 2020.03.13
* @par email:
* luchao@tuya.com
* @copyright HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
* @par company
* http://www.tuya.com
*/

#include "HC89S003F4.h"
#include "zigbee.h"
#include "string.h"
#include <stdio.h>

extern u8 xdata switchcnt;      //复位模块点击次数计数
extern u8 xdata reset_bt_bn;    //复位模块的全局变量
extern u8 xdata SWITCHflag2;   //开关灯的变量
extern u8 xdata SWITCHfXBR;    //开关雷达的变量
extern u8 xdata lightvalue;    //灯亮值
extern u8 xdata XRBoffbrightvalue;  //关雷达后的灯亮值
extern ulong xdata TH;          //雷达感应偏差阈值，数值越大代表越不灵敏
extern u8 xdata LIGHT_TH;       //感光阈值
extern u16 xdata DELAY_NUM;     //感应延时，单位为秒
extern u8 xdata lowlightDELAY_NUM;      //关灯延时，单位为分钟
extern u8 xdata light_ad;               //采到的光感的瞬时值
u8 xdata cdsvalue = 0;              //感光选择值
ulong xdata sensing_th = 0;     //雷达感应阈值，数值越大越灵敏
extern  u8 xdata Linkage_flag;	//联动的开关的全局
extern  u8 xdata Light_on_flag;	//

extern u8 xdata all_day_micro_light_enable;
extern u16 xdata radar_trig_times;
extern u8 xdata light_status_xxx;
extern u8 xdata person_in_range_flag;

void send_data(u8 d);
void reset_bt_module(void);
unsigned char PWM3init(unsigned char ab);
void savevar(void);
void Flash_EraseBlock(unsigned int fui_Address);//flash扇区擦除
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address);//flash写入
void Delay_us_1(uint q1);

void reset_bt_module(void)
{
	mcu_network_start();
}
/******************************************************************************
                                移植须知:
1:MCU必须在while中直接调用mcu_api.c内的zigbee_uart_service()函数
2:程序正常初始化完成后,建议不进行关串口中断,如必须关中断,关中断时间必须短,关中断会引起串口数据包丢失
3:请勿在中断/定时器中断内调用上报函数
******************************************************************************/
/******************************************************************************
                              第一步:初始化
1:在需要使用到zigbee相关文件的文件中include "zigbee.h"
2:在MCU初始化中调用mcu_api.c文件中的zigbee_protocol_init()函数
3:将MCU串口单字节发送函数填入protocol.c文件中uart_transmit_output函数内,并删除#error
4:在MCU串口接收函数中调用mcu_api.c文件内的uart_receive_input函数,并将接收到的字节作为参数传入
5:单片机进入while循环后调用mcu_api.c文件内的zigbee_uart_service()函数
******************************************************************************/

/******************************************************************************
                        1:dp数据点序列类型对照表
          **此为自动生成代码,如在开发平台有相关修改请重新下载MCU_SDK**         
******************************************************************************/

///> dp data list, this will be generated by cloud platform
const DOWNLOAD_CMD_S xdata download_cmd[] =
{
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
  {DPID_CDS, DP_TYPE_ENUM},
  {DPID_PIR_DELAY, DP_TYPE_VALUE},
  {DPID_SWITCH_XBR, DP_TYPE_BOOL},
  {DPID_STANDBY_TIME, DP_TYPE_VALUE},
  {DPID_SENSE_STRESS, DP_TYPE_VALUE},
  {DPID_SWITCH_LED2, DP_TYPE_BOOL},
  {DPID_SWITCH_LINKAGE, DP_TYPE_BOOL},
  {DPID_ALL_DAY_MICRO_LIGHT, DP_TYPE_BOOL},
  {DPID_RADAR_TRIGGER_TIMES, DP_TYPE_VALUE},
  {DPID_CLEAR_TRIGGER_NUMBER, DP_TYPE_BOOL},
  {DPID_LIGHT_STATUS, DP_TYPE_ENUM},
  {DPID_PERSON_IN_RANGE, DP_TYPE_ENUM},
};


/******************************************************************************
                           2:串口单字节发送函数
请将MCU串口发送函数填入该函数内,并将接收到的数据作为参数传入串口发送函数
******************************************************************************/

static void report_mcu_ota_result(unsigned char  res);


/**
* @brief encapsulates a generic send function, developer should use their own function to completing this fuction 
* @param[in] {value} send signle data 
* @return  void
*/
void uart_transmit_output(unsigned char value)
{
// #error "请将MCU串口发送函数填入该函数,并删除该行"
    send_data(value);
	
/*
  //示例:
  extern void Uart_PutChar(unsigned char value);
  Uart_PutChar(value);	                                //串口发送函数
*/
}

/******************************************************************************
                           第二步:实现具体用户函数
1:APP下发数据处理
2:数据上报处理
******************************************************************************/

/******************************************************************************
                            1:所有数据上报处理
当前函数处理全部数据上报(包括可下发/可上报和只上报)
  需要用户按照实际情况实现:
  1:需要实现可下发/可上报数据点上报
  2:需要实现只上报数据点上报
此函数为MCU内部必须调用
用户也可调用此函数实现全部数据上报
******************************************************************************/

//自动化生成数据上报函数

/**
* @brief Upload all dp information of the system, and realize the synchronization of APP and muc data
* @param[in] {void}
* @return  void
*/
void all_data_update(void)
{
    u8 light;
    u8 radius;
  //#error "请在此处理可下发可上报数据及只上报数据示例,处理完成后删除该行"
  //此代码为平台自动生成，请按照实际数据修改每个可下发可上报函数和只上报函数
	
    mcu_dp_bool_update(DPID_SWITCH_LED, reset_bt_bn); //复位模块
    mcu_dp_bool_update(DPID_SWITCH_LED2, SWITCHflag2); //灯的开关
    mcu_dp_value_update(DPID_BRIGHT_VALUE, lightvalue); //VALUE型数据上报;

	if(LIGHT_TH==255)
		light=0;
	else if(LIGHT_TH==200)
		light=2;
	else if(LIGHT_TH==40)
		light=3;		
	else if(LIGHT_TH==20)
		light=4;
	else //其它值
		light=5;

    mcu_dp_enum_update(DPID_CDS, light); //枚举型数据上报;
    mcu_dp_value_update(DPID_PIR_DELAY, DELAY_NUM); //VALUE型数据上报;
    mcu_dp_bool_update(DPID_SWITCH_XBR, SWITCHfXBR); //BOOL型数据上报;
    mcu_dp_value_update(DPID_STANDBY_TIME, lowlightDELAY_NUM); //VALUE型数据上报;

	radius=TH/10000;
	radius=50-radius;

    mcu_dp_value_update(DPID_SENSE_STRESS, radius); //VALUE型数据上报;


	
	mcu_dp_bool_update(DPID_SWITCH_LINKAGE,Linkage_flag); //BOOL型数据上报;
	
	mcu_dp_bool_update(DPID_ALL_DAY_MICRO_LIGHT,all_day_micro_light_enable); //BOOL型数据上报;
    mcu_dp_value_update(DPID_RADAR_TRIGGER_TIMES,radar_trig_times); //VALUE型数据上报;
    mcu_dp_enum_update(DPID_LIGHT_STATUS,light_status_xxx); //枚举型数据上报;
	mcu_dp_enum_update(DPID_PERSON_IN_RANGE,person_in_range_flag); //枚举型数据上报;

}

/******************************************************************************
                                WARNING!!!    
                            2:所有数据上报处理
自动化代码模板函数,具体请用户自行实现数据处理
******************************************************************************/

///> this will realize by  cloud platform

/*****************************************************************************
函数名称 : dp_download_switch_led_handle
功能描述 : 针对DPID_SWITCH_LED的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_led_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_led;
    
    switch_led = mcu_get_dp_download_bool(value,length);

    reset_bt_bn = switch_led;
  
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_LED, reset_bt_bn);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_bright_value_handle
功能描述 : 针对DPID_BRIGHT_VALUE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_bright_value_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long bright_value;
    unsigned char i;
    
    bright_value = mcu_get_dp_download_value(value,length);
	
	//DPID_BRIGHT_VALUEcount++;
	if(bright_value==lightvalue)
	{
/* 		if(DPID_BRIGHT_VALUEcount<2)
		{
			//DPID_BRIGHT_VALUEcount = 0;
			
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_BRIGHT_VALUE,bright_value,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		//DPID_BRIGHT_VALUEcount=0;
		for(i=0;i<8;i++)
		{
//			if(groupaddr[i] != 0)
//			{
//				mcu_dp_value_mesh_update(DPID_BRIGHT_VALUE,bright_value,groupaddr[i]);
//			}
		}
	}	
	
    lightvalue = bright_value;

	//if(SWITCHfXBR==0)
	{
		XRBoffbrightvalue = bright_value;
	}
	
	savevar();
		
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_BRIGHT_VALUE, lightvalue);

    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_cds_handle
功能描述 : 针对DPID_CDS的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_cds_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为ENUM
    unsigned char ret;
    unsigned char cds;
    unsigned char i;
    
    cds = mcu_get_dp_download_enum(value,length);
	
	//DPID_CDScount++;
	if(cds==cdsvalue)
	{
		//if(DPID_CDScount<2)
		{
/* 			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_enum_mesh_update(DPID_CDS,cds,groupaddr[i]);
				}
			} */
		}
		if((cds==5)&&(light_ad!=LIGHT_TH))
		{
			//DPID_CDScount=0;
		}
	}
	else
	{
		//DPID_CDScount=0;
		for(i=0;i<8;i++)
		{
//			if(groupaddr[i] != 0)
//			{
//				mcu_dp_enum_mesh_update(DPID_CDS,cds,groupaddr[i]);
//			}
		}
	}	
	
    switch(cds) {
        case 0:		//2000LUS
			LIGHT_TH=255;//cds*4;
        break;
        
        case 1:		//300LUX
			LIGHT_TH=255;//cds*4;
        break;
        
        case 2:		//50LUX
			LIGHT_TH=200;
        break;
        
        case 3:	//10LUX
			LIGHT_TH=40;
        break;
        
        case 4:	//5LUX
			LIGHT_TH=20;
        break;
        
		case 5:
			LIGHT_TH = light_ad;
		break;
				
        default:
    
        break;
    }

    cdsvalue = cds;

    savevar();
    //sprintf(temp_str, "%3d", LIGHT_TH);
    //mcu_dp_string_update(DPID_DEBUG, temp_str, strlen(temp_str));
    //处理完DP数据后应有反馈
    ret = mcu_dp_enum_update(DPID_CDS, cdsvalue);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_pir_delay_handle
功能描述 : 针对DPID_PIR_DELAY的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_pir_delay_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long pir_delay;
    unsigned char i;
    
    pir_delay = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    */
	
	//DPID_PIR_DELAYcount++;
	if(pir_delay==DELAY_NUM)
	{
/* 		if(DPID_PIR_DELAYcount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_PIR_DELAY,pir_delay,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		//DPID_PIR_DELAYcount=0;
		for(i=0;i<8;i++)
		{
//			if(groupaddr[i] != 0)
//			{
//				mcu_dp_value_mesh_update(DPID_PIR_DELAY,pir_delay,groupaddr[i]);
//			}
		}
	}
	
    DELAY_NUM = pir_delay;
	savevar();
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_PIR_DELAY, DELAY_NUM);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_switch_xbr_handle
功能描述 : 针对DPID_SWITCH_XBR的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_xbr_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_xbr;
    unsigned char i;
    
    switch_xbr = mcu_get_dp_download_bool(value,length);
	
	//DPID_SWITCH_XBRcount++;
	if(switch_xbr==SWITCHfXBR)
	{
/* 		if(DPID_SWITCH_XBRcount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_bool_mesh_update(DPID_SWITCH_XBR,switch_xbr,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		//DPID_SWITCH_XBRcount=0;
		for(i=0;i<8;i++)
		{
//			if(groupaddr[i] != 0)
//			{
//				mcu_dp_bool_mesh_update(DPID_SWITCH_XBR,switch_xbr,groupaddr[i]);
//			}
		}
	
	}
	
    if(switch_xbr == 0) {
        //开关关
        SWITCHfXBR = 0;
        //mcu_dp_string_update(DPID_DEBUG, radar_bn_off, strlen(radar_bn_off));
    }else {
        //开关开
        SWITCHfXBR = 1;
        //mcu_dp_string_update(DPID_DEBUG, radar_bn_on, strlen(radar_bn_on));
    }
  
    savevar();
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_XBR,SWITCHfXBR);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_standby_time_handle
功能描述 : 针对DPID_STANDBY_TIME的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_standby_time_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long standby_time;
    unsigned char i;
    
    standby_time = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
	//DPID_STANDBY_TIMEcount++;
	if(standby_time==lowlightDELAY_NUM)
	{
/* 		if(DPID_STANDBY_TIMEcount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_STANDBY_TIME,standby_time,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		//DPID_STANDBY_TIMEcount=0;
		for(i=0;i<8;i++)
			{
//				if(groupaddr[i] != 0)
//				{
//					mcu_dp_value_mesh_update(DPID_STANDBY_TIME,standby_time,groupaddr[i]);
//				}
			}
	
	}
	
    lowlightDELAY_NUM=standby_time;
    
    savevar();
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_STANDBY_TIME, lowlightDELAY_NUM);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_sense_stress_handle
功能描述 : 针对DPID_SENSE_STRESS的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_sense_stress_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long sense_stress;
    unsigned char i;
    
    sense_stress = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
	//DPID_SENSE_STRESScount++;
	if(sense_stress==sensing_th)
	{
/* 		if(DPID_SENSE_STRESScount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_SENSE_STRESS,sense_stress,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		//DPID_SENSE_STRESScount=0;
		for(i=0;i<8;i++)
		{
//			if(groupaddr[i] != 0)
//			{
//				mcu_dp_value_mesh_update(DPID_SENSE_STRESS,sense_stress,groupaddr[i]);
//			}
		}
	}	
	
	sensing_th = sense_stress;
	TH=(50-sense_stress)*10000;
		
	savevar();
    
    //sprintf(temp_str, "%6d", TH);
    //mcu_dp_string_update(DPID_DEBUG, temp_str, strlen(temp_str));    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_SENSE_STRESS, sensing_th);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_switch_led2_handle
功能描述 : 针对DPID_SWITCH_LED2的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_led2_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_led2;
    unsigned char i;
    
    switch_led2 = mcu_get_dp_download_bool(value,length);

    //DPID_SWITCH_LED2count++;
    if(switch_led2==SWITCHflag2)
    {
/*     	if(DPID_SWITCH_LED2count<2)
    	{
    		for(i=0;i<8;i++)
    		{
    			if(groupaddr[i] != 0)
    			{
    				mcu_dp_bool_mesh_update(DPID_SWITCH_LED2,switch_led2,groupaddr[i]);
    			}
    		}
    	} */
    }
    else
    {
    	//DPID_SWITCH_LED2count=0;
    	for(i=0;i<8;i++)
    	{
//    		if(groupaddr[i] != 0)
//    		{
//    			mcu_dp_bool_mesh_update(DPID_SWITCH_LED2,switch_led2,groupaddr[i]);
//    		}
    	}   

    }

    if(switch_led2 == 0) {
        //灯开关关
        SWITCHflag2=0;
    }else {
        //灯开关开
        //mcu_dp_string_update(DPID_DEBUG, led_bn_on, strlen(led_bn_on));
        if(SWITCHfXBR==1)
		{
			Light_on_flag=1;
		}
        SWITCHflag2=1;
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_LED2, SWITCHflag2);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_switch_linkage_handle
功能描述 : 针对DPID_SWITCH_LINKAGE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_linkage_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_Linkage;
    unsigned char i;
    switch_Linkage = mcu_get_dp_download_bool(value,length);
		
	if(switch_Linkage==Linkage_flag)
	{
		//
	}
	else
	{
		for(i=0;i<8;i++)
		{
//			if(groupaddr[i] != 0)
//			{
//				mcu_dp_bool_mesh_update(DPID_SWITCH_LINKAGE,switch_Linkage,groupaddr[i]);
//			}
		}
	}
    if(switch_Linkage == 0) {
        //雷达开关关
        //LIGHT_OFF;
        //PWM3init(0);
        Linkage_flag=0;
    }else {
        //雷达开关开
        //LIGHT_ON;
        //PWM3init(100);
        Linkage_flag=1;
    }
  	
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_LINKAGE,switch_Linkage);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;

}
/*****************************************************************************
函数名称 : dp_download_all_day_micro_light_handle
功能描述 : 针对DPID_ALL_DAY_MICRO_LIGHT的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_all_day_micro_light_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char all_day_micro_light;
		u8 i;
    
    all_day_micro_light = mcu_get_dp_download_bool(value,length);
	
    if(all_day_micro_light_enable == all_day_micro_light)
    {
		//
    }
    else
    {
    	for(i=0;i<8;i++)
    	{
    		//if(groupaddr[i] != 0)
    		//{
    			//mcu_dp_bool_mesh_update(DPID_ALL_DAY_MICRO_LIGHT,all_day_micro_light,groupaddr[i]);
    		//}
    	}   

    }
	
	all_day_micro_light_enable = all_day_micro_light;
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_ALL_DAY_MICRO_LIGHT, all_day_micro_light_enable);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;

}
/*****************************************************************************
函数名称 : dp_download_clear_trigger_number_handle
功能描述 : 针对DPID_CLEAR_TRIGGER_NUMBER的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 只下发类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_clear_trigger_number_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    //unsigned char ret;
    //0:关/1:开
    unsigned char clear_trigger_number;
    
    clear_trigger_number = mcu_get_dp_download_bool(value,length);
    if(clear_trigger_number == 0) {
        //开关关
    }else {
        //开关开
		radar_trig_times = 0;
		mcu_dp_value_update(DPID_RADAR_TRIGGER_TIMES,radar_trig_times); //VALUE型数据上报;
    }
  
  	return SUCCESS;
}




#ifdef SUPPORT_MCU_RTC_CHECK
/**
* @brief mcu check local RTC time 
* @param[in] {time} timestamp
* @return  void
*/
void mcu_write_rtctime(unsigned char time[])
{
  #error "mcu should realize RTC time wtriting fuction, and delete this line"
  /*
  time[0]~time[3]：standard time
  time[4]~time[7]: Local time
 */
	my_memcpy((void *)timestamp,(const char *)time,4);	//get timestamp
	zigbee_timestamp_to_time();	
/*
	year = _time.w_year;	//year
	month = _time.w_month;	//month
	date = _time.w_date;	//date
	hour = _time.hour + 8;	//hour(8:BeiJing time)
	min = _time.min;	//minute
	sec = _time.sec;	//second
*/
}
#endif


/**
* @brief Zigbee functional test feedback
* @param[in] {void} 
* @return  void
*/
void zigbee_test_result(void)
{
//  #error "this test is makesure the rf fuction of zigbee module, if test pass or not should do something, mcu should realize"
  unsigned char rssi = zigbee_uart_rx_buf[DATA_START];
	
  if(rssi > 0x3C)	{
    //test sucess the range of rssi is 0% ~ 100%
		//do some thing here!!!
		//
		//
		//
  }
  else{
    //test failure
  }
  
}

/******************************************************************************
                                WARNING!!!                     
以下函数用户请勿修改!!
******************************************************************************/

/**
* @brief this function will handle uart received frame data  
* @param[in] {dpid}   dp id
* @param[in] {value}  dp data 
* @param[in] {length} lenght of dp data 
* @return  handle result 
*/
unsigned char dp_download_handle(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  /* only list of function, mcu need realize these fuction*/
  unsigned char ret;
  switch(dpid){
          case DPID_SWITCH_LED:
            //开关处理函数
            ret = dp_download_switch_led_handle(value,length);
			if(ret==1)
			{
				switchcnt ++;
				if(switchcnt>=5)
				{
					switchcnt = 0;
                    reset_bt_module();
				}
			}
        break;
        case DPID_BRIGHT_VALUE:
            //亮度值处理函数
            ret = dp_download_bright_value_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_CDS:
            //光敏参数处理函数
            ret = dp_download_cds_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_PIR_DELAY:
            //感应延时处理函数
            ret = dp_download_pir_delay_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SWITCH_XBR:
            //雷达开关处理函数
            ret = dp_download_switch_xbr_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_STANDBY_TIME:
            //伴亮延时处理函数
            ret = dp_download_standby_time_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SENSE_STRESS:
            //感应强度处理函数
            ret = dp_download_sense_stress_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SWITCH_LED2:
            //开关灯处理函数
            ret = dp_download_switch_led2_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SWITCH_LINKAGE:
            //联动处理函数
            ret = dp_download_switch_linkage_handle(value,length);
			switchcnt = 0;
        break;
        case DPID_ALL_DAY_MICRO_LIGHT:
            //全天伴亮处理函数
            ret = dp_download_all_day_micro_light_handle(value,length);
			switchcnt = 0;
        break;
        case DPID_CLEAR_TRIGGER_NUMBER:
            //计数清零处理函数
            ret = dp_download_clear_trigger_number_handle(value,length);
			switchcnt = 0;
        break;

  default:
        switchcnt = 0;
    break;
  }
  return ret;
}

/**
* @brief get received cmd total number
* @param[in] {void}   
* @return  received cmd total number
*/
unsigned char get_download_cmd_total(void)
{
  return(sizeof(download_cmd) / sizeof(download_cmd[0]));
}


/**
* @brief received zigbee net_work state handle 
* @param[in] {zigbee_work_state}  zigbee current network state
* @return  void 
*/
void zigbee_work_state_event(unsigned char zigbee_work_state)
{	
	unsigned short length= 0;
	
	length = set_zigbee_uart_byte(length,0x10);
	zigbee_uart_write_frame(ZIGBEE_STATE_CMD, length);	

	switch(zigbee_work_state){
		case ZIGBEE_NOT_JION:	
			//mcu_network_start();
			//mcu_reset_zigbee();
			break;
		
		case ZIGBEE_JOIN_GATEWAY:	
					all_data_update();
					savevar();
			break;
		
		case ZIGBEE_JOIN_ERROR:	

			break;
		
		case ZIGBEE_JOINING:	

			break;
		
		default:
			break;
	}
}


/**
* @brief received reset zigbee response 
* @param[in] {state} response state 
* @return  void 
*/
void mcu_reset_zigbee_event(unsigned char state)
{	
	switch(state){
		case RESET_ZIGBEE_OK:
		
			break;
		
		case RESET_ZIGBEE_ERROR:
		
			break;
		
		default:
			break;
	}
}


/**
* @brief check mcu version response
* @param[in] {void}
* @return  void 
*/
void response_mcu_ota_version_event(void)
{
	unsigned short length = 0;
	length = set_zigbee_uart_byte(length,get_current_mcu_fw_ver());	//current fw version
	zigbee_uart_write_frame(MCU_OTA_VERSION_CMD,length);
}

#ifdef SUPPORT_MCU_OTA
/**
* @brief mcu ota update notify response
* @param[in] {offset} data offset 
* @return  void 
*/
void response_mcu_ota_notify_event(unsigned char offset)
{
	unsigned char i = 0;
	unsigned short length = 0;
	
	current_mcu_fw_pid();	//current PID
	
	while(i<8){
		ota_fw_info.mcu_ota_pid[i] = zigbee_uart_rx_buf[offset + DATA_START + i];								//ota fw PID
		i++;
	}
	ota_fw_info.mcu_ota_ver = zigbee_uart_rx_buf[offset + DATA_START + 8];											//ota fw version
	ota_fw_info.mcu_ota_fw_size = zigbee_uart_rx_buf[offset + DATA_START + 9] << 24 | \
																zigbee_uart_rx_buf[offset +DATA_START + 10] << 16 | \
																zigbee_uart_rx_buf[offset + DATA_START + 11] << 8 | \
																zigbee_uart_rx_buf[offset + DATA_START + 12];								//ota fw size
	ota_fw_info.mcu_ota_checksum = zigbee_uart_rx_buf[offset + DATA_START + 13] << 24 | \
																 zigbee_uart_rx_buf[offset + DATA_START + 14] << 16 | \
																 zigbee_uart_rx_buf[offset + DATA_START + 15] << 8 | \
																 zigbee_uart_rx_buf[offset + DATA_START + 16];								//ota fw checksum
	
	if((!strcmp_barry(&ota_fw_info.mcu_ota_pid[0],&current_mcu_pid[0])) && \
		 (ota_fw_info.mcu_ota_ver > get_current_mcu_fw_ver() &&\
		  ota_fw_info.mcu_ota_fw_size > 0)	
		){		//check fw pid and fw version and fw size
		length = set_zigbee_uart_byte(length,0x00);	//OK
	}
	else{
		length = set_zigbee_uart_byte(length,0x01);	//error
	}
    ota_fw_info.mcu_current_offset = 0;
	zigbee_uart_write_frame(MCU_OTA_NOTIFY_CMD,length);
}


/**
* @brief received mcu ota data request response
* @param[in] {fw_offset}  offset of file 
* @param[in] {data}  received data  
* @return  void 
*/

void reveived_mcu_ota_data_handle(unsigned int fw_offset, char *data0)
{
	//#error "received frame data, should save in flash, mcu should realize this fuction, and delete this line "
	unsigned int offset;
	
	offset = fw_offset;
	
	return;
}

/**
* @brief mcu send ota data request 
* @param[in] {void}  
* @return  void 
*/
void mcu_ota_fw_request_event(unsigned char offset)
{	
	unsigned int fw_offset;
	char fw_data[FW_SINGLE_PACKET_SIZE] = {-1};	//
	unsigned char i = 0;

	if(zigbee_uart_rx_buf[offset + DATA_START] == 0x01)				//status check
		return;
	while(i < 8){
		if(current_mcu_pid[i] != zigbee_uart_rx_buf[offset + DATA_START + 1 + i])	//pid check
			return;
		i++;
	}
	if(ota_fw_info.mcu_ota_ver != zigbee_uart_rx_buf[offset + DATA_START + 9]) //version check
		return;
	
	i = 0;
	while(i < 4){
		fw_offset |= (zigbee_uart_rx_buf[offset + DATA_START + 10 + i] << (24 - i * 8));		//offset
		i++;
	}
	i = 0;
	if(ota_fw_info.mcu_current_offset ==  fw_offset)
	{
		if((ota_fw_info.mcu_ota_fw_size - fw_offset) / FW_SINGLE_PACKET_SIZE != 0){
			while(i < FW_SINGLE_PACKET_SIZE){
				fw_data[i] = zigbee_uart_rx_buf[offset + DATA_START + 14 + i];   //fw data
				i++;
			}
			ota_fw_info.mcu_current_offset += FW_SINGLE_PACKET_SIZE;
		}
		else {
			i = 0;
			while(i < (ota_fw_info.mcu_ota_fw_size - fw_offset)){
				fw_data[i] = zigbee_uart_rx_buf[offset + DATA_START + 14 + i];
				i++;
			}
			if(ota_fw_info.mcu_ota_checksum !=\
				(fw_data[i -1 - 3] << 24 |\
					fw_data[i -1 - 2] << 16 |\
					fw_data[i -1 - 1] << 8 |\
					fw_data[i -1 - 0] ))	
					{
						//ota failure report ota failure and clear ota struct 
					    my_memset(&ota_fw_info,0,sizeof(ota_fw_info));
						report_mcu_ota_result(0);
						return;	
					}	
					else
					{
						//ota sucess 
						//should report ota sucess notify 
						report_mcu_ota_result(1);
					}																	
		}
	   ota_fw_data_handle(fw_offset,&fw_data[0]);	//OTA paket data handle
	}
	else
	{
		// ota request timeout, then restart ota request from  ota_fw_info.mcu_ota_fw_size
	}
}

static void report_mcu_ota_result(unsigned char  res)
{
	unsigned short length;
	if((res==0)||(res == 1))
	{
		length = set_zigbee_uart_byte(length,res);	
		zigbee_uart_write_frame(MCU_OTA_NOTIFY_CMD,length);
	}
}


/**
* @brief mcu ota data result notify
* @param[in] {void} 
* @return  void 
*/
void mcu_ota_result_event(unsigned char offset)
{
	unsigned char status = zigbee_uart_rx_buf[offset + DATA_START];
	
	if(status == 0x00){
	}
	else if(status == 0x01)	{

	}
}

/**
* @brief mcu ota data handle 
* @param[in] {fw_offset} frame offset 
* @param[in] {data} received data
* @return  void 
*/
void ota_fw_data_handle(unsigned int fw_offset,char *data0)
{
	//#error "请在该函数处理固件包数据,并删除该行"
	unsigned int offset;
	
	offset = fw_offset;	
	
	
	return;
}
#endif
