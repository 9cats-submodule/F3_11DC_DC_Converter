#include "hmi_driver.h"
#include "hmi_user_uart.h"
#include "cmd_queue.h"
#include "cmd_process.h"

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stm32f1xx_hal.h"
#include "base.h"

#include "tim.h"

extern float goodAmp[2];
extern u8 mode,autoMode,OK;
extern u16 i,j;
uint8 cmd_buffer[CMD_MAX_SIZE]; //指令缓存

/*! 
 *  \brief  消息处理流程
 *  \param msg 待处理消息
 *  \param size 消息长度
 */

void ProcessMessage(PCTRL_MSG msg, uint16 size)
{
	uint8 cmd_type = msg->cmd_type;				   //指令类型
	uint8 ctrl_msg = msg->ctrl_msg;				   //消息的类型
	uint8 control_type = msg->control_type;		   //控件类型
	uint16 screen_id = PTR2U16(&msg->screen_id);   //画面ID
	uint16 control_id = PTR2U16(&msg->control_id); //控件ID
	uint32 value = PTR2U32(msg->param);			   //数值

	switch (cmd_type)
	{
	case NOTIFY_TOUCH_PRESS:   //触摸屏按下
	case NOTIFY_TOUCH_RELEASE: //触摸屏松开
		NotifyTouchXY(cmd_buffer[1], PTR2U16(cmd_buffer + 2), PTR2U16(cmd_buffer + 4), NULL);
		break;
	case NOTIFY_WRITE_FLASH_OK: //写FLASH成功
		NotifyWriteFlash(1, NULL);
		break;
	case NOTIFY_WRITE_FLASH_FAILD: //写FLASH失败
		NotifyWriteFlash(0, NULL);
		break;
	case NOTIFY_READ_FLASH_OK:								//读取FLASH成功
		NotifyReadFlash(1, cmd_buffer + 2, size - 6, NULL); //去除帧头帧尾
		break;
	case NOTIFY_READ_FLASH_FAILD: //读取FLASH失败
		NotifyReadFlash(0, 0, 0, NULL);
		break;
	case NOTIFY_READ_RTC: //读取RTC时间
		NotifyReadRTC(cmd_buffer[1], cmd_buffer[2], cmd_buffer[3], cmd_buffer[4], cmd_buffer[5], cmd_buffer[6], cmd_buffer[7], NULL);
		break;
	case NOTIFY_CONTROL:
	{
		if (ctrl_msg == MSG_GET_CURRENT_SCREEN) //画面ID变化通知
		{
			NotifyScreen(screen_id, NULL);
		}
		else
		{
			switch (control_type)
			{
			case kCtrlButton: //按钮控件
				NotifyButton(screen_id, control_id, msg->param[1], NULL);
				break;
			case kCtrlText: //文本控件
				NotifyText(screen_id, control_id, msg->param, NULL);
				break;
			case kCtrlProgress: //进度条控件
				NotifyProgress(screen_id, control_id, value, NULL);
				break;
			case kCtrlSlider: //滑动条控件
				NotifySlider(screen_id, control_id, value, NULL);
				break;
			case kCtrlMeter: //仪表控件
				NotifyMeter(screen_id, control_id, value, NULL);
				break;
			case kCtrlMenu: //菜单控件
				NotifyMenu(screen_id, control_id, msg->param[0], msg->param[1], NULL);
				break;
			case kCtrlSelector: //选择控件
				NotifySelector(screen_id, control_id, msg->param[0], NULL);
				break;
			case kCtrlRTC: //倒计时控件
				NotifyTimer(screen_id, control_id, NULL);
				break;
			default:
				break;
			}
		}
	}
	break;
	default:
		break;
	}
}

void SetTFTText(uint16 screen_id, uint16 control_id, char *text, ...)
{
	char buffer[20];
	va_list ap;
	va_start(ap, text);
	vsprintf(buffer, text, ap);
	va_end(ap);
	SetTextValue(screen_id, control_id, (u8 *)buffer);
}

void SetTextValueInt32(uint16 screen_id, uint16 control_id, int32 value)
{
	uchar buffer[12] = {0};
	snprintf((char *)buffer, sizeof(buffer), "%ld", value); //把整数转换为字符串
	SetTextValue(screen_id, control_id, buffer);
}

void SetTextValueFloat(uint16 screen_id, uint16 control_id, float value)
{
	uchar buffer[12] = {0};
//	snprintf((char *)buffer, sizeof(buffer), "%.1f", value); //把浮点数转换为字符串(保留一位小数)
	SetTextValue(screen_id, control_id, buffer);
}

void UpdateUI()
{
}

/*! 
 *  \brief  画面切换通知
 *  \details  当前画面改变时(或调用GetScreen)，执行此函数
 *  \param screen_id 当前画面ID
 */
void NotifyScreen(uint16 screen_id, void *userdata)
{
}

/*! 
 *  \brief  触摸坐标事件响应
 *  \param press 1按下触摸屏，3松开触摸屏
 *  \param x x坐标
 *  \param y y坐标
 */
void NotifyTouchXY(uint8 press, uint16 x, uint16 y, void *userdata)
{
}

/*! 
 *  \brief  按钮控件通知
 *  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param state 按钮状态：0弹起，1按下
 */
extern u8 next_mode;
void NotifyButton(uint16 screen_id, uint16 control_id, uint8 state, void *userdata)
{
	if ((screen_id == 0) && (control_id == 20))
	{
		if(state == 1)
		{
			next_mode = 1;
			autoMode = 1;
			OK = 0;
		}
		else
		{
			next_mode = mode = i =  j = 0;
			autoMode = 0;
			OK = 0;
		}
	}
	if ((screen_id == 0) && (control_id == 24))
	{
		if(state == 1)
		{
			next_mode = 1;
			OK = 0;
		}
	}
	if ((screen_id == 0) && (control_id == 25))
	{
		if(state == 1)
		{
			next_mode = 2;
			OK = 0;
		}
	}
	if ((screen_id == 0) && (control_id == 26))
	{
		if(state == 1)
		{
			next_mode = 3;
			OK = 0;
		}
	}
}

/*! 
 *  \brief  文本控件通知
 *  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param str 文本控件内容
 */
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str, void *userdata)
{
}

/*! 
 *  \brief  进度条控件通知
 *  \details  调用GetControlValue时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value, void *userdata)
{
}

/*! 
 *  \brief  滑动条控件通知
 *  \details  当滑动条改变(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value, void *userdata)
{
	//滚动条 -> TFT 改变 进度条 和 显示的值
	//       -> STM32 改变PWM占空比
	if(!mode)
	{
		if ((screen_id == 0) && (control_id == 3))
		{
			u32 ARR      = htim1.Instance->ARR;
			u32 Next_CCR = value * ARR /100;
			u8 str[20]  = {0};

			sprintf((char*)str,"%3ld / %3ld = %5.2f%%", Next_CCR, ARR, (Next_CCR+1)*100.0/(ARR+1));
			SetProgressValue(0, 2, value);
			SetTextValue(0,34,str);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Next_CCR+1);
			//__HAL_TIM_SET_AUTORELOAD(&htim1,Next_CCR);
		}
	}
}

/*! 
 *  \brief  仪表控件通知
 *  \details  调用GetControlValue时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value, void *userdata)
{
}

/*! 
 *  \brief  菜单控件通知
 *  \details  当菜单项按下或松开时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param item 菜单项索引
 *  \param state 按钮状态：0松开，1按下
 */
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8 item, uint8 state, void *userdata)
{
}

/*! 
 *  \brief  选择控件通知
 *  \details  当选择控件变化时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param item 当前选项
 */

void NotifySelector(uint16 screen_id, uint16 control_id, uint8 item, void *userdata)
{
}

/*! 
 *  \brief  定时器超时通知处理
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 */
void NotifyTimer(uint16 screen_id, uint16 control_id, void *userdata)
{
}

/*! 
 *  \brief  读取用户FLASH状态返回
 *  \param status 0失败，1成功
 *  \param _data 返回数据
 *  \param length 数据长度
 */
void NotifyReadFlash(uint8 status, uint8 *_data, uint16 length, void *userdata)
{
}

/*! 
 *  \brief  写用户FLASH状态返回
 *  \param status 0失败，1成功
 */
void NotifyWriteFlash(uint8 status, void *userdata)
{
}

/*! 
 *  \brief  读取RTC时间，注意返回的是BCD码
 *  \param year 年（BCD）
 *  \param month 月（BCD）
 *  \param week 星期（BCD）
 *  \param day 日（BCD）
 *  \param hour 时（BCD）
 *  \param minute 分（BCD）
 *  \param second 秒（BCD）
 */
void NotifyReadRTC(uint8 year, uint8 month, uint8 week, uint8 day, uint8 hour, uint8 minute, uint8 second, void *userdata)
{
}
