#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "inc/tm4c1294ncpdt.h"

#define SYSTICK_FREQUENCY 1000 //1000hz

#define I2C_FLASHTIME 500  //500mS
#define GPIO_FLASHTIME 300 //300mS
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 0x22
#define PCA9557_I2CADDR 0x18

#define PCA9557_INPUT 0x00
#define PCA9557_OUTPUT 0x01
#define PCA9557_POLINVERT 0x02
#define PCA9557_CONFIG 0x03

#define TCA6424_CONFIG_PORT0 0x0c
#define TCA6424_CONFIG_PORT1 0x0d
#define TCA6424_CONFIG_PORT2 0x0e

#define TCA6424_INPUT_PORT0 0x00
#define TCA6424_INPUT_PORT1 0x01
#define TCA6424_INPUT_PORT2 0x02

#define TCA6424_OUTPUT_PORT0 0x04
#define TCA6424_OUTPUT_PORT1 0x05
#define TCA6424_OUTPUT_PORT2 0x06

#define CLOCK_DISPLAY 0x00
#define SOLARTERM_DISPLAY 0x01
#define DATE_DISPLAY 0x02
#define ALARM_1 0x04
#define ALARM_2 0x08
#define TIMER_DISPLAY 0x10

#define UART_SET_CLOCK 0x1
#define UART_SET_DATE 0x2
#define UART_SET_ALARM_1 0x3
#define UART_SET_ALARM_2 0x4
#define UART_SET_TIMER 0x5

void Delay(uint32_t value);
void S800_GPIO_Init(void);
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
uint8_t is_leap_year(uint32_t year);
void S800_I2C0_Init(void);
void S800_UART_Init(void);
void S800_INT_Init(void);
void GPIOJ_Handler(void);
void UARTStringGet(char *msg);
void UARTStringPut(const char *cMessage);
void UARTStringPutNonBlocking(const char *cMessage);
void UART_ECHO(char *msg);
void Initial_Sequence(void);
void Display_8tubes(uint8_t *p, uint8_t num);
void clock_carrying(void);
void date_carrying(void);
void clock_display(void);
void shut_down_tubes(void);
void solarterm_display(void);
void date_display(void);
void date_set(void);
void alarm_1(void);
void alarm_2(void);
void timer_display(void);
void timer_set(void);
void alarm_check(void);
void timer_count_down(void);
void uart_set_check(void);
void reboot(void);
void blink(void);

//systick software counter define
volatile uint16_t systick_10ms_couter, systick_100ms_couter, systick_1s_counter, systick_1ms_counter;
volatile uint8_t systick_10ms_status, systick_100ms_status, systick_1s_status, systick_1ms_status;

volatile uint8_t result, cnt, key_value, gpio_status;
volatile uint8_t rightshift = 0x01;
uint32_t ui32SysClock;
uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c};
uint8_t disp_buff[8];
uint8_t STATE = 0;
uint8_t clock_set_flag = 0;
uint8_t timer_flag = 1;
uint8_t day_carrying_flag;
uint8_t switch_byte;
uint8_t tmp_switch_byte;
uint8_t timer_counting;
uint8_t alarm_1_flag = 0;
uint8_t alarm_2_flag = 0;
uint8_t uart_set_status = 0;
uint8_t uart_error_flag = 0;
uint8_t alarm_timer_flag = 1;
uint8_t motor = 0;
uint8_t motor_control[] = {0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08};
uint8_t motor_status = 0;
uint32_t days_in_month_non_leapyear[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint32_t days_in_month_leapyear[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint32_t receive_finished_countdown = 10000;
uint32_t illuminating_level = 3;
char RxBuf[256] = " ";
char *tail = RxBuf;
int RxEndFlag = 0;
//the disp_tab and disp_tab_7seg must be the Corresponding relation
//the last character should be the space(not display) character
char const disp_tab[] = {'0', '1', '2', '3', '4', '5', //the could display char and its segment code
						 '6', '7', '8', '9', 'A', 'b',
						 'C', 'd', 'E', 'F',
						 'H', 'L', 'P', 'o',
						 '.', '-', '_', ' '};
char const disp_tab_7seg[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D,
							  0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C,
							  0x39, 0x5E, 0x79, 0x71,
							  0x76, 0x38, 0x73, 0x5c,
							  0x80, 0x40, 0x08, 0x00};
char ASCII2Disp(char *buff);
char message[50];
char tmp[10];

uint32_t switch_byte_flag;

uint32_t time_sec = 0, time_min = 0, time_hour = 0, time_day = 1, time_month = 1, time_year = 1970;
uint32_t time_solarterm;
uint32_t alarm_1_hour = 0, alarm_1_min = 0, alarm_1_sec = 0;
uint32_t alarm_2_hour = 0, alarm_2_min = 0, alarm_2_sec = 0;
uint32_t timer_min = 0, timer_sec = 0, timer_msec = 0;

int main(void)
{
	int i;
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ | SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 20000000);

	SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY);

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	S800_INT_Init();

	Initial_Sequence();

	while (1)
	{
		uart_set_check();

		if (systick_1s_status && !clock_set_flag)
		{
			systick_1s_status = 0;
			time_sec++;
			clock_carrying();
			date_carrying();
			alarm_check();
		}

		if (systick_1ms_status)
		{
			systick_1ms_status = 0;
			if (motor == 1)
			{
				for (i = 0; i < 8; i++)
				{
					for (motor_status = 0; motor_status < 8; motor_status++)
					{
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, motor_control[motor_status]);
						Delay(4000);
					}
				}
			}
		}

		if (alarm_1_flag)
			I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~STATE - 0x40);
		if (alarm_2_flag)
			I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~STATE - 0x80);
		if (alarm_1_flag && alarm_2_flag)
			I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~STATE - 0x80 - 0x40);
		if (!alarm_1_flag && !alarm_2_flag)
			I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~STATE);

		switch (STATE)
		{
		case CLOCK_DISPLAY:
			clock_display();
			break;
		case SOLARTERM_DISPLAY:
			solarterm_display();
			break;
		case DATE_DISPLAY:
			date_display();
			break;
		case ALARM_1:
			alarm_1();
			break;
		case ALARM_2:
			alarm_2();
			break;
		case TIMER_DISPLAY:
			timer_display();
			break;
		default:
			break;
		}
	}
}

void clock_carrying(void) //clock carrying rule
{
	if (time_sec == 60)
	{
		if (time_min == 59)
		{
			if (time_hour == 23)
			{
				time_sec = 0;
				time_min = 0;
				time_hour = 0;
				day_carrying_flag = 1;
			}
			else
			{
				time_sec = 0;
				time_min = 0;
				time_hour++;
			}
		}
		else
		{
			time_min++;
			time_sec = 0;
		}
	}
}

void date_carrying(void)
{
	if (day_carrying_flag)
	{
		day_carrying_flag = 0;
		if (!is_leap_year(time_year)) // Not leap year
		{
			if (time_day < days_in_month_non_leapyear[time_month - 1])
				time_day++;
			else
			{
				if (time_month == 12) //xxxx.12.31
				{
					time_year++;
					time_month = 1;
					time_day = 1;
				}
				else
				{
					time_day = 1;
					time_month++;
				}
			}
		}
		else // Is leap year
		{
			if (time_day < days_in_month_leapyear[time_month - 1])
				time_day++;
			else
			{
				if (time_month == 12) //xxxx.12.31
				{
					time_year++;
					time_month = 1;
					time_day = 1;
				}
				else
				{
					time_day = 1;
					time_month++;
				}
			}
		}
	}
}

void clock_display(void) //
{
	uint32_t cnt = 0;
	uint8_t clock[8];
	clock[0] = time_hour / 10;
	clock[1] = time_hour % 10;
	clock[3] = time_min / 10;
	clock[4] = time_min % 10;
	clock[6] = time_sec / 10;
	clock[7] = time_sec % 10;
	if (illuminating_level != 0)
		for (cnt = 0; cnt < 8; cnt++)
		{
			if (cnt == 2 || cnt == 5)
				continue;
			Display_8tubes(clock, cnt);
			SysCtlDelay(4 / (4 - illuminating_level) * ui32SysClock / 6000);
			shut_down_tubes();
			SysCtlDelay((4 - illuminating_level) * ui32SysClock / 6000);
		}

	if (tmp_switch_byte == 0xff)
	{
		tmp_switch_byte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		switch_byte = ~tmp_switch_byte;
		switch (switch_byte)
		{
		case 0x01:
			if (time_hour > 0)
				time_hour--;
			else
				time_hour = 23;
			break;

		case 0x02:
			if (time_min > 0)
				time_min--;
			else
				time_min = 59;
			break;

		case 0x04:
			if (time_sec > 0)
				time_sec--;
			else
				time_sec = 59;
			break;

		case 0x08:
			if (time_hour < 8 && illuminating_level != 0)
				illuminating_level = 0;
			else if (illuminating_level > 0)
				illuminating_level--;
			else
				illuminating_level = 3;
			break;

		case 0x10:
			if (motor == 1)
				motor = 0;
			else
				motor = 1;
			break;

		case 0x20:
			if (time_sec < 59)
				time_sec++;
			else
				time_sec = 0;
			break;

		case 0x40:
			if (time_min < 59)
				time_min++;
			else
				time_min = 0;
			break;

		case 0x80:
			if (time_hour < 23)
				time_hour++;
			else
				time_hour = 0;
			break;

		default:
			break;
		}
	}
	else if (I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == 0xff)
		tmp_switch_byte = 0xff;
}

void solarterm_display(void)
{
	uint32_t i;
	clock_set_flag = 0;
	timer_flag = 1;
	if (time_month == 2 && time_day == 3) //sp1
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x73); //p
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[1]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 2 && time_day == 18) //sp2
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x73); //p
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[2]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 3 && time_day == 5) //sp3
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x73); //p
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[3]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 3 && time_day == 20) //sp4
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x73); //p
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[4]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 4 && time_day == 4) //sp5
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x73); //p
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 4 && time_day == 20) //sp6
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x73); //p
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[6]); //6
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 5 && time_day == 5)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[1]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 5 && time_day == 21)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[2]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 6 && time_day == 5)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[3]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 6 && time_day == 21)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[4]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 7 && time_day == 7)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 7 && time_day == 22)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //s
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[6]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 8 && time_day == 7)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[15]); //f
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[10]); //a
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[1]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 8 && time_day == 23)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[15]); //f
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[10]); //a
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[2]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 9 && time_day == 7)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[15]); //f
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[10]); //a
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[3]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 9 && time_day == 23)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[15]); //f
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[10]); //a
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[4]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 10 && time_day == 8)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[15]); //f
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[10]); //a
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 10 && time_day == 23)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[15]); //f
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[10]); //a
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[6]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 11 && time_day == 7)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[1]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 11 && time_day == 22)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[2]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 12 && time_day == 7)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[3]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 12 && time_day == 21)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[4]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 1 && time_day == 5)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[5]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (time_month == 1 && time_day == 20)
	{
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x1);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x3e); //u
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x2);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[6]); //1
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x4);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
}

void date_display(void)
{
	uint32_t cnt = 0;
	uint8_t date[8];
	clock_set_flag = 0;
	timer_flag = 1;
	date[0] = time_year / 1000;
	date[1] = (time_year / 100) % 10;
	date[2] = (time_year / 10) % 10;
	date[3] = time_year % 10;
	date[4] = time_month / 10;
	date[5] = time_month % 10;
	date[6] = time_day / 10;
	date[7] = time_day % 10;
	for (cnt = 0; cnt < 8; cnt++)
	{
		Display_8tubes(date, cnt);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}

	if (tmp_switch_byte == 0xff)
	{
		tmp_switch_byte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		switch_byte = ~tmp_switch_byte;
		switch (switch_byte)
		{
		case 0x01:
			if (time_year > 1)
				time_year--;
			else
				time_year = 1970;
			break;
		case 0x02:
			if (time_month > 1)
				time_month--;
			else
				time_month = 12;
			break;
		case 0x04:
			if (time_day > 1)
				time_day--;
			else
			{
				if (is_leap_year(time_year))
					time_day = days_in_month_leapyear[time_month - 1];
				else
					time_day = days_in_month_non_leapyear[time_month - 1];
			}
			break;
		case 0x20:
			if (time_day < is_leap_year(time_year) ? days_in_month_leapyear[time_month - 1] : days_in_month_non_leapyear[time_month - 1])
				time_day++;
			else
				time_day = 1;
			break;
		case 0x40:
			if (time_month < 12)
				time_month++;
			else
				time_month = 1;
			break;
		case 0x80:
			if (time_year < 9999)
				time_year++;
			else
				time_year = 1970;
			break;

		default:
			break;
		}
	}
	else if (I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == 0xff)
		tmp_switch_byte = 0xff;
}

void alarm_1(void)
{
	uint32_t cnt = 0;
	uint8_t clock[8];
	timer_flag = 1;
	clock_set_flag = 0;
	clock[0] = alarm_1_hour / 10;
	clock[1] = alarm_1_hour % 10;
	clock[3] = alarm_1_min / 10;
	clock[4] = alarm_1_min % 10;
	clock[6] = alarm_1_sec / 10;
	clock[7] = alarm_1_sec % 10;
	for (cnt = 0; cnt < 8; cnt++)
	{
		if (cnt == 2 || cnt == 5)
			continue;
		Display_8tubes(clock, cnt);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (tmp_switch_byte == 0xff)
	{
		tmp_switch_byte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		switch_byte = ~tmp_switch_byte;
		switch (switch_byte)
		{
		case 0x01:
			if (alarm_1_hour > 0)
				alarm_1_hour--;
			else
				alarm_1_hour = 23;
			break;
		case 0x02:
			if (alarm_1_min > 0)
				alarm_1_min--;
			else
				alarm_1_min = 59;
			break;
		case 0x04:
			if (alarm_1_sec > 0)
				alarm_1_sec--;
			else
				alarm_1_sec = 59;
			break;

		case 0x08:
			if (alarm_1_flag)
				alarm_1_flag = 0;
			else
				alarm_1_flag = 1;
			alarm_timer_flag = 1;
			break;

		case 0x20:
			if (alarm_1_sec < 59)
				alarm_1_sec++;
			else
				alarm_1_sec = 0;
			break;
		case 0x40:
			if (alarm_1_min < 59)
				alarm_1_min++;
			else
				alarm_1_min = 0;
			break;
		case 0x80:
			if (alarm_1_hour < 23)
				alarm_1_hour++;
			else
				alarm_1_hour = 0;
			break;
		default:
			break;
		}
	}
	else if (I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == 0xff)
		tmp_switch_byte = 0xff;
}

void alarm_2(void)
{
	uint32_t cnt = 0;
	uint8_t clock[8];
	timer_flag = 1;
	clock_set_flag = 0;
	clock[0] = alarm_2_hour / 10;
	clock[1] = alarm_2_hour % 10;
	clock[3] = alarm_2_min / 10;
	clock[4] = alarm_2_min % 10;
	clock[6] = alarm_2_sec / 10;
	clock[7] = alarm_2_sec % 10;
	for (cnt = 0; cnt < 8; cnt++)
	{
		if (cnt == 2 || cnt == 5)
			continue;
		Display_8tubes(clock, cnt);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (tmp_switch_byte == 0xff)
	{
		tmp_switch_byte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		switch_byte = ~tmp_switch_byte;
		switch (switch_byte)
		{
		case 0x01:
			if (alarm_2_hour > 0)
				alarm_2_hour--;
			else
				alarm_2_hour = 23;
			break;
		case 0x02:
			if (alarm_2_min > 0)
				alarm_2_min--;
			else
				alarm_2_min = 59;
			break;
		case 0x04:
			if (alarm_2_sec > 0)
				alarm_2_sec--;
			else
				alarm_2_sec = 59;
			break;

		case 0x08:
			if (alarm_2_flag)
				alarm_2_flag = 0;
			else
				alarm_2_flag = 1;
			alarm_timer_flag = 1;
			break;

		case 0x20:
			if (alarm_2_sec < 59)
				alarm_2_sec++;
			else
				alarm_2_sec = 0;
			break;
		case 0x40:
			if (alarm_2_min < 59)
				alarm_2_min++;
			else
				alarm_2_min = 0;
			break;
		case 0x80:
			if (alarm_2_hour < 23)
				alarm_2_hour++;
			else
				alarm_2_hour = 0;
			break;
		default:
			break;
		}
	}
	else if (I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == 0xff)
		tmp_switch_byte = 0xff;
}

void timer_display(void)
{
	uint32_t cnt = 0;
	uint8_t timer[8];
	clock_set_flag = 0;
	timer_flag = 1;
	timer[0] = timer_min / 10;
	timer[1] = timer_min % 10;
	timer[3] = timer_sec / 10;
	timer[4] = timer_sec % 10;
	timer[6] = timer_msec / 10;
	timer[7] = timer_msec % 10;
	for (cnt = 0; cnt < 8; cnt++)
	{
		if (cnt == 2 || cnt == 5)
			continue;
		Display_8tubes(timer, cnt);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
	}
	if (tmp_switch_byte == 0xff)
	{
		tmp_switch_byte = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		switch_byte = ~tmp_switch_byte;
		switch (switch_byte)
		{
		case 0x01:
			if (timer_min > 0)
				timer_min--;
			else
				timer_min = 59;
			timer_counting = 0;
			break;

		case 0x02:
			if (timer_sec > 0)
				timer_sec--;
			else
				timer_sec = 59;
			timer_counting = 0;
			break;

		case 0x04:
			if (timer_msec > 0)
				timer_msec--;
			else
				timer_msec = 99;
			timer_counting = 0;
			break;

		case 0x20:
			if (timer_msec < 99)
				timer_msec++;
			else
				timer_msec = 0;
			timer_counting = 0;
			break;

		case 0x40:
			if (timer_sec < 59)
				timer_sec++;
			else
				timer_sec = 0;
			timer_counting = 0;
			break;

		case 0x80:
			if (timer_min < 59)
				timer_min++;
			else
				timer_min = 0;
			timer_counting = 0;
			break;

		case 0x08:
			alarm_timer_flag = 1;
			timer_counting = 1;
			break;

		case 0x10:
			timer_counting = 0;
			break;

		default:
			break;
		}
	}
	else if (I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0) == 0xff)
		tmp_switch_byte = 0xff;
}

void alarm_check(void)
{
	if (alarm_1_flag)
	{
		if (time_hour == alarm_1_hour && time_min == alarm_1_min && time_sec == alarm_1_sec && alarm_timer_flag)
		{
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
			illuminating_level = 3;
			blink();
		}
	}
	if (alarm_2_flag)
	{
		if (time_hour == alarm_2_hour && time_min == alarm_2_min && time_sec == alarm_2_sec && alarm_timer_flag)
		{
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
			illuminating_level = 3;
			blink();
		}
	}
}

void timer_count_down(void)
{

	if (timer_msec > 0)
		timer_msec--;
	else if (timer_sec > 0) //xx:00
	{
		timer_sec--;
		timer_msec = 99;
	}
	else if (timer_min > 0) //xx:00:00
	{
		timer_min--;
		timer_sec = 59;
		timer_msec = 99;
	}
	else if (alarm_timer_flag)
	{
		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5); //00:00:00
		blink();
	}
	else
		;
}

void Initial_Sequence(void)
{
	uint32_t i, j = 0;
	uint8_t code[] = {2, 1, 9, 1, 0, 0, 9, 0};
	for (i = 0; i < 2; i++)
	{
		I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0xff);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0xff);
		SysCtlDelay(200 * ui32SysClock / 3000);
		I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
		I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00);
		SysCtlDelay(200 * ui32SysClock / 3000);
	}
	while (j < 400)
	{
		Display_8tubes(code, j % 8);
		SysCtlDelay(2 * ui32SysClock / 3000);
		shut_down_tubes();
		SysCtlDelay(ui32SysClock / 6000);
		j++;
		if (j % 200 == 0)
		{
			shut_down_tubes();
			SysCtlDelay(800 * ui32SysClock / 3000);
		}
	}
}

void Display_8tubes(uint8_t *p, uint8_t num)
{
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[p[num]]);
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 1 << num);
}

void shut_down_tubes(void)
{
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00);
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
}

uint8_t is_leap_year(uint32_t year)
{
	uint8_t flag;
	flag = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
	return flag;
}

void uart_set_check(void)
{
	if (RxEndFlag && !uart_error_flag)
	{
		uint32_t i;
		RxEndFlag = 0;
		UARTStringPutNonBlocking("START SETTING BY UART \n");
		UARTStringPutNonBlocking("THE INPUT IS: ");
		UARTStringPutNonBlocking(RxBuf);
		switch (uart_set_status)
		{
		case UART_SET_CLOCK:
			UARTStringPutNonBlocking(" SETTING CLOCK NOW \n");
			time_hour = (RxBuf[6] - '0') * 10 + (RxBuf[7] - '0');
			time_min = (RxBuf[8] - '0') * 10 + (RxBuf[9] - '0');
			time_sec = (RxBuf[10] - '0') * 10 + (RxBuf[11] - '0');
			UARTStringPutNonBlocking("CLOCK SET COMPLETED \n");
			break;
		case UART_SET_DATE:
			UARTStringPutNonBlocking(" SETTING DATE NOW \n");
			time_year = 0;
			for (i = 5; i < 9; i++)
				time_year += (RxBuf[i] - '0') * pow(10, 8 - i);
			time_month = (RxBuf[9] - '0') * 10 + (RxBuf[10] - '0');
			time_day = (RxBuf[11] - '0') * 10 + (RxBuf[12] - '0');
			UARTStringPutNonBlocking("DATE SETTING COMPLETED \n");
			break;
		case UART_SET_ALARM_1:
			UARTStringPutNonBlocking(" SETTING ALARM 1 NOW \n");
			alarm_1_hour = (RxBuf[7] - '0') * 10 + (RxBuf[8] - '0');
			alarm_1_min = (RxBuf[9] - '0') * 10 + (RxBuf[10] - '0');
			alarm_1_sec = (RxBuf[11] - '0') * 10 + (RxBuf[12] - '0');
			UARTStringPutNonBlocking("ALARM 1 SETTING COMPLETED \n");
			break;
		case UART_SET_ALARM_2:
			UARTStringPutNonBlocking(" SETTING ALARM 2 NOW \n");
			alarm_2_hour = (RxBuf[7] - '0') * 10 + (RxBuf[8] - '0');
			alarm_2_min = (RxBuf[9] - '0') * 10 + (RxBuf[10] - '0');
			alarm_2_sec = (RxBuf[11] - '0') * 10 + (RxBuf[12] - '0');
			UARTStringPutNonBlocking("ALARM 2 SETTING COMPLETED \n");
			break;
		case UART_SET_TIMER:
			UARTStringPutNonBlocking(" SETTING TIMER NOW \n");
			timer_min = (RxBuf[6] - '0') * 10 + (RxBuf[7] - '0');
			timer_sec = (RxBuf[8] - '0') * 10 + (RxBuf[9] - '0');
			timer_msec = (RxBuf[10] - '0') * 10 + (RxBuf[11] - '0');
			alarm_timer_flag = 1;
			UARTStringPutNonBlocking("TIMER SETTING COMPLETED \n");
			break;

		default:
			break;
		}
		uart_set_status = 0;
	}
}

char ASCII2Disp(char *buff)
{
	char *pcDisp;
	pcDisp = (char *)strchr(disp_tab, *buff);
	if (pcDisp == NULL)
		return 0x0;
	else
		return (disp_tab_7seg[pcDisp - disp_tab]);
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for (ui32Loop = 0; ui32Loop < value; ui32Loop++)
	{
	};
}

void UARTStringPutNonBlocking(const char *cMessage)
{
	while (*cMessage != '\0')
	{
		if (UARTSpaceAvail(UART0_BASE))
			UARTCharPutNonBlocking(UART0_BASE, *cMessage++);
	}
}

void UARTStringPut(const char *cMessage)
{
	while (*cMessage != '\0')
		UARTCharPut(UART0_BASE, *(cMessage++));
}

void UARTStringGet(char *msg)
{
	while (1)
	{
		*msg = UARTCharGet(UART0_BASE);
		if (*msg == '\r')
		{
			*msg = UARTCharGet(UART0_BASE);
			break;
		}
		msg++;
	}
	*msg = '\0';
}

void S800_INT_Init(void)
{
	SysTickEnable();
	SysTickIntEnable();

	IntPrioritySet(INT_UART0, 0x00);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	IntEnable(INT_UART0);

	GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_FALLING_EDGE);
	GPIOIntRegister(GPIO_PORTJ_BASE, GPIOJ_Handler);
	GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	IntEnable(INT_GPIOJ);

	IntMasterEnable();
}

void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //Enable PortA
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
		; //Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX); // Set GPIO A0 and A1 as UART pins.
	GPIOPinConfigure(GPIO_PA1_U0TX);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
	UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello!\r\n");
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Enable PortF
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
		;										 //Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); //Enable PortJ
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
		;										 //Wait for the GPIO moduleJ ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); //Enable PortN
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
		; //Wait for the GPIO moduleN ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
		;

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3); //Set PF0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);										   //Set PN0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);										   //Set PN1 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);										   //Set PK5 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); //Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void)
{
	uint8_t result;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); //config I2C0 400k
	I2CMasterEnable(I2C0_BASE);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0x0ff); //config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x0);   //config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x0);   //config port 2 as output

	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00);	 //config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); //turn off the LED1-8
	I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x0);
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while (I2CMasterBusy(I2C0_BASE))
	{
	};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while (I2CMasterBusy(I2C0_BASE))
	{
	};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while (I2CMasterBusy(I2C0_BASE))
	{
	};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value, rop;
	while (I2CMasterBusy(I2C0_BASE))
	{
	};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	//I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while (I2CMasterBusBusy(I2C0_BASE))
		;
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(200);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while (I2CMasterBusBusy(I2C0_BASE))
		;
	value = I2CMasterDataGet(I2C0_BASE);
	Delay(200);
	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	if (systick_100ms_couter != 0)
		systick_100ms_couter--;
	else
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
		systick_100ms_couter = SYSTICK_FREQUENCY / 10;
		systick_100ms_status = 1;
	}

	if (systick_10ms_couter != 0)
		systick_10ms_couter--;
	else
	{
		if (timer_flag && timer_counting)
			timer_count_down();
		systick_10ms_couter = SYSTICK_FREQUENCY / 100;
		systick_10ms_status = 1;
	}

	if (systick_1s_counter != 0)
		systick_1s_counter--;
	else
	{
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
		systick_1s_counter = SYSTICK_FREQUENCY;
		systick_1s_status = 1;
	}

	if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)
	{
		systick_1s_status = systick_100ms_status = systick_10ms_status = 0;
	}
	else
		;

	if (systick_1ms_counter != 0)
		systick_1ms_counter--;
	else
	{
		systick_1ms_counter = SYSTICK_FREQUENCY / 1000;
		systick_1ms_status = 1;
	}

	if (receive_finished_countdown < 100)
	{
		receive_finished_countdown++;
	}
	else if (receive_finished_countdown == 100)
	{
		receive_finished_countdown = 10000;
		RxEndFlag = 1;
		uart_error_flag = 0;
		if (strncmp(RxBuf, "CLOCK+", 6) == 0)
			uart_set_status = UART_SET_CLOCK;
		else if (strncmp(RxBuf, "DATE+", 5) == 0)
			uart_set_status = UART_SET_DATE;
		else if (strncmp(RxBuf, "ALARM1+", 7) == 0)
			uart_set_status = UART_SET_ALARM_1;
		else if (strncmp(RxBuf, "ALARM2+", 7) == 0)
			uart_set_status = UART_SET_ALARM_2;
		else if (strncmp(RxBuf, "TIMER+", 6) == 0)
			uart_set_status = UART_SET_TIMER;
		else
		{
			UARTStringPutNonBlocking("ERROR!");
			uart_error_flag = 1;
		}
		tail = RxBuf;
	}
}

void UART0_Handler(void)
{
	uint32_t ulStatus;
	ulStatus = UARTIntStatus(UART0_BASE, true);
	UARTIntClear(UART0_BASE, ulStatus);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);
	while (UARTCharsAvail(UART0_BASE))
	{
		*tail++ = UARTCharGetNonBlocking(UART0_BASE);
	}
	*tail = '\0';
	receive_finished_countdown = 0;
}

void GPIOJ_Handler(void) //Using PJ0 to switch STATE
{
	unsigned long intStatus;
	intStatus = GPIOIntStatus(GPIO_PORTJ_BASE, true);
	GPIOIntClear(GPIO_PORTJ_BASE, intStatus);
	if (!GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0))
	{
		while (!GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0))
			;
		SysCtlDelay(300 * ui32SysClock / 3000);
		if (!GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1))
			reboot();
		else
		{
			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
			if (STATE == 0x00)
				STATE = 0x01;
			else if (STATE < 0x10)
				STATE = STATE << 1;
			else
				STATE = 0x00;
		}
	}

	if (!GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1))
	{
		GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
		alarm_timer_flag = 0;
	}
}

void reboot(void)
{
	shut_down_tubes();
	alarm_1_hour = 0;
	alarm_1_min = 0;
	alarm_1_sec = 0;
	alarm_2_hour = 0;
	alarm_2_min = 0;
	alarm_2_sec = 0;
	timer_min = 0;
	timer_sec = 0;
	timer_msec = 0;
	Initial_Sequence();
	SysCtlDelay(1000 * ui32SysClock / 3000);
}

void blink(void)
{
	int times;
	for (times = 0; times < 5; times++)
	{
		I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00);
		SysCtlDelay(150 * ui32SysClock / 3000);
		I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff);
		SysCtlDelay(150 * ui32SysClock / 3000);
	}
}
