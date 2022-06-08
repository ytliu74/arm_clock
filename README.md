# ARM Final


### 一、 功能实现情况

|           项目            | 完成？ |                 项目                 | 完成？ |
| :-----------------------: | :----: | :----------------------------------: | :----: |
|     日期的显示及设置      |   √    |              节气的显示              |   √    |
|     时间的显示及设置      |   √    | 闹钟蜂鸣及止闹，闹钟设置。共两个闹钟 |   √    |
|  倒计时秒表的显示及设置   |   √    |         8位按键控制调整设置          |   √    |
|      串行口控制设置       |   √    |               开机画面               |   √    |
|      组合键重启系统       |   √    |            8位LED辅助显示            |   √    |
|       步进电机旋转        |   √    |             显示亮度切换             |   √    |
| 夜间一键熄灭 闹钟自动唤醒 |   √    |              电位器控制              |   ×    |



### 二、 项目简介

* 显示

  利用8位数码管，作为主要显示，显示时钟、日期、节气、闹钟、定时器。

  并利用8位LED作为辅助显示，主要作为目前状态的指示，以及闹钟是否开启状态的显示。

* 设置

  利用红板上两个按键作为状态更换、关闭闹铃以及作为组合键关机。

  利用蓝板上8个按键作为时钟、日期、闹钟、定时器的设置以及暂停或开始状态。

  利用蓝板上按键步进电机旋转的状态设置。

  利用篮板上按键作为显示亮度切换的设置。

  利用串口通信，使用“CLOCK+120000”等语句，控制时钟、日期、闹钟、定时器。



### 三、遇到的问题

1. 用I2C获取按钮状态时，会非常频繁的输出0x00，这个状态意味着所以按钮同时按下，很明显这不符合事实。这个问题出在*uint8_t* I2C0_ReadByte(*uint8_t* *DevAddr*, *uint8_t* *RegAddr*)中，若将期中Delay(1)改为Delay(200)，问题得到解决。

2. systick_10ms_status放在外面获取时，明显有延迟，无法作为计时器的count down，于是将计时器的count down直接置于SysTick_Handler中，问题得到解决。

3. 亮度调节不明显。需要同时对开启与关断的时间进行调整，才能得到一个较为明显的亮度调节功能。

4. 在中断程序中，对组合键的判定存在错误。使用多重条件语句嵌套，解决该问题。

   



### 四、视频介绍目录

1. 开机，开机程序。

2. 红板USR_SW1控制状态调整。依次为：时钟、节气、日期、闹钟1、闹钟2、定时器。

3. 蓝板使用6个按键，分别调整时钟、日期、闹钟、定时器的各个参数的加与减。

4. 使用串口通信，分别控制时钟、日期、闹钟、定时器。

   ```
   CLOCK+083030
   DATE+20210625
   ALARM1+083500
   ALARM2+093000
   TIMER+001000
   ```

5. 设置闹钟ALARM1以及闹钟ALARM2，分别开启以及关闭，触发闹铃以及关闭闹铃。

6. 设置定时器，演示开始与暂停按键，演示定时器到0时触发闹铃以及关闭闹铃。

7. 设置日期到特定节气，显示节气。
   ```
   DATE+20210203
   DATE+20211008
   ```

8. 在白天以及凌晨的时间段，分别调整时钟的亮度。在0:00-8:00时，一键熄灭。

9. 在熄灭的时间段，闹钟触发时，重新开启显示。

   ```
   CLOCK+073000
   ALARM1+073030
   ```

10. 步进电机的开启以及关断。

11. 组合键重启系统。



### 五、 主要功能实现代码

1. 时钟显示及调节

   ```c
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
   ```

   

2. 日期显示及调节

   ```c
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
   ```

   

3. 闹钟显示及调节

   ```c
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
   
   ```

   4. 计时器显示及调节

      ```c
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
      ```

      
