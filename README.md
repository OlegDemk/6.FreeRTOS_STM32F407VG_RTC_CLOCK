# 6.FreeRTOS_STM32F407VG_RTC_CLOCK

CLOCK.

Progect base on STM32F407VG. Project was created on STM32IDE.

Components:

1. STM32F407VG
2. OLED (SSD1306_128_32)
3. DS3231 (RTC modeule)
4. Encoder (With key button)

All iteracrion with clock is in Start_RTC task. 
For avoid contact bounce effect need read button state after some time after first button pressed detection. The button connected to PE19 Pin set up as externet interrupt. In EXTI15_10_IRQHandler, Tim7 starts every time when the button pressed. In TIM7_IRQHandler we count 5 periods of time and check signal on button (PE19 Pin). This method solve problrm with contact bounc

For avoid contact bounce effect nead read button state after some rime after first faling age (press a button)
![alt text](https://github.com/OlegDemk/6.FreeRTOS_STM32F407VG_RTC_CLOCK/blob/main/project_schem.png)

![alt text](https://github.com/OlegDemk/6.FreeRTOS_STM32F407VG_RTC_CLOCK/blob/main/schem.png)

![alt text](https://github.com/OlegDemk/6.FreeRTOS_STM32F407VG_RTC_CLOCK/blob/main/photo.png)


