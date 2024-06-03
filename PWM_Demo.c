
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // 基址宏定义
#include "inc/hw_types.h"        // 数据类型宏定义，寄存器访问函数
#include "inc/hw_timer.h"        // 与定时器有关的宏定义
#include "inc/hw_ints.h"         // 与中断有关的宏定义
#include "driverlib/debug.h"     // 调试用
#include "driverlib/gpio.h"      // 通用IO口宏定义和函数原型
#include "driverlib/pin_map.h"   // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"    // 系统控制定义
#include "driverlib/systick.h"   // SysTick Driver 函数原型
#include "driverlib/interrupt.h" // NVIC中断控制驱动函数原型
#include "driverlib/timer.h"     // 与Timer有关的函数原型
#include "driverlib/pwm.h"       // 与Timer有关的函数原型
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "PWM.h"    // 与PWM相关的函数原型


#define SYSTICK_FREQUENCY 50 // SysTick频率为50Hz，即循环定时周期20ms

//*******************************************************************************************************
//
// 函数原型声明
//
//*******************************************************************************************************
void GPIOInit(void);    // GPIO初始化
void SysTickInit(void); // 设置SysTick中断
void DevicesInit(void); // MCU器件初始化，注：会调用上述函数

//*******************************************************************************************************
//
// 变量定义
//
//*******************************************************************************************************

// 8位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
uint8_t digit[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

// 8位小数点 1亮  0灭
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2、3
uint8_t pnt = 0x00;

// 8个LED指示灯状态，0灭，1亮
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
//     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// 当前按键值
volatile uint8_t key_code = 0;

// 用于记录前一次按键检测时的键盘状态，0表示无键按下，1有键按下
volatile uint8_t key_state = 0;

// 按键操作有效标记，0代表无新操作，1代表有键操作
volatile uint8_t key_flag = 0;
uint8_t times = 0;
// 系统时钟频率
uint32_t g_ui32SysClock;
uint32_t freq = 4000;                                         // PWM频率
uint32_t ui32Freq[] = {587,659,494,440,880,988,740,880,1175,1109,988,880,100000,880,988,880,740,659,740,880,740,494,587,659,100000,659,880,740,587,494,440,494,659,988,880,988,880,740,659}; // 方波频率，根据实际需要进行修改,空拍为100000Hz
uint32_t freqtime[] = {500,500,500,500,1500,500,500,500,500,250,250,1500,500,1000,500,500,500,500,1000,500,500,500,500,1500,500,500,500,500,500,1000,500,500,500,500,500,250,250,1500,500}; // 单位为s
uint32_t timecount = 0;                                       // 计时器
uint32_t i = 0;
uint32_t pwmflag = 1;
//*******************************************************************************************************
//
// 主程序
//
//*******************************************************************************************************
int main(void)
{

    DevicesInit(); //  MCU器件初始化


    // uint32_t ui32Freq = 500;  // 方波频率，根据实际需要进行修改

    while (1)
    {
        // if(key_flag == 1)    // 有键按下
        //{
        // key_flag = 0;

        // switch (key_code)
        // {
        // case 1: // 产生频率为ui32Freq的方波信号
        //     PWMStart(freq);
        //     break;
        // case 2: // 停止产生PWM信号
        //     PWMStop();
        //     break;
        // default:
        //     break;
        // }
        //}
    }
}

//*******************************************************************************************************
//
// 函数原型：void GPIOInit(void)
// 函数功能：GPIO初始化。使能PortK，设置PK4,PK5为输出；使能PortM，设置PM0为输出。
//          （PK4连接TM1638的STB，PK5连接TM1638的DIO，PM0连接TM1638的CLK）
// 函数参数：无
// 函数返回值：无
//
//*******************************************************************************************************
void GPIOInit(void)
{
    // 配置TM1638芯片管脚
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // 使能端口 K
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
    {
    }; // 等待端口 K准备完毕

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); // 使能端口 M
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
    {
    }; // 等待端口 M准备完毕

    // 设置端口 K的第4,5位（PK4,PK5）为输出引脚		PK4-STB  PK5-DIO
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // 设置端口 M的第0位（PM0）为输出引脚   PM0-CLK
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
}

//*******************************************************************************************************
//
// 函数原型：SysTickInit(void)
// 函数功能：设置SysTick中断
// 函数参数：无
// 函数返回值：无
//
//*******************************************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(g_ui32SysClock / SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms
    SysTickEnable();                                      // SysTick使能
    SysTickIntEnable();                                   // SysTick中断允许
}

//*******************************************************************************************************
//
// 函数原型：DevicesInit(void)
// 函数功能：MCU器件初始化，包括系统时钟设置、GPIO初始化和SysTick中断设置
// 函数参数：无
// 函数返回值：无
//
//*******************************************************************************************************
void DevicesInit(void)
{
    // 使用外部25MHz主时钟源，经过PLL，然后分频为16MHz
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                         SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                        16000000);

    FPULazyStackingEnable();
    FPUEnable();

    GPIOInit();        // GPIO初始化
    PWMInit();         // PWM初始化
    SysTickInit();     // 设置SysTick中断
    IntMasterEnable(); // 总中断允许
}

//*******************************************************************************************************
//
// 函数原型：void SysTick_Handler(void)
// 函数功能：SysTick中断服务程序，检测按键动作，刷新数码管显示
// 函数参数：无
// 函数返回值：无
//
//*******************************************************************************************************
void SysTick_Handler(void) // 定时周期为20ms
{
    if (pwmflag == 1)
    {
        timecount++;
        if (times == 0 && pwmflag == 1)
        {
            PWMStart(ui32Freq[0]);
            times = 1;
        }
        if (timecount >= freqtime[i] * 0.05)
        {
            i++;
            if (i >= 40)
            {
                i = 0;
            }
            timecount = 0;
            freq = ui32Freq[i];
            PWMStart(freq);
        }
    }
}
