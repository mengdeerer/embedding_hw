
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
#include "hw_i2c.h"
#include "i2c.h"
#include "string.h"
#include "ctype.h"
//#include "hibernate.h"
#include "eeprom.h"
//#include "utils/uartstdio.h"

// 变量定义 变量定义
#define SYSTICK_FREQUENCY 50
#define dot 0x80
// I2C
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

// date and time
typedef struct
{
    uint32_t year;
    uint32_t month;
    uint32_t day;
} date;

typedef struct
{
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
} time;

typedef struct
{
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
} alarm;
uint32_t alarmcount = 0;
uint32_t alarm_onoff = 1;
uint32_t alarmflag = 0;
uint32_t dateflag = 0;
uint32_t timeflag = 0;
date mydate;
time mytime;
alarm myalarm;
uint32_t onesec = 0;
int displaydelay = 2000; // 数码管延迟
int datenum[8];           //{10604202}
int timenum[8];           //{00 80 01}
int alarmnum[8];
uint8_t datedot[] = {0, 0, dot, 0, dot, 0, 0, 0}; // 2024.06.01,with space in it
//eeprom
uint32_t ui32EEPROMInit;
uint32_t pui32Data[3];
uint32_t pui32Read[3];

uint32_t ui32SysClock;
// 编辑模式
int editmode = 0;
int editpart = 0;
int updatetime = 0;
// 流水线
uint32_t date_flow_flag = 0;
uint32_t time_flow_flag = 0;
int date_flow_num[8];
int time_flow_num[8];
int flow_time = 0;
int last = 0;
int datei = 0;
int direction_flag = 0; // 0向右，1向左
int flow_speed = 100;
// PWM
uint8_t times = 0;
uint32_t pwmflag = 0;
uint32_t freq = 1000000;                                                                                                                                                                                                            // PWM???
uint32_t ui32Freq[] = {587, 659, 494, 440, 880, 988, 740, 880, 1175, 1109, 988, 880, 100000, 880, 988, 880, 740, 659, 740, 880, 740, 494, 587, 659, 100000, 659, 880, 740, 587, 494, 440, 494, 659, 988, 880, 988, 880, 740, 659}; // ?????????????????????????,?????100000Hz
uint32_t freqtime[] = {500, 500, 500, 500, 1500, 500, 500, 500, 500, 250, 250, 1500, 500, 1000, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 1500, 500, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 500, 250, 250, 1500, 500};  // ??閿熸枻鎷??s
uint32_t timecount = 0;                                                                                                                                                                                                            // ?????
uint32_t notes_num = 0;                                                                                                                                                                                                            // ?????????                                                                                                                                                                                                                // ?????????
// led
uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c, 0x40};
uint8_t name[] = {0x3F, 0x38, 0x77, 0x30, 0x1E, 0x79, 0x3E, 0x76}; // xuejialo
// 状态切换
volatile uint32_t state = 0; // state=0显示时间，state=1显示日期,state2闹钟
// UART
uint8_t uart_receive_char;
uint8_t uart_receive_str[50] = {0x00};
volatile uint8_t wordi = 0;
int endflag = 0;
// 中断
uint32_t ui32IntPriorityGroup, ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick, ui32IntPriorityUart0;
// 按钮
uint8_t button = 0xff;
int buttondelay = 0; // 防抖
uint8_t prev_button = 0xff;
uint32_t buttonstate = 0;
int buttontime = 20;
// 函数原型声明
void SysTickInit(void); //
void Delay(uint32_t value);
void S800_GPIO_Init(void);
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void S800_I2C0_Init(void);
void PWMStart(uint32_t ui32Freq_Hz);
void PWMStop(void);
void PWMInit(void);
void setdate(date *date, int year, int month, int day);
void settime(time *time, int hour, int minute, int second);
void dateupdate(date *date);
void timeupdate(time *time);
void S800_UART_Init(void);
void alarm_init(void);
void setalarm(alarm *alarm, int hour, int minute, int second);
void UARTStringPut(const char *cMessage);
void UARTStringPutNonBlocking(const char *cMessage);
void array_flow(int *flow_num);
void light_display(void);
void command_process(void);
// delay函数
void Delay(uint32_t value)
{
    uint32_t ui32Loop;
    for (ui32Loop = 0; ui32Loop < value; ui32Loop++)
    {
    };
}
void clear_display()
{
    dateflag = 0;
    timeflag = 0;
    alarmflag = 0;
    date_flow_flag = 0;
    time_flow_flag = 0;
}
// 函数定义

// date and time
void date_and_time_init(date *date, time *time)
{
    EEPROMRead(pui32Read, 0x400, sizeof(pui32Read));
    if(pui32Read[0]>=0&&pui32Read[0]<=23&&pui32Read[1]>=0&&pui32Read[1]<=59&&pui32Read[2]>=0&&pui32Read[2]<=59)
    {
        time->hour = pui32Read[0];
        time->minute = pui32Read[1];
        time->second = pui32Read[2];
    }
    else
    {
        settime(time, 10, 8, 0);
    }
    setdate(date, 2024, 6, 1);
    dateupdate(date);
    timeupdate(time);
}
void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY); // 20ms执行一次
    SysTickEnable();
    SysTickIntEnable();
}
void DevicesInit(void)
{
    // 使用外部25MHz主时钟源，经过PLL，然后分频为16MHz
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                      16000000);

    // FPULazyStackingEnable();
    // FPUEnable();
    S800_GPIO_Init();
    S800_I2C0_Init();
    S800_UART_Init();
    PWMInit();     // PWM初始化
    SysTickInit(); // 设置SysTick中断
    // IntMasterEnable(); // 总中断允许
    // 改中断优先级

    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); // Enable UART0 RX,TX interrupt
    IntMasterEnable();
    ui32IntPriorityMask = IntPriorityMaskGet();
    IntPriorityGroupingSet(3);            // Set all priority to pre-emtption priority
    IntPrioritySet(INT_UART0, 3);         // Set INT_UART0 to highest priority
    IntPrioritySet(FAULT_SYSTICK, 0x0e0); // Set INT_SYSTICK to lowest priority

    ui32IntPriorityGroup = IntPriorityGroupingGet();
    ui32IntPriorityUart0 = IntPriorityGet(INT_UART0);
    ui32IntPrioritySystick = IntPriorityGet(FAULT_SYSTICK);
    // 启用EEPROM模块。
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    // 等待EEPROM模块准备好。
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
    {
    }

    // 等待EEPROM初始化完成。
    ui32EEPROMInit = EEPROMInit();

    // 检查EEPROM初始化是否返回错误，并通知应用程序。
    if (ui32EEPROMInit != EEPROM_INIT_OK)
    {
        while (1)
        {
        }
    }
    // 时间初始化
    date_and_time_init(&mydate, &mytime);
    alarm_init();
}

void PWMInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // PWM0使能

    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true); // 使能(允许)PWM0_4的输出
    // PWMGenEnable(PWM0_BASE, PWM_GEN_2);             //使能PWM0模块的2号发生器(因为4号PWM是2号发生器产生的)
    // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, g_ui32SysClock / ui32Freq_Hz); // 根据Freq_Hz设置PWM周期

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // 使能GPIOG
    GPIOPinConfigure(GPIO_PK5_M0PWM7);           // 配置引脚复用
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5); // 引脚映射

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // 配置PWM发生器
    // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,(PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2)/ 2)); //设置占空比为50%
}

void S800_GPIO_Init(void) // initialize
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable PortF
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
        ;                                        // Wait for the GPIO moduleF ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Enable PortJ
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
        ;                                        // Wait for the GPIO moduleJ ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Enable PortN
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
        ; // Wait for the GPIO moduleN ready

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); // Set PF0 as Output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // Set PN0 as Output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1); // Set PN1 as Output pin

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set the PJ0,PJ1 as input pin
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void) // initialize
{
    uint8_t result;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); // config I2C0 400k
    I2CMasterEnable(I2C0_BASE);

    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0x0ff); // config port 0 as input
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x0);   // config port 1 as output
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x0);   // config port 2 as output

    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00);  // config port as output
    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); // turn off the LED1-8
}

void S800_UART_Init(void) // initialize
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable PortA
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
        ; // Wait for the GPIO moduleA ready

    GPIOPinConfigure(GPIO_PA0_U0RX); // Set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    // UARTStringPut((uint8_t *)"\r\nhello,world!\r\n"); // send this message once when initialized
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX2_8, UART_FIFO_RX7_8);
}

//button
void button_catch(){
    if (buttontime > 10)
    {
        button = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
        if (buttontime > 100000)
        {
            buttontime = 11;
        }
    }

    if (button != 0xff)
    {
        buttondelay++;
        if (buttondelay == 3)
        {
            // UARTStringPut((uint8_t *)"buttonget\r\n");
            button = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);

            buttondelay = 0;
            // UARTStringPut((uint8_t *)"buttonnnnn\r\n");
            buttontime = 0;
            if (button == 0xfe)
            {
                buttonstate = 1;
                state++;
                if (state == 3)
                {
                    state = 0;
                }
                if (state == 0)
                {
                    clear_display();
                    dateflag = 1;
                }
                else if (state == 1)
                {
                    clear_display();
                    timeflag = 1;
                }
                else if (state == 2)
                {
                    clear_display();
                    alarmflag = 1;
                }
                UARTStringPut((uint8_t *)"button1\r\n");
            }
            else if (button == 0xfd) // 是否流水
            {
                buttonstate = 2;
                if(dateflag == 1)
                {
                    clear_display();
                    date_flow_flag = 1;
                }
                else if(timeflag == 1)
                {
                    clear_display();
                    time_flow_flag = 1;
                }
                // if (date_flow_flag == 0)
                // {
                //     clear_display();
                //     date_flow_flag = 1;
                // }
                // else
                // {
                //     date_flow_flag = 0;
                // }
            }
            else if (button == 0xfb) // 左右流水
            {
                buttonstate = 3;
                if (direction_flag == 1)
                {
                    direction_flag = 0;
                }
                else
                {
                    direction_flag = 1;
                }
            }
            else if (button == 0xf7)
            {
                buttonstate = 4;
                pui32Data[0] = mytime.hour;
                pui32Data[1] = mytime.minute;
                pui32Data[2] = mytime.second;
                EEPROMProgram(pui32Data, 0x400, sizeof(pui32Data));
            }
            else if (button == 0xef)
            {
                buttonstate = 5;
                if (editmode == 0)
                {
                    editmode = 1;
                }
                else
                {
                    editmode = 0;
                }
            }
            else if (button == 0xdf)
            {
                buttonstate = 6;
                editpart++;
                if (editpart == 3)
                {
                    editpart = 0;
                }
            }
            else if (button == 0xbf)
            {
                buttonstate = 7;
                if (editmode == 1)
                {
                    if (dateflag == 1)
                    {
                        if (editpart == 0)
                        {
                            mydate.day++;
                        }
                        else if (editpart == 1)
                        {
                            mydate.month++;
                        }
                        else if (editpart == 2)
                        {
                            mydate.year++;
                        }
                    }
                    else if (timeflag == 1)
                    {
                        if (editpart == 0)
                        {
                            mytime.second++;
                        }
                        else if (editpart == 1)
                        {
                            mytime.minute++;
                        }
                        else if (editpart == 2)
                        {
                            mytime.hour++;
                        }
                    }
                }
            }
            else if (button == 0x7f)
            {
                buttonstate = 8;
                if (editmode == 1)
                {
                    if (dateflag == 1)
                    {
                        if (editpart == 0)
                        {
                            mydate.day--;
                        }
                        else if (editpart == 1)
                        {
                            mydate.month--;
                        }
                        else if (editpart == 2)
                        {
                            mydate.year--;
                        }
                    }
                    else if (timeflag == 1)
                    {
                        if (editpart == 0)
                        {
                            mytime.second--;
                        }
                        else if (editpart == 1)
                        {
                            mytime.minute--;
                        }
                        else if (editpart == 2)
                        {
                            mytime.hour--;
                        }
                    }
                }
            }
            button = 0xff;
        }
    }
}
// date,time set
void setdate(date *date, int year, int month, int day)
{
    date->year = year;
    date->month = month;
    date->day = day;
}

void settime(time *time, int hour, int minute, int second)
{
    time->hour = hour;
    time->minute = minute;
    time->second = second;
}
void setalarm(alarm *alarm, int hour, int minute, int second)
{
    alarm->hour = hour;
    alarm->minute = minute;
    alarm->second = second;
}
void dateupdate(date *date)
{
    datenum[0] = date->day % 10;
    datenum[1] = date->day / 10;
    datenum[2] = date->month % 10;
    datenum[3] = date->month / 10;
    datenum[4] = date->year % 10;
    datenum[5] = date->year / 10 % 10;
    datenum[6] = date->year / 100 % 10;
    datenum[7] = date->year / 1000;
}
void alarmupdate(alarm *alarm)
{
    alarmnum[0] = alarm->second % 10;
    alarmnum[1] = alarm->second / 10;
    alarmnum[2] = 17;
    alarmnum[3] = alarm->minute % 10;
    alarmnum[4] = alarm->minute / 10;
    alarmnum[5] = 17;
    alarmnum[6] = alarm->hour % 10;
    alarmnum[7] = alarm->hour / 10;
}
void showdate(date *date)
{
    uint8_t j, light;
    while (dateflag)
    {
        for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
        {
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[datenum[j]] | datedot[j]);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
            Delay(displaydelay);
            button_catch();
            //button = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
        }
    }
}
void timeupdate(time *time)
{

    timenum[0] = time->second % 10;
    timenum[1] = time->second / 10;
    timenum[2] = 0;
    timenum[3] = time->minute % 10;
    timenum[4] = time->minute / 10;
    timenum[5] = 0;
    timenum[6] = time->hour % 10;
    timenum[7] = time->hour / 10;
}
void showtime(time *time)
{
    uint8_t j, light;
    while (timeflag)
    {
        for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
        {
            if (j == 2 || j == 5)
            {
                continue;
            }
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[timenum[j]]);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
            Delay(displaydelay);
            button_catch();
            //button = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
        }
    }
}
void showalarm(alarm *alarm)
{
    uint8_t j, light;
    while (alarmflag)
    {
        for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
        {
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarmnum[j]]);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
            Delay(displaydelay);
            button_catch();
            //button = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
        }
    }
}
void time_and_date_secupdate(time *time, date *date)
{
    // time->second++;
    if (time->second >= 60)
    {
        time->second = 0;
        time->minute++;
    }
    if (time->minute >= 60)
    {
        time->minute = 0;
        time->hour++;
    }
    if (time->hour >= 24)
    {
        time->hour = 0;
        date->day++;
    }
    // 注意月份对应天数变化
    if (date->month == 2)
    {
        if (date->year % 4 == 0 && date->year % 100 != 0 || date->year % 400 == 0)
        {
            if (date->day >= 29)
            {
                date->day = 1;
                date->month++;
            }
        }
        else
        {
            if (date->day >= 28)
            {
                date->day = 1;
                date->month++;
            }
        }
    }
    else if (date->month == 4 || date->month == 6 || date->month == 9 || date->month == 11)
    {
        if (date->day >= 30)
        {
            date->day = 1;
            date->month++;
        }
    }
    else
    {
        if (date->day >= 31)
        {
            date->day = 1;
            date->month++;
        }
    }
}

// 闹钟初始化
void alarm_init()
{
    setalarm(&myalarm, 10, 10, 0);
}
// SysTick中断服务程序
void SysTick_Handler(void)
{
    // PWM
    if (pwmflag == 1)
    {
        timecount++;
        if (times == 0 && pwmflag == 1)
        {
            PWMStart(ui32Freq[0]);
            times = 1;
        }
        if (timecount >= freqtime[notes_num] * 0.05)
        {
            notes_num++;
            if (notes_num >= 40)
            {
                notes_num = 0;
            }
            timecount = 0;
            freq = ui32Freq[notes_num];
            PWMStart(freq);
        }
    }

    // 1s sec+1
    if (editmode == 0)
    {
        onesec++;
    }
    if (onesec >= 50)
    {
        onesec = 0;
        mytime.second++;
        time_and_date_secupdate(&mytime, &mydate);
        light_display();
    }
    updatetime++;
    if (updatetime == 10)
    {
        updatetime = 0;
        time_and_date_secupdate(&mytime, &mydate);
        dateupdate(&mydate);
        timeupdate(&mytime);
        alarmupdate(&myalarm);
    }
    // 闹钟
    if(alarmcount!=0)
    {
        alarmcount++;
        if (alarmcount == 250) // 5s
        {
            alarmcount = 0;
            pwmflag = 0;
            PWMStop();
        }
    }
    if (mytime.hour == myalarm.hour && mytime.minute == myalarm.minute && mytime.second == myalarm.second && alarm_onoff == 1)
    {
        pwmflag = 1;
        alarmcount++;
    }
    buttontime++;
    
    // 流水线
    if (date_flow_flag == 1 || time_flow_flag == 1)
    {
        flow_time++;
        if (flow_time == flow_speed)
        { // 1s流水一次
            flow_time = 0;
            if (date_flow_flag == 1)
            {
                array_flow(date_flow_num);
            }
            if (time_flow_flag == 1)
            {
                array_flow(time_flow_num);
            }
        }
    }
    //

}
// 左右流水
void array_flow(int *flow_num)
{
    int last = 0;
    int i = 0;
    if (direction_flag == 1)
    {
        last = flow_num[0];
        for (i = 0; i < 7; i++)
        {
            flow_num[i] = flow_num[i + 1];
        }
        flow_num[7] = last;
    }
    else
    {
        last = flow_num[7];
        for (i = 7; i > 0; i--)
        {
            flow_num[i] = flow_num[i - 1];
        }
        flow_num[0] = last;
    }
}

// PWM player
void PWMStart(uint32_t ui32Freq_Hz)
{
    PWMGenDisable(PWM0_BASE, PWM_GEN_3);                                                 // ???PWM0????2???????(???4??PWM??2?????????????)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / ui32Freq_Hz);                   // ????Freq_Hz????PWM????
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 2)); // ?????????50%
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true); 
}
void PWMStop()
{
    pwmflag = 0;
    PWMGenDisable(PWM0_BASE, PWM_GEN_3); // PWM7
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
}

// I2c

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
    uint8_t value;

    while (I2CMasterBusy(I2C0_BASE))
    {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
        return 0;
    Delay(100);

    // receive data
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    value = I2CMasterDataGet(I2C0_BASE);
    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
        return 0;
    Delay(100);

    return value;
}
// UART通信
void UARTStringPut(const char *cMessage)
{
    while (*cMessage != '\0')
        UARTCharPut(UART0_BASE, *(cMessage++));
}
void UARTStringPutNonBlocking(const char *cMessage)
{
    while (*cMessage != '\0')
        UARTCharPutNonBlocking(UART0_BASE, *(cMessage++));
}

void UART0_Handler(void)
{

    int32_t uart0_int_status;
    
    uart0_int_status = UARTIntStatus(UART0_BASE, true);    // Get the interrrupt status.
    UARTIntClear(UART0_BASE, uart0_int_status);            // Clear the asserted interrupts
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1); // PN1 on

    while (UARTCharsAvail(UART0_BASE)) // while receive FIFO is not empty
    {
        uart_receive_str[wordi] = UARTCharGetNonBlocking(UART0_BASE);
        if(uart_receive_str[wordi] == '\n')
        {
            continue;
        }
        if (uart_receive_str[wordi] == '\r')//回车键
        {
            endflag = 1;
            uart_receive_str[wordi] = '\0';
            UARTStringPut(uart_receive_str);
            UARTStringPut("\r\n");
            command_process();
            break;
        }
        wordi++;
    }

    if(endflag == 1){
        for (wordi = 0; wordi < 50; wordi++)
        {
            uart_receive_str[wordi] = 0x00;
        }
        wordi = 0;
        endflag = 0;
    }
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0); // PN1 off
}
//命令处理
void command_process(){
    char new_str[50];
    char set_str[50];
    int i = 0, j = 0;
    int k = 0;
    int numi = 0;
    int hour = 0, minute = 0, second = 0;
    int year = 0, month = 0, day = 0;
    bool has_space = false;
    while(uart_receive_str[i] != '\0'){
        if(uart_receive_str[i]>127){
            UARTStringPut("full width command\n");
            wordi = 0;
            return;
        }
        i++;
    }
    if (strcmp("?", uart_receive_str) == 0)
    {
        UARTStringPut("Available commands:\n");
        UARTStringPut("\tinit clock                       : intialize the clock to 10:08:00\n");
        UARTStringPut("\tget <TIME/DATE/ALARM>            : get status\n");
        UARTStringPut("\tset <TIME/ALARM TIME/DATE> <xx:xx:xx> : set clock status\n");
        wordi = 0;
        return;
    }
    i=0;
    while (uart_receive_str[i] != '\0') // 遍历字符串，直到遇到字符串结束符
    {
        if (uart_receive_str[i] == ' ')
        {
            has_space = true; // 找到空格，设置标记为 true
            //UARTStringPut("has space\r\n");
            break;
        }
        i++;
    }
    if(has_space == false){
        UARTStringPut("no space command\n");
        wordi = 0;
        return;
    }
    if (has_space)
    {
        for (i = 0; uart_receive_str[i] != '\0'; i++) // 连续空格变成一个
        {
            if (uart_receive_str[i] != ' ')
            {
                new_str[j++] = uart_receive_str[i];
            }
            else if (uart_receive_str[i + 1] != ' ')
            {
                new_str[j++] = ' ';
            }
        }
        new_str[j] = '\0';                 // 添加字符串结束符
        strcpy(uart_receive_str, new_str); // 将新的字符串复制回原始字符串
        i = 0;
        while (uart_receive_str[i] != '\0')
        {
            if (uart_receive_str[i] >= 'A' && uart_receive_str[i] <= 'Z')
            {
                uart_receive_str[i] += 32;
            }
            i++;
        }
    }
    if(strcmp("speed high", uart_receive_str) == 0)
    {
        flow_speed = 50;
        UARTStringPut("OK,Speed high\n");
        return;
    }
    if(strcmp("speed low", uart_receive_str) == 0)
    {
        flow_speed = 100;
        UARTStringPut("OK,Speed low\n");
        return;
    }
    if (strcmp("alarm on", uart_receive_str) == 0)
    {
        alarm_onoff = 1;
        UARTStringPut("OK,Alarm on\n");
        return;
    }
    if(strcmp("alarm off", uart_receive_str) == 0)
    {
        alarm_onoff = 0;
        UARTStringPut("OK,Alarm off\n");
        return;
    }
    if (strcmp("init clock", uart_receive_str) == 0)
    {
        settime(&mytime, 10, 8, 0);
        setdate(&mydate, 2024, 6, 1);
        timeupdate(&mytime);
        dateupdate(&mydate);
        UARTStringPut("Clock initialized\n");
        wordi = 0;
        return;
    }
    if (strcmp("get time", uart_receive_str) == 0)
    {
        UARTStringPut("Time: ");
        UARTCharPut(UART0_BASE, mytime.hour / 10 + '0');
        UARTCharPut(UART0_BASE, mytime.hour % 10 + '0');
        UARTCharPut(UART0_BASE, ':');
        UARTCharPut(UART0_BASE, mytime.minute / 10 + '0');
        UARTCharPut(UART0_BASE, mytime.minute % 10 + '0');
        UARTCharPut(UART0_BASE, ':');
        UARTCharPut(UART0_BASE, mytime.second / 10 + '0');
        UARTCharPut(UART0_BASE, mytime.second % 10 + '0');
        UARTStringPut("\r\n");
        wordi = 0;
        return;
    }
    if (strcmp("get date", uart_receive_str) == 0)
    {
        UARTStringPut("Date: ");
        UARTCharPut(UART0_BASE, mydate.year / 1000 + '0');
        UARTCharPut(UART0_BASE, mydate.year / 100 % 10 + '0');
        UARTCharPut(UART0_BASE, mydate.year / 10 % 10 + '0');
        UARTCharPut(UART0_BASE, mydate.year % 10 + '0');
        UARTCharPut(UART0_BASE, '/');
        UARTCharPut(UART0_BASE, mydate.month / 10 + '0');
        UARTCharPut(UART0_BASE, mydate.month % 10 + '0');
        UARTCharPut(UART0_BASE, '/');
        UARTCharPut(UART0_BASE, mydate.day / 10 + '0');
        UARTCharPut(UART0_BASE, mydate.day % 10 + '0');
        UARTStringPut("\r\n");
        wordi = 0;
        return;
    }
    if (strcmp("get alarm", uart_receive_str) == 0)
    {
        UARTStringPut("Alarm: ");
        UARTCharPut(UART0_BASE, myalarm.hour / 10 + '0');
        UARTCharPut(UART0_BASE, myalarm.hour % 10 + '0');
        UARTCharPut(UART0_BASE, ':');
        UARTCharPut(UART0_BASE, myalarm.minute / 10 + '0');
        UARTCharPut(UART0_BASE, myalarm.minute % 10 + '0');
        UARTCharPut(UART0_BASE, ':');
        UARTCharPut(UART0_BASE, myalarm.second / 10 + '0');
        UARTCharPut(UART0_BASE, myalarm.second % 10 + '0');
        UARTStringPut("\r\n");
        wordi = 0;
        return;
    }
    for (i = 0; uart_receive_str[i] != '\0'; i++)
    {
        if(uart_receive_str[i] >='0' && uart_receive_str[i] <= '9'){
            numi = i;
            break;
        }
        set_str[i] = uart_receive_str[i];
    }
    while(set_str[i-1] == ' '){
        set_str[i-1] = '\0';
        i--;
    }
    if(strcmp("set time",set_str)==0){

        for (i = numi; uart_receive_str[i] != '\0'; i++)
        {
            if(uart_receive_str[i] == ' '){
                continue;
            }
            if (uart_receive_str[i] == ':')
            {
                k++;
                continue;
            }
            if (k == 0)
            {
                hour = hour * 10 + uart_receive_str[i] - '0';
            }
            else if (k == 1)
            {
                minute = minute * 10 + uart_receive_str[i] - '0';
            }
            else if (k == 2)
            {
                second = second * 10 + uart_receive_str[i] - '0';
            }
        }
        settime(&mytime, hour, minute, second);
        timeupdate(&mytime);
        return;
    }
    if(strcmp("set date",set_str)==0){
        for (i = numi; uart_receive_str[i] != '\0'; i++)
        {
            if (uart_receive_str[i] == ' ')
            {
                continue;
            }
            if (uart_receive_str[i] == ':')
            {
                k++;
                continue;
            }
            if (k == 0)
            {
                year = year * 10 + uart_receive_str[i] - '0';
            }
            else if (k == 1)
            {
                month = month * 10 + uart_receive_str[i] - '0';
            }
            else if (k == 2)
            {
                day = day * 10 + uart_receive_str[i] - '0';
            }
        }
        setdate(&mydate, year, month, day);
        dateupdate(&mydate);
        return;
    }
    if(strcmp("set alarm time",set_str)==0){
        for (i = numi; uart_receive_str[i] != '\0'; i++)
        {
            if (uart_receive_str[i] == ' ')
            {
                continue;
            }
            if (uart_receive_str[i] == ':')
            {
                k++;
                continue;
            }
            if (k == 0)
            {
                hour = hour * 10 + uart_receive_str[i] - '0';
            }
            else if (k == 1)
            {
                minute = minute * 10 + uart_receive_str[i] - '0';
            }
            else if (k == 2)
            {
                second = second * 10 + uart_receive_str[i] - '0';
            }
        }
        setalarm(&myalarm, hour, minute, second);
        alarmupdate(&myalarm);
        return; 
    }
    UARTStringPut("Invalid command\n");
}
// 流水线显示时间日期
void date_flow_display(date *date)
{
    uint8_t j, light;
    for (j = 0; j < 8; j++)
    {
        date_flow_num[j] = datenum[j];
    }
    while (date_flow_flag)
    {
        for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
        {
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[date_flow_num[j]]);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
            Delay(displaydelay);
            button_catch();
        }
    }
}

void time_flow_display(time *time)
{
    uint8_t j, light;
    for (j = 0; j < 8; j++)
    {
        time_flow_num[j] = timenum[j];
        time_flow_num[2] = 17;
        time_flow_num[5] = 17;
    }
    while (time_flow_flag)
    {
        for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
        {
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[time_flow_num[j]]);
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
            Delay(displaydelay);
            button_catch();
        }
    }
}

// 指示灯作用
void light_display()
{
    if (alarm_onoff == 1)
    {
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00);
    }
    else
    {
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xff);
    }
}
// 启动阶段
void setup()
{
    int timer0 = 0;
    int timer1 = 0;
    uint32_t j = 0;
    uint8_t light = 0x80;
    uint32_t xuehao[8] = {5, 4, 0, 0, 1, 9, 1, 3};          // 学号31910045
    I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); // 关闭LED1-8
    // UARTStringPut("Hello World!\r\n");
    pwmflag = 1; // 开启音乐播放
    for (timer1 = 0; timer1 < 3; timer1++)
    {
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00); // 开启LED1-8
        for (timer0 = 0; timer0 < 200; timer0++)
        {
            for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
            {
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[xuehao[j]]);
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
                Delay(displaydelay);
            }
        }
        I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff);
        Delay(1e6); // 大约0.5s
    }

    // 显示姓名字母
    for (timer1 = 0; timer1 < 3; timer1++)
    {
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00); //
        for (timer0 = 0; timer0 < 200; timer0++)
        {
            for (j = 0, light = 0x80; j < 8; j++, light >>= 1)
            {
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, name[j]);
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, light);
                Delay(displaydelay);
            }
        }
        I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff);
        Delay(1e6); //
    }
    
    PWMStop(); // 关闭音乐播放
}

int main()
{
    DevicesInit();
    setup();

    UARTStringPut("Hello World!\r\n");
    // pui32Data[0] = 49;
    // pui32Data[1] = 3;
    // pui32Data[2] = 5;
    // EEPROMProgram(pui32Data, 0x400, sizeof(pui32Data));
    // EEPROMRead(pui32Read, 0x400, sizeof(pui32Read));
    // if(pui32Read[0] == 49 && pui32Read[1] == 3 && pui32Read[2] == 5){
    //     UARTStringPut("EEPROM test pass\n");
    // }
    // else{
    //     UARTStringPut("EEPROM test fail\n");
    // }
    while (1)
    {
        if (date_flow_flag == 0 && time_flow_flag == 0 && dateflag == 0 && timeflag == 0 && alarmflag == 0)
        {
            timeflag = 1;
        }
        showdate(&mydate);
        showtime(&mytime);
        showalarm(&myalarm);
        date_flow_display(&mydate);
        time_flow_display(&mytime);
    }
}