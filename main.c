
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // ��ַ�궨��
#include "inc/hw_types.h"        // �������ͺ궨�壬�Ĵ������ʺ���
#include "inc/hw_timer.h"        // �붨ʱ���йصĺ궨��
#include "inc/hw_ints.h"         // ���ж��йصĺ궨��
#include "driverlib/debug.h"     // ������
#include "driverlib/gpio.h"      // ͨ��IO�ں궨��ͺ���ԭ��
#include "driverlib/pin_map.h"   // TM4Cϵ��MCU��Χ�豸�ܽź궨��
#include "driverlib/sysctl.h"    // ϵͳ���ƶ���
#include "driverlib/systick.h"   // SysTick Driver ����ԭ��
#include "driverlib/interrupt.h" // NVIC�жϿ�����������ԭ��
#include "driverlib/timer.h"     // ��Timer�йصĺ���ԭ��
#include "driverlib/pwm.h"       // ��Timer�йصĺ���ԭ��
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "hw_i2c.h"
#include "i2c.h"
#include "string.h"
#include "ctype.h"
//#include "hibernate.h"
#include "eeprom.h"
//#include "utils/uartstdio.h"

// �������� ��������
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
int displaydelay = 2000; // ������ӳ�
int datenum[8];           //{10604202}
int timenum[8];           //{00 80 01}
int alarmnum[8];
uint8_t datedot[] = {0, 0, dot, 0, dot, 0, 0, 0}; // 2024.06.01,with space in it
//eeprom
uint32_t ui32EEPROMInit;
uint32_t pui32Data[3];
uint32_t pui32Read[3];

uint32_t ui32SysClock;
// �༭ģʽ
int editmode = 0;
int editpart = 0;
int updatetime = 0;
// ��ˮ��
uint32_t date_flow_flag = 0;
uint32_t time_flow_flag = 0;
int date_flow_num[8];
int time_flow_num[8];
int flow_time = 0;
int last = 0;
int datei = 0;
int direction_flag = 0; // 0���ң�1����
int flow_speed = 100;
// PWM
uint8_t times = 0;
uint32_t pwmflag = 0;
uint32_t freq = 1000000;                                                                                                                                                                                                            // PWM???
uint32_t ui32Freq[] = {587, 659, 494, 440, 880, 988, 740, 880, 1175, 1109, 988, 880, 100000, 880, 988, 880, 740, 659, 740, 880, 740, 494, 587, 659, 100000, 659, 880, 740, 587, 494, 440, 494, 659, 988, 880, 988, 880, 740, 659}; // ?????????????????????????,?????100000Hz
uint32_t freqtime[] = {500, 500, 500, 500, 1500, 500, 500, 500, 500, 250, 250, 1500, 500, 1000, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 1500, 500, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 500, 250, 250, 1500, 500};  // ??锟斤�??s
uint32_t timecount = 0;                                                                                                                                                                                                            // ?????
uint32_t notes_num = 0;                                                                                                                                                                                                            // ?????????                                                                                                                                                                                                                // ?????????
// led
uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c, 0x40};
uint8_t name[] = {0x3F, 0x38, 0x77, 0x30, 0x1E, 0x79, 0x3E, 0x76}; // xuejialo
// ״̬�л�
volatile uint32_t state = 0; // state=0��ʾʱ�䣬state=1��ʾ����,state2����
// UART
uint8_t uart_receive_char;
uint8_t uart_receive_str[50] = {0x00};
volatile uint8_t wordi = 0;
int endflag = 0;
// �ж�
uint32_t ui32IntPriorityGroup, ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick, ui32IntPriorityUart0;
// ��ť
uint8_t button = 0xff;
int buttondelay = 0; // ����
uint8_t prev_button = 0xff;
uint32_t buttonstate = 0;
int buttontime = 20;
// ����ԭ������
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
// delay����
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
// ��������

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
    SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY); // 20msִ��һ��
    SysTickEnable();
    SysTickIntEnable();
}
void DevicesInit(void)
{
    // ʹ���ⲿ25MHz��ʱ��Դ������PLL��Ȼ���ƵΪ16MHz
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                      16000000);

    // FPULazyStackingEnable();
    // FPUEnable();
    S800_GPIO_Init();
    S800_I2C0_Init();
    S800_UART_Init();
    PWMInit();     // PWM��ʼ��
    SysTickInit(); // ����SysTick�ж�
    // IntMasterEnable(); // ���ж�����
    // ���ж����ȼ�

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
    // ����EEPROMģ�顣
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    // �ȴ�EEPROMģ��׼���á�
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
    {
    }

    // �ȴ�EEPROM��ʼ����ɡ�
    ui32EEPROMInit = EEPROMInit();

    // ���EEPROM��ʼ���Ƿ񷵻ش��󣬲�֪ͨӦ�ó���
    if (ui32EEPROMInit != EEPROM_INIT_OK)
    {
        while (1)
        {
        }
    }
    // ʱ���ʼ��
    date_and_time_init(&mydate, &mytime);
    alarm_init();
}

void PWMInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // PWM0ʹ��

    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true); // ʹ��(����)PWM0_4�����
    // PWMGenEnable(PWM0_BASE, PWM_GEN_2);             //ʹ��PWM0ģ���2�ŷ�����(��Ϊ4��PWM��2�ŷ�����������)
    // PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, g_ui32SysClock / ui32Freq_Hz); // ����Freq_Hz����PWM����

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // ʹ��GPIOG
    GPIOPinConfigure(GPIO_PK5_M0PWM7);           // �������Ÿ���
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5); // ����ӳ��

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // ����PWM������
    // PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,(PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2)/ 2)); //����ռ�ձ�Ϊ50%
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
            else if (button == 0xfd) // �Ƿ���ˮ
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
            else if (button == 0xfb) // ������ˮ
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
    // ע���·ݶ�Ӧ�����仯
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

// ���ӳ�ʼ��
void alarm_init()
{
    setalarm(&myalarm, 10, 10, 0);
}
// SysTick�жϷ������
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
    // ����
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
    
    // ��ˮ��
    if (date_flow_flag == 1 || time_flow_flag == 1)
    {
        flow_time++;
        if (flow_time == flow_speed)
        { // 1s��ˮһ��
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
// ������ˮ
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
// UARTͨ��
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
        if (uart_receive_str[wordi] == '\r')//�س���
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
//�����
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
    while (uart_receive_str[i] != '\0') // �����ַ�����ֱ�������ַ���������
    {
        if (uart_receive_str[i] == ' ')
        {
            has_space = true; // �ҵ��ո����ñ��Ϊ true
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
        for (i = 0; uart_receive_str[i] != '\0'; i++) // �����ո���һ��
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
        new_str[j] = '\0';                 // ����ַ���������
        strcpy(uart_receive_str, new_str); // ���µ��ַ������ƻ�ԭʼ�ַ���
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
// ��ˮ����ʾʱ������
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

// ָʾ������
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
// �����׶�
void setup()
{
    int timer0 = 0;
    int timer1 = 0;
    uint32_t j = 0;
    uint8_t light = 0x80;
    uint32_t xuehao[8] = {5, 4, 0, 0, 1, 9, 1, 3};          // ѧ��31910045
    I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); // �ر�LED1-8
    // UARTStringPut("Hello World!\r\n");
    pwmflag = 1; // �������ֲ���
    for (timer1 = 0; timer1 < 3; timer1++)
    {
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x00); // ����LED1-8
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
        Delay(1e6); // ��Լ0.5s
    }

    // ��ʾ������ĸ
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
    
    PWMStop(); // �ر����ֲ���
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