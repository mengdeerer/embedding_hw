
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
#include "hw_types.h"
#include "i2c.h"

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
uint32_t dateflag = 0;
uint32_t timeflag = 0;
date mydate;
time mytime;
uint32_t onesec = 0;
int datenum[8];                                   //{10604202}
int timenum[8];                                   //{00 80 01}
uint8_t datedot[] = {0, 0, dot, 0, dot, 0, 0, 0}; // 2024.06.01,with space in it

uint32_t ui32SysClock;
// PWM
uint8_t times = 0;
uint32_t pwmflag = 0;
uint32_t freq = 100000;                                                                                                                                                                                                            // PWM???
uint32_t ui32Freq[] = {587, 659, 494, 440, 880, 988, 740, 880, 1175, 1109, 988, 880, 100000, 880, 988, 880, 740, 659, 740, 880, 740, 494, 587, 659, 100000, 659, 880, 740, 587, 494, 440, 494, 659, 988, 880, 988, 880, 740, 659}; // ?????????????????????????,?????100000Hz
uint32_t freqtime[] = {500, 500, 500, 500, 1500, 500, 500, 500, 500, 250, 250, 1500, 500, 1000, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 1500, 500, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 500, 250, 250, 1500, 500};  // ??锟斤�??s
uint32_t timecount = 0;                                                                                                                                                                                                            // ?????
uint32_t notes_num = 0;                                                                                                                                                                                                            // ?????????                                                                                                                                                                                                                // ?????????
// led
uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c};
uint8_t name[] = {0x3F, 0x38, 0x77, 0x30, 0x1E, 0x79, 0x3E, 0x76}; // xuejialo
// ����ԭ������
void SysTickInit(void); // ����SysTick�ж�
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
// delay����
void Delay(uint32_t value)
{
    uint32_t ui32Loop;
    for (ui32Loop = 0; ui32Loop < value; ui32Loop++)
    {
    };
}
// ��������

// date and time
void date_and_time_init(date *date, time *time)
{
    setdate(date, 2024, 6, 1);
    settime(time, 10, 8, 0);
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
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                         SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                        16000000);

    FPULazyStackingEnable();
    FPUEnable();

    GPIOInit();        // GPIO��ʼ��
    PWMInit();         // PWM��ʼ��
    SysTickInit();     // ����SysTick�ж�
    IntMasterEnable(); // ���ж�����
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

void S800_GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable PortF
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
        ;                                        // Wait for the GPIO moduleF ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Enable PortJ
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
        ; // Wait for the GPIO moduleJ ready

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);             // Set PF0 as Output pin
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set the PJ0,PJ1 as input pin
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void)
{
    uint8_t result;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);     // ?????i2c???
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // ???I2C???0???????????I2C0SCL--PB2??I2C0SDA--PB3
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);             // ????PB2?I2C0SCL
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);             // ????PB3?I2C0SDA
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // I2C??GPIO_PIN_2????SCL
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);    // I2C??GPIO_PIN_3????SDA

    I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); // config I2C0 400k
    I2CMasterEnable(I2C0_BASE);

    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0x0ff); // config port 0 as input
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x0);   // config port 1 as output
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x0);   // config port 2 as output

    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00);  // config port as output
    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); // turn off the LED1-8
    I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x00);
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
            Delay(2000);
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
            Delay(2000);
        }
    }
}

void time_and_date_secupdate(time *time, date *date)
{
    time->second++;
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
    onesec++;
    if (onesec >= 50)
    {
        onesec = 0;
        time_and_date_secupdate(&mytime, &mydate);
        dateupdate(&mydate);
        timeupdate(&mytime);
    }
}

// PWM player
void PWMStart(uint32_t ui32Freq_Hz)
{
    PWMGenDisable(PWM0_BASE, PWM_GEN_3);                                                 // ???PWM0????2???????(???4??PWM??2?????????????)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / ui32Freq_Hz);                   // ????Freq_Hz????PWM????
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 2)); // ?????????50%
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);                                                  // ???PWM0????2???????(???4??PWM??2?????????????)
}
void PWMStop()
{
    pwmflag = 0;
    PWMGenDisable(PWM0_BASE, PWM_GEN_3); // PWM7 
}

// I2c

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
    uint8_t rop;
    while (I2CMasterBusy(I2C0_BASE))
    {
    }; // ???I2C0?????????
    //
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    // ??????????????????????????false???????锟斤�??????true????????????

    I2CMasterDataPut(I2C0_BASE, RegAddr);                         // ????锟斤�??锟斤�?????????
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // ??????锟斤�??????
    while (I2CMasterBusy(I2C0_BASE))
    {
    };

    rop = (uint8_t)I2CMasterErr(I2C0_BASE); // ??????

    I2CMasterDataPut(I2C0_BASE, WriteData);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // ??????锟斤�????????????
    while (I2CMasterBusy(I2C0_BASE))
    {
    };

    rop = (uint8_t)I2CMasterErr(I2C0_BASE); // ??????

    return rop; // ???????????????????0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t value, rop;
    while (I2CMasterBusy(I2C0_BASE))
    {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    //	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND); // ??锟斤�????锟斤�??????
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    rop = (uint8_t)I2CMasterErr(I2C0_BASE);
    Delay(100);
    // receive data
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);            // ?????????
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // ??锟斤�???锟斤�??????
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    value = I2CMasterDataGet(I2C0_BASE); // ????????????
    Delay(100);
    return value;
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
    // pwmflag = 1;// �������ֲ���
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
                Delay(2000);
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
                Delay(2000);
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
    // setup();

    date_and_time_init(&mydate, &mytime);
    timeflag = 1;
    showtime(&mytime);

    // dateflag = 1;
    // showdate(&mydate);
    // I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x80);
    // I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x81);//
    while (1)
    {
    }
}