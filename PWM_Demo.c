
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
#include "PWM.h"    // ��PWM��صĺ���ԭ��


#define SYSTICK_FREQUENCY 50 // SysTickƵ��Ϊ50Hz����ѭ����ʱ����20ms

//*******************************************************************************************************
//
// ����ԭ������
//
//*******************************************************************************************************
void GPIOInit(void);    // GPIO��ʼ��
void SysTickInit(void); // ����SysTick�ж�
void DevicesInit(void); // MCU������ʼ����ע���������������

//*******************************************************************************************************
//
// ��������
//
//*******************************************************************************************************

// 8λ�������ʾ�����ֻ���ĸ����
// ע����������λ�������������Ϊ4��5��6��7��0��1��2��3
uint8_t digit[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

// 8λС���� 1��  0��
// ע����������λС����������������Ϊ4��5��6��7��0��1��2��3
uint8_t pnt = 0x00;

// 8��LEDָʾ��״̬��0��1��
// ע������ָʾ�ƴ������������Ϊ7��6��5��4��3��2��1��0
//     ��ӦԪ��LED8��LED7��LED6��LED5��LED4��LED3��LED2��LED1
uint8_t led[] = {0, 0, 0, 0, 0, 0, 0, 0};

// ��ǰ����ֵ
volatile uint8_t key_code = 0;

// ���ڼ�¼ǰһ�ΰ������ʱ�ļ���״̬��0��ʾ�޼����£�1�м�����
volatile uint8_t key_state = 0;

// ����������Ч��ǣ�0�������²�����1�����м�����
volatile uint8_t key_flag = 0;
uint8_t times = 0;
// ϵͳʱ��Ƶ��
uint32_t g_ui32SysClock;
uint32_t freq = 4000;                                         // PWMƵ��
uint32_t ui32Freq[] = {587,659,494,440,880,988,740,880,1175,1109,988,880,100000,880,988,880,740,659,740,880,740,494,587,659,100000,659,880,740,587,494,440,494,659,988,880,988,880,740,659}; // ����Ƶ�ʣ�����ʵ����Ҫ�����޸�,����Ϊ100000Hz
uint32_t freqtime[] = {500,500,500,500,1500,500,500,500,500,250,250,1500,500,1000,500,500,500,500,1000,500,500,500,500,1500,500,500,500,500,500,1000,500,500,500,500,500,250,250,1500,500}; // ��λΪs
uint32_t timecount = 0;                                       // ��ʱ��
uint32_t i = 0;
uint32_t pwmflag = 1;
//*******************************************************************************************************
//
// ������
//
//*******************************************************************************************************
int main(void)
{

    DevicesInit(); //  MCU������ʼ��


    // uint32_t ui32Freq = 500;  // ����Ƶ�ʣ�����ʵ����Ҫ�����޸�

    while (1)
    {
        // if(key_flag == 1)    // �м�����
        //{
        // key_flag = 0;

        // switch (key_code)
        // {
        // case 1: // ����Ƶ��Ϊui32Freq�ķ����ź�
        //     PWMStart(freq);
        //     break;
        // case 2: // ֹͣ����PWM�ź�
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
// ����ԭ�ͣ�void GPIOInit(void)
// �������ܣ�GPIO��ʼ����ʹ��PortK������PK4,PK5Ϊ�����ʹ��PortM������PM0Ϊ�����
//          ��PK4����TM1638��STB��PK5����TM1638��DIO��PM0����TM1638��CLK��
// ������������
// ��������ֵ����
//
//*******************************************************************************************************
void GPIOInit(void)
{
    // ����TM1638оƬ�ܽ�
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // ʹ�ܶ˿� K
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
    {
    }; // �ȴ��˿� K׼�����

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM); // ʹ�ܶ˿� M
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
    {
    }; // �ȴ��˿� M׼�����

    // ���ö˿� K�ĵ�4,5λ��PK4,PK5��Ϊ�������		PK4-STB  PK5-DIO
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // ���ö˿� M�ĵ�0λ��PM0��Ϊ�������   PM0-CLK
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);
}

//*******************************************************************************************************
//
// ����ԭ�ͣ�SysTickInit(void)
// �������ܣ�����SysTick�ж�
// ������������
// ��������ֵ����
//
//*******************************************************************************************************
void SysTickInit(void)
{
    SysTickPeriodSet(g_ui32SysClock / SYSTICK_FREQUENCY); // ������������,��ʱ����20ms
    SysTickEnable();                                      // SysTickʹ��
    SysTickIntEnable();                                   // SysTick�ж�����
}

//*******************************************************************************************************
//
// ����ԭ�ͣ�DevicesInit(void)
// �������ܣ�MCU������ʼ��������ϵͳʱ�����á�GPIO��ʼ����SysTick�ж�����
// ������������
// ��������ֵ����
//
//*******************************************************************************************************
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

//*******************************************************************************************************
//
// ����ԭ�ͣ�void SysTick_Handler(void)
// �������ܣ�SysTick�жϷ�����򣬼�ⰴ��������ˢ���������ʾ
// ������������
// ��������ֵ����
//
//*******************************************************************************************************
void SysTick_Handler(void) // ��ʱ����Ϊ20ms
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
