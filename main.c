
//--------------------------------------//
//          ADC 7 channel               //
//          3.0, 3.1, 3.2, 3.3 PIN      //
//          A12, A13, A14, A15          //
//          4.0, 4.1, 4.2 PIN           //
//          A8,  A9,  A10               //
//                                      //
//          PWM 4 channel               //
//          3.4, 3.5, 3.6, 3.7 PIN      //
//          TB0.3, TB0.4 TB0.5 TB0.6    //
//                                      //
//          UART ( <-> Displayer)       //
//          P2.5, P2.6                  //
//          UCA1TXD, UCA1RXD            //
//                                      //
//          UART ( <-> PC )             //
//          P5.4, P5.5                  //
//          UCA2TXD,, UCA2RXD           //
//                                      //
//          Temperature Send            //
//          PID Control                 //
//          AutoTuning Mode             //
//--------------------------------------//


// Normal Mode
// temperature controller 0x90 -> PC
// temperature controller <- PC 0x80

// AT Mode START
// temperature controller 0x92 -> PC
// temperature controller <- PC 0x82

// AT Mode STOP
// temperature controller 0x91 -> PC
// temperature controller <- PC 0x81


//-------------------------------------------------------------------
//RX data is 1byte - 2byte - 2byte - 2byte - 2byte - 1byte - 1byte
//RX data[11]
//Rx data([1]+[2])/10 = CH1 target temp
//RX data([3]+[4])/10 = CH2 target temp
//RX data([5]+[6])/10 = CH3 target temp
//RX data([7]+[8])/10 = CH4 target temp
//RX data[9] = MSB --- Heater On/Off (State/Control bit)

// UCA3TXD -> UCA2TXD
// UCA3RXD -> UCA2RXD

//-------------------------------------------------------------------

#include <msp430.h>
#include "HAL/include/uart.h"
#include "HAL/include/adc.h"
#include "HAL/include/pwm.h"
#include "HAL/include/i2c.h"
//#include "driverlib.h"
/////// we will use FRAM CTRL


#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
//#include <time.h>
#include <math.h>
#include "Driver/include/LNJT103F_NTC.h"

#define OFFSETVOLTAGE 0.1
#define MSP_CPU_CLK 4000000

//---------------------- PID Command ---------------------------



float Kp_Value[4] = {1.8, 1.75, 1.65, 1.75};
float Ki_Value[4] = {0.6, 0.5, 0.55, 0.5};
float Kd_Value[4] = {3.75, 3.65, 3.55, 3.65};


// 현재 PID는 근사치에 가까움 PID 설정셋팅 그만

//float Kp_Value[4] = {0.06, 0.06, 0.03, 0.15};
//float Ki_Value[4] = {0.0, 0.0, 0.0, 0.0};
//float Kd_Value[4] = {0.0, 0.0, 0.0, 0.0};

//백업
//float Kp_Value[4] = {0.012, 0.06, 0.006, 0.15};
//float Ki_Value[4] = {0.0048, 0.0, 0.0024, 0.0};
//float Kd_Value[4] = {0.0008, 0.0, 0.0004, 0.0};
// zigler nicholes sample
// 1.8 - 0.6 - /3.75
// 1.2 - 0.4  2.5
// 1.0   0.33  2.08

// D값 만 조절하면 될듯
//

// 1.44     0.072   0.03
// 2.4      0.018   0.01
// 2.4      0.018   0.01
// 2.4      0.018   0.01
// Ziegler Nicholes Methode 참조




////////// arm:   1.8,  0.6,  3.75
////////// back:
////////// leg: 1.65, 0.55, 3.55
////////// roller:  1.8, 0.6, 3.75


unsigned char P_CastingBuffer[4][4];
unsigned char I_CastingBuffer[4][4];
unsigned char D_CastingBuffer[4][4];

unsigned char pid_CastingBuffer[4][4];

unsigned char FtoCBuffer[48];
// unsigned char CtoFBuffer[48];
volatile unsigned char CtoFBuffer[48];
unsigned int ADC_Chenck[4];
// CH1 PID: P(0~3) I(4~7) D(8~11)
// CH2 PID
// CH3 PID
// CH4 PID: P(36~39) I(40~43) D(44~47)
//unsigned char PID_TuningBuffer[48];

// sample Tuning base 2
//double Kp_Value[4] = {17, 17, 17, 17};           // PID k1 = 1.2 , k2 = 2, k3 = 0.5 , T = 3.4sec , L = 0.2sec
//double Ki_Value[4] = {0.4, 0.4, 0.4, 0.4};               // Kp = k1*T/L = 17, Ki = 2L = 0.4, Kd = 0.5L = 0.1
//double Kd_Value[4] = {0.1, 0.1, 0.1, 0.1};

volatile unsigned int PID_Flag[4] = {0, };     // if receive 'on' -> start PID, / 'off' -> end PID
//volatile float pidResult;

volatile float pidResult[4] = {0, 0, 0, 0};     // pull up
float AT_pidResult[4] = {0, 0, 0, 0};

//volatile int ConfigTemp[4] = {35, 35, 35, 35};

volatile float ConfigTemp[4] = {35, 35, 35, 35};
volatile unsigned int SendGpioFlag[4] = {0, };

float TB_Temp;

float sensitivity = 0.3;       // 0에 가까워 질 수록 필터링 잘됨
float filteringValue[4];
float noFlilterTemp[4];
//--------------------------------------------------------------

//---------------------- UART Command --------------------------

unsigned char CheckQueue[11];

unsigned char PC_SendMessageFlag;
unsigned char DP_SendMessageFlag;

volatile unsigned int xFlag = 0;
volatile int a;

unsigned short int tempIntVal_ch1;
unsigned short int targetIntVal_ch1;

unsigned short int tempIntVal_ch2;
unsigned short int targetIntVal_ch2;

unsigned short int tempIntVal_ch3;
unsigned short int targetIntVal_ch3;

unsigned short int tempIntVal_ch4;
unsigned short int targetIntVal_ch4;

unsigned short tempIntVal_i2c;

unsigned char PC_reQuestFlag = 0;
unsigned char PC_TransmitStart = 0;
unsigned char DP_reQuestFlag = 0;
unsigned char DP_TransmitStart = 0;

//unsigned char TuningStartFlag = 0;
//unsigned char TransmitStart = 0;

//--------------------------------------------------------------

//////////////////////////       ADC       //////////////////////////

float voltageArray[4];
float TempVal[8];
float PreTempVal[4] = {100, 100, 100, 100};

float TempI2C = 45.2;          // ���� ���ؼ� �ӽ� �µ� ����


unsigned int ADC_Result[8];
volatile int ADC_CalcurationFlag;
//unsigned int ADC_Result[9];
//unsigned int ADC_Result;
//float ;
float ADC_Sample[4];
volatile unsigned int ADC_Temp[8];
unsigned char SaveOnOffState = 0x00;



//volatile unsigned int ADC_Temp[5];
int sample_index = 0;
int ADC_CH1_Controller;
int I2C_Sensor;

int SampleComplete;
int ADC_FLAG;
int END_FLAG;
//------------------------ Clock Check -------------------------
//clock_t start, finish;
//double duration;
//--------------------------------------------------------------

void _commandZone(unsigned char *_newCommand);
void Init_PWM_GPIO(void);
void Init_SMCLK_16MHZ(void);
void Init_ADC_GPIO(void);
void Init_ADC(void);
void processPcCommand(Data command);
void Init_TIMER_A0(void);

// change parameter and return value as float -> double
float PID_Contorller(float kp, float ki, float kd, float targetValue, float readValue);

int AnalogRead(uint8_t channel);
void MessageTx(void);
void MessageTx2(void);
void Tx_PID_Tuning(unsigned char commandTx, unsigned char SelectedChannel);


void stopWatchDog(void)
{
    WDTCTL = WDTPW + WDTHOLD;         // Stop Watch Dog timer
}


int timeSamplingCount;
int timeCount_PID;
int monitoringTime;

extern unsigned char CheckBreak;

//------------------------------- TB I2C -----------------------------------------------------------

int TB_Sensor_Flag;

#define TBP_ADDR 0x3A

int     RX_Byte_Ctr,        // Coutner to make sure all of the information is received
        TX_Byte_Ctr,        // Counter to make sure all of the information is sent
        i;                  // Integer used for counting sent bytes
char    i2cData[3],          // Creates an array to store data
        //sending[1],         // Creates an array to store the sent data
        *pointer,           // Creates a pointer to access the array
        *txPtr;

int16_t _rawObject;
int16_t result_object;
//int16_t _rawSensor;
uint8_t BUF[5];
int16_t *dest_call;
int PEC_ture;

const unsigned char crc8_table[256]=          //CRC table (Please don't change this value)
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


float CalcTemp(int rawTemp);                    // �µ����
uint8_t GetObject(void);                   // ���µ� �б�
uint8_t CalPEC(uint8_t *crc, uint8_t nBytes);  // PEC ����





void getData(void){

    pointer = &i2cData;                       // Sets the pointer to the height array
    RX_Byte_Ctr = 3;                        // Determines the number of bytes received
    TX_Byte_Ctr = 1;                        // Determines the number of bytes sent
    UCB2I2CSA = TBP_ADDR;           // Sets slave address

    UCB2TBCNT = 0x01;   // Expecting to receive 3 bytes of data
    UCB2CTLW0 |= UCTR | UCTXSTT;            // Enables TX Mode, Sends start condition
    //__bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt ???

    //UCB2CTLW1 |= UCASTP_2;  // Sends stop bit when UCTBCNT is reached
    UCB2TBCNT = 3;   // Expecting to receive 3 bytes of data
    //__delay_cycles(20);
    //__delay_cycles(2000);
    __delay_cycles(2000);
    UCB2CTLW0 &= ~UCTR;                      // Enters RX Mode
    UCB2CTLW0 |= UCTXSTT;                    // Sends start condition
    //__bis_SR_register(LPM0_bits | GIE);     // Enters Low-Power mode and enables global interrupt  ???
}

//----------------------------------------------------------------------------------------------------------------------

//-------------------------------------- FRAM ACCESS ----------------------------------------------------
// 4byte (variables Type = float) * 3 (control number = P, I, D) * 4 (Channel number) = 48

#define WRITE_SIZE      48              // 90 +48

#define FRAM_ADDR       0x004000     // allocation 300(decimal)

unsigned char RAMData;

//void FRAMWrite(unsigned char inputData);
void FRAMWrite(unsigned char *inputData);
void FloatToByte(void);
void ByteToFloat(void);

unsigned char P_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
unsigned char I_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
unsigned char D_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte

float Kp_Temp[4];
float Ki_Temp[4];
float Kd_Temp[4];

int FLASH_FLAG;


#if defined(__TI_COMPILER_VERSION__)
#pragma PERSISTENT(FRAM_write)
unsigned char FRAM_write[WRITE_SIZE] = {0};
#elif defined(__IAR_SYSTEMS_ICC__)
__persistent unsigned char FRAM_write[WRITE_SIZE] = {0};

#elif defined(__GNUC__)
unsigned char __attribute__((persistent)) FRAM_write[WRITE_SIZE] = {0};

#else
#error Compiler not supported!
#endif


extern int BufferCount;
extern int ShiftCount;
extern unsigned char ucRxBuffer[12];
extern Queue rxPcDataQueue;
extern int BufferCount2;
extern unsigned char BufferChecking2[12];
//int AT_MODE_FLAG;

unsigned char AT_NORMAL_FLAG = 0;
unsigned char AT_STOP_FLAG = 0;
unsigned char AT_START_FLAG = 0;
unsigned char DP_NORMAL_FLAG = 0;

unsigned char Selected_Channel;
unsigned short int AT_currentTemp;
unsigned short int AT_targetTemp;

unsigned char TuningStartFlag = 0;
unsigned char TuningStopFlag = 0;

float voltage_check[4];
float roller_voltage;

unsigned char Relay_Check[4];
unsigned char checkTemp[4];
unsigned char RelayCheckFlag;
unsigned char SaveKey[4];

unsigned char rxReFlag;
int pwmResult;

int main(void)
{
    int i, k = 0;
    //int adc_index = 0;
    Init_ADC_GPIO();
    Init_PWM_GPIO();

    //test GPIO
    P4DIR |= BIT3;

    // Relay GPIO
    P2DIR |= BIT2;


    Init_SMCLK_16MHZ();
    stopWatchDog();

    i2cInit();
    __delay_cycles(4800000);        // 16MHz / 2 about 0.5sec
    Init_UART();                        // uart.h
    Init_UART2();
    QueueInit(&rxPcDataQueue);                     // Queue initialize

    Init_TIMER_A0();


    int readIndex = 0;
    //int mIndex = 0;

    //__disable_interrupt();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    RAMData = ( *(volatile unsigned char *)(FRAM_ADDR) );      // Initialize Fram Position


        for(readIndex = 0; readIndex < 48; readIndex++)
        {
            // array to save

            int tempP;
            CtoFBuffer[readIndex] = (*(volatile unsigned char *)(FRAM_ADDR + readIndex));
        }

    ByteToFloat();
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, interrupts enabled
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;


    // Interrupt enable for global FRAM memory protection
    GCCTL0 |= WPIE;


    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR1 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR3 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR4 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR5 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle
    TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
    TB0CCR6 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

    //__bis_SR_register(LPM0_bits | GIE); // Enter LPM0, interrupts enabled
    __enable_interrupt();

    // Relay GPIO HIGH -> Relay Off state
    // Relay GPIO LOW -> RElay ON state
    P2OUT |= BIT2;              // Relay GPIO HIGH
    //P2OUT &= ~BIT2;           // RElay GPIO LOW

    while (1)
    {
        // temperature controller normally operate mode
        // Connected Verify

        if(ADC_CalcurationFlag == 1)
        {

            sample_index++;

            if(sample_index == 5)
            {
                for(i=0;i<8;i++){

                    TempVal[i] = temp_Calculator(ADC_Result[i] /= 5);
                    TempVal[i] = 100 - TempVal[i];
                    ADC_Result[i] = 0;
                }
                sample_index = 0;
            }

            ADC_CalcurationFlag = 0;
        }

        // 1. recognize connection status and mode setting

        if(QIsEmpty(&rxPcDataQueue) != 1)
        {
            processPcCommand(Dequeue(&rxPcDataQueue));
        }


        // 1. 전시기 명령어 처리 구간 / 튜닝 진입시 처리 안함
        if(RxData2 == 0xa5)
        {
            if(BufferCount2 >= 12)
            {
                // Displayer command process
                BufferCount2 = 0;
                if(BufferChecking2[0] == 0xfc && BufferChecking2[11] == 0xa5)
                {
                    _commandZone(BufferChecking2);
                }
            }
        }

        /*
        if(AT_NORMAL_FLAG == 1 || AT_START_FLAG == 1)
        {
            // 2. I2C sensor process
            if(TB_Sensor_Flag)
            {
                getData();                                  // period per

                //printf("LOW_BYTE %d\r\n", (uint8_t)i2cData[0]);
                //printf("HIGH BYTE %d\r\n", (uint8_t)i2cData[1]);
                //printf("PEC %d\r\n", (uint8_t)i2cData[2]);

                //PEC ������ ���� ������ ����
                BUF[0] = TBP_ADDR<<1;
                BUF[1] = 0x07;
                BUF[2] = (TBP_ADDR<<1) | 0x01;
                BUF[3] = (uint8_t)i2cData[0];
                BUF[4] = (uint8_t)i2cData[1];

                if ((uint8_t)i2cData[2] == CalPEC(BUF, 5))              // ������ ������ PEC ���� �´��� ����
                {
                  *dest_call = ((uint8_t)i2cData[1]<<8) | (uint8_t)i2cData[0];    // ������ ������ �µ� ������ ����
                  dest_call = &_rawObject;
                  PEC_ture = 1;
                }
                else
                {
                    PEC_ture = 0;
                }

                if(GetObject())              // ���µ� �� �����µ� Read �Ϸ�Ǹ�
                {
                    TempI2C =  CalcTemp(result_object);
                }

                TB_Sensor_Flag = 0;
            }

        }
        */
        // common process heater ON/OFF
        if(AT_START_FLAG == 1)
        {
            if(SendGpioFlag[0])
            {
                // PWM Control Start
                // duty is PID result ( = range)
                SendGpioFlag[0] = 0;

                TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

                TB0CCR3 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[0] * 100));                          // CCR1 PWM duty cycle

                TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

            }

            if(SendGpioFlag[1])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[1] = 0;

                TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set

                TB0CCR4 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[1] * 100));                          // CCR1 PWM duty cycle

                TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
            }
            if(SendGpioFlag[2])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[2] = 0;

                TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set

                TB0CCR5 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[2] * 100));                          // CCR1 PWM duty cycle

                TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
            }
            if(SendGpioFlag[3])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[3] = 0;

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set

                //TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[3] * 100));                          // CCR1 PWM duty cycle

                TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set

                //TB0CCR1 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[3] * 100));                          // CCR1 PWM duty cycle

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set

                TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[2] * 100));                          // CCR1 PWM duty cycle

                //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
            }
        }

        if((DP_NORMAL_FLAG || AT_NORMAL_FLAG) && AT_START_FLAG != 1)
        {
            if(SendGpioFlag[0])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[0] = 0;

                //-----------------------------Save Trigger zone ---------------------------
                /*
                if(TempVal[0] > ConfigTemp[0] + 3.6)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR3 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                    SaveKey[0] = 1;
                }
                else if(SaveKey[0] == 1 && TempVal[0] <= ConfigTemp[0] + 3.6 && TempVal[0] >= ConfigTemp[0]-0.5)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR3 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else if( (SaveKey[0] == 1 && TempVal[0] < ConfigTemp[0]-0.9) || SaveKey[0] == 0)
                {
                    SaveKey[0] = 0;
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR3 = (( (MSP_CPU_CLK/5000)-1 )/100)*((int)( (1-pidResult[0]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                //------------------------------------------------------------------------------------------------------------------------

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR3 = ((MSP_CPU_CLK/5000)/100)*((int)(pidResult[0] * 100));                          // CCR1 PWM duty cycle
                /*
                if(pidResult[0] <= 0)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR3 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR3 = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[0]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/

                //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period




                //TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

                //pwmResult = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[0]) * 100));
                //pwmResult = ((float)( (MSP_CPU_CLK/5000)-2 )/100.0)*((int)( (1-pidResult[0]) * 100));

                //TB0CCR3 = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[0]) * 100));                          // CCR1 PWM duty cycle
                //TB0CCR3 = ((float)( (MSP_CPU_CLK/5000)-1 )/100.0)*((int)( (1-pidResult[0]) * 100));                 // CCR1 PWM duty cycle
                //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                TB0CCR3 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[0]) * 100));                 // CCR1 PWM duty cycle
                TB0CTL &= ~(TBIFG);                 // Clear FLAG

                //if( (TempVal[0] > ConfigTemp[0] + 1)  || (TempVal[1] > ConfigTemp[1] + 1) || (TempVal[2] > ConfigTemp[2] + 1) || (TempVal[3] > ConfigTemp[3] + 1))
                //{
                //    P2OUT |= BIT2;
                //}
                //if( (TempVal[0] < ConfigTemp[0])  && (TempVal[1] < ConfigTemp[1] ) && (TempVal[2] < ConfigTemp[2]) && (TempVal[3] < ConfigTemp[3]) )
                //{
                //    P2OUT &= ~BIT2;
                //}

            }

            if(SendGpioFlag[1])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[1] = 0;

                //-----------------------------Save Trigger zone ---------------------------
                /*
                if(TempVal[1] > ConfigTemp[1] + 3.6)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR4 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                    SaveKey[1] = 1;
                }
                else if(SaveKey[1] == 1 && TempVal[1] <= ConfigTemp[1] + 3.6 && TempVal[1] >= ConfigTemp[1]-0.9)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR4 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else if( (SaveKey[1] == 1 && TempVal[1] < ConfigTemp[1]-0.9) || SaveKey[1] == 0)
                {
                    SaveKey[1] = 0;
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR4 = (( (MSP_CPU_CLK/5000)-1 )/100)*((int)( (1-pidResult[1]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                //--------------------------------------------------------

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR4 = ((MSP_CPU_CLK/5000)/100)*((int)(pidResult[1] * 100));                          // CCR1 PWM duty cycle
                /*
                if(pidResult[1] <= 0)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR4 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR4 = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[1]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
                //TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR4 = ((float)( (MSP_CPU_CLK/5000)-1 )/100.0)*((int)( (1-pidResult[1]) * 100));                 // CCR1 PWM duty cycle

                //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                TB0CCR4 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[1]) * 100));                 // CCR1 PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG

            }
            if(SendGpioFlag[2])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[2] = 0;

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR5 = ((MSP_CPU_CLK/5000)/100)*((int)(pidResult[2] * 100));                          // CCR1 PWM duty cycle
                /*
                if(TempVal[2] > ConfigTemp[2] + 3.6)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR5 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                    SaveKey[2] = 1;
                }
                else if(SaveKey[2] == 1 && TempVal[2] <= ConfigTemp[2] + 3.6 && TempVal[2] >= ConfigTemp[2]-0.9)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR5 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else if( (SaveKey[2] == 1 && TempVal[2] < ConfigTemp[2]-0.9) || SaveKey[2] == 0)
                {
                    SaveKey[2] = 0;
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set

                    TB0CCR5 = (( (MSP_CPU_CLK/5000)-1 )/100)*((int)( (1-pidResult[2]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                /*
                if(pidResult[2] <= 0)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR5 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR5 = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[2]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
                //TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR5 = ((float)( (MSP_CPU_CLK/5000)-1 )/100.0)*((int)( (1-pidResult[2]) * 100));                 // CCR1 PWM duty cycle                          // CCR1 PWM duty cycle

                //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                TB0CCR5 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[2]) * 100));                 // CCR1 PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
                //if( (TempVal[0] > ConfigTemp[0] + 1)  || (TempVal[1] > ConfigTemp[1] + 1) || (TempVal[2] > ConfigTemp[2] + 1) || (TempVal[3] > ConfigTemp[3] + 1))
                //{
                //    P2OUT |= BIT2;
                //}
                //if( (TempVal[0] < ConfigTemp[0])  && (TempVal[1] < ConfigTemp[1] ) && (TempVal[2] < ConfigTemp[2]) && (TempVal[3] < ConfigTemp[3]) )
                //{
                //    P2OUT &= ~BIT2;
                //}
            }
            if(SendGpioFlag[3])
            {
                // PWM Control Start
                // duty is PID result ( = range)

                SendGpioFlag[3] = 0;

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)(pidResult[3] * 100));                          // CCR1 PWM duty cycle

                //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                //SendGpioFlag[3] = 0;

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR1 = (( MSP_CPU_CLK/5000 )/100)*((int)(pidResult[3] * 100));                          // CCR1 PWM duty cycle

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)(pidResult[3] * 100));                          // CCR1 PWM duty cycle

                /*
                if(TempVal[3] > ConfigTemp[3] + 3.6)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR1 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR6 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                    SaveKey[3] = 1;
                }
                else if(SaveKey[3] == 1 && TempVal[3] <= ConfigTemp[3] + 3.6 && TempVal[3] >= ConfigTemp[3]-0.9)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR1 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR6 = ( (MSP_CPU_CLK/5000)-1 );                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                }
                else if( (SaveKey[3] == 1 && TempVal[3] < ConfigTemp[3]-0.9) || SaveKey[3] == 0)
                {
                    SaveKey[3] = 0;
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR1 = (( (MSP_CPU_CLK/5000)-1 )/100)*((int)( (1-pidResult[3]) * 100));                          // CCR1 PWM duty cycle

                    //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR6 = (( (MSP_CPU_CLK/5000)-1 )/100)*((int)( (1-pidResult[3]) * 100));                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                /*
                if(pidResult[3] <= 0)
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                    //TB0CCR6 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                    //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR1 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }
                else
                {
                    TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                    TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                    TB0CCR1 = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[3]) * 100));                          // CCR1 PWM duty cycle
                    TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                }*/
                //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
                //TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR1 = ((float)( (MSP_CPU_CLK/5000)-1 )/100.0)*((int)( (1-pidResult[3]) * 100));                 // CCR1 PWM duty cycle
                //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

                TB0CCR1 = ((float)( (MSP_CPU_CLK/5000) )/100.0)*((int)( (1-pidResult[3]) * 100));                 // CCR1 PWM duty cycle
                TB0CTL &= ~(TBIFG);     // Clear FLAG
                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                //TB0CCR6 = (( (MSP_CPU_CLK/5000)-2 )/100)*((int)( (1-pidResult[3]) * 100));                          // CCR1 PWM duty cycle


                //if( (TempVal[0] > ConfigTemp[0] + 1)  || (TempVal[1] > ConfigTemp[1] + 1) || (TempVal[2] > ConfigTemp[2] + 1) || (TempVal[3] > ConfigTemp[3] + 1))
                //{
                //    P2OUT |= BIT2;
                //}
                //if( (TempVal[0] < ConfigTemp[0])  && (TempVal[1] < ConfigTemp[1] ) && (TempVal[2] < ConfigTemp[2]) && (TempVal[3] < ConfigTemp[3]) )
                //{
                //    P2OUT &= ~BIT2;
                //}
            }
            /*
            if(checkTemp[0] == 1)
            {
                checkTemp[0]= 0;

                if((TempVal[0] > PreTempVal[0] + 3) || (TempVal[1] > PreTempVal[1] + 3) || (TempVal[2] > PreTempVal[2] + 3) || (TempVal[3] > PreTempVal[3] + 3))
                {
                    // Relay GPIO HIGH (ENABLE)
                    P2OUT |= BIT2;
                }
                //if(TempVal[0] < PreTempVal[0]+1 && TempVal[1] < PreTempVal[1]+1 && TempVal[2] < PreTempVal[2]+1 && TempVal[3] < PreTempVal[3]+1)
                //{
                    // Relay GPIO LOW (DISABLE)
                //   P2OUT &= ~BIT2;
                //}

                PreTempVal[0] = TempVal[0];
                //PreTempVal[1] = TempVal[1];
                //PreTempVal[2] = TempVal[2];
                //PreTempVal[3] = TempVal[3];
            }
            if(checkTemp[1] == 1)
            {
                checkTemp[1] = 0;
                if( (TempVal[0] > PreTempVal[0] + 3) || (TempVal[1] > PreTempVal[1] + 3) || (TempVal[2] > PreTempVal[2] + 3) || (TempVal[3] > PreTempVal[3] + 3))
                {
                    // Relay GPIO HIGH (ENABLE)
                    P2OUT |= BIT2;
                }
                //if(TempVal[0] < PreTempVal[0]+1 && TempVal[1] < PreTempVal[1]+1 && TempVal[2] < PreTempVal[2]+50 && TempVal[3] < PreTempVal[3]+50)
                //{
                //    // Relay GPIO LOW (DISABLE)
                //    P2OUT &= ~BIT2;
                //}

                PreTempVal[1] = TempVal[1];
            }
            if(checkTemp[2] == 1)
            {
                checkTemp[2] = 0;

                if(TempVal[2] > PreTempVal[2] + 3)
                {
                    // Relay GPIO HIGH (ENABLE)
                    P2OUT |= BIT2;
                }
                //if(TempVal[0] < PreTempVal[0]+1 && TempVal[1] < PreTempVal[1]+1 && TempVal[2] < PreTempVal[2]+1 && TempVal[3] < PreTempVal[3]+1)
                //{
                //    // Relay GPIO LOW (DISABLE)
                //    P2OUT &= ~BIT2;
                //}

                PreTempVal[2] = TempVal[2];
            }
            if(checkTemp[3] == 1)
            {
                checkTemp[3] = 0;

                if(TempVal[3] > PreTempVal[3] + 3)
                {
                    // Relay GPIO HIGH (ENABLE)
                    P2OUT |= BIT2;
                }
                //if(TempVal[0] < PreTempVal[0]+1 && TempVal[1] < PreTempVal[1]+1 && TempVal[2] < PreTempVal[2]+1 && TempVal[3] < PreTempVal[3]+1)
                //{
                    // Relay GPIO LOW (DISABLE)
                //    P2OUT &= ~BIT2;
                //}

                PreTempVal[0] = TempVal[0];
            }
            */
            /*
            // monitoring in 1sec
            if(RelayCheckFlag == 1)
            {
                RelayCheckFlag = 0;

                // monitoring SW State

                if(PID_Flag[0] == 1)
                {
                    // SW1 Arm ON
                    if( (TempVal[0] > ConfigTemp[0] + 3) )
                    {
                        // Relay Enable
                        P2OUT &= ~BIT2;
                    }
                }
                else
                {
                    // SW1 OFF
                    if(ConfigTemp[0] < 40)
                    {
                        if(TempVal[0] > 40)
                        {
                            P2OUT &= ~BIT2;
                        }
                    }

                    else
                    {

                        if(TempVal[0] >= 40 && TempVal[0] < 60)
                        {
                            if( TempVal[0] >= PreTempVal[0] + 2.8 )
                            {
                                P2OUT &= ~BIT2;
                            }
                            PreTempVal[0] = TempVal[0];
                        }
                    }

                }

                if(PID_Flag[1] == 1)
                {
                    // SW2 Back ON
                    if( (TempVal[1] > ConfigTemp[1] + 3))
                    {
                        P2OUT &= ~BIT2;
                    }
                }
                else
                {
                    // SW2 OFF
                    if(ConfigTemp[1] < 40)
                    {
                        if(TempVal[1] > 40)
                        {
                            P2OUT &= ~BIT2;
                        }
                    }

                    else
                    {
                        if(TempVal[1] >= 40 && TempVal[1] < 60)
                        {
                            if( TempVal[1] >= PreTempVal[1] + 2.8)
                            {
                                P2OUT &= ~BIT2;
                            }
                            PreTempVal[1] = TempVal[1];
                        }
                    }
                }
                if(PID_Flag[2] == 1)
                {
                    // SW3 Leg ON
                    if( (TempVal[2] > ConfigTemp[2] + 3) )
                    {
                        P2OUT &= ~BIT2;
                    }
                }
                else
                {
                    // SW3 OFF
                    if(ConfigTemp[2] < 40)
                    {
                        if(TempVal[2] > 40)
                        {
                            P2OUT &= ~BIT2;
                        }
                    }

                    else
                    {
                        if(TempVal[2] >= 40 && TempVal[2] < 60)
                        {
                            if( TempVal[2] >= PreTempVal[2] + 2.8 )
                            {
                                P2OUT &= ~BIT2;
                            }
                            PreTempVal[2] = TempVal[2];
                        }
                    }
                }
                if(PID_Flag[3] == 1)
                {
                    // SW4 Roller ON
                    if( (TempVal[3] >= ConfigTemp[3] + 3) )
                    {
                        P2OUT &= ~BIT2;
                    }
                }
                else
                {
                    // SW4 OFF
                    if(ConfigTemp[3] < 40)
                    {
                        if(TempVal[3] > 40)
                        {
                            P2OUT &= ~BIT2;
                        }
                    }

                    else
                    {
                        if(TempVal[3] >= 40 && TempVal[3] < 60)
                        {
                            if( TempVal[3] >= PreTempVal[3] + 2.8 )
                            {
                                P2OUT &= ~BIT2;
                            }
                            PreTempVal[3] = TempVal[3];
                        }
                    }
                }

            }*/


        }

        /*
        if(SendGpioFlag[0])
        {
            // PWM Control Start
            // duty is PID result ( = range)

            SendGpioFlag[0] = 0;

            TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set

            if(AT_START_FLAG == 1)
            {
                TB0CCR3 = ((MSP_CPU_CLK/5000)/100)*((int)((1-AT_pidResult[0]) * 100));                          // CCR1 PWM duty cycle
            }
            else
            {
                TB0CCR3 = ((MSP_CPU_CLK/5000)/100)*((int)((1-pidResult[0]) * 100));                          // CCR1 PWM duty cycle
            }

            //TB0CCR3 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[0] * 100));                          // CCR1 PWM duty cycle
            TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR

        }

        if(SendGpioFlag[1])
        {
            // PWM Control Start
            // duty is PID result ( = range)

            SendGpioFlag[1] = 0;

            TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set

            if(AT_START_FLAG == 1)
            {
                TB0CCR4 = ((MSP_CPU_CLK/5000)/100)*((int)((1-AT_pidResult[1]) * 100));                          // CCR1 PWM duty cycle
            }
            else
            {
                TB0CCR4 = ((MSP_CPU_CLK/5000)/100)*((int)((1-pidResult[1]) * 100));                          // CCR1 PWM duty cycle
            }

            //TB0CCR4 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[1] * 100));                          // CCR1 PWM duty cycle
            TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }
        if(SendGpioFlag[2])
        {
            // PWM Control Start
            // duty is PID result ( = range)

            SendGpioFlag[2] = 0;

            TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set

            if(AT_START_FLAG == 1)
            {
                TB0CCR5 = ((MSP_CPU_CLK/5000)/100)*((int)((1-AT_pidResult[2]) * 100));                          // CCR1 PWM duty cycle
            }
            else
            {
                TB0CCR5 = ((MSP_CPU_CLK/5000)/100)*((int)((1-pidResult[2]) * 100));                          // CCR1 PWM duty cycle
            }

            //TB0CCR5 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[2] * 100));                          // CCR1 PWM duty cycle
            TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }
        if(SendGpioFlag[3])
        {
            // PWM Control Start
            // duty is PID result ( = range)

            SendGpioFlag[3] = 0;

            TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set

            if(AT_START_FLAG == 1)
            {
                TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)((1-AT_pidResult[3]) * 100));                          // CCR1 PWM duty cycle
            }
            else
            {
                TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)((1-pidResult[3]) * 100));                          // CCR1 PWM duty cycle
            }

            //TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*((int)(AT_pidResult[3] * 100));                          // CCR1 PWM duty cycle
            TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }
        */
        if(AT_NORMAL_FLAG || AT_START_FLAG)
        {
            MessageTx();
        }

        if(DP_NORMAL_FLAG)
        {
            MessageTx2();
        }

        if(AT_NORMAL_FLAG == 1 && AT_START_FLAG != 0)
        {
            if(FLASH_FLAG == 1)
            {
                __disable_interrupt();
                FRAMWrite(FtoCBuffer);
                __enable_interrupt();

                FLASH_FLAG = 0;
            }
        }
    }
}

void Init_TIMER_A0(void)
{
    // Timer

    //TA0CCR0 = 8000;                         // 2KHz = 0.5ms
    TA0CCR0 = 1000;                           // Frequency -> (16Mhz/1600)/8 = 1250Hz
    //TA0CTL = TASSEL__SMCLK | MC__CONTINOUS; // SMCLK, continuous mode
    TA0CTL = TASSEL_2 | ID_3 | MC_1;          // SMCLK, continuous mode
    TA0CCTL0 = CCIE;                          // TACCR0 interrupt enabled
}



void Init_SMCLK_16MHZ(void)
{
    /////////////////// SMCLK = 16MHZ ////////////////////////////////
    P5DIR |= BIT5;
    P5SEL0 |= BIT5;                         // Output ACLK
    P5SEL1 |= BIT5;

    P5DIR |= BIT6;
    P5SEL1 |= BIT6;                         // Output SMCLK
    P5SEL0 |= BIT6;

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                     // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz
    // Set SMCLK = MCLK = DCO, ACLK = VLOCLK
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;   // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz
    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    //CSCTL3 = DIVA__1 | DIVS__4 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL3 = DIVA__1 | DIVS__4 | DIVM__1;   // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                            // Lock CS registers                      // Lock CS registers
    /////////////////// SMCLK = 16MHZ ////////////////////////////////
    ////////////////// SMCLK -> 4MHz  ////////////////////////////////
}


void processPcCommand(Data command)
{
    switch(command.mode)
    {
    case 0x80:  //connect requeast
        PC_SendMessageFlag = 1;
        PC_reQuestFlag = 1;
        MessageTx();    // Feedback Message
        break;
    case 0xc0:  //connect Ack Req
        PC_reQuestFlag = 0;
        AT_NORMAL_FLAG = 1;
        break;


    }

}

void _commandZone(unsigned char *_newCommand)
{
    // Displayer - normal mode
    if(_newCommand[0] == 0xfc && _newCommand[1] == 0xB0 &&_newCommand[11] == 0xa5 && DP_reQuestFlag == 0)
    {
        // Receive Configuration Temperature and Heater ON/OFF Status (DP 3.1)

        SaveOnOffState = _newCommand[10];

        if(((_newCommand[2]<<8)+_newCommand[3]) <= 3000)
        //if(((_newCommand[2]<<8)+_newCommand[3]) <= 650)
        {
            //CH1 Temperature
            ConfigTemp[0] = ((float)((_newCommand[2]<<8)+_newCommand[3]))/10.0;
        }
        if(((_newCommand[4]<<8)+_newCommand[5]) <= 3000)
        //if(((_newCommand[4]<<8)+_newCommand[5]) <= 650)
        {
            //CH2 Temperature
            ConfigTemp[1] = ((float)((_newCommand[4]<<8)+_newCommand[5]))/10.0;
        }
        if(((_newCommand[6]<<8)+_newCommand[7]) <= 3000)
        //if(((_newCommand[6]<<8)+_newCommand[7]) <= 650)
        {
            //CH3 Temperature
            ConfigTemp[2] = ((float)((_newCommand[6]<<8)+_newCommand[7]))/10.0;
        }
        if(((_newCommand[8]<<8)+_newCommand[9]) <= 3000)
        //if(((_newCommand[8]<<8)+_newCommand[9]) <= 65.0)
        {
            //CH4 Temperature
            ConfigTemp[3] = ((float)((_newCommand[8]<<8)+_newCommand[9]))/10.0;
        }


        // Heater On/Off
        if( (SaveOnOffState & (0x1 << 7 )) >= 1)
        {
            PID_Flag[0] = 1;
        }
        else
        {
            PID_Flag[0] = 0;
        }
        if( (SaveOnOffState & (0x1 << 6 )) >= 1)
        {
            PID_Flag[1] = 1;
        }
        else
        {
            PID_Flag[1] = 0;
        }
        if( (SaveOnOffState & (0x1 << 5 )) >= 1)
        {
            PID_Flag[2] = 1;
        }
        else
        {
            PID_Flag[2] = 0;
        }
        if( (SaveOnOffState & (0x1 << 4 )) >= 1)
        {
            PID_Flag[3] = 1;
        }
        else
        {
            PID_Flag[3] = 0;
        }
        if( (SaveOnOffState & (0x1 << 3 )) >= 1)
        {
            ADC_CH1_Controller = 0;
            I2C_Sensor = 1;
        }
        else
        {
            ADC_CH1_Controller = 1;
            I2C_Sensor = 0;
        }
        AT_NORMAL_FLAG = 0;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 0;
    }


    // PC - normal mode
    if(_newCommand[0] == 0xfd && _newCommand[1] == 0x80 &&_newCommand[11] == 0xa5 && PC_reQuestFlag == 0)
    {

        // 1byte shift
        // receive Gain setting command (PC 3.2)
        if(_newCommand[10] == 0x07)
        {
            unsigned char ChannelTemp = _newCommand[6];     // ChannelTemp = Channel - 1

            // it is command that set Gain (���̺��ϰ� ���� ���� ���� �Ȱ� �ѹ� ����)
            if(_newCommand[7] == 0x00)
            {
                float FloatP_Gain;
                // P- Gain Set
                // P- Gain Buffer (Channel Select)

                // test RealTerm Test
                P_CastingBuffer[ChannelTemp][0] = _newCommand[5];
                P_CastingBuffer[ChannelTemp][1] = _newCommand[4];
                P_CastingBuffer[ChannelTemp][2] = _newCommand[3];
                P_CastingBuffer[ChannelTemp][3] = _newCommand[2];

                memcpy(&FloatP_Gain, P_CastingBuffer[ChannelTemp], sizeof(float));              // unsisgned char  ->  Float
                Kp_Value[ChannelTemp] = FloatP_Gain;

                // Flash Memory Write
                FloatToByte();
                //FRAMWrite(FtoCBuffer);
                FLASH_FLAG = 1;

                Tx_PID_Tuning(1, ChannelTemp);
            }
            else if(_newCommand[7] == 0x01)
            {
                // I-Gain Set
                float FloatI_Gain;
                // I- Gain Set
                // I- Gain Buffer (Channel Select)
                // test RealTerm Test
                I_CastingBuffer[ChannelTemp][0] = _newCommand[5];
                I_CastingBuffer[ChannelTemp][1] = _newCommand[4];
                I_CastingBuffer[ChannelTemp][2] = _newCommand[3];
                I_CastingBuffer[ChannelTemp][3] = _newCommand[2];

                memcpy(&FloatI_Gain, I_CastingBuffer[ChannelTemp], sizeof(float));

                Ki_Value[ChannelTemp] = FloatI_Gain;

                // Flash Memory Write
                FloatToByte();
                FLASH_FLAG = 1;
                //FRAMWrite(FtoCBuffer);
                Tx_PID_Tuning(1, ChannelTemp);

            }
            else if(_newCommand[7] == 0x02)
            {
                // D-Gain Set
                //unsigned char GainTemp[4];
                float FloatD_Gain;
                // D- Gain Set
                // D- Gain Buffer (Channel Select)
                // test RealTerm Test
                D_CastingBuffer[ChannelTemp][0] = _newCommand[5];
                D_CastingBuffer[ChannelTemp][1] = _newCommand[4];
                D_CastingBuffer[ChannelTemp][2] = _newCommand[3];
                D_CastingBuffer[ChannelTemp][3] = _newCommand[2];

                memcpy(&FloatD_Gain, D_CastingBuffer[ChannelTemp], sizeof(float));
                Kd_Value[ChannelTemp] = FloatD_Gain;

                // Flash Memory Write
                FloatToByte();
                //FRAMWrite(FtoCBuffer);
                FLASH_FLAG = 1;

                Tx_PID_Tuning(1, ChannelTemp);

            }
        }
        else if(_newCommand[10] == 0x05)
        {
            unsigned char ChannelTemp = _newCommand[6];
            // ��� ä���� ���� ���¸� �ѹ��� ����

            memcpy(P_CastingBuffer[0], &Kp_Value[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
            memcpy(P_CastingBuffer[1], &Kp_Value[1], sizeof(float));
            memcpy(P_CastingBuffer[2], &Kp_Value[2], sizeof(float));
            memcpy(P_CastingBuffer[3], &Kp_Value[3], sizeof(float));

            memcpy(I_CastingBuffer[0], &Ki_Value[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
            memcpy(I_CastingBuffer[1], &Ki_Value[1], sizeof(float));
            memcpy(I_CastingBuffer[2], &Ki_Value[2], sizeof(float));
            memcpy(I_CastingBuffer[3], &Ki_Value[3], sizeof(float));


            memcpy(D_CastingBuffer[0], &Kd_Value[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
            memcpy(D_CastingBuffer[1], &Kd_Value[1], sizeof(float));
            memcpy(D_CastingBuffer[2], &Kd_Value[2], sizeof(float));
            memcpy(D_CastingBuffer[3], &Kd_Value[3], sizeof(float));

            Tx_PID_Tuning(2, ChannelTemp);
        }
        else if(_newCommand[10] == 0x02)
        {
            unsigned char ChannelTemp = _newCommand[6];

            memcpy(P_CastingBuffer[ChannelTemp], &Kp_Value[ChannelTemp], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1

            memcpy(I_CastingBuffer[ChannelTemp], &Ki_Value[ChannelTemp], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1

            memcpy(D_CastingBuffer[ChannelTemp], &Kd_Value[ChannelTemp], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1

            // ä���� �о �� ä���� ���¸� ����
            Tx_PID_Tuning(3, ChannelTemp);
        }
        else
        {
            // Configuration target Temperature (PC 2.2)

            SaveOnOffState = _newCommand[10];

            if(((_newCommand[2]<<8)+_newCommand[3]) <= 3000)
            {
                //CH1 Temperature
                ConfigTemp[0] = ((float)((_newCommand[2]<<8)+_newCommand[3]))/10.0;
            }
            if(((_newCommand[4]<<8)+_newCommand[5]) <= 3000)
            {
                //CH2 Temperature
                ConfigTemp[1] = ((float)((_newCommand[4]<<8)+_newCommand[5]))/10.0;
            }
            if(((_newCommand[6]<<8)+_newCommand[7]) <= 3000)
            {
                //CH3 Temperature
                ConfigTemp[2] = ((float)((_newCommand[6]<<8)+_newCommand[7]))/10.0;
            }
            if(((_newCommand[8]<<8)+_newCommand[9]) <= 3000)
            {
                //CH4 Temperature
                ConfigTemp[3] = ((float)((_newCommand[8]<<8)+_newCommand[9]))/10.0;
            }


            // Heater On/Off
            if( (SaveOnOffState & (0x1 << 7 )) >= 1)
            {
                PID_Flag[0] = 1;
            }
            else
            {
                PID_Flag[0] = 0;
            }
            if( (SaveOnOffState & (0x1 << 6 )) >= 1)
            {
                PID_Flag[1] = 1;
            }
            else
            {
                PID_Flag[1] = 0;
            }
            if( (SaveOnOffState & (0x1 << 5 )) >= 1)
            {
                PID_Flag[2] = 1;
            }
            else
            {
                PID_Flag[2] = 0;
            }
            if( (SaveOnOffState & (0x1 << 4 )) >= 1)
            {
                PID_Flag[3] = 1;
            }
            else
            {
                PID_Flag[3] = 0;
            }
            if( (SaveOnOffState & (0x1 << 3 )) >= 1)
            {
                ADC_CH1_Controller = 0;
                I2C_Sensor = 1;
            }
            else
            {
                ADC_CH1_Controller = 1;
                I2C_Sensor = 0;
            }
        }

        AT_NORMAL_FLAG = 1;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 0;
    }
    else if(_newCommand[0] == 0xfd && _newCommand[1] == 0x82 &&_newCommand[11] == 0xa5 && TuningStartFlag == 1)
    {
        // AT-mode Start
        // receive P-I-D result, channel (PC 4.3)

        unsigned char ChannelTemp = _newCommand[10];
        if(ChannelTemp == 0x00)
        {
            // channel 1
            Selected_Channel = 0x80;
        }
        else if(ChannelTemp == 0x01)
        {
            // channel 2
            Selected_Channel = 0x40;
        }
        else if(ChannelTemp == 0x02)
        {
            // channel 3
            Selected_Channel = 0x20;
        }
        else if(ChannelTemp == 0x03)
        {
            // channel 4
            Selected_Channel = 0x10;
        }


        //float AT_curretTemp;

        unsigned char pidNgain = _newCommand[9];
        SaveOnOffState = Selected_Channel;
        // Tuning START
        /*
        if(((_newCommand[2]<<8)+_newCommand[3]) <= 3000)
        {
            //CH1 Temperature
            ConfigTemp[ChannelTemp] = ((float)((_newCommand[2]<<8)+_newCommand[3]))/10.0;
        }
        */
        if(pidNgain == 0x00)
        {
            // pidResult
            float Floatpid_Gain;
            // D- Gain Set
            // D- Gain Buffer (Channel Select)
            // test RealTerm Test
            pid_CastingBuffer[ChannelTemp][0] = _newCommand[5];
            pid_CastingBuffer[ChannelTemp][1] = _newCommand[4];
            pid_CastingBuffer[ChannelTemp][2] = _newCommand[3];
            pid_CastingBuffer[ChannelTemp][3] = _newCommand[2];

            memcpy(&Floatpid_Gain, pid_CastingBuffer[ChannelTemp], sizeof(float));
            AT_pidResult[ChannelTemp] = Floatpid_Gain;
        }

        // Heater On/Off
        if( (SaveOnOffState & (0x1 << 7 )) >= 1)
        {
            PID_Flag[0] = 1;
        }
        else
        {
            PID_Flag[0] = 0;
        }
        if( (SaveOnOffState & (0x1 << 6 )) >= 1)
        {
            PID_Flag[1] = 1;
        }
        else
        {
            PID_Flag[1] = 0;
        }
        if( (SaveOnOffState & (0x1 << 5 )) >= 1)
        {
            PID_Flag[2] = 1;
        }
        else
        {
            PID_Flag[2] = 0;
        }
        if( (SaveOnOffState & (0x1 << 4 )) >= 1)
        {
            PID_Flag[3] = 1;
        }
        else
        {
            PID_Flag[3] = 0;
        }
        /*
        if( (SaveOnOffState & (0x1 << 3 )) >= 1)
        {
            ADC_CH1_Controller = 0;
            I2C_Sensor = 1;
        }
        else
        {
            ADC_CH1_Controller = 1;
            I2C_Sensor = 0;
        }
        */
        AT_NORMAL_FLAG = 0;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 1;
    }
    else if(_newCommand[0] == 0xfd && _newCommand[1] == 0x82 &&_newCommand[11] == 0xa5 && TuningStartFlag == 0)
    {
        // response Tuning Start (PC 4.1)
        TuningStartFlag = 1;
        Selected_Channel = _newCommand[10];
        AT_NORMAL_FLAG = 0;
        AT_STOP_FLAG = 0;
        AT_START_FLAG = 1;

        //TuningStopFlag = 0;
        MessageTx();            // Feedback Message (Start Tuning mode)
    }
    else if(_newCommand[0] == 0xfd && _newCommand[1] == 0x81 &&_newCommand[11] == 0xa5 && TuningStartFlag == 1)
    {
        // Auto Tuning Stop (PC 4.4)
        // pid-Gain

        // Send PID Stop Signal
        // Exit Tunning StartFlag = 0
        Selected_Channel = 0x00;
        SaveOnOffState = 0x00;

        TuningStartFlag = 0;
        TuningStopFlag = 1;

        AT_START_FLAG = 0;
        AT_STOP_FLAG = 1;
        // Send Stop feedback

        MessageTx();        // Feedback Message

        // return Normal Mode
        AT_NORMAL_FLAG = 1;
        AT_STOP_FLAG = 0;

        TuningStartFlag = 0;
        //TuningStopFlag = 0;
    }


    // With PC
    //////////////////////  read: 12 byte ////////////////////////

    if(_newCommand[0] == 0xfd && _newCommand[1] == 0xC0 && _newCommand[11] == 0xa5 && PC_reQuestFlag == 1)
    {
        // Start Transmit (PC 1.3)
        PC_reQuestFlag = 0;
        AT_NORMAL_FLAG = 1;
    }

    if(_newCommand[0] == 0xfd && _newCommand[1] == 0xA0 && _newCommand[11] == 0xa5 && PC_reQuestFlag == 0)
    {
        // Verify connect request (PC 1.1)
        PC_reQuestFlag = 1;

        //AT_START_FLAG = 0;
        //AT_STOP_FLAG = 0;
        //AT_NORMAL_FLAG = 0;
        MessageTx();    // Feedback Message
    }


    // With Displayer

    if(_newCommand[0] == 0xfc && _newCommand[1] == 0x90 && _newCommand[11] == 0xa5 && DP_reQuestFlag == 1 )
    {
        // Start Transmit (DP 1.3)
        //DP_TransmitStart = 1;
        DP_reQuestFlag = 0;
        DP_NORMAL_FLAG = 1;
    }

    if(_newCommand[0] == 0xfc && _newCommand[1] == 0x70 && _newCommand[11] == 0xa5 && (DP_reQuestFlag == 0 || rxReFlag == 0))
    {
        // Verify connect request (DP 1.1)
        rxReFlag = 1;
        DP_reQuestFlag = 1;
        //AT_START_FLAG = 0;
        //AT_STOP_FLAG = 0;
        //DP_NORMAL_FLAG = 0;
        MessageTx2();    // Feedback Message
    }

}

// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    int i = 0;
    //static int timeCount_PID = 0;
    //static int timeSamplingCount = 0;
    //static int timeSendMessage = 0;
    //static int timeCount_I2C = 0;

    //timeSendMessage += 1;
    timeCount_PID += 1;
    timeSamplingCount += 1;
    //timeCount_I2C += 1;
    //monitoringTime += 1;


    if(timeSamplingCount >= 10)         // 0.02sec       51ms
    {
        // P4OUT ^= BIT3;
        // ADC value
        // Sampling until 0.1sec
        // ADC sampling
        // 1 channel

        // adc 8 ch
        ADC_Result[0] += AnalogRead(ADC12INCH_12);
        ADC_Result[1] += AnalogRead(ADC12INCH_13);
        ADC_Result[2] += AnalogRead(ADC12INCH_14);
        ADC_Result[3] += AnalogRead(ADC12INCH_15);
        ADC_Result[4] += AnalogRead(ADC12INCH_8);
        ADC_Result[5] += AnalogRead(ADC12INCH_9);
        ADC_Result[6] += AnalogRead(ADC12INCH_10);
        ADC_Result[7] += AnalogRead(ADC12INCH_11);


       //__bic_SR_register_on_exit(LPM0_bits);
        ADC_CalcurationFlag = 1;
        timeSamplingCount = 0;
    }

    if(timeCount_PID >= 50)            // 125 = 0.1s                25 = 178.6ms         5 = 51ms    10 = 83ms   15   83
    {
        TB_Sensor_Flag = 1;
        /*
        if(SampleComplete)
        {
            int k = 0;
            for(k =0; k<4; k++){
                voltageArray[k] = ((ADC_Sample[k])/4095.0) * 3.3;

                // Resistor Change : 2.4K -> 2.5K
                //TempVal[k] = ((-3.0257)*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                (33.586*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                ((-146.22)*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                (317.92*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                ((-365.75)*voltageArray[k]*voltageArray[k]) +
                //                                (244.58 * voltageArray[k]) -57.066;

                //TempVal[k] = ((-36.606)*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                (268.19*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                ((-770.59)*voltageArray[k]*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                                (1105.8*voltageArray[k]*voltageArray[k]*voltageArray[k]) +
                //                               ((-839.64)*voltageArray[k]*voltageArray[k]) +
                //                                (370.58 * voltageArray[k]) -57.066;
                //TempVal[k] = (voltageArray[k] + 0.05) / 0.04923 + 0.00;

                TempVal[k] = (0.01586)*ADC_Sample[k];
            }

            SampleComplete = 0;


        }
        */
        if(TempVal[0] < 0)
        {
            TempVal[0] = 0;
        }
        else if(TempVal[0] > 300)
        {
            TempVal[0] = 300;
        }
        if(TempVal[1] < 0)
        {
            TempVal[1] = 0;
        }
        else if(TempVal[1] > 300)
        {
            TempVal[1] = 300;
        }
        if(TempVal[2] < 0)
        {
            TempVal[2] = 0;
        }
        else if(TempVal[2] > 300)
        {
            TempVal[2] = 300;
        }
        if(TempVal[3] < 0)
        {
            TempVal[3] = 0;
        }
        else if(TempVal[3] > 300)
        {
            TempVal[3] = 300;
        }

        // ch1 byte split
        tempIntVal_ch1 = ((floor(TempVal[0]*10)/10) *10);
        targetIntVal_ch1 = (ConfigTemp[0]*10);


        tempIntVal_ch2 = ((floor(TempVal[1]*10)/10) *10);
        targetIntVal_ch2 = (ConfigTemp[1]*10);

        // ch3 byte split
        tempIntVal_ch3 = ((floor(TempVal[2]*10)/10) *10);
        targetIntVal_ch3 = (ConfigTemp[2]*10);

        // ch4 byte split
        tempIntVal_ch4 = ((floor(TempVal[3]*10)/10) *10);
        targetIntVal_ch4 = (ConfigTemp[3]*10);


        // I2C byte split
        tempIntVal_i2c = ((floor(TempI2C*10)/10) *10);
        //*p_I2C_temp = (char *)&tempIntVal_i2c;

        if(AT_START_FLAG == 1)
        {
            AT_currentTemp = ((floor(TempVal[Selected_Channel] *10) /10) *10);
            //AT_targetTemp = (ConfigTemp[Selected_Channel]*10);
        }

        PC_SendMessageFlag = 1;
        DP_SendMessageFlag = 1;

        if(PID_Flag[0])
        {

             if(ADC_CH1_Controller == 1)
             {
                 /*
                 if(TempVal[0] > ConfigTemp[0]+0.5)
                  {
                      pidResult[0] = 0;
                      TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                      TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
                      TB0CCR3 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                      TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                      SendGpioFlag[0] = 0;
                  }
                 else
                 {
                     pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], TempVal[0]);
                     SendGpioFlag[0] = 1;
                 }*/
                 pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], TempVal[0]);
                 //pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], 44.8);
                 SendGpioFlag[0] = 1;
             }
             if(I2C_Sensor == 1)
             {
                 pidResult[0] = PID_Contorller(Kp_Value[0], Ki_Value[0], Kd_Value[0], ConfigTemp[0], TempI2C);
                 SendGpioFlag[0] = 1;
             }

        }
        else
        {
            // Heater Off
            //Relay_Check[0] = 1;

            pidResult[0] = 0;
            //P3OUT &= ~BIT4;         // must change output LOW
            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
            //TB0CCR3 = ((MSP_CPU_CLK/5000)/100)*(0);                          // CCR1 PWM duty cycle

            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL3 = OUTMOD_7;                    // CCR1 reset/set
            TB0CCR3 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

            TB0CTL &= ~(TBIFG);
            //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }

        if(PID_Flag[1])
        {
            /*
            if(TempVal[1] > ConfigTemp[1]+0.5)
             {
                 pidResult[1] = 0;
                 TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                 TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
                 TB0CCR4 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                 TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                 SendGpioFlag[1] = 0;
             }
             else
             {
                 pidResult[1] = PID_Contorller(Kp_Value[1], Ki_Value[1], Kd_Value[1], ConfigTemp[1], TempVal[1]);
                 SendGpioFlag[1] = 1;
             }*/
             pidResult[1] = PID_Contorller(Kp_Value[1], Ki_Value[1], Kd_Value[1], ConfigTemp[1], TempVal[1]);
             SendGpioFlag[1] = 1;
        }
        else
        {
            // Heater Off
            //Relay_Check[1] = 1;

            pidResult[1] = 0;
            //P3OUT &= ~BIT5;         // must change output LOW
            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
            //TB0CCR4 = ((MSP_CPU_CLK/5000)/100)*(0);                          // CCR1 PWM duty cycle

            //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            //TB0CCTL4 = OUTMOD_7;                    // CCR1 reset/set
            TB0CCR4 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

            TB0CTL &= ~(TBIFG);
            //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }

        if(PID_Flag[2])
        {
            /*
             if(TempVal[2] > ConfigTemp[2]+0.5)
             {
                 pidResult[2] = 0;
                 TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                 TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
                 TB0CCR5 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                 TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                 SendGpioFlag[2] = 0;
             }
             else
             {
                 pidResult[2] = PID_Contorller(Kp_Value[2], Ki_Value[2], Kd_Value[2], ConfigTemp[2], TempVal[2]);
                 SendGpioFlag[2] = 1;
             }*/
             pidResult[2] = PID_Contorller(Kp_Value[2], Ki_Value[2], Kd_Value[2], ConfigTemp[2], TempVal[2]);
             SendGpioFlag[2] = 1;
        }
        else
        {
            // Heater Off
            //Relay_Check[2] = 1;

            pidResult[2] = 0;
            //P3OUT &= ~BIT6;         // must change output LOW
            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
            //TB0CCR5 = ((MSP_CPU_CLK/5000)/100)*(0);                          // CCR1 PWM duty cycle

            //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            //TB0CCTL5 = OUTMOD_7;                    // CCR1 reset/set
            TB0CCR5 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

            TB0CTL &= ~(TBIFG);

            //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }

        if(PID_Flag[3])
        {
            /*
            if(TempVal[3] > ConfigTemp[3])
            {
                pidResult[3] = 0;
                TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
                TB0CCR6 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
                TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
                TB0CCR1 = (MSP_CPU_CLK/5000)-2;                          // CCR1 PWM duty cycle

                TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
                SendGpioFlag[3] = 0;
            }
            else
            {
                pidResult[3] = PID_Contorller(Kp_Value[3], Ki_Value[3], Kd_Value[3], ConfigTemp[3], TempVal[3]);
                SendGpioFlag[3] = 1;
            }*/
            pidResult[3] = PID_Contorller(Kp_Value[3], Ki_Value[3], Kd_Value[3], ConfigTemp[3], TempVal[3]);
            SendGpioFlag[3] = 1;
        }
        else
        {
            // Heater Off
            //Relay_Check[3] = 1;

            pidResult[3] = 0;
            //P3OUT &= ~BIT7;         // must change output LOW
            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
            //TB0CCR6 = ((MSP_CPU_CLK/5000)/100)*(0);                          // CCR1 PWM duty cycle

            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
            //TB0CCR1 = ((MSP_CPU_CLK/5000)/100)*(0);                          // CCR1 PWM duty cycle


            //TB0CCR0 = (MSP_CPU_CLK/5000);                       // PWM Period
            //TB0CCTL6 = OUTMOD_7;                    // CCR1 reset/set
            TB0CCR6 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

            //TB0CCR0 = (MSP_CPU_CLK/5000)-1;                       // PWM Period
            //TB0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
            TB0CCR1 = (MSP_CPU_CLK/5000);                          // CCR1 PWM duty cycle

            TB0CTL &= ~(TBIFG);

            //TB0CTL = TBSSEL__SMCLK | MC__UP | TBCLR;// SMCLK, up mode, clear TBR
        }




        timeCount_PID = 0;
    }
    /*
    if(monitoringTime >= 1000)
    {
        RelayCheckFlag = 1;

        monitoringTime = 0;
    }*/

}

//float PID_Contorller(float kp, float ki, float kd, float targetValue, float readValue)
float PID_Contorller(float kp, float ki, float kd, float targetValue, float readValue)
{
    //static float PID_Value, pidP, pidI, pidD, pidError, preError;
    static float PID_Value, pidP, pidI, pidD, pidError, preError;

    pidError = targetValue - readValue;

    pidP = kp * pidError;
    pidI = ki * (pidI + pidError);
    pidD = kd * ( (pidError - preError));

    PID_Value = pidP + pidI + pidD;
    /*
    if(PID_Value < 0)
    {
        PID_Value = 0;
    }
    if(PID_Value > 100)
    {
        PID_Value = 100;
    }
    */
    if(PID_Value <= 0)
    {
        PID_Value = 0;
    }
    if(PID_Value > 1)
    {
        PID_Value = 1;
    }

    preError = pidError;
    return PID_Value;
}


void MessageTx(void)
{
    // Verify Request connect (PC 1.2)
    if(PC_SendMessageFlag && PC_reQuestFlag)
    {
        //UCA1IE &= ~UCRXIE;
        //UCA2IE &= ~UCRXIE;

        fput_data(0xfe);

        fput_data(0xB0);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0xa5);

        PC_SendMessageFlag = 0;

        //UCA2IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }

    // Send message about current Temperature and target temperature (PC 1.4 / 2.1 / 3.1)
    if(PC_SendMessageFlag == 1 && AT_NORMAL_FLAG == 1)
    {

        if( ((tempIntVal_ch1 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch2 & (0xFF << 8)) >> 8)  < 0x03
                && ((tempIntVal_ch3 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch4 & (0xFF << 8)) >> 8)  < 0x03
                && ((targetIntVal_ch1 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch2 & (0xFF << 8)) >> 8) < 0x03
                && ((targetIntVal_ch3 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch4 & (0xFF << 8)) >> 8) < 0x03
        )
        {
            //UCA1IE &= ~UCRXIE;
            //UCA3IE &= ~UCRXIE;

            fput_data(0xfe);

            fput_data(0x90);

            fput_data( (tempIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data( (tempIntVal_i2c & (0xFF << 8)) >> 8 );
            fput_data( (tempIntVal_i2c & (0xFF << 0)) >> 0 );

            fput_data( SaveOnOffState );

            fput_data( 0xa5 );

            PC_SendMessageFlag = 0;

            //UCA3IE |= UCRXIE;
            //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
        }
    }

    // Send message about current Temperature in Tuning Start (4.2)
    if(PC_SendMessageFlag && (AT_START_FLAG))
    {
        if( ((AT_currentTemp & (0xFF << 8)) >> 8)  < 0x03 )
        {
            //UCA1IE &= ~UCRXIE;
            //UCA3IE &= ~UCRXIE;                  // prevent to occur overrun (rx)

            fput_data(0xfe);                                            // 1

            fput_data(0x92);                                            // 2

            fput_data( (AT_currentTemp & (0xFF << 8)) >> 8 );            // 3
            fput_data( (AT_currentTemp & (0xFF << 0)) >> 0 );            // 4

            //fput_data( (AT_curretTemp & (0xFF << 8)) >> 8 );
            //fput_data( (AT_curretTemp & (0xFF << 0)) >> 0 );

            fput_data(0x00);                                            // 5
            fput_data(0x00);                                            // 6
            fput_data(0x00);                                            // 7
            fput_data(0x00);                                            // 8
            fput_data(0x00);                                            // 9
            fput_data(0x00);                                            // 10

            fput_data(0x00);                                            // 11
            fput_data(0x00);                                            // 12
            fput_data(0x00);                                            // 13
            fput_data(0x00);                                            // 14
            fput_data(0x00);                                            // 15

            fput_data(0x00);                                            // 16
            fput_data(0x00);                                            // 17
            fput_data(0x00);                                            // 18
            fput_data(0x00);                                            // 19
            fput_data(0x00);                                            // 20

            //fput_data2( SaveOnOffState );

            fput_data(Selected_Channel);                                  // 21

            fput_data( 0xa5 );                                          // 22

            PC_SendMessageFlag = 0;

            //UCA3IE |= UCRXIE;
            //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
        }
    }

    // Send message about Tuning Stop (4.5)
    if(PC_SendMessageFlag && AT_STOP_FLAG)
    {
        //UCA1IE &= ~UCRXIE;
        //UCA3IE &= ~UCRXIE;                  // prevent to occur overrun (rx)

        fput_data(0xfe);                                            // 1

        fput_data(0x91);                                            // 2

        fput_data(0x00);                                            // 3
        fput_data(0x00);                                            // 4

        //fput_data( (AT_curretTemp & (0xFF << 8)) >> 8 );
        //fput_data( (AT_curretTemp & (0xFF << 0)) >> 0 );

        fput_data(0x00);                                            // 5
        fput_data(0x00);                                            // 6
        fput_data(0x00);                                            // 7
        fput_data(0x00);                                            // 8
        fput_data(0x00);                                            // 9
        fput_data(0x00);                                            // 10

        fput_data(0x00);                                            // 11
        fput_data(0x00);                                            // 12
        fput_data(0x00);                                            // 13
        fput_data(0x00);                                            // 14
        fput_data(0x00);                                            // 15

        fput_data(0x00);                                            // 16
        fput_data(0x00);                                            // 17
        fput_data(0x00);                                            // 18
        fput_data(0x00);                                            // 19
        fput_data(0x00);                                            // 20

        //fput_data2( SaveOnOffState );

        fput_data(Selected_Channel);                                  // 21

        fput_data( 0xa5 );                                          // 22

        PC_SendMessageFlag = 0;

        //UCA3IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }
}

void MessageTx2(void)
{
    // Verify Request connect (DP 1.2)
    if(DP_SendMessageFlag && (DP_reQuestFlag || rxReFlag))
    {
        //UCA1IE &= ~UCRXIE;
        //UCA3IE &= ~UCRXIE;
        rxReFlag = 0;
        fput_data2(0xfb);

        fput_data2(0x80);

        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
        fput_data2( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);
        fput_data2(0x00);

        fput_data2(0xa5);

        DP_SendMessageFlag = 0;

        //UCA3IE |= UCRXIE;
        //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
    }

    // Send message about current Temperature and target temperature (DP 2.1 / 3.2)
    if(DP_SendMessageFlag == 1 && DP_NORMAL_FLAG == 1)
    {

        if( ((tempIntVal_ch1 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch2 & (0xFF << 8)) >> 8)  < 0x03
                && ((tempIntVal_ch3 & (0xFF << 8)) >> 8)  < 0x03 && ((tempIntVal_ch4 & (0xFF << 8)) >> 8)  < 0x03
                && ((targetIntVal_ch1 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch2 & (0xFF << 8)) >> 8) < 0x03
                && ((targetIntVal_ch3 & (0xFF << 8)) >> 8) < 0x03 && ((targetIntVal_ch4 & (0xFF << 8)) >> 8) < 0x03
        )

        //if(tempIntVal_ch1 < 0x2BC && tempIntVal_ch1 < 0x)
        {
            //UCA1IE &= ~UCRXIE;                  // prevent to occur overrun (rx)
            //UCA3IE &= ~UCRXIE;

            fput_data2(0xfb);

            fput_data2(0xA0);

            fput_data2( (tempIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch1 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch1 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );
            //fput_data2( (targetIntVal_ch2 & (0xFF << 8)) >> 8 );
            //fput_data2( (targetIntVal_ch2 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );
            //fput_data2( (targetIntVal_ch3 & (0xFF << 8)) >> 8 );
            //fput_data2( (targetIntVal_ch3 & (0xFF << 0)) >> 0 );

            fput_data2( (tempIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data2( (tempIntVal_ch4 & (0xFF << 0)) >> 0 );

            fput_data2( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            fput_data2( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );
            //fput_data2( (targetIntVal_ch4 & (0xFF << 8)) >> 8 );
            //fput_data2( (targetIntVal_ch4 & (0xFF << 0)) >> 0 );

            //fput_data2( (tempIntVal_i2c & (0xFF << 8)) >> 8 );
            //fput_data2( (tempIntVal_i2c & (0xFF << 0)) >> 0 );

            fput_data2( SaveOnOffState );

            fput_data2( 0xa5 );

            DP_SendMessageFlag = 0;

            //UCA3IE |= UCRXIE;
            //UCA1IE |= UCRXIE;                  // prevent to occur overrun (rx)
        }
    }
}


void FRAMWrite(unsigned char *inputData)
{

    unsigned int i = 0;

    for (i = 0; i < WRITE_SIZE; i++)
    {
        FRAM_write[i] = inputData[i];
    }

}


void Tx_PID_Tuning(unsigned char commandTx, unsigned char SelectedChannel)
{
    unsigned char tIndex = 0;
    if(commandTx == 1)
    {
        // save to return one cycle

        fput_data(0xfe);

        fput_data(0x90);

        fput_data( P_CastingBuffer[SelectedChannel][3] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][2] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][1] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][0] );     // P-Gain

        fput_data( I_CastingBuffer[SelectedChannel][3] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][2] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][1] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][0] );     // I-Gain

        fput_data( D_CastingBuffer[SelectedChannel][3] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][2] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][1] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][0] );     // D-Gain

        fput_data(SelectedChannel);     // Channel      0~3 (4-channels)

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x07);        // block command

        fput_data(0xa5);
    }
    if(commandTx == 2)
    {
        // save to return four cycle (all channels)
        for(tIndex = 0; tIndex < 4; tIndex++)
        {
            fput_data(0xfe);

            fput_data(0x90);

            fput_data( P_CastingBuffer[tIndex][3] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][2] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][1] );     // P-Gain
            fput_data( P_CastingBuffer[tIndex][0] );     // P-Gain

            fput_data( I_CastingBuffer[tIndex][3] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][2] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][1] );     // I-Gain
            fput_data( I_CastingBuffer[tIndex][0] );     // I-Gain

            fput_data( D_CastingBuffer[tIndex][3] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][2] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][1] );     // D-Gain
            fput_data( D_CastingBuffer[tIndex][0] );     // D-Gain

            fput_data(tIndex);     // Channel      0~3 (4-channels)

            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);
            fput_data(0x00);

            fput_data(0x07);        // block command

            fput_data(0xa5);
        }
    }
    if(commandTx == 3)
    {
        // save to return one cycle about SelectedChannel

        fput_data(0xfe);

        fput_data(0x90);

        fput_data( P_CastingBuffer[SelectedChannel][3] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][2] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][1] );     // P-Gain
        fput_data( P_CastingBuffer[SelectedChannel][0] );     // P-Gain

        fput_data( I_CastingBuffer[SelectedChannel][3] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][2] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][1] );     // I-Gain
        fput_data( I_CastingBuffer[SelectedChannel][0] );     // I-Gain

        fput_data( D_CastingBuffer[SelectedChannel][3] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][2] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][1] );     // D-Gain
        fput_data( D_CastingBuffer[SelectedChannel][0] );     // D-Gain

        fput_data(SelectedChannel);     // Channel      0~3 (4-channels)

        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);
        fput_data(0x00);

        fput_data(0x07);        // block command

        fput_data(0xa5);
    }

}

void FloatToByte(void)
{

    // from Writing to save
    unsigned char TempBuffer[48];       // Float Type data size 4*12

    unsigned char P_TempBuffer[4][4];
    unsigned char I_TempBuffer[4][4];
    unsigned char D_TempBuffer[4][4];

    unsigned int jIndex = 0;
    unsigned int mIndex = 0;
    float Kp_Temp[4];
    float Ki_Temp[4];
    float Kd_Temp[4];

    for(jIndex = 0; jIndex < 4; jIndex++)
    {
        Kp_Temp[jIndex] = Kp_Value[jIndex];
        Ki_Temp[jIndex] = Ki_Value[jIndex];
        Kd_Temp[jIndex] = Kd_Value[jIndex];
    }


    memcpy(P_TempBuffer[0], &Kp_Temp[0], sizeof(float));        //  float -> unsigned char  P-Gain Channel 1
    memcpy(P_TempBuffer[1], &Kp_Temp[1], sizeof(float));        //  float -> unsigned char  P-Gain Channel 2
    memcpy(P_TempBuffer[2], &Kp_Temp[2], sizeof(float));        //  float -> unsigned char  P-Gain Channel 3
    memcpy(P_TempBuffer[3], &Kp_Temp[3], sizeof(float));        //  float -> unsigned char  P-Gain Channel 4

    memcpy(I_TempBuffer[0], &Ki_Temp[0], sizeof(float));        //  float -> unsigned char  I-Gain Channel 1
    memcpy(I_TempBuffer[1], &Ki_Temp[1], sizeof(float));        //  float -> unsigned char  I-Gain Channel 2
    memcpy(I_TempBuffer[2], &Ki_Temp[2], sizeof(float));        //  float -> unsigned char  I-Gain Channel 3
    memcpy(I_TempBuffer[3], &Ki_Temp[3], sizeof(float));        //  float -> unsigned char  I-Gain Channel 4

    memcpy(D_TempBuffer[0], &Kd_Temp[0], sizeof(float));        //  float -> unsigned char  D-Gain Channel 1
    memcpy(D_TempBuffer[1], &Kd_Temp[1], sizeof(float));        //  float -> unsigned char  D-Gain Channel 2
    memcpy(D_TempBuffer[2], &Kd_Temp[2], sizeof(float));        //  float -> unsigned char  D-Gain Channel 3
    memcpy(D_TempBuffer[3], &Kd_Temp[3], sizeof(float));        //  float -> unsigned char  D-Gain Channel 4
    //unsigned char ���·� �ٲ�



    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(jIndex = 0; jIndex < 4; jIndex++)
        {
            FtoCBuffer[(12*mIndex)+jIndex] = P_TempBuffer[mIndex][jIndex];        // ���� ����
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(jIndex = 0; jIndex < 4; jIndex++)
        {
            FtoCBuffer[(12*mIndex)+jIndex+4] = I_TempBuffer[mIndex][jIndex];      // ���� ����
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(jIndex = 0; jIndex < 4; jIndex++)
        {
            FtoCBuffer[(12*mIndex)+jIndex+8] = D_TempBuffer[mIndex][jIndex];      // ���� ����
        }
    }
}

void ByteToFloat(void)
{
    int fIndex = 0;
    int mIndex = 0;

    // Memory value save to PID value
    //unsigned char P_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
    //unsigned char I_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
    //unsigned char D_MemoryBuffer[4][4];      // P-Gain  4 Channel and 4byte
    float Kp_Temp1;
    float Kp_Temp2;
    float Kp_Temp3;
    float Kp_Temp4;

    float Ki_Temp1;
    float Ki_Temp2;
    float Ki_Temp3;
    float Ki_Temp4;

    float Kd_Temp1;
    float Kd_Temp2;
    float Kd_Temp3;
    float Kd_Temp4;
    //float Kp_Temp[4];
    //float Ki_Temp[4];
    //float Kd_Temp[4];


    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(fIndex = 0; fIndex < 4; fIndex++)
        {
            P_MemoryBuffer[mIndex][fIndex] = CtoFBuffer[(12*mIndex)+fIndex];            // ���� ���
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(fIndex = 0; fIndex < 4; fIndex++)
        {
            I_MemoryBuffer[mIndex][fIndex] = CtoFBuffer[(12*mIndex)+fIndex+4];          // ���� ���
        }
    }
    for(mIndex = 0; mIndex <4; mIndex++)
    {
        for(fIndex = 0; fIndex < 4; fIndex++)
        {
            D_MemoryBuffer[mIndex][fIndex] = CtoFBuffer[(12*mIndex)+fIndex+8];          // ���� ���
        }
    }


    memcpy(&Kp_Temp1, P_MemoryBuffer[0], sizeof(float));        //  unsigned char -> float  P-Gain Channel 1
    memcpy(&Kp_Temp2, P_MemoryBuffer[1], sizeof(float));        //  unsigned char -> float  P-Gain Channel 2
    memcpy(&Kp_Temp3, P_MemoryBuffer[2], sizeof(float));        //  unsigned char -> float  P-Gain Channel 3
    memcpy(&Kp_Temp4, P_MemoryBuffer[3], sizeof(float));        //  unsigned char -> float  P-Gain Channel 3

    memcpy(&Ki_Temp1, I_MemoryBuffer[0], sizeof(float));        //  unsigned char -> float  I-Gain Channel 1
    memcpy(&Ki_Temp2, I_MemoryBuffer[1], sizeof(float));        //  unsigned char -> float  I-Gain Channel 2
    memcpy(&Ki_Temp3, I_MemoryBuffer[2], sizeof(float));        //  unsigned char -> float  I-Gain Channel 3
    memcpy(&Ki_Temp4, I_MemoryBuffer[3], sizeof(float));        //  unsigned char -> float  I-Gain Channel 3

    memcpy(&Kd_Temp1, D_MemoryBuffer[0], sizeof(float));        //  unsigned char -> float  D-Gain Channel 1
    memcpy(&Kd_Temp2, D_MemoryBuffer[1], sizeof(float));        //  unsigned char -> float  D-Gain Channel 2
    memcpy(&Kd_Temp3, D_MemoryBuffer[2], sizeof(float));        //  unsigned char -> float  D-Gain Channel 3
    memcpy(&Kd_Temp4, D_MemoryBuffer[3], sizeof(float));        //  unsigned char -> float  D-Gain Channel 3

    Kp_Value[0] = Kp_Temp1;
    Kp_Value[1] = Kp_Temp2;
    Kp_Value[2] = Kp_Temp3;
    Kp_Value[3] = Kp_Temp4;

    Ki_Value[0] = Ki_Temp1;
    Ki_Value[1] = Ki_Temp2;
    Ki_Value[2] = Ki_Temp3;
    Ki_Value[3] = Ki_Temp4;

    Kd_Value[0] = Kd_Temp1;
    Kd_Value[1] = Kd_Temp2;
    Kd_Value[2] = Kd_Temp3;
    Kd_Value[3] = Kd_Temp4;

    /*
    for(fIndex = 0; fIndex < 4; fIndex++)
    {
        Kp_Value[fIndex] = Kp_Temp[fIndex];
        Ki_Value[fIndex] = Ki_Temp[fIndex];
        Kd_Value[fIndex] = Kd_Temp[fIndex];
    }*/

}

float CalcTemp(int rawTemp)                    // �µ����
{
  float retTemp;
  retTemp = ((float)rawTemp)*0.02;
  retTemp -= 273.15;
  return retTemp;
}

uint8_t GetObject(void)                   // ���µ� �б�
{
  int16_t rawObj;
  if(PEC_ture == 1)
  {
    if (rawObj & 0x8000)                         // �ֻ��� ��Ʈ�� 1 �̸� ����
    {
      return 0;
    }
    result_object = _rawObject;
    return 1;
  }
}

uint8_t CalPEC(uint8_t *crc, uint8_t nBytes)  // PEC ����
{
  uint8_t data, count;
  uint16_t remainder = 0;

  for(count=0; count<nBytes; ++count)
  {
     data = *(crc++) ^ remainder;
     remainder = crc8_table[data] ^ (remainder >> 8);
  }
  return (uint8_t)remainder;
}


#pragma vector = USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
{
    switch(__even_in_range(UCB2IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_I2C_UCNACKIFG:                            // NACK Interrupt
            UCB2CTLW0 |= UCTXSTT;                            // I2C start condition
        break;

        case USCI_I2C_UCRXIFG0:                             // I2C RX Interrupt
            if (RX_Byte_Ctr > 1){                               // Checks if there is more data to be received
                *pointer = UCB2RXBUF;                       // Loads the data array
                pointer++;                                  // Increments the pointer
                RX_Byte_Ctr--;                              // Decrement RX byte counter
            } else if(RX_Byte_Ctr == 1){                                        // If all of the data is received
                *pointer = UCB2RXBUF;
                pointer = 0;
                //__bic_SR_register_on_exit(LPM0_bits);       // Exit LPM0
            }
        break;

        case USCI_I2C_UCTXIFG0:                             // I2C TX Interrupt
            if(TX_Byte_Ctr > 1){                                // If there is more data to send
                UCB2TXBUF = *txPtr;                     // Loads TX buffer from array
                i++;                                        // Increments array position
                TX_Byte_Ctr--;                              // Decrements TX byte counter
            } else if(TX_Byte_Ctr == 1){                                        // If there is nothing left to say
                UCB2TXBUF = 0x07;
                //__bic_SR_register_on_exit(LPM0_bits);       // Exits Low-Power mode
            }
        break;
    }
}
