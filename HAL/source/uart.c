/*
 * uart.c
 *
 *  Created on: 2022. 4. 15.
 *      Author: hc.ro
 */


#ifndef UART_C_
#define UART_C_

#include "HAL/include/uart.h"

unsigned int BufferCount;
unsigned int BufferCountFromSlave;
int ShiftCount;
unsigned char ucRxBuffer[20];
unsigned char ucRxBufferFromSlave[20];
unsigned char OverrunChecking;
unsigned int ByteStatus;
//volatile unsigned char errData[11];
//volatile unsigned char Index = 0;

unsigned char orData[11];
unsigned char orCount;

unsigned char OverRun_Flag;



void Init_UART(void)
{
    // Connect PC (UCA2 TYPE)
    //P6SEL1 &= ~(BIT0 | BIT1);
    //P6SEL0 |= (BIT0 | BIT1);                // USCI_A3 UART operation
    P5SEL1 &= ~(BIT4 | BIT5);
    P5SEL0 |= (BIT4 | BIT5);                // USCI_A2 UART operation


    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure USCI_A3 for UART mode
    UCA2CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA2CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    //UCA3BRW = 138;                            // 16000000/115200 = 138.88
    //UCA3MCTLW = 0xF700;                     // 16000000/115200 - INT(1000000/115200)=0.8888
    //UCA3BRW = 125;                            // 16000000/128000 = 125
    //UCA3MCTLW = 0;                          // 16000000/128000 - INT(1000000/115200)=0
                                             // UCBRSx value = 0xF7 (See UG)

    //UCA3BRW = 13;                           // 4MHz -> 19200
    //UCA3MCTLW = 0x8401;
    UCA2CTLW1 = 3;                          // deglitch time test 200ns
    //UCA3BRW = 17;
    //UCA3MCTLW = 0xDD51;                     // 16MHz -> 57600

    UCA2BRW = 4;
    UCA2MCTLW = 0x5551;                     // 4MHz -> 57600

    UCA2CTLW0 &= ~UCSWRST;                  // release from reset
    UCA2IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
}


void Init_UART2(void)
{
    // Connect Displayer (UCA1 TYPE)
    P2SEL0 &= ~(BIT5 | BIT6);
    P2SEL1 |= (BIT5 | BIT6);                // USCI_A1 UART operation

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure USCI_A3 for UART mode
    UCA1CTLW0 = UCSWRST;                    // Put eUSCI in reset
    UCA1CTLW0 |= UCSSEL__SMCLK;             // CLK = SMCLK
    //UCA3BRW = 138;                            // 16000000/115200 = 138.88
    //UCA3MCTLW = 0xF700;                     // 16000000/115200 - INT(1000000/115200)=0.8888
    //UCA3BRW = 125;                            // 16000000/128000 = 125
    //UCA3MCTLW = 0;                          // 16000000/128000 - INT(1000000/115200)=0
                                            // UCBRSx value = 0xF7 (See UG)

    //UCA3BRW = 13;                           // 4MHz -> 19200
    //UCA3MCTLW = 0x8401;
    UCA1CTLW1 = 3;                          // deglitch time test 200ns
    //UCA3BRW = 17;
    //UCA3MCTLW = 0xDD51;                     // 16MHz -> 57600

    UCA1BRW = 4;
    UCA1MCTLW = 0x5551;                     // 4MHz -> 57600

    UCA1CTLW0 &= ~UCSWRST;                  // release from reset
    UCA1IE |= UCRXIE;                       // Enable USCI_A3 RX interrupt
}


void UART_TX_DATA(void)
{
    int i = 0;

    Data temp;

    if(rxPcDataQueue.state == 1)
    {
        UCA3IE &= ~UCRXIE;
        for(i = 0; i<11; i++)
        {
            temp = Dequeue(&rxPcDataQueue);
            fput_data( temp.mode );
        }
        UCA3IE |= UCRXIE;
    }
}

void fput_data(unsigned char _c)
{
    // temperature controller -> PC

    while (!(UCTXIFG & UCA2IFG));
    UCA2TXBUF = _c;
}

void fput_data2(unsigned char _c)
{
    // temperature controller -> Displayer
    while (!(UCTXIFG & UCA1IFG));
    UCA1TXBUF = _c;
}


// transmit as 'PRINTF' function
int fputc(int _c, register FILE *_fp)
{
    while (!(UCTXIFG & UCA3IFG));
    UCA3TXBUF = _c;
    return(_c);
}
int fputs(const char *_ptr, register FILE *_fp)
{
    int i, len;
    len = strlen(_ptr);
    for(i=0 ; i<len ; i++)
    {
        while (!(UCTXIFG & UCA3IFG));
        UCA3TXBUF = _ptr[i];
    }
    return len;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A2_VECTOR
__interrupt void USCI_A2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A2_VECTOR))) USCI_A2_ISR (void)
#else
#error Compiler not supported!
#endif
{

    switch(__even_in_range(UCA2IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
                //ByteStatus = UCA3STATW;



                //TA0CCTL0 = CCIE;                          // TACCR0 interrupt enabled
                //TA0CCTL0 = CCIE_0;                          // TACCR0 interrupt disabled

                UCB2IE   &= ~(UCTXIE0    // Data received interrupt enable
                          |  UCRXIE0    // Data ready to transmit interrupt enable
                          |  UCNACKIE);  // NACK interrupt enable

                RxData = UCA2RXBUF;

                ucRxBuffer[BufferCount++] = RxData;

                if(RxData == 0xa5 && BufferCount >= 12)
                {
                    int i=0;
                    Data temp;

                    if(ucRxBuffer[BufferCount-12] == 0xfd && ucRxBuffer[BufferCount-1] == 0xa5)
                    {
                        temp.mode = ucRxBuffer[BufferCount-11];
                        for(i=0;i<9;i++)
                        {
                            temp.data[i] = ucRxBuffer[i+(BufferCount-10)];
                        }
                        Enqueue(&rxPcDataQueue, temp);
                    }
                    BufferCount = 0;
                }

                //_commandZone(ucRxBuffer);


                //TA0CCTL0 = CCIE;                          // TACCR0 interrupt disabled

                UCB2IE   |= (UCTXIE0    // Data received interrupt enable
                          |  UCRXIE0    // Data ready to transmit interrupt enable
                          |  UCNACKIE);  // NACK interrupt enable

            __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0 on reti

            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=EUSCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(EUSCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{

    switch(__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
                //ByteStatus = UCA3STATW;

                UCB2IE   &= ~(UCTXIE0    // Data received interrupt enable
                      |  UCRXIE0    // Data ready to transmit interrupt enable
                      |  UCNACKIE);  // NACK interrupt enable

                RxDataFromSlave = UCA1RXBUF;

                ucRxBufferFromSlave[BufferCountFromSlave++] = RxDataFromSlave;


                if(RxDataFromSlave == 0xa5 && BufferCountFromSlave >= 14)
                {
                    int i=0;
                    Data temp;

                    if(ucRxBufferFromSlave[BufferCountFromSlave-14] == 0xfd && ucRxBufferFromSlave[BufferCountFromSlave-1] == 0xa5)
                    {
                        temp.mode = ucRxBufferFromSlave[BufferCountFromSlave-13];
                        for(i=0;i<11;i++)
                        {
                            temp.data[i] = ucRxBufferFromSlave[i+(BufferCountFromSlave-12)];
                        }
                        Enqueue(&rxSlaveDataQueue, temp);
                    }
                    BufferCountFromSlave = 0;
                }

                UCB2IE   |= (UCTXIE0    // Data received interrupt enable
                          |  UCRXIE0    // Data ready to transmit interrupt enable
                          |  UCNACKIE);  // NACK interrupt enable


            __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0 on reti

            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}


#endif /* UART_C_ */
