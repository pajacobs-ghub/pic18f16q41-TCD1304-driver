// tcd1304-driver.c
// Drive the TCD1304DG clock signals with a PIC18F16Q41-I/P.
// PJ 2024-11-25

// PIC18F16Q41 Configuration Bit Settings (generated in Memory View)
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
#pragma config BBSIZE = BBSIZE_512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config DEBUG = OFF

// CONFIG8
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG9
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#define ICGpin LATCbits.LATC4
#define CLMpin LATCbits.LATC5
#define SHpin LATCbits.LATC6

void init_pins()
{
    ICGpin = 1; ANSELCbits.ANSELC4 = 0; TRISCbits.TRISC4 = 0;
    CLMpin = 0; ANSELCbits.ANSELC5 = 0; TRISCbits.TRISC5 = 0;
    SHpin = 0; ANSELCbits.ANSELC6 = 0; TRISCbits.TRISC6 = 0;
}

void init_pwm123()
{
    // Use PWM1 module for the master clock signal, CLM.
    // We want a 2MHz 50% duty signal, once started.
    PWM1CONbits.EN = 0;
    PWM1CLKbits.CLK = 0b0010;  // FOSC (period of incoming clock 1/64 us)
    PWM1CPRE = 8-1; // period for PWM clock = 1/8 us
    PWM1PR = 4-1; // period for CLM is 0.5 us, count ticks 0 1 2 3
    PWM1S1CFGbits.MODE = 0; // left aligned, so we start active
    PWM1S1CFGbits.POL1 = 1; // invert polarity to start low
    PWM1S1P1 = 2; // one PWM clock period high for 50% duty
    //
    // Use PWM2 for SH, presently we want a fixed t_INT
    PWM2CONbits.EN = 0;
    PWM2CLKbits.CLK = 0b0010;  // FOSC (period of incoming clock 1/64 us)
    PWM2CPRE = 16-1; // period for PWM clock = 1/4 us
    PWM2PR = 400*4-1; // period for SH
    PWM2S1CFGbits.MODE = 0; // left aligned, so we start active
    PWM2S1P1 = 8; // 2 us pulse
    //
    // Use PWM3 for ICG, presently gives overall cycle of 8ms.
    // This sets the read cycle time.
    PWM3CONbits.EN = 0;
    PWM3CLKbits.CLK = 0b0010;  // FOSC (period of incoming clock 1/64 us)
    PWM3CPRE = 16-1; // period for PWM clock = 1/4 us
    PWM3PR = 8000*4-1; // period for ICG
    PWM3S1CFGbits.MODE = 0; // left aligned, so we start active
    PWM3S1CFGbits.POL1 = 1; // invert polarity to start low
    PWM3S1P1 = 28; // 7 us low pulse
    //
    // Connect the output of CLCs to the relevant output pins.
    GIE = 0; // We run without interrupt.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RC5PPS = 0x0a; // PWM1S1P1_OUT
    RC6PPS = 0x0c; // PWM2S1P1_OUT
    RC4PPS = 0x0e; // PWM3S1P1_OUT
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    // Start the modules in a sequence.
    PWM3CONbits.EN = 1; // ICG
    // We start the master clock slightly later so that we are reasonably sure
    // that it is high when the ICG goes high to start reading pixel data.
    PWM1CONbits.EN = 1; // Master clock
    //
    // Start SH about 0.5us later
    NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP(); NOP();
    PWM2CONbits.EN = 1;
}

int main() {
    init_pins();
    init_pwm123();
    while (1) {
        __delay_ms(1);
        CLRWDT();
    }
}
