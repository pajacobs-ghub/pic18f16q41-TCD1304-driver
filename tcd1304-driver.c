// tcd1304-driver.c
// Drive the TCD1304DG clock signals with a PIC18F16Q41-I/P.
// PJ 2024-11-25 Basic clocking working with fixed periods.
//    2024-12-10 Adjustable periods and opamp as a buffer for VOS.

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

void init_pins_for_pwm()
{
    ICGpin = 1; ANSELCbits.ANSELC4 = 0; TRISCbits.TRISC4 = 0;
    CLMpin = 0; ANSELCbits.ANSELC5 = 0; TRISCbits.TRISC5 = 0;
    SHpin = 0; ANSELCbits.ANSELC6 = 0; TRISCbits.TRISC6 = 0;
    //
    // Connect the output of PWMs to the relevant output pins.
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
}

void init_pwm123(uint16_t period_SH_us, uint16_t period_ICG_us)
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
    PWM2CPRE = 64-1; // period for PWM clock = 1 us
    PWM2PR = period_SH_us-1;
    PWM2S1CFGbits.MODE = 0; // left aligned, so we start active
    PWM2S1P1 = 2; // 2 us pulse
    //
    // Use PWM3 for ICG.
    // This sets the read cycle time and should be an exact multiple of t_INT.
    PWM3CONbits.EN = 0;
    PWM3CLKbits.CLK = 0b0010;  // FOSC (period of incoming clock 1/64 us)
    PWM3CPRE = 64-1; // period for PWM clock = 1 us
    PWM3PR = period_ICG_us-1;
    PWM3S1CFGbits.MODE = 0; // left aligned, so we start active
    PWM3S1CFGbits.POL1 = 1; // invert polarity to start low
    PWM3S1P1 = 7; // 7 us low pulse
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

void init_opa1()
{
    // Set up for unity gain with
    //   Input on pin 12, OPA1IN0+, RB5/ANB5
    //   Output on pin 14, OPA1OUT, RC2/ANC2
    TRISBbits.TRISB5 = 1; ANSELBbits.ANSELB5 = 1;
    TRISCbits.TRISC2 = 1; ANSELCbits.ANSELC2 = 1;
    OPA1CON2bits.PCH = 0b010; // non-inverting input is OPA1IN+ (PSS)
    OPA1CON3bits.PSS = 0; // pin select OPA1IN0+ (RB5)
    OPA1HWCbits.OREN = 0; // no hardware override
    OPA1CON0bits.UG = 1; // unity gain
    OPA1CON0bits.CPON = 1; // charge-pump on
    OPA1CON0bits.EN = 1; // turn module on
}

int main() {
    init_opa1();
    init_pins_for_pwm();
    init_pwm123(200, 10000);
    while (1) {
        __delay_ms(1);
        CLRWDT();
    }
}
