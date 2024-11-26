// demo-1-flash-led.c
// Bring up the TCD1304DG clock prototype with a PIC18F16Q41-I/P.
// It's output is a fast oscillating pin pretending to be something
// like the master clock needed to drive the CCD.
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

#define FOSC 64000000L
#define _XTAL_FREQ FOSC

#include <stdint.h>
#include <stdlib.h>

#define LED LATBbits.LATB4

int main() {
    TRISBbits.TRISB4 = 0;
    LED = 0;
    while (1) {
        // Neat, I did not realize that we can do fractional microsecond delays.
        // Remember to RTFM.
        // Together with LED change and watch-dog pat, 0.1us gets us about 310ns
        // measured time for a single pass of this loop.
        // __delay_us(0.1);
        // NOP();  // Each NOP measures 62ns on the scope, as it should.
        // The bare minimum of toggling the LED and kicking the watch dog
        // gets us about 248ns (4 instruction cycles).
        LED ^= 1;
        CLRWDT();
    }
}
