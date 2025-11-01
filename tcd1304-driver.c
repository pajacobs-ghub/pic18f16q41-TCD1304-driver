// tcd1304-driver.c
// Drive the TCD1304DG clock signals with a PIC18F16Q41-I/P.
// Accept period values from the I2C bus.
//
// PJ 2024-11-25 Basic clocking working with fixed periods.
//    2024-12-10 Adjustable periods and op-amp as a buffer for VOS.
//               Bring over the simple I2C client functions from
//               the simple I2C client example for the PIC18F16Q41.
//    2025-11-02 Don't take message until I2C module is done with it.

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

// Our chosen slave address.
// Note that this number appears in a couple of the messages below.
#define I2C_ADDR7  0x51
// The largest Newhaven serial LCD command seems to be 11 bytes.
#define BUFFER_LEN 16
static volatile unsigned char receive_buf[BUFFER_LEN];
static unsigned char send_buf[BUFFER_LEN];
static volatile unsigned char bytes_received = 0;
static volatile unsigned char i2c_is_active = 0;
static volatile unsigned char send_indx = 0;

void i2c_init()
{
    // Configure PPS I2C1_SCL=RB6, I2C1_SDA=RB4
    // Open-drain outputs, with pull-up and fast-mode slew-rate 
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB6 = 0;
    ODCONBbits.ODCB4 = 1;
    ODCONBbits.ODCB6 = 1;
    WPUBbits.WPUB4 = 1;
    WPUBbits.WPUB6 = 1;
    RB4I2Cbits.I2CPU = 0b01; // 2X pull-up
    RB6I2Cbits.I2CPU = 0b01;
    RB4I2Cbits.SLEW = 0b01; // fast-mode slew rate enabled
    RB6I2Cbits.SLEW = 0b01;
    ANSELBbits.ANSELB4 = 0;
    ANSELBbits.ANSELB6 = 0;
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    I2C1SCLPPS = 0b001110; // RB6
    RB6PPS = 0x21; // I2C1 SCL
    I2C1SDAPPS = 0b001100; // RB4
    RB4PPS = 0x22; // I2C1 SDA
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    // Slave mode with 7-bit address.
    I2C1CON0 = 0;
    // Clock stretching is enabled.
    // I2C1CON1 = 0;
    // Fast-mode enable, allow address buffer, 30ns hold, 8 clk pulses for bus free
    I2C1CON2 = 0x28;
    //
    I2C1ADR0 = I2C_ADDR7 << 1;  // LSB is don't care
    //
    // Interrupts
    I2C1PIR = 0; // Clear flags in module
    I2C1PIEbits.ACKTIE = 1; // Acknowledge-time (9th clock pulse)
    I2C1PIEbits.PCIE = 1; // Stop condition
    PIR7bits.I2C1IF = 0;
    PIE7bits.I2C1IE = 1;
    INTCON0bits.IPEN = 0; // No priority for interrupts; all high priority
    INTCON0bits.GIEH = 1; // Enable high-priority interrupts
    //
    I2C1CON0bits.EN = 1; // Enable module
    return;
}

void i2c_close()
{
    di();
    I2C1CON0bits.EN = 0;
    return;
}

void __interrupt() i2c_service()
// Implement the state-table approach shown in Microchip App Note 734
// and the slave mode example (6) from TB3159.
{
    unsigned char junk, i;
    if (PIR7bits.I2C1IF) {
        PIR7bits.I2C1IF = 0;
        if (I2C1PIRbits.ACKTIF) {
            I2C1PIRbits.ACKTIF = 0;
            // Slave mode is active following a start condition.
            // If a stop condition has been seen, slave mode will not be active.
            if (!I2C1STAT0bits.SMA) {
                i2c_is_active = 0;
                I2C1CON0bits.CSTR = 0; // Release clock.
                return;
            } else {
                i2c_is_active = 1;
            }
            // At this point slave mode is active, so we should do something.
            if (!I2C1STAT0bits.R && !I2C1STAT0bits.D) {
                // State 1: i2c write operation, incoming byte is an address
                // and that matching address would have been loaded into I2C1ADB0.
                // Prepare to collect incoming data bytes.
                for ( i = 0; i < BUFFER_LEN; ++i ) receive_buf[i] = 0;
                bytes_received = 0;
                I2C1CON0bits.CSTR = 0; // Release clock.
                return;
            }
            if (!I2C1STAT0bits.R && I2C1STAT0bits.D) {
                // State 2: i2c write operation, incoming byte is data.
                while (!I2C1STAT1bits.RXBF) { /* wait */ } 
                if (bytes_received < BUFFER_LEN) {
                    receive_buf[bytes_received] = I2C1RXB;
                    bytes_received++;
                } else {
                    // Buffer is full, discard incoming byte.
                    junk = I2C1RXB;
                }
                I2C1CON0bits.CSTR = 0; // Release clock.
                return;
            }
            if (I2C1STAT0bits.R && !I2C1STAT0bits.D) {
                // State 3: i2c read operation, incoming byte is address
                // and it will have been loaded into I2C1ADB0 already.
                send_indx = 0;
                bytes_received = 0; // Discard the last received message, too.
                I2C1STAT1bits.TXWE = 0;
                while (!I2C1STAT1bits.TXBE) { /* wait */ } 
                I2C1TXB = send_buf[send_indx];
                I2C1CON0bits.CSTR = 0; // Release clock.
                send_indx++;
                if (send_indx == BUFFER_LEN) send_indx = 0; // wrap around
                return;
            }
            if (I2C1STAT0bits.R && I2C1STAT0bits.D) {
                // State 4: i2c read operation, last byte was data
                I2C1STAT1bits.TXWE = 0;
                if (I2C1CON1bits.ACKSTAT == 0) {
                    // Acknowledge received for previous byte; master wants more.
                    while (!I2C1STAT1bits.TXBE) { /* wait */ } 
                    I2C1TXB = send_buf[send_indx];
                    send_indx++;
                    if (send_indx == BUFFER_LEN) send_indx = 0; // wrap around
                } else {
                    // Master sent NACK; no need to send any more data.
                }
                I2C1CON0bits.CSTR = 0; // Release clock.
                return;
            }
            if (I2C1STAT0bits.D && I2C1CON1bits.ACKSTAT) {
                // State 5: Master NACK after data byte.
                // Nothing special to do.
                I2C1CON0bits.CSTR = 0; // Release clock.
                return;
            }
            // If we get this point, something has gone wrong.
            while ( 1 ) ; // Block until watch dog barks.
        }
        if (I2C1PIRbits.PCIF) {
            I2C1PIRbits.PCIF = 0;
            // If a stop condition has been seen, slave mode will not be active.
            if (!I2C1STAT0bits.SMA) {
                i2c_is_active = 0;
                return;
            }
        }
    }
} // end i2c_service()

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

int main()
{
    unsigned char cmd[BUFFER_LEN];
    unsigned char new_cmd = 0;
    uint8_t nbytes;
    uint16_t period_SH = 200; // Sets t_INT in microseconds
    uint16_t period_ICG = 10000; // Sets read period in microseconds.
    //
    init_opa1();
    init_pins_for_pwm();
    // Start the TCD1304 clocking signals with default values
    // for the SH and IGC periods.
    init_pwm123(period_SH, period_ICG);
    //
    // Start the I2C interrupt service and just sit back waiting for commands.
    i2c_init();
    ei(); // The i2c_service routine is called via interrupt.
    // Put something in the send buffer so that the I2C master can ask for it.
    for (unsigned char i = 0; i < BUFFER_LEN; ++i) send_buf[i] = i;
    // Main loop.
    while (1) {
        // Disable interrupts while we access the I2C buffer.
        di();
        // We will make a copy of the I2C receive buffer to allow
        // the command interpreter to work on it safely.
        if (bytes_received && !i2c_is_active) {
            for (unsigned char i = 0; i < bytes_received; ++i) cmd[i] = receive_buf[i];
            nbytes = bytes_received;
            bytes_received = 0; // Indicate that we have taken the bytes.
            new_cmd = 1; // and flag that we have a command to work on.
        } else {
            new_cmd = 0;
        }
        ei();
        if (!new_cmd) {
            // There is no new command, so just waste a bit of time.
            __delay_ms(1);
            CLRWDT();
            continue;
        }
        // Interpret the command bytes.
        if (nbytes == 4)
        {
            // Bytes arrive over the wire in big-endian order.
            period_SH = (uint16_t) cmd[0]<<8 | cmd[1];
            period_ICG = (uint16_t) cmd[2]<<8 | cmd[3];
            init_pwm123(period_SH, period_ICG);
        }
        //
        // Now we have interpreted the command bytes, clean up.
        for (unsigned char i = 0; i < BUFFER_LEN; ++i) cmd[i] = 0;
        new_cmd = 0;
        CLRWDT();
    } // end while 1
    //
    // Don't expect to arrive here...
    i2c_close();
    return 0;
}
