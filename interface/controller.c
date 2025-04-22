#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- ENCODER
void setupEncoder();                            // Setup encoder inputs on <TODO: pins> as an interrupt on change
int p1_prev = 0;                                // Previous state of encoder channel 1
int p2_prev = 0;                                // Previous state of encoder channel 2
int encoder_cnt = 0;                            // Encoder count

//-- MOTOR DRIVER
void setupDriver();                             // Setup motor driver with enable pins on <TODO: pins> and PWM on <TODO: pin>

//-- PWM
void setupPWM();                                // Setup hardware PWM on pin <TODO: pin>
int pwm_duty = 0;                               // Global to hold current PWM duty

//-- CONTROL LOOP
void setupControl();                            // Setup timer to drive control calculations

//-- I2C
void setupI2C();                                // Setup I2C with SCL on P1.3 and SDA on P1.2
volatile unsigned int temp_angle = 0;           // Angle received over I2C (one byte at a time)
volatile unsigned int angle = 0;                // Angle received over I2C
volatile unsigned int byte_n = 0;               // Keep track of byte number received

//-- STATE MACHINE

//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    //------------------------------------------------INITIALIZATIONS------------------------------------------------
    //-- ENCODER
    setupEncoder();

    //-- MOTOR DRIVER
    setupDriver();

    //-- PWM
    setupPWM();

    //-- CONTROL LOOP
    setupControl();

    //-- I2C
    setupI2C();

    //-- STATE MACHINE

    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    //------------------------------------------------STATE MACHINE------------------------------------------------
    while(1)
    {

    }
    //----------------------------------------------END STATE MACHINE----------------------------------------------

}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------
//-- ENCODER
void setupEncoder() {
    // TODO: setup encoder on pins and add hardware interrupts for a change
}

    // TODO: add ISR for change in encoder pins here

//-- MOTOR DRIVER
void setupDriver() {
    // TODO: setup motor driver
}

//-- PWM
void setupPWM(){
    // TODO: setup hardware PWM on a different clock than the control loop
}

    // TODO: add ISR for the PWM with a way to update duty based on pwm_duty

//-- CONTROL LOOP
void setupControl() {
    // TODO: setup timer to drive control updates
}
    // TODO: add ISR for timer where control output is calculated and states are updated.

//-- I2C
void setupI2C() {
    UCB0CTLW0 |= UCSWRST;       // Software reset

    // Configure pins P1.2 (SDA) and P1.3 (SCL)
    P1SEL1 &= ~(BIT2 | BIT3);   // Clear bits in SEL1
    P1SEL0 |= (BIT2 | BIT3);    // Set bits in SEL0

    UCB0CTLW0 = UCSWRST | UCMODE_3 | UCSYNC;   // I2C mode, sync, hold in reset
    UCB0CTLW0 &= ~UCMST;        // Slave mode
    UCB0I2COA0 = 0x68 | UCOAEN; // Own address + enable
    UCB0CTLW1 = 0;              // No auto STOP
    UCB0CTLW0 &= ~UCTR;         // Receiver mode

    UCB0CTLW0 &= ~UCSWRST;      // Exit reset

    UCB0IE |= UCRXIE0;          // Enable receive interrupt
}

// I2C Interrupt Service Routine
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void) {

    if (byte_n==0) {
        temp_angle = UCB0RXBUF << 8;
        byte_n++;
    } else {
        temp_angle += UCB0RXBUF;
        angle = temp_angle;
        byte_n = 0;
    }
}


//-- STATE MACHINE

//----------------------------------------------END FUNCTIONS AND ISRS----------------------------------------------