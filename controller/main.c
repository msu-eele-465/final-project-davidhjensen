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
void setupI2C();                                // Setup I2C with SCL on __ and SDA on <TODO: pins>
void rxCommand();                               // Receive a command from the master

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
    // TODO: Setup I2C with SCL on __ and SDA on <TODO: pins>
}
void rxCommand() {
    // Receive a command from the master
}
    // TODO: I2C ISRs as needed here

//-- STATE MACHINE

//----------------------------------------------END FUNCTIONS AND ISRS----------------------------------------------