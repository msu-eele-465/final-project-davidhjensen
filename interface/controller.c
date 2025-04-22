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
typedef struct {                                // 2-state system with 1 input
    float A[2][2];  // 2x2 matrix
    float B[2];     // 2x1 input matrix
    float x[2];     // current state
} DiscreteSystem2x1;
typedef struct {                                // 1-state low-pass filter
    float a;        // scalar
    float b;        // scalar
    float x;        // current state
} DiscreteLPF;
float theta1 = 0.0f;            // Adaptive gain 1 (updated by MRAS)
float theta2 = 0.0f;            // Adaptive gain 2 (updated by MRAS)
float gamma = 2.0f;             // Adaptation gain
float Ts = .01;                 // Sampling time
float y_measured = 0.0f;        // Angle from encoder
float y_prev = 0.0f;            // Previous angle from encoder
float v = 0.0f;                 // Velocity from backwards difference
float v_filtered = 0.0f;        // Filtered velocity
void setupControl();                            // Setup timer to drive control calculations
void updateSystems(int);                        // Update all models
DiscreteSystem2x1 model = {                     // Model
    .A = {
        {0.9988f, 0.00975f},
        {-0.2438f, 0.95f}
    },
    .B = {0.001229f, 0.2438f},
    .x = {0.0f, 0.0f}
};
DiscreteSystem2x1 plant = {                     // Plant
    .A = {
        {1.0f, 0.009939f},
        {0.0f, 0.9877f}
    },
    .B = {0.0002827f, 0.05643f},
    .x = {0.0f, 0.0f}
};
DiscreteSystem2x1 spt1 = {                      // Theta1 sensitivity partial
    .A = {
        {0.9988f, 0.00975f},
        {-0.2438f, 0.95f}
    },
    .B = {0.001229f, 0.2438f},
    .x = {0.0f, 0.0f}
};
DiscreteSystem2x1 spt2 = {                      // Theta2 sensitivity partial
    .A = {
        {0.9988f, 0.00975f},
        {-0.2438f, 0.95f}
    },
    .B = {-0.001229f, -0.2438f},
    .x = {0.0f, 0.0f}
};
DiscreteLPF vlpf = {                            // Low-pass velocity filter
    .a = 0.9048f,
    .b = 0.09516f,
    .x = 0.0f
};

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

void updateSystems(int uc) {
    // Read actual plant output from encoder (user-defined function)
    // TODO: convert current angle from encoder
    v = (y_measured - y_prev) / Ts;         // Derivative
    y_prev = y_measured;

    // Update velocity low-pass filter
    vlpf.x = vlpf.a * vlpf.x + vlpf.b * v;
    v_filtered = vlpf.x;

    // Error signal: e = y_model - y_plant
    float error = model.x[0] - y_measured;

    // Control signal: u = theta1 * (uc - y) - theta2 * v_filtered
    float u = theta1 * ((float) uc - y_measured) - theta2 * v_filtered;

    // --- Update Reference Model ---
    float x1m = model.x[0];
    float x2m = model.x[1];
    model.x[0] = model.A[0][0]*x1m + model.A[0][1]*x2m + model.B[0]*uc;
    model.x[1] = model.A[1][0]*x1m + model.A[1][1]*x2m + model.B[1]*uc;

    // --- Update Sensitivity Model 1 ---
    float x1s1 = spt1.x[0];
    float x2s1 = spt1.x[1];
    float spt1_u = uc - y_measured;
    spt1.x[0] = spt1.A[0][0]*x1s1 + spt1.A[0][1]*x2s1 + spt1.B[0]*spt1_u;
    spt1.x[1] = spt1.A[1][0]*x1s1 + spt1.A[1][1]*x2s1 + spt1.B[1]*spt1_u;

    // --- Update Sensitivity Model 2 ---
    float x1s2 = spt2.x[0];
    float x2s2 = spt2.x[1];
    spt2.x[0] = spt2.A[0][0]*x1s2 + spt2.A[0][1]*x2s2 + spt2.B[0]*v_filtered;
    spt2.x[1] = spt2.A[1][0]*x1s2 + spt2.A[1][1]*x2s2 + spt2.B[1]*v_filtered;

    // --- Update Adaptive Parameters (MIT Rule) ---
    theta1 -= gamma * spt1.x[0] * error;
    theta2 -= gamma * spt2.x[0] * error;

    // Apply control signal to motor (user-defined function)
    // TODO: PWM duty to motors
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