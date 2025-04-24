#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- ENCODER
void setupEncoder();                            // Setup encoder inputs on P3.2 (B) and P3.3 (A)
float getAngle();                               // Convert current encoder count to angle in degrees
volatile int prev = 0;                          // Previous state of encoder
volatile long encoder_cnt = 0;                   // Encoder count

//-- MOTOR DRIVER
void setupDriver();                             // Setup motor driver with ENA1 P3.1 and ENA2 on P3.5

//-- PWM
void setupPWM();                                // Setup hardware PWM on P1.6
int pwm_duty = 0;                               // Global to hold current PWM duty

//-- CONTROL TIMER
void setupSampleClock();                        // Setup clock on TB3 to sample ADC every 0.5s
void enableSampleClock();                       // Enable sampling clock interrupts
void disableSampleClock();                      // Disable sampling clock interrupts

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
float gamma = 0.003f;             // Adaptation gain
float Ts = .01;                 // Sampling time
float y_measured = 0.0f;        // Angle from encoder
float y_prev = 0.0f;            // Previous angle from encoder
float v = 0.0f;                 // Velocity from backwards difference
float v_filtered = 0.0f;        // Filtered velocity
void setupControl();                            // Setup timer to drive control calculations
void updateSystems(float);                      // Update all models
float this_is_uc =  0;                          // Cleans up var when passed to function
float hardcoded_angle[] = {0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15};
int hardcode_i = 0;
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
volatile int temp_angle = 0;                    // Angle received over I2C (one byte at a time)
volatile int angle = 180;                       // Angle received over I2C
volatile unsigned int byte_n = 0;               // Keep track of byte number received

//-- UART
void setupUART();                               // Setup UART with baud rate of 115200 on USB output
void addFloat(unsigned int, float);             // Write a float to messsage starting at index i (7 chars: XXXX.XX)    
volatile unsigned int i_uart = 0;               // Index of byte in message to send
char message[] = ".......,.......,.......,.......,.......,.......,.......,.......\n\r";      // Message to send over UART

//-- STATE MACHINE

//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    //------------------------------------------------INITIALIZATIONS------------------------------------------------
    //-- ENCODER
    setupEncoder();

    //-- MOTOR DRIVER
    setupDriver();

    //-- PWM
    setupPWM();

    //-- CONTROL TIMER
    setupSampleClock();

    //-- CONTROL LOOP
    setupControl();

    //-- I2C
    setupI2C();

    //-- UART
    setupUART();
    //message[64] = 0x0D;                             // End in carraige return
    addFloat(0, 123.45f);
    addFloat(8, -678.90f);

    //-- STATE MACHINE

    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    //------------------------------------------------STATE MACHINE------------------------------------------------
    // TODO: for testing, put this as a mode that can be toggled
    enableSampleClock();
    while(1)
    {

    }
    //----------------------------------------------END STATE MACHINE----------------------------------------------

}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------
//-- ENCODER
void setupEncoder() {
    // TODO: setup encoder on pins and add hardware interrupts for a change
    P3DIR &= ~(BIT2 | BIT3);        // Inputs on P3.2 and P3.3
    P3SEL1 &= ~(BIT2 | BIT3);       // Clear SEL1 to allow for interrupts on P3.2 and P3.3
    P3SEL0 &= ~(BIT2 | BIT3);       // Clear SEL0 to allow for interrupts on P3.2 and P3.3
    P3IES &= ~(BIT2 | BIT3);        // Set sensitivity to RISING edge initally
    P3IFG &= ~(BIT2 | BIT3);        // clear flags
    P3IE |= (BIT2 | BIT3);          // Enable interrupts   
}

#pragma vector = PORT3_VECTOR
__interrupt void ISR_P3_ENCODER(void)
{
    switch (__even_in_range(P3IV, P3IV_P3IFG7)) {
        case P3IV_P3IFG2:
        case P3IV_P3IFG3:
            {
            int MSB = (P3IN & BIT2) >> 2;               // Read P3.2
            int LSB = (P3IN & BIT3) >> 3;               // Read P3.3

            int encoded = (MSB << 1) | LSB;             // Convert to number
            int sum = (prev << 2) | encoded;    // Add to previous encoder value 

            // Update encoder count
            if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder_cnt ++; 
            if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder_cnt --; 
            prev = encoded;                     //store this value for next time 

            // Update sensitivities
            if (MSB) P3IES |= BIT2;                     // Set to falling-edge sensitive if it is high
            else P3IES &= ~BIT2;                        // Set to rising-edge sensitive if it is low
            if (LSB) P3IES |= BIT3;                     // Set to falling-edge sensitive if it is high
            else P3IES &= ~BIT3;                        // Set to rising-edge sensitive if it is low
            break;
            }
    }
}

float getAngle() {
    return (float) encoder_cnt / 7.0 * 2 * 3.1415 / 150.0;
}

//-- MOTOR DRIVER
void setupDriver() {
    P3DIR |= BIT1 | BIT5;       // Set P3.1 and P3.5 as outputs
    P3OUT &= ~(BIT1 | BIT5);    // Clear outputs
}

void v2pwm(float v) {
    // Limit voltage to ±12V
    if (v > 12.0) v = 12.0;
    if (v < -12.0) v = -12.0;

    unsigned int pwm_val;

    // Set direction and calculate PWM value
    if (v >= 0) {
        // Forward direction: set enable pins accordingly
        P3OUT |= BIT1;    // P3.1 high for forward
        P3OUT &= ~BIT5;   // P3.5 low
        pwm_val = (unsigned int)(v * (99.0 / 12.0) + 0.5); // Round to nearest int
    } else {
        // Reverse direction: set enable pins accordingly
        P3OUT &= ~BIT1;   // P3.1 low for reverse
        P3OUT |= BIT5;    // P3.5 high
        pwm_val = (unsigned int)((-v) * (99.0 / 12.0) + 0.5);
    }

    // Clamp PWM value to 0-99 (just in case)
    if (pwm_val > 99) pwm_val = 99;

    // Set PWM duty cycle
    TB0CCR1 = pwm_val;
}

//-- PWM
void setupPWM(){
    TB0CCR0 = 100-1;                                // PWM Period
    TB0CCTL1 = OUTMOD_7;                            // CCR1 reset/set
    TB0CCR1 = 0;                                    // CCR1 PWM duty cycle
    TB0CTL = TBSSEL_1 | MC_1 | TBCLR;               // ACLK, up mode, clear TAR
    P1DIR |= BIT6;                                  // P1.6 output
    P1SEL1 |= BIT6;                                 // P1.6 option select
    P1SEL0 &= !BIT6;                                // P1.6 option select
}

//-- SAMPLING TIMER
void setupSampleClock() {
    TB3CTL |= TBCLR;                            // reset settings
    TB3CTL |= TBSSEL__ACLK | MC__UP | ID__8;    // 32.768 kHz / 8 = 4096
    TB3CCR0 = 41-1;                             // 102.4 Hz = 41-1
    TB3CCTL0 &= ~CCIE;                          // Disable capture compare
    TB3CCTL0 &= ~CCIFG;                         // Clear IFG
}

void enableSampleClock() {
    TB3CCTL0 |= CCIE;                           // Enable capture compare
    TB3CCTL0 &= ~CCIFG;                         // Clear IFG
}

void disableSampleClock() {
    TB3CCTL0 &= ~CCIE;                          // Disable capture compare
    TB3CCTL0 &= ~CCIFG;                         // Clear IFG
}

#pragma vector = TIMER3_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    hardcode_i = (hardcode_i + 1) % 120;
    this_is_uc = ((hardcoded_angle[hardcode_i / 5])*3.1415f/180.0f);
    updateSystems(this_is_uc);
    // clear CCR0 IFG
    TB3CCTL0 &= ~CCIFG;
}

//-- CONTROL LOOP
void setupControl() {
    // TODO: setup timer to drive control updates
}

void updateSystems(float uc) {
    // Read actual plant output from encoder (user-defined function)
    y_measured = getAngle();                // Get current angle from encoder
    v = (y_measured - y_prev) / Ts;         // Derivative
    y_prev = y_measured;

    // Update velocity low-pass filter
    vlpf.x = vlpf.a * vlpf.x + vlpf.b * v;
    v_filtered = vlpf.x;

    // Error signal: e = y_model - y_plant
    float error = model.x[0] - y_measured;

    // Control signal: u = theta1 * (uc - y) - theta2 * v_filtered
    float u = theta1 * ((float) uc - y_measured) - theta2 * v_filtered;

    // Apply control signal to motor (user-defined function)
    v2pwm(u);

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

    // Send current states over UART
    //addFloat(0, y_measured);
    //addFloat(8, v_filtered);
    //addFloat(16, x1m);
    //addFloat(24, x2m);
    //addFloat(32, error);
    //addFloat(40, u);
    //addFloat(48, theta1);
    //addFloat(56, theta2);

    //i_uart = 0;                 // Reset message index
    //UCA1IFG &=~ UCTXCPTIFG;     // Clear flag
    //UCA1TXBUF = message[0];     // Start message at beginning
    //UCA1IE |= UCTXCPTIE;        // Enable interrupt
}

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

//-- UART
void setupUART() {
    UCA1CTLW0 |= UCSWRST;           // Reset
    UCA1CTLW0 |= UCSSEL__SMCLK;     // Use 1M clock
    UCA1BRW = 8;                    // Set to low freq baud rate mode (divides by 8)
    UCA1MCTLW |= 0xD600;            // Gets really close to 115200
    P4SEL1 &= ~BIT3;                // Config P4.3 to use UCATDX
    P4SEL0 |= BIT3;                 // Config P4.3 to use UCATDX
    UCA1CTLW0 &= ~UCSWRST;          // Take out of software reset
}

void addFloat(unsigned int i, float num) {
    if ((num>=1000.0) | (num<=-1000.0)) {
        message[i] = '+';
        message[i+1] = 'x';
        message[i+2] = 'x';
        message[i+3] = 'x';
        message[i+4] = '.';
        message[i+5] = 'x';
        message[i+6] = 'x';
    } else {
        if (num>=0) {
            message[i] = '+';
        } else {
            message[i] = '-';
            num *= -1;
        }
        unsigned long n = (unsigned long)(num * 100);
        unsigned int hundreds = n / 10000;
        unsigned int tens = (n % 10000) / 1000;
        unsigned int ones = (n % 1000) / 100;
        unsigned int tenths = (n % 100) / 10;
        unsigned int hundredths = n % 10;

        message[i+1] = hundreds + '0';
        message[i+2] = tens + '0';
        message[i+3] = ones + '0';
        message[i+5] = tenths + '0';
        message[i+6] = hundredths + '0';
    }
}

#pragma vector=EUSCI_A1_VECTOR
__interrupt void ISR_EUSCI_A1(void)
{
    // If message sent or space is reached, disable TX interrupts
    if(message[i_uart+1] == ' ' || i_uart == sizeof(message) - 1) {
        UCA1IE &=~ UCTXCPTIE;
    }
    else {
        i_uart++;                     // Inc position and transmit
        UCA1TXBUF = message[i_uart];
    }
    UCA1IFG &=~ UCTXCPTIFG;
}

//-- STATE MACHINE

//----------------------------------------------END FUNCTIONS AND ISRS----------------------------------------------
