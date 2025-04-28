#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- KEYPAD
void setupKeypad();                         // Init
char readKeypad();                          // Checks for pressed keys on keypad
int checkCols();                            // Used internally by readKeypad()
char lastKey = 'X';                         // Used internally for debouncing

//-- ADC SAMPLING
void setupADC();                            // Init ADC on P5.2
void readADC();                             // Start ADC read into adc_val
int adc_val;                                // ADC code updated in ADC ISR

//-- CONVERSIONS
#define ADC_SCALER (270.0 / 4095.0)         // 270 / (2^12 - 1)
float adc2angle(int);                       // Convert ADC code to float [0, 1]

//-- SAMPLING TIMER
void setupSampleClock();                    // Setup clock on TB3 to sample ADC every 0.5s
void enableSampleClock();                   // Enable sampling clock interrupts
void disableSampleClock();                  // Disable sampling clock interrupts

//-- I2C MASTER
volatile uint8_t rx_byte_count = 0;         // Number of bytes received
volatile uint8_t rx_data[100];              // Array to hold received bytes
void setupI2C();                            // Setup I2C on P4.6 (SDA) and P4.7 (SCL)
void sendInt(unsigned int);                // Send an integer passed as a char to the function to slave MSP 

//-- SLAVE MSP (I2C INFO + vars?)
#define MSP_ADDR  0x68

//-- LCD
#define LCD_RS BIT0     // Register Select
#define LCD_RW BIT1     // Read/Write
#define LCD_E  BIT2     // Enable
#define LCD_DATA P2OUT  // Data bus on Port 1
char message[] = "                                ";   // 33 characters long, 16 first row, 16 top row, \0
void lcd_init();                            // Initialize the LCD display
void lcd_display_message(char *str);        // Display a 32 character message
void updateAngle(float angle);              // Update angle in bottom left [16-21]
void delay(unsigned int count);                         // INTERNAL
void lcd_enable_pulse();                                // INTERNAL
void lcd_write_command(unsigned char cmd);              // INTERNAL
void lcd_write_data(unsigned char data);                // INTERNAL
void lcd_set_cursor(unsigned char address);             // INTERNAL
void lcd_display_string(char *str);                     // INTERNAL
void lcd_clear();                                       // INTERNAL

//-- STATE MACHINE
// STATE:
        // 0    Main Menu
        // 1    Experiment
        // 2    Command follow based on POT
int state = 0;
//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    //------------------------------------------------INITIALIZATIONS------------------------------------------------
    //-- KEYPAD
    setupKeypad();

    //-- ADC SAMPLING
    setupADC();

    //-- SAMPLING TIMER
    setupSampleClock();

    //-- I2C MASTER
    setupI2C(); 

    //-- SLAVE MSP

    //-- LCD
    lcd_init();  
    memcpy(&message[0], "Select Function                 ", 32);            // update message for LCD
    lcd_display_message(message);

    //-- STATE MACHINE
    char key_val = 'X';                                     // value read from keypad - 'X' if no new input read

    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    //------------------------------------------------STATE MACHINE------------------------------------------------

    while(1)
    {

        key_val = readKeypad();
        if (key_val != 'X') {

            // OFF
            if (state == 0) {

                // start experiment
                if (key_val == '1') {
                    state = 1;
                    memcpy(&message[0], "Starting Expmnt ", 16);    // update message for LCD
                    lcd_display_message(message);                   // update LCD
                    // TODO: send command to slave MSP to start experiment
                    __delay_cycles(1e6);                            // wait for experiment to finish & display results
                    memcpy(&message[0], "Expmnt Finished ", 16);    // update message for LCD
                    lcd_display_message(message);                   // update LCD
                    // TODO: send command to slave MSP to idle
                }

                // command following mode
                if (key_val == '2') {
                    state = 2;
                    memcpy(&message[0], "Cmd Follow: POT ", 16);    // update message for LCD
                    lcd_display_message(message);                   // update LCD
                    // TODO: send command to slave MSP to start command following
                    enableSampleClock();                           // enable sampling timer (reads ADC, updates LCD, TODO: sends angle)
                }

            } else if ((state == 1) | (state == 2)) {
                if (key_val == '*') {
                    state = 0;
                    disableSampleClock();                           // enable sampling timer (reads ADC, updates LCD, TODO: sends angle)
                    memcpy(&message[0], "Select Function                 ", 32);            // update message for LCD
                    lcd_display_message(message);                   // update LCD
                    // TODO: send command to slave MSP to idle
                }
            
            } else {
                // TODO: send command to slave MSP to idle
                // TODO: stop sampling timer (reads ADC, updates LCD, sends angle)
                state = 0;
            }
        }
    }
    //----------------------------------------------END STATE MACHINE----------------------------------------------
}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------
//-- KEYPAD
void setupKeypad() {
    
    // columns as outputs on P4.4, P6.6, P6.5, P6.4 initialized to 0
    P4DIR |= BIT4;
    P6DIR |= BIT6 | BIT5 | BIT4;
    P6OUT &= ~BIT4;
    P6OUT &= ~(BIT6 | BIT5 | BIT4);

    // rows as inputs pulled down internally on P6.3, P6.2, P6.1, P6.0
    P6DIR &= ~(BIT3 | BIT2 | BIT1 | BIT0);      // inputs
    P6REN |= BIT3 | BIT2 | BIT1 | BIT0;         // internal resistors
	P6OUT &= ~(BIT3 | BIT2 | BIT1 | BIT0);        // pull-downs

}

char readKeypad() {
    // columns on P4.4, P6.6, P6.5, P6.4
    // rows on P6.3, P6.2, P6.1, P6.0

    char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

    char pressed = 'X';

    // check row 1
    P4OUT |= BIT4;
    int col = checkCols();
    if (col!=-1) {
        pressed = keys[0][col];
    } 
    P4OUT &= ~BIT4;

    // check row 2
    P6OUT |= BIT6;
    col = checkCols();
    if (col!=-1) {
        pressed =  keys[1][col];
    } 
    P6OUT &= ~BIT6;

    // check row 3
    P6OUT |= BIT5;
    col = checkCols();
    if (col!=-1) {
        pressed =  keys[2][col];
    }
    P6OUT &= ~BIT5;

    // check row 4
    P6OUT |= BIT4;
    col = checkCols();
    if (col!=-1) {
        pressed =  keys[3][col];
    }
    P6OUT &= ~BIT4;

    if (pressed != lastKey) {
        lastKey = pressed;
        return pressed;
    } else {
        return 'X';
    }
}

int checkCols() {
    // check inputs for rows on P6.3, P6.2, P6.1, P6.0 (rows 1-4)
    if (P6IN & BIT3) {
        return 0;
    } else if (P6IN & BIT2) {
        return 1;
    } else if (P6IN & BIT1) {
        return 2;
    } else if (P6IN & BIT0) {
        return 3;
    } else {
        return -1;
    }
}

//-- ADC SAMPLING
void setupADC() {
    // Configure ADC A10 pin on P5.2
    P5SEL0 |= BIT2;
    P5SEL1 |= BIT2;

    // Configure ADC12
    ADCCTL0 |= ADCSHT_2 | ADCON;                             // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;                                       // ADCCLK = MODOSC; sampling timer
    ADCCTL2 &= ~ADCRES;                                      // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;                                     // 12-bit conversion results
    ADCMCTL0 |= ADCINCH_10;                                   // A10 ADC input select; Vref=AVCC
    ADCIE |= ADCIE0;                                         // Enable ADC conv complete interrupt
}


void readADC() {
    // start sampling and conversion
    ADCCTL0 |= ADCENC | ADCSC;
}

#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch(ADCIV)
    {
        case ADCIV_ADCIFG:
            adc_val = ADCMEM0;
            break;
        default:
            break;
    }
}

//-- CONVERSIONS
float adc2angle(int code) {
    // 12-bit adc conversion with 3.3V reference to angle in [0, 270]
    float prop = ((float) code) * ADC_SCALER;
    return prop;
}

//-- SAMPLING TIMER
void setupSampleClock() {
    TB3CTL |= TBCLR;                            // reset settings
    TB3CTL |= TBSSEL__ACLK | MC__UP | ID__8;    // 32.768 kHz / 8 = 4096 / 2 - 1 = 2047
    TB3CCR0 = 127;                              // 32 Hz
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
    readADC();                                  // Start ADC conversion
    while(!(ADCIFG & ADCIFG0));                 // Wait for ADC conversion to finsih
    float angle = adc2angle(adc_val);           // Convert to angle
    updateAngle(angle);                         // Update LCD message
    lcd_display_message(message);               // Update LCD
    // TODO: send angle to slave MSP

    // clear CCR0 IFG
    TB3CCTL0 &= ~CCIFG;
}

//-- I2C MASTER
void setupI2C(){
    UCB1CTLW0 |= UCSWRST;           // Put into software reset
    UCB1CTLW0 |= UCSSEL__SMCLK;     // Use SMCLK 
    UCB1BRW = 10;                   // Divide by 10 (100kHz)

    UCB1CTLW0 |= UCMODE_3 | UCMST;  // Master
    UCB1CTLW1 |= UCASTP_2;          // Autostop on byte count

    P4SEL1 &= ~(BIT6 | BIT7);       // SDA=P4.6, SCL=P4.7
    P4SEL0 |=  (BIT6 | BIT7);       // SDA=P4.6, SCL=P4.7

    UCB1TBCNT = 2;                  // Expect 2 bytes
    UCB1I2CSA = MSP_ADDR;           // RTC address

    UCB1CTLW0 |= UCTR;              // TX mode (clear for RX)
    UCB1CTLW0 &= ~UCSWRST;          // Take out of software reset

    //UCB1IE |= UCRXIE0;              // Enable RX interrupt
}

// Integer send function (for led bar)
void sendInt(unsigned int data) {
    UCB1CTLW0 |= UCTR;                      // TX mode

    while (UCB1CTLW0 & UCTXSTP);            // Wait for STOP if needed
    UCB1I2CSA = MSP_ADDR;                   // Slave address

    UCB1TBCNT = 2;                          // Transmit 2 bytes
    UCB1CTLW0 |= UCTXSTT;                   // Generate START

    while (!(UCB1IFG & UCTXIFG0));          // Wait for TX buffer ready
    UCB1TXBUF = (0xFF00 & data) >> 8;       // First byte

    while (!(UCB1IFG & UCTXIFG0));          // Wait for TX buffer ready
    UCB1TXBUF = (0xFF & data);       // Second byte

    while (UCB1CTLW0 & UCTXSTP);            // Wait for STOP to finish
}

#pragma vector = EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_ISR(void) {
    switch (__even_in_range(UCB1IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCRXIFG0:
            rx_data[rx_byte_count++] = UCB1RXBUF;
            break;
        default:
            break;
    }
}

//-- SLAVE MSP

//-- LCD
void updateAngle(float angle) {
    if ((angle>270.0) | (angle<0.0)) {
        message[16] = 'x';
        message[17] = 'x';
        message[18] = 'x';
        message[20] = 'x';
    } else {
        unsigned int n = (int) (angle*10);
        unsigned int hundreds = n / 1000;
        unsigned int tens = (n - 1000*hundreds) / 100;
        unsigned int ones = (n-1000*hundreds-100*tens) / 10;
        unsigned int tenths = n-1000*hundreds-100*tens-10*ones;
        message[16] = hundreds+48;
        message[17] = tens+48;
        message[18] = ones+48;
        message[20] = tenths+48;
        sendInt(100*hundreds+10*tens+ones);
    }
    message[19] = '.';      // Decimal place
    message[21] = 223;      // Degree symbol
}

void delay(unsigned int count) {
    while(count--) __delay_cycles(1000);
}

void lcd_enable_pulse() {
    P3OUT |= LCD_E;
    delay(1);
    P3OUT &= ~LCD_E;
}

void lcd_write_command(unsigned char cmd) {
    P3OUT &= ~LCD_RS;   // RS = 0 for command
    P3OUT &= ~LCD_RW;   // RW = 0 for write
    LCD_DATA = cmd;     // Write command to data bus
    lcd_enable_pulse();
    delay(1);           // Command execution delay
}

void lcd_write_data(unsigned char data) {
    P3OUT |= LCD_RS;    // RS = 1 for data
    P3OUT &= ~LCD_RW;   // RW = 0 for write
    LCD_DATA = data;    // Write data to data bus
    lcd_enable_pulse();
    delay(1);           // Data write delay
}

void lcd_init() {
    P3DIR |= LCD_RS | LCD_RW | LCD_E;
    P2DIR |= 0xFF;      // Set Port 1 as output for data bus

    delay(50);          // Power-on delay

    lcd_write_command(0x38);    // Function set: 8-bit, 2 lines, 5x8 dots
    lcd_write_command(0x0C);    // Display ON, Cursor OFF
    lcd_clear();                // Clear
    lcd_write_command(0x06);    // Entry mode set: Increment cursor
}

void lcd_set_cursor(unsigned char address) {
    lcd_write_command(0x80 | address);
    delay(1);
}

void lcd_display_string(char *str) {
    while(*str) {
        lcd_write_data(*str++);
    }
}

void lcd_clear(){
    lcd_write_command(0x01);    // Clear display
    delay(1);                   // Delay for clear command
}

void lcd_display_message(char *str ){   // Write all 32 characters to display
    int i;
    lcd_set_cursor(0x00);
    for (i=0; i<16; i++){
        lcd_write_data(*str++);
    }

    lcd_set_cursor(0x40);
    for (i=0; i<16; i++){
        lcd_write_data(*str++);
    }
}
//--------------------------------------------END FUNCTIONS AND ISRS--------------------------------------------