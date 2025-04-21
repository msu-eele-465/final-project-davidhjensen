#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- SOMETHING

//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    //------------------------------------------------INITIALIZATIONS------------------------------------------------
    //-- SOMETHING

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
//-- SOMETHING

//----------------------------------------------END FUNCTIONS AND ISRS----------------------------------------------