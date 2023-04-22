/*
 * Fencing Hit Indicator
 * Copyright (c) 2023 David Rice
 * 
 * github.com/davidsmakerworks
 * 
 * This is a simple fencing hit indicator that implements the hit time and
 * lockout time as specified by current foil fencing rules.
 * 
 * Designed for PIC 16F18325
 * 
 * INPUTS:
 * RC4 - Red fencer B-line
 * RC3 - Green fencer B-line
 * 
 * OUTPUTS:
 * RC5 - Red LED
 * RA2 - Green LED
 * RC2 - Active buzzer
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set only once)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow/Underflow causes Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define FALSE 0
#define TRUE !FALSE

// Pins where the fencers' foils are connected
#define RED_FOIL PORTCbits.RC4
#define GREEN_FOIL PORTCbits.RC3

// Pins where LED driver is connected
#define RED_LED LATCbits.LATC5
#define GREEN_LED LATAbits.LATA2

// Pin where active buzzer is connected
#define BUZZER LATCbits.LATC2

// State values for finite state machine:
//
// STATE_READY: Monitoring for hits by one or both fencers
// STATE_LOCKOUT: Hit has been recorded and lockout time expired
// STATE_RESET: Resetting to prepare for next hit
#define STATE_READY 0
#define STATE_LOCKOUT 1
#define STATE_RESET 2

// Per USA Fencing 2023 rules, the tip of the foil must be depressed for 14 ms
// to register a touch. 300 ms after the first touch is registered, the other
// fencer is no longer able to register a touch.
#define MIN_HIT_TIME 14 
#define LOCKOUT_TIME 300

// Amount of time in msec that buzzer should sound when hit is registered
#define BUZZER_TIME 1500

// Amount of additional time in msec that LEDs should stay on after buzzer has stopped sounding
#define ADDL_LIGHT_TIME 1500

// If a hit is detected within this many ms after the last hit, count it as possibly being due to a disconnected foil
#define DISCONNECT_DETECT_TIME 500

// If this many "hits" are detected in rapid succession, assume that one or both foils are disconnected
// Minimum is 2
#define DISCONNECT_DETECT_COUNT 3

// The time stamps (in ms) at which the fencer's hit was first detected
// This condition must persist for MIN_HIT_TIME ms to be considered a valid hit
uint16_t red_start_timestamp;
uint16_t green_start_timestamp;

// The time stamp (in ms) when the first hit was registered
// No hits can be registered LOCKOUT_TIME ms after the first hit
uint16_t lockout_start_timestamp;

// The time stamp (in ms) when the state was last reset. This is used
// to determine if the buzzer has been sounding continuously due to one
// or both foils being disconnected and disable it until a valid hit
// is recorded
uint16_t last_reset_timestamp;

// The timestamp when the READY state was entered, used to determine when
// a disconnect condition has been cleared
uint16_t ready_timestamp;

// This is the number of times that a "hit" has been detected in rapid succession. If
// this is larger than DISCONNECT_DETECT_COUNT, it means one or both foils are probably
// disconnected.
uint8_t consecutive_activations;

// TRUE if buzzer is armed (i.e., both foils are connected and hits are
// valid). FALSE if the buzzer has been counding continuously due to one
// or both foils beind disconnected.
uint8_t buzzer_armed;

// State variable for finite state machine in main loop
uint8_t state;

// Indicates if the tip is currently depressed (i.e., a hit will be registered
// if the tip remains depressed for MIN_HIT_TIME ms) 
uint8_t red_pending;
uint8_t green_pending;

// Indicated that a hit has been registered by one fencer, and now the other fencer
// has LOCKOUT_TIME ms to also register a hit
uint8_t lockout_pending;

volatile uint16_t ticks = 0; // 1 tick = 1 msec

void __interrupt() isr(void) {
    if (INTCONbits.PEIE) {
        if (PIE1bits.TMR2IE && PIR1bits.TMR2IF) {
            PIR1bits.TMR2IF = 0; // Interrupt flag must be cleared manually
            ticks++; // Count 1 msec
        }
    }
}

void init_osc(void) {
    OSCCON1bits.NDIV = 0b0011; // 32 MHz HFINTOSC / 8 = 4 MHz Fosc = 1 MHz instruction clock

    while (!OSCCON3bits.ORDY); // Wait for clock switch to complete
}

void init_timer(void) {
    T2CONbits.T2CKPS = 0b00; // 1:1 prescaler = 1 MHz clock with Fosc = 4 MHz
    T2CONbits.T2OUTPS = 0b1001; // 1:10 postscaler = 100 kHz count rate
    PR2 = 100; // 100 counts = 1 msec
    T2CONbits.TMR2ON = 1; // Turn on Timer2
}

void init_interrupts(void) {
    PIE1bits.TMR2IE = 1; // Enable Timer2 period match interrupt
}

void init_ports(void) {
    // Disable all analog functions
    ANSELA = 0x00;
    ANSELC = 0x00;

    // Set port directions
    TRISA = 0x00;
    TRISC = 0x00;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;

    // Enable weak pull-ups on inputs
    WPUCbits.WPUC3 = 1;
    WPUCbits.WPUC4 = 1;
}

void init_system(void) {
    init_osc();
    init_ports();
    init_timer();
    init_interrupts();

    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    INTCONbits.GIE = 1; // Enable global interrupts
}

void delay_ms(uint16_t delay_time) {
    uint16_t start_time;
    
    start_time = ticks;
    
    while ((ticks - start_time) < delay_time); // Spin loop until time is expired
}

void main(void) {
    init_system();
    
    state = STATE_RESET;
    buzzer_armed = TRUE;
    last_reset_timestamp = 0;
    consecutive_activations = 0;

    while (1) {
        switch (state) {
            case STATE_RESET:            
                // Record timestamp for disconnect detection
                last_reset_timestamp = ticks;
                
                // Turn off LEDs
                RED_LED = 0;
                GREEN_LED = 0;

                // Turn off buzzer
                BUZZER = 0;

                red_start_timestamp = 0;
                green_start_timestamp = 0;
                lockout_start_timestamp = 0;

                red_pending = FALSE;
                green_pending = FALSE;
                lockout_pending = FALSE;

                ready_timestamp = ticks;
                state = STATE_READY;
                break;
            case STATE_READY:
                // Check to see if buzzer should be rearmed (i.e., disconnect condition is cleared)
                if (!buzzer_armed) {
                    if (ticks - ready_timestamp > DISCONNECT_DETECT_TIME) {
                        buzzer_armed = TRUE;
                        consecutive_activations = 0;
                    }
                }
                
                if (RED_FOIL) {
                    // If tip is depressed...
                    if (red_pending) {
                        // ...and if it has been depressed for MIN_HIT_TIME ms...
                        if ((ticks - red_start_timestamp) > MIN_HIT_TIME) {
                            // ...then if this is the first hit recorded in the sequence...
                            if (!lockout_pending) {
                                // ...set the lockout timer to allow time for the other fencer to hit
                                lockout_start_timestamp = red_start_timestamp;
                                lockout_pending = TRUE;
                            }
                            RED_LED = 1; // Light up red LED
                            if (buzzer_armed) {
                                BUZZER = 1; // Sound buzzer if armed
                            }
                        }
                    } else {
                        // If this is the first moment that the tip has been depressed,
                        // record that time for comparison with MIN_HIT_TIME
                        red_start_timestamp = ticks;
                        red_pending = TRUE;
                    }
                } else {
                    // If tip is not depressed, or is released after less
                    // than MIN_HIT_TIME ms, cancel pending hit
                    red_pending = FALSE;
                }
                
                if (GREEN_FOIL) {
                    // If tip is depressed...
                    if (green_pending) {
                        // ...and if it has been depressed for MIN_HIT_TIME ms...
                        if ((ticks - green_start_timestamp) > MIN_HIT_TIME) {
                            // ...then if this is the first hit recorded in the sequence...
                            if (!lockout_pending) {
                                // ...set the lockout timer to allow time for the other fencer to hit
                                lockout_start_timestamp = green_start_timestamp;
                                lockout_pending = TRUE;
                            }
                            GREEN_LED = 1; // Light up green LED
                            if (buzzer_armed) {
                                BUZZER = 1; // Sound buzzer if armed
                            }
                        }
                    } else {
                        // If this is the first moment that the tip has been depressed,
                        // record that time for comparison with MIN_HIT_TIME
                        green_start_timestamp = ticks;
                        green_pending = TRUE;
                    }
                } else {
                    // If tip is not depressed, or is released after less
                    // than MIN_HIT_TIME ms, cancel pending hit
                    green_pending = FALSE;
                }

                // Continue to check for a hit scored by the other fencer until
                // lockout time has expired
                if (lockout_pending && (ticks - lockout_start_timestamp > LOCKOUT_TIME)) {
                    state = STATE_LOCKOUT; // Go into lockout state after LOCKOUT_TIME ms
                }
                break;
            case STATE_LOCKOUT:
                 // Check for multiple activations in rapid succession which can indicate a disconnected foil
                if (ticks - last_reset_timestamp < DISCONNECT_DETECT_TIME) {
                    consecutive_activations++;
                    
                    // Subtract 2 as workaround for the way consecutive activations are counted
                    if (consecutive_activations > DISCONNECT_DETECT_COUNT - 2) {
                        buzzer_armed = FALSE;
                    }
                }
                
                // Allow buzzer to sound for BUZZER_TIME ms, keep LEDs on for
                // additional ADD_LIGHT_TIME ms after that
                delay_ms(BUZZER_TIME);
                BUZZER = 0;
                delay_ms(ADDL_LIGHT_TIME);

                state = STATE_RESET; // Reset for next hit
                break;
        }
    }
}
