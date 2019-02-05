//******************************************************************************//
// INFO                                                                         //
//******************************************************************************//
// File            : main.c                                                     //
// Author          : Aditya Mall                                                //
// Date            : 11/14/2018                                                 //
// Copyright       : (c) 2018, Aditya Mall, Mentor: Dr. Jason Losh,             //
//                   The University of Texas at Arlington.                      //
// Project         : LCR Meter using EK-TM4C123GXL Evaluation Board.            //
// Target Platform : EK-TM4C123GXL Evaluation Board                             //
// Target uC       : TM4C123GH6PM                                               //
// IDE             : Code Composer Studio v7                                    //
// System Clock    : 40 MHz                                                     //
// UART Baudrate   : 115200                                                     //
// Data Length     : 8 Bits                                                     //
// Version         : 2.4.4                                                      //
//                                                                              //
// Hardware configuration:                                                      //
//  - Red LED at PF1 drives an NPN transistor that powers the red LED           //
//  - Blue LED at PF2 drives an NPN transistor that powers the blue LED         //
//  - Green LED at PF3 drives an NPN transistor that powers the green LED       //
//  - Pushbutton at SW1 pulls pin PF4 low (internal pull-up is used)            //
//  - UART Interface:                                                           //
//       U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller          //
//       Configured to 115,200 baud, 8N1                                        //
//  - Project Specific Interface:                                               //
//       Measure LR enable connected to PF3                                     //
//       Measure C enable connected to PF2                                      //
//       Measure HIGH_R enable connected to PE3                                 //
//       Measure LOW_R enable connected to PE1                                  //
//       INTEGRATE enable connected to PE2                                      //
//       DUT1 connected to PE5                                                  //
//       DUT2 connected to PD2 and PC7                                          //
//       ST7565R Graphics LCD Display Interface:                                //
//       MOSI (SSI2Tx) on PB7                                                   //
//       MISO (SSI2Rx) is not used by the LCD display but                       //
//            the pin is used for GPIO for A0                                   //
//       SCLK (SSI2Clk) on PB4                                                  //
//       A0 connected to PB6                                                    //
//      ~CS connected to PB1                                                    //
//                                                                              //
//******************************************************************************//
// ATTENTION                                                                    //
//******************************************************************************//
//                                                                              //
// This Software was made by Aditya Mall, under the guidance of Dr. Jason Losh, //
// The University of Texas at Arlington. Any UNAUTHORIZED use of this software, //
// without the prior permission and consent of Dr. Jason Losh or any of the,    //
// mentioned contributors is a direct violation of Copyright.                   //
//                                                                              //
// THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED   //
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF           //
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. //
// ADITYA MALL OR ANY MENTIONED CONTRIBUTORS SHALL NOT, IN ANY CIRCUMSTANCES,   //
// BE LIABLE FOR SPECIAL, INCIDENTAL,OR CONSEQUENTIAL DAMAGES,                  //
// FOR ANY REASON WHATSOEVER.                                                   //
//                                                                              //
// For more info please contact: aditya.mall@mavs.uta.edu                       //
//                                                                              //
//******************************************************************************//



//***** References and Citations ******//
//
// 1) ANSI VT100 escape sequence sourced from: "http://ascii-table.com/ansi-escape-sequences-vt-100.php"
// 2) IR Filtering and Timer Code reference from: EE5314 Lectures and code distributed by Professor: Dr. Jason Losh.
// 3) VT100 Operating system control sequences sourced from "http://rtfm.etla.org/xterm/ctlseq.html"
// 4) LCD graphics driver library given by Dr. Jason Losh as a part of course work.
//
//*************************************//



//************* Versions **************//
//
// Version 2.4.4 - 12/02/2018
//   Info:-
//        - All steps 1 - 12 supported on Terminal
//        - L, R and C steps supported on Pushbuttons along with "auto" and additional "Terminal" Mode
//        - Documentation Pending
//
// Version 2.4.3 - 12/02/2018
//   Info:-
//        - Push button supported for "auto" and "capacitor" command
//        - Additional button for getting into terminal mode supported
//
// Version 2.4.2 - 12/02/2018
//   Info:-
//        - Voltage dut1 and dut2 negative value bug removed
//        - auto command stall at resistance bug removed
//        - ports off command gets displayed on LCD
//
// Version 2.4 - 12/02/2018
//   Info:-
//        - Supports "auto" Command.
//        - Needs resistance value calibration
//        - Documentation Pending.
//
// Version 2.3 - 12/01/2018
//   Info:-
//        - LCD Graphics library added (Provided by Dr. Jason Losh)
//        - All values for L, R, C and ESR gets displayed on LCD
//        - LCD display prints USER info on startup.
//
// Version 2.2 - 11/27/2018 (EC2, 2nd intermediate defense)
//   Info:-
//        - Capacitance and Inductance functions added
//        - "capacitor" command supported
//        - Blue background color on xterm
//        - Font and text colors supported on all vt100 based terminals
//
// Version 2.1 - 11/20/2018
//   Info:-
//        - Background Color supported on Xterm, Tera Term
//        - Changed fonts and foreground color supported on all vt100 based terminals
//        - Resistance measurement accuracy improved through better equation
//
// Version 2.0 - 11/14/2018 (EC1, 1st Intermediate defense)
//   Info:-
//        - Steps 1 - 8 of LCR meter project using EK-TM4C123GXL Evaluation Board
//        - Supports "voltage" Command
//        - Supports "resistor" Command
//        - Doesn't have hardware echo, local echo has to be enabled in the terminal settings
//
// Version 1.0 - 11/05/2018
//   Info:-
//        - Steps 1 - 5 of LCR meter project using EK-TM4C123GXL Evaluation Board
//        - Includes Hardware echo
//        - Backspace bug detected after project submission
//
//
//***********************************//



//*****************************************************************************//
//                                                                             //
//              STANDARD LIBRARIES AND BOARD SPECIFIC HEADER FILES             //
//                                                                             //
//*****************************************************************************//

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "graphics_lcd.h"


//*****************************************************************************//
//                                                                             //
//                MACRO DEFINITIONS, DIRECTIVES and STRUCTURES                 //
//                                                                             //
//*****************************************************************************//


//*******************Debug and Code test defines**********************//
#ifndef DEBUG
#define DEBUG
#endif

#ifndef TEST
#define TEST
#endif

#ifndef TEST1
//#define TEST1
#endif

#ifndef EXP
//#define EXP
#endif


//****************** Bit Banding defines for Pins *********************//

//PORT E
#define PE1               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define PE2               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4)))
#define PE3               (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4)))

//Port F
#define PF1               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define PF2               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PF3               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PF4               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

//Port A
#define PA5               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define PA6               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))

//Port D
#define PD1               (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))

//PORT C (!! CAUTION !!, see data sheet)
#define PC5               (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))



//************************ Board Modules Pins ***********************//
#define RED_LED           PF1
#define BLUE_LED          PF2    // Also CAP pin
#define GREEN_LED         PF3    // Also LR pin
#define PUSH_BUTTON       PF4


//*********************** Project Specific Pins ********************//
#define CAP               PF2
#define LR                PF3

#define LOW_R             PE1
#define INTEGRATE         PE2
#define HIGH_R            PE3

#define AUTO_BUTTON       PA6
#define TERM_BUTTON       PUSH_BUTTON
#define CAP_BUTTON        PD1
#define INDUC_BUTTON      PA5
#define RES_BUTTON        PC5


//************************** Project Specific Defines **********************//

#define MAX_SIZE          80
#define MAX_ARGS          80

#define SPECIAL_CHARS     ( (string[i] >= 33 && string[i] <= 47) || \
                            (string[i] >= 58 && string[i] <= 64) || \
                            (string[i] >= 58 && string[i] <= 64) || \
                            (string[i] >= 91 && string[i] <= 96) )

#define ARGS_CHECK(num)   ( args_updated < num || args_updated > num )

#define ADC_RESOLUTION    4096.0
#define SYSTEM_VOLTAGE    3.320
#define CALIB_VAL_DUT1    0.211
#define CALIB_VAL_DUT2    0.268


// Structure to define the current state of the the measurement
typedef struct measure_state
{
    uint8_t res;                            // Structure member to store the state for resistance measurement
    uint8_t cap;                            // Structure member to store the state for capacitance measurement
    uint8_t induc;                          // Structure member to store the state for Inductance measurement
    uint8_t auto_mode;                      // Structure member to store the state for Auto Mode measurement
    uint8_t term;                           // Structure member to store the current mode of operation for LCR meter

}State_ty;


//*****************************************************************************//
//                                                                             //
//                          Function Prototypes                                //
//                                                                             //
//*****************************************************************************//

//Delay and Blocking Functions
uint8_t waitPbPress(void);
void waitMicrosecond(uint32_t us);


//Project Specific Pin Initialization Functions
void init_LR_Pin(void);
void init_C_Pin(void);
void init_HIGHSIDE_R_Pin(void);
void init_LOWSIDE_R_Pin(void);
void init_INTEGRATE_Pin(void);


//UART IO Control functions
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0(void);
void clear_screen(void);
void term_getsUart0(void);


//User test commands
void test_getsUART0(void);
void getsTerminal(void);
void test_commands(void);


//String Parsing functions
void parse_string(void);
uint8_t is_command(char* command, uint8_t arg);


//Project Command Functions
void project_info(void);
void flash_led(void);
void all_pins_zero(void);
void project_commands(void);
void user_lcd_cmd_info(void);

float VAL_DUT1(void);
float VAL_DUT2(void);

void resistor(void);
void capacitor(void);
void inductor(void);
float esr(void);
void auto_mode(void);

void cap_button_specific(void);
void induc_button_specific(void);
void res_button_specific(void);

void _Analog_Comparator0ISR(void);

void system_reset(void);

//Buffer Reset Control Functions
void reset_buffer(void);
void reset_new_string(void);


