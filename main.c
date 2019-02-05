
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



//*****************************************************************************//
//                                                                             //
//              STANDARD LIBRARIES AND BOARD SPECIFIC HEADER FILES             //
//                                                                             //
//*****************************************************************************//

#include "headers.h"



//*****************************************************************************//
//                                                                             //
//                          GLOBAL VARIABLES                                   //
//                                                                             //
//*****************************************************************************//


// String variables
char string[MAX_SIZE]               = {0};   // Array to store the chars received from UART
char new_string[MAX_ARGS][MAX_SIZE] = {0};   // Array to store the words after dividing the string to tokens


// Char category variables
uint8_t a[MAX_ARGS] = {0};                   // Array to store the record of Alpha characters
uint8_t n[MAX_ARGS] = {0};                   // Array to store the record of Numeric characters
uint8_t s[MAX_ARGS] = {0};                   // Array to store the record of Special characters


// Measurement result variables
char str[10];                                // Array to store printable measurement value
uint32_t time_constant;                      // Variable for storing the value of the time constant
float resistance;                            // Variable for storing the value of resistance measurement result
float capacitance;                           // Variable for storing the value of resistance measurement result
float inductance;                            // Variable for storing the value of resistance measurement result
float esr_value;                             // Variable for storing the value of resistance measurement result
uint16_t raw_DUT1;                           // Variable for storing the ADC1 FIFO Value
uint16_t raw_DUT2;                           // Variable for storing the ADC0 FIFO Value


// Argument count variables
uint8_t args_no      = 0;                    // Variable for indexing initial number of arguments
uint8_t args_str     = 0;                    // Variable for indexing the number of characters per argument
uint8_t args_updated;                        // Variable for indexing final number of arguments, not initialized to zero
uint8_t try_counter  = 0;                    // Variable for indexing the number of counts of measurement

// Structure Variables
State_ty state;                              // Structure variable declaration for current measurement state

// Test Variables
char ch;
char buff_int[MAX_SIZE] = {0};



//*****************************************************************************//
//                                                                             //
//                     HARDWARE INTIALIZATION FUNCTION                         //
//                                                                             //
//*****************************************************************************//

void initHw()
{
    // Configure System clock as 40Mhz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (0x04 << SYSCTL_RCC_SYSDIV_S);

    // UART on port A must use APB, default added for clarification
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, F, E, C, D and B peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF| SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOB;


    // Configure RED led and Pushbutton Pins
    GPIO_PORTF_DIR_R &= ~(0x10);                                                   // Enable push button as Input
    GPIO_PORTF_DIR_R |= 0x02;                                                      // Enable PF1 as Output for Red Led
    GPIO_PORTF_DEN_R |= 0x12;                                                      // Enable Digital for Pushbuttons and Red Led
    GPIO_PORTF_PUR_R |= 0x10;                                                      // Enable internal pull-up for push button

    GPIO_PORTA_DIR_R &=  ~(1 << 5) | ~(1 << 6);
    GPIO_PORTA_DEN_R |=  (1 << 5) | (1 << 6);
    GPIO_PORTA_PUR_R |=  (1 << 5) | (1 << 6);

    GPIO_PORTD_DIR_R &=  ~(1 << 1);
    GPIO_PORTD_DEN_R |=  (1 << 1);
    GPIO_PORTD_PUR_R |=  (1 << 1);

    //(!! ATTENTION!!, please data sheet before configuring PORTC)
    GPIO_PORTC_DIR_R &=  ~(1 << 5);
    GPIO_PORTC_DEN_R |=  (1 << 5);
    GPIO_PORTC_PUR_R |=  (1 << 5);


    // Configure UART0 pins
    SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R0;                                      // Turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R   |= 3;                                                       // Turn on Digital Operations on PA0 and PA1
    GPIO_PORTA_AFSEL_R |= 3;                                                       // Select Alternate Functionality on PA0 and PA1
    GPIO_PORTA_PCTL_R  |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;                 // Select UART0 Module

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R   = 0;                                                             // turn-off UART0 to allow safe programming
    UART0_CC_R   |= UART_CC_CS_SYSCLK;                                             // use system clock (40 MHz)
    UART0_IBRD_R  = 21;                                                            // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R  = 45;                                                            // round(fract(r)*64)=45
    UART0_LCRH_R |= UART_LCRH_WLEN_8 | UART_LCRH_FEN;                              // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R  |= UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;                 // enable TX, RX, and module

    // Configure ADC on DUT 1 through GPIO registers
    SYSCTL_RCGCADC_R   |= SYSCTL_RCGCADC_R1;                                       // Turn on clock for ADC Module 0
    GPIO_PORTE_DEN_R   &= ~(1 << 5);                                               // Turn off digital operation on on PE5
    GPIO_PORTE_AFSEL_R |= (1 << 5);                                                // Select Alternate Functionality on PE5
    GPIO_PORTE_AMSEL_R |= (1 << 5);                                                // Select Analog Mode on PE5
    GPIO_PORTE_PCTL_R  |= GPIO_PCTL_PE5_AIN8;                                      // Default Added for clarification

    // Configure ADC registers for DUT1
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                                              // Disable SS3 for safe programming
    ADC1_EMUX_R   = ADC_EMUX_EM3_PROCESSOR;                                        // Select SS3 bit in ADCPSSI as trigger, default value
    ADC1_SSMUX3_R = 8;                                                             // Select SS3 MUX to set 1st sample sequence to AIN8
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                                               // Mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                                               // Enable SS3 for operation

    // Configure ADC on DUT 2 through GPIO registers
    SYSCTL_RCGCADC_R   |= SYSCTL_RCGCADC_R0;                                       // Turn on clock for ADC Module 0
    GPIO_PORTD_DEN_R   &= ~(1 << 2);                                               // Turn off digital operation on
    GPIO_PORTD_DIR_R   &= ~(1 << 2);                                               // ??
    GPIO_PORTD_AFSEL_R |= (1 << 2);                                                // Select Alternate Functionality on PD2
    GPIO_PORTD_AMSEL_R |= (1 << 2);                                                // Select Analog Mode on PD2
    GPIO_PORTD_PCTL_R  |= GPIO_PCTL_PD2_AIN5;                                      // Default Added for clarification

    // Configure ADC registers for DUT2
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                                              // Disable SS3 for safe programming
    ADC0_EMUX_R   = ADC_EMUX_EM2_PROCESSOR;                                        // Select SS3 bit in ADCPSSI as trigger, default value
    ADC0_SSMUX3_R = 5;                                                             // Select SS3 MUX to set 1st sample sequence to AIN5
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                                               // Mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                                               // Enable SS3 for operation

    // TIMER Configure
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;                                   // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                                              // turn-off counter before reconfiguring
    WTIMER5_CFG_R  = 4;                                                            // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;   // configure for edge time mode, count up
    WTIMER5_CTL_R  = TIMER_CTL_TAEVENT_POS;                                        // measure time from positive edge to positive edge
    WTIMER5_TAV_R  = 0;                                                            // zero counter for first period

    // Comparator GPIO configure
    SYSCTL_RCGCACMP_R  |= 0x01;                                                    // Enable comparator clock
    GPIO_PORTC_DEN_R   &= ~(1 << 7);                                               // Turn off digital Operation on PC7
    GPIO_PORTC_DIR_R   &= ~(1 << 7);                                               // Make PC7 as Input
    GPIO_PORTC_AFSEL_R |= (1 << 7);                                                // Select Alternate Functionality on PC7
    GPIO_PORTC_AMSEL_R |= (1 << 7);                                                // Select Analog Mode on PC7

    // Comparator Register configure
    COMP_ACREFCTL_R = 0xF | (1 << 9);                                              // Select Internal reference voltage as 2.464 Volts
    COMP_ACCTL0_R  |= (0x02 << 9) | (0x02 << 2) | (0x01 << 1);                     // Configure for internal volatge reference, rising edge sense and inverted output
    //NVIC_EN0_R     |= (1 << INT_COMP0 - 16);                                     //

    // Configure A0 and ~CS for graphics LCD
    GPIO_PORTB_DIR_R |= 0x42;                                                      // Make bits 1 and 6 outputs
    GPIO_PORTB_DR2R_R |= 0x42;                                                     // Set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x42;                                                      // Enable bits 1 and 6 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;                                         // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                                                      // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                                                     // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0x90;                                                    // select alternative functions for MOSI, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB4_SSI2CLK;              // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0x90;                                                      // enable digital operation on TX, CLK pins
    GPIO_PORTB_PUR_R |= 0x10;                                                      // must be enabled when SPO=1

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                                                    // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                                                // select master mode
    SSI2_CC_R = 0;                                                                 // select system clock as the clock source
    SSI2_CPSR_R = 40;                                                              // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;     // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                                                     // turn on SSI2

}

//*****************************************************************************//
//                                                                             //
//                          DELAY FUNCTIONS                                    //
//                                                                             //
//*****************************************************************************//



void _Analog_Comparator0ISR(void)
{
    if(state.induc)
    {
        float voltage_1, voltage_2;
        time_constant = WTIMER5_TAV_R;      //

        WTIMER5_TAV_R = 0;                  //

        sprintf(str, "%u", time_constant);
        putsUart0("\r\n");
        putsUart0("time_const_inductor:");
        putsUart0(str);
        putsUart0("\r\n");
        /*
        waitMicrosecond(500000);

        voltage_2 = VAL_DUT2();
        voltage_1 = VAL_DUT1();

        esr_value = ( 33 * (voltage_1 / 2.464 ) ) - 33;
         */
        COMP_ACMIS_R = 0x01;                // clear the comparator interrupt
        WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;

    }
    else
    {
        time_constant = WTIMER5_TAV_R;      //

        WTIMER5_TAV_R = 0;                  //

        time_constant /= 40.0;              //

        //RED_LED ^= 1;

        sprintf(str, "%u", time_constant);
        putsUart0("\r\n");
        putsUart0("time_const:");
        putsUart0(str);
        putsUart0("\r\n");

        COMP_ACMIS_R = 0x01;                // clear the comparator interrupt
        WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;

    }
}

//micro second delay function
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6"       );
    __asm("WMS_LOOP1:   SUB  R1, #1"       );
    __asm("             CBZ  R1, WMS_DONE1");
    __asm("             NOP"               );
    __asm("             NOP"               );
    __asm("             B    WMS_LOOP1"    );
    __asm("WMS_DONE1:   SUB  R0, #1"       );
    __asm("             CBZ  R0, WMS_DONE0");
    __asm("             NOP"               );
    __asm("             B    WMS_LOOP0"    );
    __asm("WMS_DONE0:"                     );
}

// Blocking function that returns only when SW1 is pressed
uint8_t waitPbPress(void)
{
    if (PUSH_BUTTON)
        return 0;

    else
    {
        waitMicrosecond(50000);
        return 1;
    }
}

//*****************************************************************************//
//                                                                             //
//                     LCR PIN INTIALIZATION FUNCTIONS                         //
//                                                                             //
//*****************************************************************************//

//!! LR and C pin if set to high at the same time can lead to Hardware damage !!//

void init_LR_Pin(void)
{
    //GPIO configs for LR pin on PF3 (Also board GREEN led)

    GPIO_PORTF_DIR_R |= (1 << 3);       //Set pin 3 of Port F as output
    GPIO_PORTF_DEN_R |= (1 << 3);       //Enable digital function for pin 3
}

void init_C_Pin(void)
{
    //GPIO configs for LR pin on PF2 (Also board Blue led)

    GPIO_PORTF_DIR_R |= (1 << 2);       //Set pin 2 of Port F as output
    GPIO_PORTF_DEN_R |= (1 << 2);       //Enable digital function for pin 2

}

void init_HIGHSIDE_R_Pin(void)
{
    GPIO_PORTE_DIR_R |= (1 << 3);       //Set pin 3 of Port E as output
    GPIO_PORTE_DEN_R |= (1 << 3);       //Enable digital function for pin 3

}

void init_LOWSIDE_R_Pin(void)
{
    GPIO_PORTE_DIR_R |= (0x02);         //Set pin 3 of Port E as output
    GPIO_PORTE_DEN_R |= (0x02);         //Enable digital function for pin 3

}

void init_INTEGRATE_Pin(void)
{
    GPIO_PORTE_DIR_R |= (1 << 2);       //Set pin 3 of Port E as output
    GPIO_PORTE_DEN_R |= (1 << 2);       //Enable digital function for pin 3

}

//*****************************************************************************//
//                                                                             //
//                     UART IO Control Functions                               //
//                                                                             //
//*****************************************************************************//

// Function for Clearing the Terminal Screen via UART
void clear_screen(void)
{
    putsUart0("\x1b[2J\x1b[H");         //ANSI VT100 escape sequence, clear screen and set cursor to home.
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;

    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0(void)
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

// Blocking Function for getting the input as string once the buffer is not empty
void getsUart0(void)
{
    char input;

    while(1)
    {
        input = getcUart0();
        putcUart0(input);
    }

}

// Blocking Function for getting the input as string once the buffer is not empty,
// Checks for max string size of 80 characters, Backspace, Uppercase characters and
// Terminates function when Carriage return is received.
void term_getsUart0(void)
{
    char c_input;
    uint8_t count = 0;

    while (1)
    {
        // Get input from terminal
        c_input = getcUart0();

        // Echo the input to the terminal, only for test
        //putcUart0(c_input);

        // Check if string is more than 80 characters
        if (count == MAX_SIZE)
        {
            putsUart0("Can't exceed more than 80 chars");   // Let the User know that character count has been exceeded
            reset_buffer();                                 // Reset the buffer, call function
            string[count] = 0;                              // Return null at the end of the string
            break;                                          // Break out of the loop
        }

        // Implement Backspace and Carriage Return
        if(c_input == 8)
        {

            putcUart0(' ');                                 // Clear previous characters when backspace is received
            putsUart0("\x1b[D");                            // Shift cursor to previous position, ANSI Escape sequences for VT100

            if(c_input == 0)
                continue;

            else
            {
                count--;                                    // Decrement loop if char != 0
                continue;
            }

        }
        else if(c_input == 13)
        {
            string[count]=0;
            break;
        }

        // Check for Upper case characters and convert them to lower
        if (c_input >= 65 && c_input <= 90)
            string[count++] = c_input + 32;

        else
            string[count++] = c_input;

    }

    putsUart0("\r\n");
}

//*****************************************************************************//
//                                                                             //
//                     ADC IO Control Functions                                //
//                                                                             //
//*****************************************************************************//

int16_t read_Adc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

int16_t read_Adc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}



//*****************************************************************************//
//                                                                             //
//                     STRING PARSING FUNCTIONS                                //
//                                                                             //
//*****************************************************************************//

//Function for tokenizing string
void parse_string(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t array_shift = 0;

    //Convert character into string blocks and tokenize these blocks with delimiters
    for (i = 0; i <= strlen(string); i++)
    {
        if (string[i]== ' '|| string[i] == '\0' || string[i] == 9 || SPECIAL_CHARS)
        {
            new_string[args_no][args_str] = 0;
            args_no++;
            args_str = 0;
        }
        else
        {
            new_string[args_no][args_str] = string[i];
            args_str++;
        }
    }

    array_shift = 1;

    //shift the words to be printed to the starting position of the array
    while (array_shift)
    {
        array_shift = 0;

        //keep swapping elements to the right
        for (j = 0; j < args_no - 1; j++)
        {
            // Check for null elements and accordingly sort Array
            if (strncmp(new_string[j], "\0", 1) == 0 && strncmp(new_string[j + 1], "\0", 1) != 0)
            {
                array_shift = 1;

                // Exchange elements
                strcpy(new_string[j], new_string[j + 1]);
                strcpy(new_string[j + 1], "\0");
            }

        }
    }

    // Determine type of string for every argument
    for (j = 0; j < args_no; j++)
    {
        for (i = 0; i < strlen(new_string[j]); i++)
        {
            if (new_string[j][i] >= 97 && new_string[j][i] <= 122)          // Check if character is between a to z for the particular argument position
                a[j] = 1;                                                   // Store 1 if true at that particular argument position

            else if (new_string[j][i] >= 48 && new_string[j][i] <= 57)      // Check if character is between 0 to 9 for the particular argument position
                n[j] = 1;                                                   // Store 1 if true at that particular argument position
        }
    }

    // Update argument number from type of characters
    for (j = 0; j < args_no; j++)
    {
        if (a[j] == 0 && n[j] == 1)
        {
            putsUart0("numeric string \r\n");
            args_updated++;
        }

        else if (a[j] == 1 && n[j] == 0)
        {
            putsUart0("alpha string \r\n");
            args_updated++;
        }

        else if (a[j] == 1 && n[j] == 1)
        {
            putsUart0("alpha numeric string \r\n");
            args_updated++;
        }
    }
}

// Function to check the argument for a particular string/verb,
// return value: 1, if conditions are satisfied.
uint8_t is_command(char* command, uint8_t arg)
{
    arg = arg + 1;
    if (strcmp(new_string[0], command) == 0 && ARGS_CHECK(arg))    //if ARGS_CHECK is false, function returns 0
        return 0;

    return 1;
}



//*****************************************************************************//
//                                                                             //
//                     USER TEST FUNCTIONS                                     //
//                                                                             //
//*****************************************************************************//

#ifdef TEST

int count = 0;
void test_getsUART0(void)
{
    /* Loop variables */
    uint8_t i, y = 0;

    /* Let user type chars till reaches 80 chars */
    for (i = 0; i < MAX_SIZE; i++)
    {
        string[i] = getcUart0();

        //putcUart0(string[i]);

        /* Lowercase the string */
        if (string[i] >= 65 && string[i] <= 90)
            string[i] += 32;

        /* Test condition for  backspace */


        else if (string[i] == 8)
        {
            --i;
            string[i] = '\0';

        }

        /* Test condition for carriage return */
        else if (string[i] == 13)
        {
            string[i] = '\0';
            break;
        }

        /* Test condition for Max Character size */
        else if (i == (MAX_SIZE - 1))
        {
            putsUart0("\r\n");
            putsUart0("Can't Exceed more than 80 Chars\r\n");
            break;
        }

    }

    putcUart0(13);
    putcUart0(10);

}
#endif


#ifdef TEST
void test_commands(void)
{
    //********************************************************** Step 4 ******************************************************************//
    // LED commands Test //

    //Check arguments for string = set
    if (is_command("set", 2))
    {
        // Compare received string and then turn on Green Led
        if (strcmp(new_string[0], "set") == 0 && (strcmp(new_string[1], "green") == 0) && (strcmp(new_string[2], "on") == 0))
        {
            putsUart0("!! Green on !! \n\r");
            GPIO_PORTF_DIR_R |= (1 << 3);        // Set pin 3 of Port F as output
            GPIO_PORTF_DEN_R |= (1 << 3);        // Enable digital function for pin 3
            BLUE_LED  = 0;                       //
            RED_LED   = 0;                       //
            GREEN_LED = 1;                       // Turn Green Led on
        }

        // Compare received string and then turn on Red Led
        else if (strcmp(new_string[0], "set") == 0 && (strcmp(new_string[1], "red") == 0) && (strcmp(new_string[2], "on") == 0))
        {
            putsUart0("!! Red on !! \n\r");
            BLUE_LED  = 0;                       //
            GREEN_LED = 0;                       //
            RED_LED   = 1;                       // Turn Red Led on
        }

        // Compare received string and then turn On Blue Led
        else if (strcmp(new_string[0], "set") == 0 && (strcmp(new_string[1], "blue") == 0) && (strcmp(new_string[2], "on") == 0))
        {
            putsUart0("!! Blue on !! \n\r");
            GPIO_PORTF_DIR_R |= (1 << 2);        // Set pin 2 of Port F as output
            GPIO_PORTF_DEN_R |= (1 << 2);        // Enable digital function for pin 2
            GREEN_LED = 0;                       //
            RED_LED   = 0;                       //
            BLUE_LED  = 1;                       // Turn Blue on
        }
    }
    else
    {
        putsUart0("This Command Takes at least 2 args \n\r");
        putsUart0("Example \"set\" \"green \\ red\" \" on \" \n\r");
    }

    //*********************************************************** STEP 5 *******************************************************************//

    // Check agrs for string = Measure
    if (is_command("enable", 1))
    {
        // Compare received string and then enable LR
        if ((strcmp(new_string[0], "enable") == 0) && (strcmp(new_string[1], "lr") == 0))
        {
            putsUart0("!! Enable LR Pin !! \n\r");             //
            init_LR_Pin();                                     // test function that turns on LR pin which is also green led on board
            CAP = 0;                                           // Turn off Measure Capacitance for preventing damage to Daughter Board
            LR = 1;                                            // Turn on Measure LR pin

        }

        // Compare received string and then enable C
        else if ((strcmp(new_string[0], "enable") == 0) && (strcmp(new_string[1], "c") == 0))
        {
            putsUart0("!! Enable Capacitance Pin !! \n\r");    //
            init_C_Pin();                                      // test function that turns on C pin which is also blue led on board
            LR  = 0;                                           //
            CAP = 1;                                           //
        }

        // Compare received string and then enable Highside R
        else if ((strcmp(new_string[0], "enable") == 0) && (strcmp(new_string[1], "highr") == 0))
        {
            putsUart0("!! Enable Highside_R Pin !! \n\r");     //
            init_HIGHSIDE_R_Pin();                             //
            LOW_R  = 0;                                        //
            HIGH_R = 1;                                        //

        }

        // Compare received string and then enable Lowside R
        else if ((strcmp(new_string[0], "enable") == 0) && (strcmp(new_string[1], "lowr") == 0))
        {
            putsUart0("!! Enable Lowside_R Pin !! \n\r");      //
            init_LOWSIDE_R_Pin();                              //
            HIGH_R = 0;                                        //
            LOW_R  = 1;                                        //
        }

        // Compare received string and then Integrate
        else if((strcmp(new_string[0], "enable") == 0) && (strcmp(new_string[1], "integrate") == 0))
        {

            putsUart0("!! Enable Integrate Pin !! \n\r");      //
            init_INTEGRATE_Pin();                              //
            INTEGRATE = 1;                                     //
        }
    }
    else
    {
        putsUart0("This Command Takes 1 argument \n\r");
        putsUart0("Example \"Enable\" \"LR, C etc.\" \n\r");
    }


    // Turn off all ports
    if (strcmp(new_string[0], "off") == 0)
    {
        GPIO_PORTF_DATA_R &= ~(0xFF);                          //
        GPIO_PORTE_DATA_R &= ~(0xFF);                          //
        GPIO_PORTC_DATA_R &= ~(0xFF);                          //
        GPIO_PORTD_DATA_R &= ~(0xFF);                          //
    }

    // Clear the Terminal Screen
    if (strcmp(new_string[0], "clear") == 0)
    {
        clear_screen();                                        // Call Clear Screen Function
        putsUart0("Screen Cleared \r\n");                      // Print to tell user that screen is cleared
    }

}
#endif



//*****************************************************************************//
//                                                                             //
//                     PROJECT COMMAND FUNCTIONS                               //
//                                                                             //
//*****************************************************************************//

void all_pins_zero(void)
{
    GPIO_PORTF_DATA_R &= ~(0xFF);
    GPIO_PORTE_DATA_R &= ~(0xFF);
    GPIO_PORTC_DATA_R &= ~(0xFF);
    GPIO_PORTD_DATA_R &= ~(0xFF);
}

// Flash led for 500 milliseconds
void flash_led(void)
{
    RED_LED = 1;
    waitMicrosecond(500000);  //500 ms delay
    RED_LED = 0;
}

void project_info(void)
{
    putsUart0("\033]2;| Name:Aditya Mall | ID:1001626048 | (c) 2018 |\007");                                            // Window Title Information
    putsUart0("\033]10;#FFFFFF\007");                                                                                   // Text Color
    putsUart0("\033]11;#4169E1\007");                                                                                   // Background Color

    putsUart0("\r\n");
    putsUart0("Project: LCR Meter using EK-TM4C123GXL Evaluation Board.\r\n");                                          // Project Name
    putsUart0("Name   : Aditya Mall \r\n");                                                                             // Author Name
    putsUart0("ID     : 1001626048 \r\n" );                                                                             // Author ID
    putsUart0("email  : \033[38;5;51;4maditya.mall@mavs.uta.edu\033[0m \r\n");                                          // Email Info, Foreground color:Cyan

    // Tell User to activate local echo from respectuve terminal setting if available
    putsUart0("\r\n");
    putsUart0("\033[33;1m!! This Program requires Local Echo, please enable Local Echo from settings !!\033[0m \r\n");  // Foreground color:Yellow
    putsUart0("\033[33;1m!! Set Stack Size to 8K bytes if you wish to run the source code !!\033[0m \r\n");
    putsUart0("\r\n");

}

void project_info_lcd(void)
{
    clearGraphicsLcd();
    setGraphicsLcdTextPosition(35, 3);
    putsGraphicsLcd("LCR Meter");
    waitMicrosecond(2000000);

    clearGraphicsLcd();

    setGraphicsLcdTextPosition(0, 0);
    putsGraphicsLcd("Name: Aditya Mall");
    setGraphicsLcdTextPosition(0, 2);
    putsGraphicsLcd("ID  : 1001626048");
    waitMicrosecond(2000000);

    clearGraphicsLcd();

}

void user_lcd_cmd_info(void)
{
    setGraphicsLcdTextPosition(0, 0);
    putsGraphicsLcd("Press L,C or R button");

    setGraphicsLcdTextPosition(35, 1);
    putsGraphicsLcd("or");

    setGraphicsLcdTextPosition(0, 2);
    putsGraphicsLcd("Type on terminal");

}


void project_commands(void)
{
    // Test project specific commands //

    // Voltage command
    if(is_command("voltage", 0))
    {
        if(strcmp(new_string[0], "voltage") == 0 )
        {
            sprintf(str, "%u", raw_DUT1);
            putsUart0("RAW_VAL_DUT1:");
            putsUart0(str);
            putsUart0("\r\n");

            sprintf(str, "%3.3f", fabs(VAL_DUT1()));
            putsUart0("DUT1_VAL:");
            putsUart0(str);
            putsUart0("\r\n");

            sprintf(str, "%u", raw_DUT2);
            putsUart0("RAW_VAL_DUT2:");
            putsUart0(str);
            putsUart0("\r\n");

            sprintf(str, "%3.3f", fabs(VAL_DUT2()));
            putsUart0("DUT2_VAL:");
            putsUart0(str);
            putsUart0("\r\n");

            sprintf(str, "%3.3f", (VAL_DUT2() - VAL_DUT1()));

            putsUart0("\r\n");
            putsUart0("Voltage:");
            putsUart0(str);
            putsUart0("\r\n");

            clearGraphicsLcd();
            setGraphicsLcdTextPosition(0, 2);
            putsGraphicsLcd("Voltage:-");
            setGraphicsLcdTextPosition(0, 3);
            putsGraphicsLcd(str);

        }
    }
    else
    {
        putsUart0("voltage Command Takes No arguments \n\r");

    }

    //resistance command
    if(is_command("resistor", 0))
    {
        if(strcmp(new_string[0], "resistor") ==0 )
        {
            while(1)
            {
                putsUart0("\r\n");
                putsUart0("Measuring Resistance....:");
                putsUart0("\r\n");

                clearGraphicsLcd();
                setGraphicsLcdTextPosition(0, 1);
                putsGraphicsLcd("Measuring Resistance...");

                NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
                waitMicrosecond(500000);

                resistor();

                waitMicrosecond(1000000);

                //resistance = (time_constant - 14.61) / 1.3191;

                //22 - 58
                //resistance = (time_constant - 9.3547)/1.2585;

                resistance = (time_constant - 15.564)/1.104;

                sprintf(str, "%.2f Ohms", resistance);
                putsUart0("Resistance Value");
                putsUart0(str);
                putsUart0("\r\n");

                clearGraphicsLcd();
                setGraphicsLcdTextPosition(0, 2);
                putsGraphicsLcd("Resistance Value:-");
                setGraphicsLcdTextPosition(0, 3);
                putsGraphicsLcd(str);

                if(try_counter >= 2)
                {
                    NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                    COMP_ACINTEN_R &= ~(0x01);
                    all_pins_zero();
                    WTIMER5_TAV_R   = 0;
                    break;
                }

                try_counter++;

                sprintf(str, "%u", try_counter);
                putsUart0("try count:");
                putsUart0(str);
                putsUart0("\r\n");


                if(resistance > 0 || time_constant > 10000000)
                {
                    NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                    COMP_ACINTEN_R &= ~(0x01);
                    all_pins_zero();
                    WTIMER5_TAV_R   = 0;
                    break;
                }
                else
                {
                    putsUart0("Trying Again \r\n");
                    continue;
                }

            }//while loop

        }//sub if statement

    }//main if statement
    else
    {
        putsUart0("resistor Command Takes No arguments \n\r");

    }

    uint8_t loop_cap;


    if(is_command("capacitor", 0))
    {
        if(strcmp(new_string[0], "capacitor") ==0 )
        {
            for(loop_cap = 0; loop_cap <2; loop_cap ++)
            {
                while(1)
                {
                    putsUart0("\r\n");
                    putsUart0("Measuring Capacitance....:");
                    putsUart0("\r\n");

                    clearGraphicsLcd();
                    setGraphicsLcdTextPosition(0, 1);
                    putsGraphicsLcd("Measuring Capacitance...");

                    NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
                    waitMicrosecond(500000);

                    capacitor();

                    waitMicrosecond(1000000);

                    sprintf(str, "%d", time_constant);
                    putsUart0("Time Constant_cap:");
                    putsUart0(str);
                    putsUart0("\r\n");

                    capacitance  = (time_constant - 23.761 ) / 147.238;
                    capacitance /= 1000;

                    sprintf(str, "%f uF", capacitance);
                    putsUart0("Compacitance in uF:");
                    putsUart0(str);
                    putsUart0("\r\n");

                    clearGraphicsLcd();
                    setGraphicsLcdTextPosition(0, 2);
                    putsGraphicsLcd("Capacitance Value:-");
                    setGraphicsLcdTextPosition(0, 3);
                    putsGraphicsLcd(str);

                    if(try_counter >= 2)
                    {
                        NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                        COMP_ACINTEN_R &= ~(0x01);
                        all_pins_zero();
                        WTIMER5_TAV_R   = 0;
                        break;
                    }

                    try_counter++;

                    sprintf(str, "%u", try_counter);
                    putsUart0("try count:");
                    putsUart0(str);
                    putsUart0("\r\n");


                    if(time_constant > 0 && time_constant < 10000000)
                    {
                        NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                        COMP_ACINTEN_R &= ~(0x01);
                        all_pins_zero();
                        WTIMER5_TAV_R   = 0;
                        break;
                    }
                    else
                    {
                        putsUart0("Trying Again \r\n");
                        continue;
                    }

                }//while loop
            }//for loop
        }//sub if statement
    }//main if statement

    else
    {
        putsUart0("Capacitor Command Takes No arguments \n\r");

    }


    if(is_command("inductor", 0))
    {
        if(strcmp(new_string[0], "inductor") == 0 )
        {
            while(1)
            {

                putsUart0("\r\n");
                putsUart0("Measuring Inductance....:");
                putsUart0("\r\n");

                clearGraphicsLcd();
                setGraphicsLcdTextPosition(0, 1);
                putsGraphicsLcd("Measuring Inductance...");

                NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
                waitMicrosecond(500000);

                state.induc = 1;

                inductor();

                waitMicrosecond(5000000);

                sprintf(str, "%d", time_constant);
                putsUart0("Time Constant_induc:");
                putsUart0(str);
                putsUart0("\r\n");

                //inductance = (time_constant - 22.267) / 1.437;

                inductance = (time_constant - 30.127) / 1.3991;

                //inductance = ( log(time_constant / 182.41) ) / 0.0011;

                sprintf(str, "%.2f uH", inductance);
                putsUart0("inductance Value:");
                putsUart0(str);
                putsUart0("\r\n");

                clearGraphicsLcd();
                setGraphicsLcdTextPosition(0, 2);
                putsGraphicsLcd("Inductor Value:-");
                setGraphicsLcdTextPosition(0, 3);
                putsGraphicsLcd(str);
                /*
                sprintf(str, "%f", esr_value);
                putsUart0("ESR:");
                putsUart0(str);
                putsUart0("\r\n");
                 */

                if(try_counter >= 2)
                {
                    NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                    COMP_ACINTEN_R &= ~(0x01);
                    all_pins_zero();
                    WTIMER5_TAV_R = 0;
                    state.induc = 0;
                    break;
                }

                try_counter++;

                sprintf(str, "%u", try_counter);
                putsUart0("try count:");
                putsUart0(str);
                putsUart0("\r\n");

                if(time_constant > 0 && time_constant < 10000000)
                {
                    NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                    COMP_ACINTEN_R &= ~(0x01);
                    all_pins_zero();
                    WTIMER5_TAV_R = 0;
                    state.induc = 0;
                    break;
                }
                else
                {
                    putsUart0("Trying Again \r\n");
                    continue;
                }
            }//while loop
        }//sub if statement
    }//main if statement
    else
    {
        putsUart0("Inductor Command Takes No arguments \n\r");

    }

    if(is_command("esr", 0))
    {
        if(strcmp(new_string[0], "esr") ==0 )
        {

            putsUart0("\r\n");
            putsUart0("Measuring ESR....:");
            putsUart0("\r\n");

            clearGraphicsLcd();
            setGraphicsLcdTextPosition(0, 1);
            putsGraphicsLcd("Measuring ESR...");

            esr_value = esr();

            if(esr_value < 0)
                esr_value = 0;

            sprintf(str, "%f", esr_value);
            putsUart0("ESR:");
            putsUart0(str);
            putsUart0("\r\n");

            clearGraphicsLcd();
            setGraphicsLcdTextPosition(0, 2);
            putsGraphicsLcd("ESR Value:-");
            setGraphicsLcdTextPosition(0, 3);
            putsGraphicsLcd(str);

            all_pins_zero();


        }//sub if statement
    }//main if statement
    else
    {
        putsUart0("esr Command Takes No arguments \n\r");

    }

    if(is_command("auto", 0))
    {
        if(strcmp(new_string[0], "auto") ==0 )
        {

            putsUart0("\r\n");
            putsUart0("Auto Command....:");
            putsUart0("\r\n");

            clearGraphicsLcd();
            setGraphicsLcdTextPosition(0, 1);
            putsGraphicsLcd("Auto Command...");

            auto_mode();

            all_pins_zero();


        }//sub if statement
    }//main if statement
    else
    {
        putsUart0("auto Command Takes No arguments \n\r");

    }

    //Turn off all ports
    if (strcmp(new_string[0], "off") == 0)
    {
        putsUart0("ALL PORTS OFF \r\n");
        GPIO_PORTF_DATA_R &= ~(0xFF);
        GPIO_PORTE_DATA_R &= ~(0xFF);

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 2);
        putsGraphicsLcd("PORTS OFF");
    }

    //Clear the Terminal Screen
    if (strncmp(new_string[0], "clear", 5) == 0)
    {
        putsUart0("\x1b[2J\x1b[H");          // VT 100 escape sequence to clear screen and return to Home.
        putsUart0("Screen Cleared \r\n");    // Print screen is cleared
        //reset_buffer();                      // Clear the Data Buffer
    }

}


float VAL_DUT1(void)
{
    float volt, iir_DUT1;
    uint8_t firstUpdate = 1;
    float alpha         = 0.99;

    //DUT 1 values
    raw_DUT1 = read_Adc1Ss3();

    volt = (raw_DUT1 / (ADC_RESOLUTION / SYSTEM_VOLTAGE)) - CALIB_VAL_DUT1;

    //filter DUT2 raw value
    if (firstUpdate)
    {
        iir_DUT1    = volt;
        firstUpdate = 0;
    }
    else
    {

        iir_DUT1 = iir_DUT1 * alpha + volt * (1-alpha);

    }

    return iir_DUT1;
}

float VAL_DUT2(void)
{
    float volt;
    float iir_DUT2;
    uint8_t firstUpdate = 1;
    float alpha         = 0.99;

    //DUT 2 values
    raw_DUT2 = read_Adc0Ss3();

    volt = (raw_DUT2 / (ADC_RESOLUTION / SYSTEM_VOLTAGE)) - CALIB_VAL_DUT2;

    //filter DUT2 raw value
    if (firstUpdate)
    {
        iir_DUT2 = volt;
        firstUpdate = 0;
    }
    else
    {

        iir_DUT2 = iir_DUT2 * alpha + volt * (1-alpha);

    }

    return iir_DUT2;
}


void resistor(void)
{
    //initialize
    init_LOWSIDE_R_Pin();
    init_LR_Pin();
    init_INTEGRATE_Pin();
    init_C_Pin();

    CAP = 0;                                // Making sure measure c is turned off
    LR  = 0;

    INTEGRATE = 1;                          // Enable Integrate
    LOW_R     = 1;                          // Discharge the cap

    waitMicrosecond(8000000);               // Wait for discharge

    WTIMER5_TAV_R  = 0;                     // Set Timer count to zero

    //now cap will charge

    COMP_ACINTEN_R = 0x01;                  // Enable interrupt

    LOW_R = 0;

    LR    = 1;
    COMP_ACMIS_R = 0x01;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;        // turn-on counter


}

void capacitor(void)
{
    init_C_Pin();
    init_LR_Pin();
    init_HIGHSIDE_R_Pin();
    init_LOWSIDE_R_Pin();

    LR = 0;                                 //make sure LR pin is off
    INTEGRATE = 0;

    LOW_R = 1;
    CAP = 1;
    waitMicrosecond(1000000);

    COMP_ACINTEN_R = 0x01;

    LOW_R = 0;

    WTIMER5_TAV_R  = 0;

    COMP_ACMIS_R = 0x01;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;
    HIGH_R = 1;

    waitMicrosecond(15000000);
}

void inductor(void)
{
    //initialize
    init_LOWSIDE_R_Pin();
    init_LR_Pin();
    init_C_Pin();


    CAP = 0;                                // Making sure measure_c pin is turned off when LR has to be ON
    LR  = 0;

    LOW_R     = 1;                          // turn this on
    waitMicrosecond(1000000);

    WTIMER5_TAV_R  = 0;                     // Set Timer count to zero

    //now cap will charge

    COMP_ACINTEN_R = 0x01;                  // Enable interrupt

    //LOW_R = 0;

    WTIMER5_CTL_R |= TIMER_CTL_TAEN;        // turn-on counter
    LR    = 1;

}

float esr(void)
{
    float voltage_1, voltage_2;
    float esr_val;

    init_LOWSIDE_R_Pin();
    init_LR_Pin();
    init_C_Pin();

    CAP = 0;                                // Making sure measure_c pin is turned OFF when LR has to be ON

    LOW_R = 1;
    LR = 1;

    waitMicrosecond(500000);

    voltage_2 = VAL_DUT2();
    voltage_1 = VAL_DUT1();

    esr_val = ( 33 * (voltage_1 / (voltage_2)) ) - 33;

    return esr_val;
}


void auto_mode(void)
{
    uint16_t time_cap, time_induc, time_res = 0;
    char cap_str[20];

    state.auto_mode = 1;

    clearGraphicsLcd();

    while(1)
    {
        putsUart0("\r\n");
        putsUart0("Measuring Inductance....:");
        putsUart0("\r\n");

        setGraphicsLcdTextPosition(0, 2);
        putsGraphicsLcd("Check Inductance....");

        NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
        waitMicrosecond(500000);

        state.induc = 1;

        inductor();

        waitMicrosecond(5000000);

        time_induc = time_constant;

        inductance = (time_constant - 30.127) / 1.3991;

        if(state.auto_mode && try_counter >= 1)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            state.induc = 0;
            break;
        }

        try_counter++;

        if(time_constant > 0 && time_constant < 10000000)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            state.induc = 0;
            break;
        }
        else
        {
            continue;
        }
    }//while loop

    LOW_R = 1;
    waitMicrosecond(2000000);
    all_pins_zero();
    time_constant = 0;
    try_counter = 0;
    LOW_R = 0;

    uint8_t i = 0;
    for(i = 0; i < 2; i++)
    {
        while(1)
        {
            putsUart0("\r\n");
            putsUart0("Measuring Resistance....:");
            putsUart0("\r\n");


            setGraphicsLcdTextPosition(0, 2);
            putsGraphicsLcd("Check Resistance....");

            NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
            waitMicrosecond(500000);

            resistor();

            waitMicrosecond(1000000);

            //resistance = (time_constant - 14.61) / 1.3191;

            //22 - 58
            //resistance = (time_constant - 9.3547)/1.2585;

            resistance = (time_constant - 15.564)/1.104;

            try_counter++;

            sprintf(str, "%d", try_counter);
            putsUart0("Try_count:");
            putsUart0(str);
            putsUart0("\r\n");


            if(time_constant > 0 || time_constant > 10000000 || try_counter >= 2)
            {
                NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                COMP_ACINTEN_R &= ~(0x01);
                all_pins_zero();
                WTIMER5_TAV_R = 0;
                time_res = time_constant;
                break;
            }


            if(state.auto_mode && try_counter >= 2 && time_constant > 0 )
            {
                NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                COMP_ACINTEN_R &= ~(0x01);
                all_pins_zero();
                WTIMER5_TAV_R = 0;
                time_res = time_constant;
                break;
            }
            else
            {
                continue;
            }

        }//while loop
    }

    LOW_R = 1;
    waitMicrosecond(2000000);
    all_pins_zero();
    time_constant = 0;
    try_counter = 0;

    while(1)
    {
        putsUart0("\r\n");
        putsUart0("Measuring Capacitance....:");
        putsUart0("\r\n");

        setGraphicsLcdTextPosition(0, 2);
        putsGraphicsLcd("Check Capacitance....");

        NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
        waitMicrosecond(500000);

        capacitor();

        waitMicrosecond(1000000);

        //time_cap = time_constant;

        capacitance = (time_constant - 23.761 ) / 147.238;
        capacitance /= 1000;

        if(state.auto_mode && try_counter >= 1)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            time_cap = time_constant;
            break;
        }

        try_counter++;

        if(time_constant > 0 && time_constant < 10000000)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            time_cap = time_constant;
            break;
        }
        else
        {
            continue;
        }

    }//while loop

    waitMicrosecond(1000000);
    all_pins_zero();
    time_constant = 0;
    try_counter = 0;

    sprintf(str, "%u", time_induc);
    putsUart0("Time Constant_induc:");
    putsUart0(str);
    putsUart0("\r\n");

    sprintf(str, "%d", time_res);
    putsUart0("Time Constant_res:");
    putsUart0(str);
    putsUart0("\r\n");

    sprintf(cap_str, "%u", time_cap);
    putsUart0("Time Constant_cap:");
    putsUart0(cap_str);
    putsUart0("\r\n");


    if(time_induc > time_cap && time_induc > time_res)
    {
        putsUart0("\r\n");
        putsUart0("Inductor it is \r\n");

        sprintf(str, "%.2f uH", inductance);
        putsUart0("inductance Value:");
        putsUart0(str);
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 2);
        putsGraphicsLcd("Inductor Value:-");
        setGraphicsLcdTextPosition(0, 3);
        putsGraphicsLcd(str);
    }
    else if(time_cap > time_induc && time_cap > time_res)
    {
        putsUart0("\r\n");
        putsUart0("Capacitor it is \r\n");

        sprintf(str, "%f uF", capacitance);
        putsUart0("Capacitance:");
        putsUart0(str);
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 0);
        putsGraphicsLcd("It is a Resistance");
        setGraphicsLcdTextPosition(0, 3);
        putsGraphicsLcd("Capacitor Value:-");
        setGraphicsLcdTextPosition(0, 4);
        putsGraphicsLcd(str);
    }
    else if(time_res > time_induc && time_res > time_cap)
    {
        putsUart0("\r\n");
        putsUart0("Resistor it is \r\n");

        sprintf(str, "%f Ohms", resistance);
        putsUart0("Resistance:");
        putsUart0(str);
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 0);
        putsGraphicsLcd("It is a Resistance");
        setGraphicsLcdTextPosition(0, 3);
        putsGraphicsLcd("Resistance Value:-");
        setGraphicsLcdTextPosition(0, 4);
        putsGraphicsLcd(str);
    }

}

void cap_button_specific(void)
{
    uint8_t loop_cap = 0;

    for(loop_cap = 0; loop_cap <2; loop_cap ++)
    {
        while(1)
        {
            putsUart0("\r\n");
            putsUart0("Measuring Capacitance....:");
            putsUart0("\r\n");

            clearGraphicsLcd();
            setGraphicsLcdTextPosition(0, 1);
            putsGraphicsLcd("Measuring Capacitance...");

            NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
            waitMicrosecond(500000);

            capacitor();

            waitMicrosecond(1000000);

            sprintf(str, "%d", time_constant);
            putsUart0("Time Constant_cap:");
            putsUart0(str);
            putsUart0("\r\n");

            capacitance = (time_constant - 23.761 ) / 147.238;
            capacitance /= 1000;

            sprintf(str, "%f uF", capacitance);
            putsUart0("Compacitance in uF:");
            putsUart0(str);
            putsUart0("\r\n");

            clearGraphicsLcd();
            setGraphicsLcdTextPosition(0, 2);
            putsGraphicsLcd("Capacitance Value:-");
            setGraphicsLcdTextPosition(0, 3);
            putsGraphicsLcd(str);

            if(try_counter >= 2)
            {
                NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                COMP_ACINTEN_R &= ~(0x01);
                all_pins_zero();
                WTIMER5_TAV_R = 0;
                break;
            }

            try_counter++;

            sprintf(str, "%u", try_counter);
            putsUart0("try count:");
            putsUart0(str);
            putsUart0("\r\n");


            if(time_constant > 0 && time_constant < 10000000)
            {
                NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
                COMP_ACINTEN_R &= ~(0x01);
                all_pins_zero();
                WTIMER5_TAV_R = 0;
                break;
            }
            else
            {
                putsUart0("Trying Again \r\n");
                continue;
            }

        }//while loop
    }//for loop
}

void induc_button_specific(void)
{
    while(1)
    {
        putsUart0("\r\n");
        putsUart0("Measuring Inductance....:");
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 1);
        putsGraphicsLcd("Measuring Inductance...");

        NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
        waitMicrosecond(500000);

        state.induc = 1;

        inductor();

        waitMicrosecond(5000000);

        sprintf(str, "%d", time_constant);
        putsUart0("Time Constant_induc:");
        putsUart0(str);
        putsUart0("\r\n");

        //inductance = (time_constant - 22.267) / 1.437;

        inductance = (time_constant - 30.127) / 1.3991;

        //inductance = ( log(time_constant / 182.41) ) / 0.0011;

        sprintf(str, "%.2f uH", inductance);
        putsUart0("inductance Value:");
        putsUart0(str);
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 2);
        putsGraphicsLcd("Inductor Value:-");
        setGraphicsLcdTextPosition(0, 3);
        putsGraphicsLcd(str);
        /*
        sprintf(str, "%f", esr_value);
        putsUart0("ESR:");
        putsUart0(str);
        putsUart0("\r\n");
         */

        if(try_counter >= 2)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            state.induc = 0;
            break;
        }

        try_counter++;

        sprintf(str, "%u", try_counter);
        putsUart0("try count:");
        putsUart0(str);
        putsUart0("\r\n");

        if(time_constant > 0 && time_constant < 10000000)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            state.induc = 0;
            break;
        }
        else
        {
            putsUart0("Trying Again \r\n");
            continue;
        }
    }//while loop
}

void res_button_specific(void)
{
    while(1)
    {
        putsUart0("\r\n");
        putsUart0("Measuring Resistance....:");
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 1);
        putsGraphicsLcd("Measuring Resistance...");

        NVIC_EN0_R     |= (1 << INT_COMP0 - 16);
        waitMicrosecond(500000);

        resistor();

        waitMicrosecond(1000000);

        //resistance = (time_constant - 14.61) / 1.3191;

        //22 - 58
        //resistance = (time_constant - 9.3547)/1.2585;

        resistance = (time_constant - 15.564)/1.104;

        sprintf(str, "%.2f Ohms", resistance);
        putsUart0("Resistance Value");
        putsUart0(str);
        putsUart0("\r\n");

        clearGraphicsLcd();
        setGraphicsLcdTextPosition(0, 2);
        putsGraphicsLcd("Resistance Value:-");
        setGraphicsLcdTextPosition(0, 3);
        putsGraphicsLcd(str);

        if(try_counter >= 2)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            break;
        }

        try_counter++;

        sprintf(str, "%u", try_counter);
        putsUart0("try count:");
        putsUart0(str);
        putsUart0("\r\n");


        if(resistance > 0 || time_constant > 10000000)
        {
            NVIC_EN0_R     &= ~(1 << INT_COMP0 - 16);
            COMP_ACINTEN_R &= ~(0x01);
            all_pins_zero();
            WTIMER5_TAV_R = 0;
            break;
        }
        else
        {
            putsUart0("Trying Again \r\n");
            continue;
        }

    }//while loop

}


void system_reset(void)
{
    if(strcmp(new_string[0], "reset") == 0)
    {
        putsUart0("Reset \r\n");

        waitMicrosecond(1000000);

        NVIC_APINT_R = 0x04 | (0x05FA << 16);
        /*
            __asm("    .global _c_int00\n"              //entry point to the main program reset
                  "    b.w     _c_int00");
         */
    }

}


//*****************************************************************************//
//                                                                             //
//                  BUFFER CLEAR AND RESRET FUNCTIONS                          //
//                                                                             //
//*****************************************************************************//

// Function that resets the new_string array
void reset_new_string(void)
{
    uint8_t i = 0;
    uint8_t j = 0;

    for (i = 0; i < MAX_ARGS; i++)
    {
        for (j = 0; j < MAX_SIZE; j++)
        {
            new_string[i][j] = '\0';
        }
    }

}

// Function of reseting the input buffers and variables
void reset_buffer(void)
{
    uint8_t i = 0;

    for (i = 0; i < MAX_SIZE; i++)
    {
        string[i] = '\0';
        a[i] = '\0';
        n[i] = '\0';
        s[i] = '\0';
        buff_int[i] = '\0';
    }

    args_updated = 0;
    args_no      = 0;
    args_str     = 0;
    time_constant = 0;
    try_counter = 0;

    reset_new_string();

}


//*****************************************************************************//
//                                                                             //
//                      MAIN FUNCTION ROUTINE                                  //
//                                                                             //
//*****************************************************************************//

int main(void)
{
    //***************** Setup ************************//

#ifdef TEST
    uint8_t i = 0;

#endif

    initHw();           // Initialize Hardware

    initGraphicsLcd();  // Init LCD

    all_pins_zero();    // Set all pins to low

    flash_led();        // Flash LED for 500ms

    clear_screen();

    putsUart0("\r\n");
    putsUart0("\033[37;1mIntializing.....\033[0m\r\n");

    project_info_lcd();
    user_lcd_cmd_info();

    clear_screen();     // Clear Terminal Screen

    project_info();     // Display Project Info


    //waitPbPress();    // Wait for PB press

    START:
    //**************** Event Loop ******************//
    while (1)
    {
        if(!state.term)
        {

            while(PUSH_BUTTON && AUTO_BUTTON && CAP_BUTTON && INDUC_BUTTON && RES_BUTTON);

            if(!AUTO_BUTTON)
                auto_mode();

            else if(!CAP_BUTTON)
                cap_button_specific();

            else if(!INDUC_BUTTON)
                induc_button_specific();

            else if(!RES_BUTTON)
                res_button_specific();

            else if(!PUSH_BUTTON)
            {
                putsUart0("Entering Terminal Mode\r\n");

                clearGraphicsLcd();
                setGraphicsLcdTextPosition(0, 3);
                putsGraphicsLcd("!! TERMINAL MODE !!");

                waitMicrosecond(2000000);
                goto TERMINAL;
            }

            //reset
            goto START;
        }

        TERMINAL:

        state.term = 1;

        // Get Input from User
        putsUart0("\r\n");
        putsUart0("\033[37;1mInput:-\033[0m\r\n");          //Print Input to let user know to start typing, foreground color:Yellow

        term_getsUart0();

        //test_getsUART0();

        // Parse the string received from user
        parse_string();

#ifdef TEST
        putsUart0("\r\n");
        putsUart0("String tokenized to words:-\r\n");

        for (i = 0; i < args_updated; i++)
        {
            putsUart0(new_string[i]);
            putsUart0("\r\n");
        }

        putsUart0("\r\n");
        sprintf(buff_int, "%d", args_updated);
        putsUart0("Number of Words  :");
        putsUart0(buff_int);
        putsUart0("\r\n");

        sprintf(buff_int, "%d", args_updated - 1);
        putsUart0("Command Arguments:");
        putsUart0(buff_int);
        putsUart0("\r\n");
#endif

        // Get Test commands for LED
        test_commands();

        // Get Project Commands from user for LCR Pins
        project_commands();

        // Reset function call check
        system_reset();

        //project functions


        // Clear the buffer before the loop ends
        reset_buffer();

    }

    return 0;
}

