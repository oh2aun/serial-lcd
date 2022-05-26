/* 
 * File:   serial-lcd.c
 * Author: mikael
 * PIC16F18426
 * Created on May 23, 2022, 11:40 PM
 */

/*
MIT License

Copyright (c) 2022 Mikael Nordman

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = OFF      // Master Clear Enable bit (MCLR pin function is port defined function)
#pragma config PWRTS = PWRT_64  // Power-up Timer Enable bit (PWRT set at 64 ms)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) is set to 2.7V)
#pragma config ZCDDIS = OFF     // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTD = OFF       // Data EEPROM write protection bit (Data EEPROM NOT write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
/*
 * 
 */

// PIC16F18426
// Hardware port definitions
// EUSART RX defaults to RC5

#define     RS              PORTAbits.RA5 // PIN2
#define     R_W             PORTAbits.RA4 // PIN3 
#define     E               PORTCbits.RC3 // PIN7

#define     D4              PORTAbits.RA2 // PIN11
#define     D5              PORTCbits.RC0 // PIN10
#define     D6              PORTCbits.RC1 // PIN9
#define     D7              PORTCbits.RC2 // PIN8

// LCD commands

#define     CLR_DISP        0b00000001 //Clear display


#define     CUR_HOME        0b00000010    //Move cursor home and clear screen memory
#define     CUR_RIGHT       0b00010100    //Move cursor one to right
#define     CUR_LEFT        0b00010000    //Move cursor one to left


#define     SCROLL_RIGHT    0b00011100    //Scroll entire screen right one space
#define     SCROLL_LEFT     0b00011000    //Scroll entire screen left one space


#define     DISP_ON         0b00001100    //Turn visible LCD on
#define     DISP_OFF        0b00001000    //Turn visible LCD off


#define     UNDERLINE_ON    0b00001110    //Turn on underlined cursor
#define     UNDERLINE_OFF   0b00001100    //Turn off underlined cursor


#define     BLINKCUR_ON     0b00001101    //Turn on blinking box cursor
#define     BLINKCUR_OFF    0b00001100    //Turn off blinking box cursor


#define     DUALCUR_ON      0b00001111    //Turn on blinking box and underline cursor
#define     DUALCUR_OFF     0b00001100    //Turn off blinking box and underine cursor


#define     SET_CURSOR      0b10000000    //SET_CURSOR + X : Sets cursor position to X


#define     ENTRY_INC       0b00000110 //
#define     DD_RAM_ADDR     0b10000000 //
#define     DD_RAM_ADDR2    0b11000000 //
#define     CG_RAM_ADDR     0b01000000 //

#define _XTAL_FREQ 1000000

void LCD_wait(void);
void send_char(char c);
void send_cmd(char d);
void init_lcd(void);

void init() {
    SP1BRG = 25;        // 9615 baud @ 1 MHz
    BAUD1CON = 0x08;    // 16bit baudgenerator
    TX1STA =0x04;
    TRISC = 0x20;       // RC5 is the USART RX 
    TRISA = 0x00;
    PMD0 = 0x47;        // FVR disabled
    PMD1 = 0xff;
    PMD2 = 0xff;
    PMD3 = 0xff;
    PMD4 = 0xff;
    PMD5 = 0xff;
    PMD6 = 0x03;         // EUSART enabled
    PMD7 = 0xff;
    ANSELA = 0;
    ANSELC = 0;
    
    init_lcd();
}

unsigned char getch(void) {
    while(1) {
        if (PIR3bits.RC1IF) {
            if (RC1STAbits.OERR) {
                RC1STAbits.CREN = 0;
                RC1STAbits.CREN = 1;                
            }
            return RC1REG;
        }
    }
}

void send_string(const char *s) {
    while (*s) {
        send_char(*s++);
    }
}

void info() {
    send_cmd(0x80+0); // first line first column
    send_string("      OH2AUN");
    send_cmd(0x80+20); // third line first column
    send_string("    Serial LCD");
    __delay_ms(2000);
    send_cmd(CLR_DISP);
}

int main(int argc, char** argv) {
    char rxchar;
    init();
    info();
    RC1STA = 0x90;  
    while(1) {
        rxchar = getch();
        switch(rxchar) {
            case 0xfe:
                rxchar = getch();
                if (rxchar != 0x30) { // ignore 'set 8 bit mode' command
                    send_cmd(rxchar);
                }
                break;
            default:
                send_char(rxchar);
        }
    }
    return (EXIT_SUCCESS);
}

//Checks the busy flag and waits until LCD is ready for next command
void LCD_wait(void)
{
    char i;
    
    TRISC = 0x27; //0 = Output, 1 = Input
    TRISA = 0x04;

    R_W = 1;    // Tell LCD to output status
    RS = 0;     // Command mode          

    i = 1;
    while(i)
    {
        E = 1; 
        i = D7; //Read data bit 7 - Busy Flag
        E = 0;
    
        E = 1;
        E = 0; //Toggle E to get the second four bits of the status byte - but we don't care
    }
    
    TRISC = 0x20;
    TRISA = 0x00;
}


//Sends an ASCII character to the LCD
void send_char(char byte)
{
    LCD_wait();

    RS = 1;  //set LCD to data mode
    R_W = 0; //set LCD to write mode
    
    D4 = 0; if (byte&0x10) D4 = 1;
    D5 = 0; if (byte&0x20) D5 = 1;
    D6 = 0; if (byte&0x40) D6 = 1;
    D7 = 0; if (byte&0x80) D7 = 1;
    E = 1; E = 0;
    
    D4 = 0; if (byte&0x01) D4 = 1;
    D5 = 0; if (byte&0x02) D5 = 1;
    D6 = 0; if (byte&0x04) D6 = 1;
    D7 = 0; if (byte&0x08) D7 = 1;
    E = 1; E = 0; //Toggle the Enable Pin
}


//Sends an LCD command
void send_cmd(char d)
{
    LCD_wait();
    
    RS = 0;  //set LCD to command mode        
    R_W = 0; //set LCD to write mode

    if (d&0x10) D4 = 1; else D4 = 0;
    if (d&0x20) D5 = 1; else D5 = 0;
    if (d&0x40) D6 = 1; else D6 = 0;
    if (d&0x80) D7 = 1; else D7 = 0;
    E = 1; E = 0;

    if (d&0x01) D4 = 1; else D4 = 0;
    if (d&0x02) D5 = 1; else D5 = 0;
    if (d&0x04) D6 = 1; else D6 = 0;
    if (d&0x08) D7 = 1; else D7 = 0;
    E = 1; E = 0;
}
//Initializes the 4-bit parallel interface to the HD44780
void init_lcd(void)
{
    //Wait for LCD busy bit to clear
    LCD_wait();
    
    RS = 0;               
    R_W = 0;

    //Tell the LCD we are using 4bit data communication
    //===============================================================
    __delay_ms(100);
    D4 = 1; D5 = 1;
    E = 1; E = 0;

    __delay_ms(50);
    E = 1; E = 0;

    __delay_ms(10);
    E = 1; E = 0;

    __delay_ms(10);
    D4 = 0;
    E = 1; E = 0;

    send_cmd(DISP_ON);
    send_cmd(CLR_DISP);
}
