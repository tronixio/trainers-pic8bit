// Configuration Registers.
#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF
#pragma config BOREN = OFF, CLKOUTEN = ON, IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PPS1WAY = ON, ZCD = OFF, PLLEN = OFF
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000
// PIC16F1778 - Compile with XC8(v2.40).
// PIC16F1778 - @8MHz Internal Oscillator.

// DRAFT Do not use.

// Rotary encoder code from:
// https://www.mikrocontroller.net/articles/Drehgeber

// PIC8-Bit Mini Trainer.
// 2x ADC CHANNELS.
// 1x LCD NHD-C0220BiZ - ST7036.
// 1x EUSART RX/TX ASYNCHRONOUS.
// 1x ROTARY ENCODER with SWITCH.
// 2x SWITCHS.

// JUMPER.URX - Close.
// JUMPER.UTX - Close.
// JUMPER.SDA - Not Use.
// JUMPER.SCL - Not Use.
// JUMPER.VCAP - Open.
// JUMPER.BCKL - Open.

// Pinout.
// MCU.RA0 <- ANALOG.AN1.
// MCU.RA1 <- ANALOG.AN2.
// MCU.RA6 -> OSCILLOSCOPE.PROBE.A.
// MCU.RB0 <- SWITCH.S1.
// MCU.RB1 <- SWITCH.S2.
// MCU.RB2 <- ROTARY.A.
// MCU.RB3 <- ROTARY.B.
// MCU.RB4 <- ROTARY.S3.

// Definitions.
// ADC.
#define ADC_TRIGGER           5
// EUSART.
#define BAUDRATE              9600
#define BAUDRATE_GENERATOR    ((_XTAL_FREQ/BAUDRATE/4)-1)
// ASCII Characters.
#define ASCII_CR              0x0D
// Rotary Encoder.
#define ROTARY_PHASE_A        PORTBbits.RB2
#define ROTARY_PHASE_B        PORTBbits.RB3
#define ROTARY_SWITCH         PORTBbits.RB4
// Switchs.
#define SWITCH_S1             PORTBbits.RB0
#define SWITCH_S2             PORTBbits.RB1

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
int8_t rotary_i8encoderFilter(void);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings.
const uint8_t au8Tronix[] = "\r\n\r\nTronix I/O";
const uint8_t au8WWW[] = "\r\nhttps://www.tronix.io/\r\n";
const uint8_t au8Ready[] = "\r\nREADY> ";
const uint8_t au8Adc0[] = "\r\nADC CHANNEL 0> ";
const uint8_t au8Adc1[] = "\r\nADC CHANNEL 1> ";
const uint8_t au8Encoder[] = "\r\nROTARY ENCODER> ";
const uint8_t au8Encodersw[] = "\r\nROTARY ENCODER SWITCH> ";
const uint8_t au8Eusart[] = "\r\nEUSART ECHO> ";
const uint8_t au8Switch1[] = "\r\nSWITCH 1> ";
const uint8_t au8Switch2[] = "\r\nSWITCH 2> ";
const uint8_t au8Pressed[] = "PRESSED";
const uint8_t au8Released[] = "RELEASED";

const uint8_t au8encoder[16] = {0, 1, 255, 0, 255, 0, 0, 1, 1, 0, 0, 255, 0, 255, 1, 0};

// Global Variables.
int8_t i8encoderDelta;
uint16_t u16adcTimer;

// Interrupts Service Routines.
void __interrupt() ISR(void)
{
    if(INTCONbits.TMR0IF){
        static int8_t u8encoderLast = 0;
        u8encoderLast = (u8encoderLast<<2) & 0x0F;
        if(ROTARY_PHASE_A) u8encoderLast |= 0b01; // CW.
        if(ROTARY_PHASE_B) u8encoderLast |= 0b10; // CCW.
        i8encoderDelta += au8encoder[u8encoderLast];
        INTCONbits.TMR0IF = 0b0;
    }
    u16adcTimer++;
}

// Main.
void main(void)
{
    // MCU Initialization.
    // Internal Oscillator Settings.
    OSCTUNE = 0b00000000;
    OSCCON = 0x70;
    while(!OSCSTATbits.HFIOFR){};
    // Ports Settings.
    // PORT Data Register.
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    // TRIS Data Direction.
    TRISA = 0b00000011;
    TRISB = 0b00011111;
    TRISC = 0b10000000;
    // WPU Disable.
    OPTION_REGbits.nWPUEN = 0b1;
    // LATCH Outputs.
    LATA = 0b00000000;
    LATB = 0b00000000;
    LATC = 0b00000000;
    // ANSEL Analog.
    ANSELA = 0b00000011;
    ANSELB = 0b00000000;
    ANSELC = 0b00000000;
    // WPU Weak Pull-up.
    WPUA = 0b00000000;
    WPUB = 0b00000000;
    WPUC = 0b00000000;
    // ODCON Open-drain.
    ODCONA = 0b00000000;
    ODCONB = 0b00000000;
    ODCONC = 0b00000000;
    // SRLCON Slew Rate.
    SLRCONA = 0b11111111;
    SLRCONB = 0b11111111;
    SLRCONC = 0b11111111;
    // INLVL Input Level.
    INLVLA  = 0b00000000;
    INLVLB  = 0b00000000;
    INLVLC  = 0b00000000;
    // HIDRVB High Drive.
    HIDRVB  = 0b00000000;
    // PPS Settings.
    // PPS Unlock Sequence.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b0;
    // PPS Inputs.
    RXPPSbits.RXPPS = 0x17;    // RC7 - EUSART.URX.
    // PPS Outputs.
    RC6PPSbits.RC6PPS = 0x24;  // RC6 - EUSART.UTX.
    // PPS Lock Sequence.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b1;

    // ADC Settings.
    ADRESL = 0;
    ADRESH = 0;
    ADCON0 = 0x00;
    ADCON1 = 0xD0;
    ADCON2 = 0x00;
    // ADC Enable.
    ADCON0bits.ADON = 0b1;

    // EUSART Settings.
    RC1REG = 0;
    TX1REG = 0;
    SP1BRG = BAUDRATE_GENERATOR;
    RC1STA = 0x10;
    TX1STA = 0x24;
    BAUD1CON = 0x08;
    // EUSART Enable.
    RC1STAbits.SPEN = 0b1;

    // Timer0 Settings.
    // ~500Hz @8MHz.
    OPTION_REGbits.PS = 0b010;
    OPTION_REGbits.PSA = 0b0;
    OPTION_REGbits.TMR0CS = 0b0;
    // Timer0 Interrupts Enable.
    INTCONbits.TMR0IE = 0b1;
    INTCONbits.TMR0IF = 0b0;

    // Display Strings.
    eusart_writeString(au8Tronix);
    eusart_writeString(au8WWW);
    eusart_writeString(au8Ready);

    // Interrupts Enable.
    INTCONbits.GIE = 0b1;

    uint8_t u8Rx;
    uint8_t au8Buffer[6];
    uint16_t u16ADCRead, u16ADC0Last, u16ADC1Last;
    uint8_t u8encoderSwitchPressed;
    uint8_t u8encoderRead, u8encoderLast;
    uint8_t u8switchS1Pressed, u8switchS2Pressed;
    while(1){
        // ADC Read every ~1s.
        if(u16adcTimer>1000){
            ADCON0bits.CHS = 0b00000;
            __delay_us(5);
            ADCON0bits.GO = 0b1;
            while(ADCON0bits.GO){};
            u16ADCRead = (uint16_t)(ADRESH<<8) + ADRESL;
            if(u16ADC0Last < (u16ADCRead - ADC_TRIGGER) || u16ADC0Last > (u16ADCRead + ADC_TRIGGER)){
                u16toa(u16ADCRead, au8Buffer, 10);
                eusart_writeString(au8Adc0);
                eusart_writeString(au8Buffer);
            }
            u16ADC0Last = u16ADCRead;
            ADCON0bits.CHS = 0b00001;
            __delay_us(5);
            ADCON0bits.GO = 0b1;
            while(ADCON0bits.GO){};
            u16ADCRead = (uint16_t)(ADRESH<<8) + ADRESL;
            if(u16ADC1Last < (u16ADCRead - ADC_TRIGGER) || u16ADC1Last > (u16ADCRead + ADC_TRIGGER)){
                u16toa(u16ADCRead, au8Buffer, 10);
                eusart_writeString(au8Adc1);
                eusart_writeString(au8Buffer);
            }
            u16ADC1Last = u16ADCRead;
            u16adcTimer = 0;
        }

        // EUSART.
        if(PIR1bits.RCIF){
            u8Rx = eusart_readCharacter();
            eusart_writeString(au8Eusart);
            eusart_writeCharacter(u8Rx);
            if(u8Rx == ASCII_CR)
                eusart_writeString(au8Ready);
        }

        // ROTARY ENCODER.
        if(!ROTARY_SWITCH){
            __delay_ms(100);
            u8encoderSwitchPressed = 1;
            eusart_writeString(au8Encodersw);
            eusart_writeString(au8Pressed);
            while(!ROTARY_SWITCH){};
        }else if(ROTARY_SWITCH){
            if(u8encoderSwitchPressed){
                u8encoderSwitchPressed = 0;
                eusart_writeString(au8Encodersw);
                eusart_writeString(au8Released);
                u8encoderRead = 0;
            }
        }

        u8encoderRead += rotary_i8encoderFilter();
        if(u8encoderLast != u8encoderRead){
            u16toa(u8encoderRead, au8Buffer, 10);
            eusart_writeString(au8Encoder);
            eusart_writeString(au8Buffer);
            u8encoderLast = u8encoderRead;
        }

        // SWITCHS.
        if(!SWITCH_S1){
            __delay_ms(100);
            u8switchS1Pressed = 1;
            eusart_writeString(au8Switch1);
            eusart_writeString(au8Pressed);
        }else if(SWITCH_S1){
            if(u8switchS1Pressed){
                eusart_writeString(au8Switch1);
                eusart_writeString(au8Released);
                u8switchS1Pressed = 0;
            }
        }
        if(!SWITCH_S2){
            __delay_ms(100);
            u8switchS2Pressed = 1;
            eusart_writeString(au8Switch2);
            eusart_writeString(au8Pressed);
        }else if(SWITCH_S2){
            if(u8switchS2Pressed){
                eusart_writeString(au8Switch2);
                eusart_writeString(au8Released);
                u8switchS2Pressed = 0;
            }
        }
    }
}

// Functions.
uint8_t eusart_readCharacter(void)
{
    if(RC1STAbits.OERR){
        RC1STAbits.CREN = 0b0;
        RC1STAbits.CREN = 0b1;
    }

    while(!PIR1bits.RCIF){};
    return(RC1REG);
}

void eusart_writeCharacter(uint8_t u8Data)
{
    while(!PIR1bits.TXIF){};
    TX1REG = u8Data;
}

void eusart_writeString(const uint8_t * u8Data)
{
    while(*u8Data != '\0')
        eusart_writeCharacter(*u8Data++);
}

int8_t rotary_i8encoderFilter(void)
{
    int8_t i8encoderFilter;

    INTCONbits.TMR0IE = 0b0;
    i8encoderFilter = i8encoderDelta;
    i8encoderDelta = i8encoderFilter & 3;
    INTCONbits.TMR0IE = 0b1;

    return(i8encoderFilter>>2);
}

void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base)
{
    uint8_t u8Buffer;
    uint16_t data = u16Data;

    while(data != '\0'){
        data /= u8Base;
        au8Buffer++;
    }
    *au8Buffer-- = 0;

    while(u16Data != '\0'){
        u8Buffer = (uint8_t)(u16Data % u8Base);
        u16Data /= u8Base;
        if(u8Buffer >= 10)
            u8Buffer += 'A' - '0' - 10;
        u8Buffer += '0';
        *au8Buffer-- = u8Buffer;
    }
}
