# PIC8-Bit Mini Trainer.

- TODO - faire la difference avec le  langage

## 0.Contents.

- [1.DRAFT - EUSART - ADC - SWITCHS - ROTARY ENCODER.](#1draft---eusart---adc---switchs---rotary-encoder)
- [2.DRAFT - LCD - ADC - SWITCHS - ROTARY ENCODER.](#2draft---lcd---adc---switchs---rotary-encoder)
- [3.TODO - EUSART - ADC - SWITCHS - ROTARY ENCODER.](#3todo---eusart)
- [4.DRAFT - LCD.](#4draft---lcd)

## 1.DRAFT - EUSART - ADC - SWITCHS - ROTARY ENCODER.

```c
// Configuration Registers.
#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF
#pragma config BOREN = OFF, CLKOUTEN = ON, IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PPS1WAY = ON, ZCD = OFF, PLLEN = OFF
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000
// PIC16F1778 - Compile with XC8(v2.32).
// PIC16F1778 - @8MHz Internal Oscillator.
// v0.2 - 12/2021.

// Rotary encoder code from:
// https://www.mikrocontroller.net/articles/Drehgeber

// PIC8-Bit Mini Trainer.
// 2x ADC CHANNELS.
// 1x EUSART RX/TX ASYNCHRONOUS.
// 1x ROTARY ENCODER with SWITCH.
// 2x SWITCHS.

// JUMPER.URX - Close.
// JUMPER.UTX - Close.
// JUMPER.SDA - Open.
// JUMPER.SCL - Open.
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
#define ADC_TRIGGER             5
// EUSART.
#define BAUDRATE                9600
#define BAUDRATE_GENERATOR      ((_XTAL_FREQ/BAUDRATE/4)-1)
// ASCII Characters.
#define ASCII_CR                0x0D
// Rotary Encoder.
#define ROTARY_ENCODER_A        PORTBbits.RB2
#define ROTARY_ENCODER_B        PORTBbits.RB3
#define ROTARY_ENCODER_SWITCH   PORTBbits.RB4
// Switchs.
#define SWITCH_S1               PORTBbits.RB0
#define SWITCH_S2               PORTBbits.RB1

// Function Prototypes.
uint8_t eusart_readCharacter(void);
void eusart_writeCharacter(uint8_t u8Data);
void eusart_writeString(const uint8_t * u8Data);
int8_t rotary_u8encoderRead(void);
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

// Global Variables.
int8_t i8EncoderDelta;
uint16_t u16AdcTimer;
const int8_t i8EncoderFull[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// Interrupts Service Routines.
void __interrupt() ISR(void)
{
    if(INTCONbits.TMR0IF){
        static uint8_t u8encoderLast = 0;
        u8encoderLast = (u8encoderLast<<2) & 0x0F;
        if(ROTARY_ENCODER_A) u8encoderLast |= 1;
        if(ROTARY_ENCODER_B) u8encoderLast |= 2;
        i8EncoderDelta += i8EncoderFull[u8encoderLast];
        INTCONbits.TMR0IF = 0b0;
    }
    u16AdcTimer++;
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
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b0;
    // PPS Inputs.
    RXPPSbits.RXPPS = 0x17;    // RC7 - EUSART.URX.
    // PPS Outputs.
    RC6PPSbits.RC6PPS = 0x24;  // RC6 - EUSART.UTX.
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
    uint16_t u8encoderRead=0, u8encoderLast;
    uint8_t u8switchS1Pressed, u8switchS2Pressed;
    while(1){
        // ADC Read every ~1s.
        if(u16AdcTimer>1000){
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
            u16AdcTimer = 0;
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
        if(!ROTARY_ENCODER_SWITCH){
            __delay_ms(100);
            u8encoderSwitchPressed = 1;
            eusart_writeString(au8Encodersw);
            eusart_writeString(au8Pressed);
            while(!ROTARY_ENCODER_SWITCH){};
        }else if(ROTARY_ENCODER_SWITCH){
            if(u8encoderSwitchPressed){
                u8encoderSwitchPressed = 0;
                eusart_writeString(au8Encodersw);
                eusart_writeString(au8Released);
                u8encoderRead = 0;
            }
        }

        u8encoderRead += rotary_u8encoderRead();
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

int8_t rotary_u8encoderRead(void)
{
    int8_t u8encoderRead;

    INTCONbits.TMR0IE = 0b0;
    u8encoderRead = i8EncoderDelta;
    i8EncoderDelta = u8encoderRead & 3;
    INTCONbits.TMR0IE = 0b1;

    return(u8encoderRead>>2);
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
```

## 2.DRAFT - LCD - ADC - SWITCHS - ROTARY ENCODER.

```c
// Configuration Registers.
#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF
#pragma config BOREN = OFF, CLKOUTEN = ON, IESO = OFF, FCMEN = OFF
#pragma config WRT = OFF, PPS1WAY = ON, ZCD = OFF, PLLEN = OFF
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000
// PIC16F1778 - Compile with XC8(v2.36).
// PIC16F1778 - @8MHz Internal Oscillator.

// Rotary encoder code from:
// https://www.mikrocontroller.net/articles/Drehgeber

// todo : toggle rotatry / backlight
// todo : limit 0 to 32
// todo : filtrer ADC

// PIC8-Bit Mini Trainer.
// 2x ADC CHANNELS.
// 1x LCD NHD-C0220BiZ - ST7036.
// 1x ROTARY ENCODER with SWITCH.
// 2x SWITCHS.

// Jumpers.
// URX - Open.
// UTX - Open.
// SDA - Close.
// SCL - Close.
// VCAP - Open.
// BCKL - Close.

// Pinout.
// MCU.RA0 <- ANALOG.AN1.
// MCU.RA1 <- ANALOG.AN2.
// MCU.RA6 -> OSCILLOSCOPE.PROBE.A.
// MCU.RB0 <- SWITCH.S1.
// MCU.RB1 <- SWITCH.S2.
// MCU.RB2 <- ROTARY.A.
// MCU.RB3 <- ROTARY.B.
// MCU.RB4 <- ROTARY.S3.
// MCU.RB5 -> LCD.BACKLIGHT.EN.

// Definitions.
// I2C.
#define I2C_READ                                           0b1
#define I2C_WRITE                                          0b0
#define I2C_FSCL_HZ                                        200000
#define I2C_BAUDRATE                                       ((_XTAL_FREQ/(4*I2C_FSCL_HZ))-1)
// CAT4002A.
#define CAT4002_DELAY_HIGH_US                              1                               
#define CAT4002_DELAY_LED_US                               10
#define CAT4002_DELAY_LOW_US                               5
#define CAT4002_DELAY_DOWN_MS                              3
// ST7036 I2C Address.
#define ST7036_I2C_ADDRESS_78                              0x78
#define ST7036_I2C_CONTROL_CONTINUOUS_COMMAND              0x00
#define ST7036_I2C_CONTROL_CONTINUOUS_DATA                 0x40
#define ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND           0x80
#define ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA              0xC0
// ST7036 Instruction Table IS2=0, IS1=0.
#define ST7036_CLEAR_DISPLAY                               0x01
#define ST7036_RETURN_HOME                                 0x02
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_NOSHIFT      0x04
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_SHIFT_RIGHT  0x05
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT      0x06
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_SHIFT_LEFT   0x07
#define ST7036_DISPLAY_OFF                                 0x08
#define ST7036_DISPLAY_ON_CURSOR_OFF                       0x0C
#define ST7036_DISPLAY_ON_CURSOR_ON_NOBLINK                0x0E
#define ST7036_DISPLAY_ON_CURSOR_ON_BLINK                  0x0F
#define ST7036_DISPLAY_CURSOR_SHIFT_LEFT                   0x10
#define ST7036_DISPLAY_CURSOR_SHIFT_RIGHT                  0x14
#define ST7036_DISPLAY_DISPLAY_SHIFT_LEFT                  0x18
#define ST7036_DISPLAY_DISPLAY_SHIFT_RIGH                  0x1C
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8         0x20
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS1     0x21
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS2     0x22
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_FONT5x8         0x28
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_DHFONT5x8       0x24
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_DHFONT5x8       0x2C
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8         0x30
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1     0x31
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS2     0x32
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_DHFONT5x8       0x34
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8         0x38
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_DHFONT5x8       0x3C
#define ST7036_SET_ICON_RAM_ADDRESS                        0X40
#define ST7036_SET_CGRAM_ADDRESS                           0x40
#define ST7036_DDRAM_ADDRESS_FIRST_LINE                    0x80
#define ST7036_DDRAM_ADDRESS_SECOND_LINE                   0xC0
// ST7036 Instruction Table IS2=0, IS1=1.
#define ST7036_BIAS_SET_1_5                                0x14
#define ST7036_BIAS_SET_1_5_3_LINE                         0x15
#define ST7036_BIAS_SET_1_4                                0x1C
#define ST7036_BIAS_SET_1_4_3_LINE                         0x1D
#define ST7036_POWER_ICON_OFF_BOOST_OFF_NO_CONTRAST        0x50
#define ST7036_POWER_ICON_ON_BOOST_OFF_NO_CONTRAST         0x58
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_0      0x54
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_1      0x55
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_2      0x56
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_3      0x57
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_0       0x5C
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1       0x5D
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_2       0x5E
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_3       0x5F
#define ST7036_FOLLOWER_CONTROL_OFF                        0x60
#define ST7036_FOLLOWER_CONTROL_ON_RAB_0                   0x68
#define ST7036_FOLLOWER_CONTROL_ON_RAB_1                   0x69
#define ST7036_FOLLOWER_CONTROL_ON_RAB_2                   0x6A
#define ST7036_FOLLOWER_CONTROL_ON_RAB_3                   0x6B
#define ST7036_FOLLOWER_CONTROL_ON_RAB_4                   0x6C
#define ST7036_FOLLOWER_CONTROL_ON_RAB_5                   0x6D
#define ST7036_FOLLOWER_CONTROL_ON_RAB_6                   0x6E
#define ST7036_FOLLOWER_CONTROL_ON_RAB_7                   0x6F
#define ST7036_CONTRAST_LSB_0                              0x70
#define ST7036_CONTRAST_LSB_1                              0x71
#define ST7036_CONTRAST_LSB_2                              0x72
#define ST7036_CONTRAST_LSB_3                              0x73
#define ST7036_CONTRAST_LSB_4                              0x74
#define ST7036_CONTRAST_LSB_5                              0x75
#define ST7036_CONTRAST_LSB_6                              0x76
#define ST7036_CONTRAST_LSB_7                              0x77
#define ST7036_CONTRAST_LSB_8                              0x78
#define ST7036_CONTRAST_LSB_9                              0x79
#define ST7036_CONTRAST_LSB_10                             0x7A
#define ST7036_CONTRAST_LSB_11                             0x7B
#define ST7036_CONTRAST_LSB_12                             0x7C
#define ST7036_CONTRAST_LSB_13                             0x7D
#define ST7036_CONTRAST_LSB_14                             0x7E
#define ST7036_CONTRAST_LSB_15                             0x7F
// ST7036 Instruction Table IS2=1, IS1=0.
#define ST7036_DOUBLE_HEIGHT_FONT_COM9_COM24               0x10
#define ST7036_DOUBLE_HEIGHT_FONT_COM1_COM16               0x11
// ST7036 Delays.
#define ST7036_CLEAR_DISPLAY_DELAY_MS                      2
#define ST7036_INITIALIZATION_DELAY_MS                     50
// NHD-C0220BiZ Configuration.
#define C0220BiZ_CONFIGURATION_I2C_ADDRESS                 ST7036_I2C_ADDRESS_78
#define C0220BiZ_CONFIGURATION_FIRST_LINE                  ST7036_DDRAM_ADDRESS_FIRST_LINE
#define C0220BiZ_CONFIGURATION_SECOND_LINE                 ST7036_DDRAM_ADDRESS_SECOND_LINE
#define C0220BiZ_CONFIGURATION_CHARACTERS                  20
// ASCII Characters.
#define ASCII_SPACE                                        0x20
// Patterns.
#define PATTERN_BATTERY_FULL                               0x04
#define PATTERN_BATTERY_3_5                                0x03
#define PATTERN_BATTERY_2_5                                0x02
#define PATTERN_BATTERY_1_5                                0x01
#define PATTERN_BATTERY_EMPTY                              0x00
// LCD.
#define LCD_BACKLIGHT_OFF                                  LATBbits.LATB5 = 0b0
#define LCD_BACKLIGHT_ON                                   LATBbits.LATB5 = 0b1
// Rotary Encoder.
#define ROTARY_ENCODER_A                                   PORTBbits.RB2
#define ROTARY_ENCODER_B                                   PORTBbits.RB3
#define ROTARY_ENCODER_SWITCH                              PORTBbits.RB4
// Switchs.
#define SWITCH_S1                                          PORTBbits.RB0
#define SWITCH_S2                                          PORTBbits.RB1

// Function Prototypes.
void i2c_restart(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t u8Data);
void lcd_clearDisplay(void);
void lcd_clearLine(uint8_t u8Line);
void lcd_initialize(void);
void lcd_setBacklight(uint8_t u8Backlight);
void lcd_setCursor(uint8_t u8Cursor);
void lcd_writeCharacter(uint8_t u8Data);
void lcd_writeInstruction(uint8_t u8Data);
void lcd_writeString(const uint8_t * u8Data);
void lcd_writeStringSetCursor(const uint8_t * u8Data, uint8_t u8Cursor);
int8_t rotary_u8encoderRead(void);
void u16toa(uint16_t u16Data, uint8_t * au8Buffer, uint8_t u8Base);

// Strings & Custom Patterns.
const uint8_t au8Tronix[] = "Tronix I/O";
const uint8_t au8WWW[] = "www.tronix.io";
const uint8_t au8Adc0[] = "ADC CHANNEL 0> ";
const uint8_t au8Adc1[] = "ADC CHANNEL 1> ";
const uint8_t au8Encoder[] = "ROTARY> ";
const uint8_t au8Encodersw[] = "ROTARY SW> ";
const uint8_t au8Switch1[] = "SWITCH 1> ";
const uint8_t au8Switch2[] = "SWITCH 2> ";
const uint8_t au8Pressed[] = "PRESSED";
const uint8_t au8Released[] = "RELEASED";

const uint8_t au8BatteryPattern[5][8] = {
    {0x0e, 0x1b, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f},
    {0x0e, 0x1b, 0x11, 0x11, 0x11, 0x1f, 0x1f, 0x1f},
    {0x0e, 0x1b, 0x11, 0x11, 0x1f, 0x1f, 0x1f, 0x1f},
    {0x0e, 0x1b, 0x11, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f},
    {0x0e, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f}
};

// Global Variables.
int8_t i8EncoderDelta;
uint16_t u16AdcTimer;
const int8_t i8EncoderFull[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// Interrupts Service Routines.
void __interrupt() ISR(void)
{
    if(INTCONbits.TMR0IF){
        static uint8_t u8encoderLast = 0;
        u8encoderLast = (u8encoderLast<<2) & 0x0F;
        if(ROTARY_ENCODER_A) u8encoderLast |= 1;
        if(ROTARY_ENCODER_B) u8encoderLast |= 2;
        i8EncoderDelta += i8EncoderFull[u8encoderLast];
        INTCONbits.TMR0IF = 0b0;
    }
    u16AdcTimer++;
}

// Main.
void main(void)
{
    // MCU Initialization.
    // Internal Oscillator Settings.
    OSCTUNE = 0b00000000;
    OSCCON = 0x70;
    // Ports Settings.
    // PORT Data Register.
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    // TRIS Data Direction.
    TRISA = 0b00000011;
    TRISB = 0b00011111;
    TRISC = 0b00011000;
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
    // PPS Enabled Writes.
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0b0;
    // PPS Inputs.
    SSPCLKPPSbits.SSPCLKPPS = 0x13; // RC3 - MSSP.SCL.
    SSPDATPPSbits.SSPDATPPS = 0x14; // RC4 - MSSP.SDA.
    // PPS Outputs.
    RC3PPSbits.RC3PPS = 0x21;       // RC3 - MSSP.SCL.
    RC4PPSbits.RC4PPS = 0x22;       // RC4 - MSSP.SDA.
    // PPS Disabled Writes.
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

    // I2C Master Settings.
    SSP1BUF = 0;
    SSP1ADD = I2C_BAUDRATE;
    SSP1STAT = 0x00;
    SSP1CON1 = 0x08;
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x00;
     // I2C Enable.
    SSP1CON1bits.SSPEN = 0b1;

    // Timer0 Settings.
    // ~500Hz @8MHz.
    OPTION_REGbits.PS = 0b010;
    OPTION_REGbits.PSA = 0b0;
    OPTION_REGbits.TMR0CS = 0b0;
    // Timer0 Interrupts Enable.
    INTCONbits.TMR0IE = 0b1;
    INTCONbits.TMR0IF = 0b0;

    // ST7036 Initialization.
    lcd_initialize();

    // Write 5x8 Dots Custom Patterns in ST7036 CGRAM.
    uint8_t u8Line, u8Pattern;
    lcd_writeInstruction(ST7036_SET_CGRAM_ADDRESS);
    for(u8Pattern=0; u8Pattern<5; u8Pattern++){
        for(u8Line=0; u8Line<8; u8Line++){
            lcd_writeCharacter(au8BatteryPattern[u8Pattern][u8Line]);
        }
    }

    // Display Strings & Pattern.
    LCD_BACKLIGHT_ON;
    lcd_clearDisplay();
    lcd_writeStringSetCursor(au8Tronix, C0220BiZ_CONFIGURATION_FIRST_LINE);
    lcd_writeStringSetCursor(au8WWW, C0220BiZ_CONFIGURATION_SECOND_LINE);
    lcd_setCursor(C0220BiZ_CONFIGURATION_FIRST_LINE + 19);
    lcd_writeCharacter(PATTERN_BATTERY_FULL);

    // Interrupts Enable.
    INTCONbits.GIE = 0b1;

    uint8_t au8Buffer[6];
    uint16_t u16ADCRead, u16ADC0Last, u16ADC1Last;
    uint8_t u8LCDBacklight = 0, u8encoderRead=0, u8encoderLast=0, u8encoderSwitchPressed;
    uint8_t u8switchS1Pressed, u8switchS2Pressed;
    while(1){
        // ADC Read every ~1s.
        if(u16AdcTimer>1000){
            u16ADCRead = 0;
            ADCON0bits.CHS = 0b00000;
            __delay_us(5);
            ADCON0bits.GO = 0b1;
            while(ADCON0bits.GO){};
            u16ADCRead = (uint16_t)(ADRESH<<8) + ADRESL;
            if(u16ADC0Last != u16ADCRead){
                u16toa(u16ADCRead, au8Buffer, 10);
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Adc0);
                lcd_writeString(au8Buffer);
                u16ADC0Last = u16ADCRead;
            }
            u16ADCRead = 0;
            ADCON0bits.CHS = 0b00001;
            __delay_us(5);
            ADCON0bits.GO = 0b1;
            while(ADCON0bits.GO){};
            u16ADCRead = (uint16_t)(ADRESH<<8) + ADRESL;
            if(u16ADC1Last != u16ADCRead){
                u16toa(u16ADCRead, au8Buffer, 10);
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Adc1);
                lcd_writeString(au8Buffer);
                u16ADC1Last = u16ADCRead;
            }
            u16AdcTimer = 0;
        }

        // Rotary Encoder.
        if(!ROTARY_ENCODER_SWITCH){
            __delay_ms(100);
            u8encoderSwitchPressed = 1;
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            lcd_writeString(au8Encodersw);
            lcd_writeString(au8Pressed);
        }else if(ROTARY_ENCODER_SWITCH){
            if(u8encoderSwitchPressed){
                u8encoderSwitchPressed = 0;
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Encodersw);
                lcd_writeString(au8Released);
                u8encoderRead = 0;
            }
        }

        u8encoderRead += rotary_u8encoderRead();
        if(u8encoderLast != u8encoderRead){
            lcd_setCursor(C0220BiZ_CONFIGURATION_FIRST_LINE + 19);
            if(u8encoderRead<51)
                lcd_writeCharacter(PATTERN_BATTERY_EMPTY);
            else if(u8encoderRead>50 && u8encoderRead<101)
                lcd_writeCharacter(PATTERN_BATTERY_1_5);
            else if(u8encoderRead>100 && u8encoderRead<151)
                lcd_writeCharacter(PATTERN_BATTERY_2_5);
            else if(u8encoderRead>150 && u8encoderRead<201)
                lcd_writeCharacter(PATTERN_BATTERY_3_5);
            else
                lcd_writeCharacter(PATTERN_BATTERY_FULL);
            u16toa(u8encoderRead, au8Buffer, 10);
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            lcd_writeString(au8Encoder);
            lcd_writeString(au8Buffer);
            u8LCDBacklight = 1;
            u8encoderLast = u8encoderRead;
            u8LCDBacklight = u8encoderRead;
        }

        // Switchs.
        if(!SWITCH_S1){
            __delay_ms(100);
            u8switchS1Pressed = 1;
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            lcd_writeString(au8Switch1);
            lcd_writeString(au8Pressed);
            LCD_BACKLIGHT_ON;
        }else if(SWITCH_S1){
            if(u8switchS1Pressed){
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Switch1);
                lcd_writeString(au8Released);
                u8switchS1Pressed = 0;
            }
        }
        if(!SWITCH_S2){
            __delay_ms(100);
            u8switchS2Pressed = 1;
            lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
            lcd_writeString(au8Switch2);
            lcd_writeString(au8Pressed);
            LCD_BACKLIGHT_OFF;
        }else if(SWITCH_S2){
            if(u8switchS2Pressed){
                lcd_clearLine(C0220BiZ_CONFIGURATION_SECOND_LINE);
                lcd_writeString(au8Switch2);
                lcd_writeString(au8Released);
                u8switchS2Pressed = 0;
            }
        }

        // LCD Backlight.
        if(u8LCDBacklight){
            lcd_setBacklight(u8LCDBacklight);
            u8LCDBacklight = 0;
        }
    }
}

// Functions.
void i2c_restart(void)
{
    while(SSP1CON2 & 0x1F){};
    SSP1CON2bits.RSEN = 0b1;
    while(SSP1CON2bits.RSEN){};
}

void i2c_start(void)
{
    while(SSP1CON2 & 0x1F){};
    SSP1CON2bits.SEN = 0b1;
    while(SSP1CON2bits.SEN){};
}

void i2c_stop(void)
{
    while(SSP1CON2 & 0x1F){};
    SSP1CON2bits.PEN = 0b1;
    while(SSP1CON2bits.PEN){};
}

void i2c_write(uint8_t u8Data)
{
    SSP1BUF = u8Data;
    while(SSP1STATbits.BF){};
}

void lcd_clearDisplay(void)
{
    lcd_writeInstruction(ST7036_CLEAR_DISPLAY);
    __delay_ms(ST7036_CLEAR_DISPLAY_DELAY_MS);
}

void lcd_clearLine(uint8_t u8Line)
{
    uint8_t character = C0220BiZ_CONFIGURATION_CHARACTERS;

    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Line);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_DATA);
    while(character--)
        i2c_write(ASCII_SPACE);
    i2c_restart();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Line);
    i2c_stop();
}

void lcd_initialize(void)
{
    __delay_ms(ST7036_INITIALIZATION_DELAY_MS);

    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_COMMAND);
    // Instruction Table IS2=0, IS1=1.
    i2c_write(ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1);
    i2c_write(ST7036_BIAS_SET_1_5);
    i2c_write(ST7036_CONTRAST_LSB_12);
    i2c_write(ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1);
    i2c_write(ST7036_FOLLOWER_CONTROL_ON_RAB_5);
    // Instruction Table IS2=0, IS1=0.
    i2c_write(ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8);
    i2c_write(ST7036_DISPLAY_ON_CURSOR_OFF);
    i2c_write(ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT);
    i2c_write(ST7036_CLEAR_DISPLAY);
    i2c_stop();
    __delay_ms(ST7036_CLEAR_DISPLAY_DELAY_MS);
}

void lcd_setBacklight(uint8_t u8Backlight)
{
    // CAT4002 DIM Reset.
    LCD_BACKLIGHT_OFF;
    __delay_ms(CAT4002_DELAY_DOWN_MS);
    LCD_BACKLIGHT_OFF;
    __delay_us(CAT4002_DELAY_LED_US);

    // CAT4002 DIM Pulses.
    INTCONbits.GIE = 0b0;
    do{
        LCD_BACKLIGHT_OFF;
        __delay_us(CAT4002_DELAY_LOW_US);
        LCD_BACKLIGHT_ON;
        __delay_us(CAT4002_DELAY_HIGH_US);
    } while(u8Backlight--);
    INTCONbits.GIE = 0b1;
}

void lcd_setCursor(uint8_t u8Cursor)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Cursor);
    i2c_stop();
}

void lcd_writeCharacter(uint8_t u8Data)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA);
    i2c_write(u8Data);
    i2c_stop();
}

void lcd_writeInstruction(uint8_t u8Data)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Data);
    i2c_stop();
}

void lcd_writeString(const uint8_t * u8Data)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_DATA);
    while(*u8Data != '\0')
        i2c_write(*u8Data++);
    i2c_stop();
}

void lcd_writeStringSetCursor(const uint8_t * u8Data, uint8_t u8Cursor)
{
    i2c_start();
    i2c_write(C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE);
    i2c_write(ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND);
    i2c_write(u8Cursor);
    i2c_write(ST7036_I2C_CONTROL_CONTINUOUS_DATA);
    while(*u8Data != '\0')
        i2c_write(*u8Data++);
    i2c_stop();
}

int8_t rotary_u8encoderRead(void)
{
    int8_t u8encoderRead;

    INTCONbits.TMR0IE = 0b0;
    u8encoderRead = i8EncoderDelta;
    i8EncoderDelta = u8encoderRead & 3;
    INTCONbits.TMR0IE = 0b1;

    return(u8encoderRead>>2);
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
```

## 3.TODO - EUSART.

```as

```

## 4.DRAFT - LCD.

```as
; Configuration Registers.
CONFIG FOSC=INTOSC
CONFIG WDTE=OFF
CONFIG PWRTE=OFF
CONFIG MCLRE=ON
CONFIG CP=OFF
CONFIG BOREN=OFF
CONFIG CLKOUTEN=OFF
CONFIG IESO=OFF
CONFIG FCMEN=OFF
CONFIG WRT=OFF
CONFIG PPS1WAY=ON
CONFIG ZCD=OFF
CONFIG PLLEN=OFF
CONFIG STVREN=ON
CONFIG BORV=LO
CONFIG LPBOR=OFF
CONFIG LVP=ON

#include <xc.inc>
; PIC16F1778 - Compile with PIC-AS(v2.36).
; PIC16F1778 - @8MHz Internal Oscillator.
; -preset_vec=0000h, -pcinit=0005h, -pstringtext=3FC0h.
; Instruction ~500ns @8MHz.

; GPR BANK0.
PSECT	cstackBANK0,class=BANK0,space=1,delta=1
__pcstackBANK0:	    DS  80

; Common RAM.
PSECT	cstackCOMM,class=COMMON,space=1,delta=1
__pcstackCOMMON:    DS	16

; MCU Definitions.
; BANKS.
#define	BANK0   0x0
#define	BANK1   0x1
#define	BANK2   0x2
#define	BANK3   0x3
#define	BANK4   0x4
#define	BANK5   0x5
#define	BANK6   0x6
#define	BANK7   0x7
#define	BANK8   0x8
#define	BANK9   0x9
#define	BANK10  0xA
#define	BANK11  0xB
#define	BANK12  0xC
#define	BANK13  0xD
#define	BANK14  0xE
#define	BANK15  0xF
#define	BANK16  0x10
#define	BANK17  0x11
#define	BANK18  0x12
#define	BANK19  0x13
#define	BANK20  0x14
#define	BANK21  0x15
#define	BANK22  0x16
#define	BANK23  0x17
#define	BANK24  0x18
#define	BANK25  0x19
#define	BANK26  0x1A
#define	BANK27  0x1B
#define	BANK28  0x1C
#define	BANK29  0x1D
#define	BANK30  0x1E
#define	BANK31  0x1F
; SFR STATUS Bits.
#define	C	0x0
#define	Z	0x2

; Definitions.
; MCU Clock Frequency.
#define _XTAL_FREQ 8000000
; I2C.
#define I2C_READ                                           0b1
#define I2C_WRITE                                          0b0
#define I2C_FSCL_HZ                                        400000
#define I2C_BAUDRATE                                       ((_XTAL_FREQ/(4*I2C_FSCL_HZ))-1)
; ST7036 I2C Address.
#define ST7036_I2C_ADDRESS_78                              0x78
#define ST7036_I2C_CONTROL_CONTINUOUS_COMMAND              0x00
#define ST7036_I2C_CONTROL_CONTINUOUS_DATA                 0x40
#define ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND           0x80
#define ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA              0xC0
; ST7036 Instruction Table IS2=0, IS1=0.
#define ST7036_CLEAR_DISPLAY                               0x01
#define ST7036_RETURN_HOME                                 0x02
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_NOSHIFT      0x04
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_SHIFT_RIGHT  0x05
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT      0x06
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_SHIFT_LEFT   0x07
#define ST7036_DISPLAY_OFF                                 0x08
#define ST7036_DISPLAY_ON_CURSOR_OFF                       0x0C
#define ST7036_DISPLAY_ON_CURSOR_ON_NOBLINK                0x0E
#define ST7036_DISPLAY_ON_CURSOR_ON_BLINK                  0x0F
#define ST7036_DISPLAY_CURSOR_SHIFT_LEFT                   0x10
#define ST7036_DISPLAY_CURSOR_SHIFT_RIGHT                  0x14
#define ST7036_DISPLAY_DISPLAY_SHIFT_LEFT                  0x18
#define ST7036_DISPLAY_DISPLAY_SHIFT_RIGH                  0x1C
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8         0x20
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS1     0x21
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS2     0x22
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_FONT5x8         0x28
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_DHFONT5x8       0x24
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_DHFONT5x8       0x2C
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8         0x30
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1     0x31
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS2     0x32
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_DHFONT5x8       0x34
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8         0x38
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_DHFONT5x8       0x3C
#define ST7036_SET_ICON_RAM_ADDRESS                        0X40
#define ST7036_SET_CGRAM_ADDRESS                           0x40
#define ST7036_DDRAM_ADDRESS_FIRST_LINE                    0x80
#define ST7036_DDRAM_ADDRESS_SECOND_LINE                   0xC0
; ST7036 Instruction Table IS2=0, IS1=1.
#define ST7036_BIAS_SET_1_5                                0x14
#define ST7036_BIAS_SET_1_5_3_LINE                         0x15
#define ST7036_BIAS_SET_1_4                                0x1C
#define ST7036_BIAS_SET_1_4_3_LINE                         0x1D
#define ST7036_POWER_ICON_OFF_BOOST_OFF_NO_CONTRAST        0x50
#define ST7036_POWER_ICON_ON_BOOST_OFF_NO_CONTRAST         0x58
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_0      0x54
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_1      0x55
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_2      0x56
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_3      0x57
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_0       0x5C
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1       0x5D
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_2       0x5E
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_3       0x5F
#define ST7036_FOLLOWER_CONTROL_OFF                        0x60
#define ST7036_FOLLOWER_CONTROL_ON_RAB_0                   0x68
#define ST7036_FOLLOWER_CONTROL_ON_RAB_1                   0x69
#define ST7036_FOLLOWER_CONTROL_ON_RAB_2                   0x6A
#define ST7036_FOLLOWER_CONTROL_ON_RAB_3                   0x6B
#define ST7036_FOLLOWER_CONTROL_ON_RAB_4                   0x6C
#define ST7036_FOLLOWER_CONTROL_ON_RAB_5                   0x6D
#define ST7036_FOLLOWER_CONTROL_ON_RAB_6                   0x6E
#define ST7036_FOLLOWER_CONTROL_ON_RAB_7                   0x6F
#define ST7036_CONTRAST_LSB_0                              0x70
#define ST7036_CONTRAST_LSB_1                              0x71
#define ST7036_CONTRAST_LSB_2                              0x72
#define ST7036_CONTRAST_LSB_3                              0x73
#define ST7036_CONTRAST_LSB_4                              0x74
#define ST7036_CONTRAST_LSB_5                              0x75
#define ST7036_CONTRAST_LSB_6                              0x76
#define ST7036_CONTRAST_LSB_7                              0x77
#define ST7036_CONTRAST_LSB_8                              0x78
#define ST7036_CONTRAST_LSB_9                              0x79
#define ST7036_CONTRAST_LSB_10                             0x7A
#define ST7036_CONTRAST_LSB_11                             0x7B
#define ST7036_CONTRAST_LSB_12                             0x7C
#define ST7036_CONTRAST_LSB_13                             0x7D
#define ST7036_CONTRAST_LSB_14                             0x7E
#define ST7036_CONTRAST_LSB_15                             0x7F
; ST7036 Instruction Table IS2=1, IS1=0.
#define ST7036_DOUBLE_HEIGHT_FONT_COM9_COM24               0x10
#define ST7036_DOUBLE_HEIGHT_FONT_COM1_COM16               0x11
; ST7036 Delays.
#define ST7036_CLEAR_DISPLAY_DELAY			   5
#define ST7036_INITIALIZATION_DELAY			   130
; NHD-C0220BiZ Configuration.
#define C0220BiZ_CONFIGURATION_I2C_ADDRESS                 ST7036_I2C_ADDRESS_78
#define C0220BiZ_CONFIGURATION_FIRST_LINE                  ST7036_DDRAM_ADDRESS_FIRST_LINE
#define C0220BiZ_CONFIGURATION_SECOND_LINE                 ST7036_DDRAM_ADDRESS_SECOND_LINE
#define C0220BiZ_CONFIGURATION_CHARACTERS                  20

; Reset Vector.
PSECT	reset_vec,class=CODE,space=0,delta=2
resetVector:
    GOTO    main

; Main.
PSECT	cinit,class=CODE,space=0,delta=2
main:
    ; MCU Initialization.
    ; Internal Oscillator Settings.
    MOVLB   BANK1
    MOVLW   0b00000000
    MOVWF   OSCTUNE
    MOVLW   0x70
    MOVWF   OSCCON
    BTFSS   HFIOFR
    BRA	    $-1
    ; Ports Settings.
    ; PORT Data Register.
    MOVLB   BANK0
    MOVLW   0b00000000
    MOVWF   PORTA
    MOVLW   0b00000000
    MOVWF   PORTB
    MOVLW   0b00000000
    MOVWF   PORTC
    MOVLW   0b00000000
    MOVWF   PORTE
    ; TRIS Data Direction.
    MOVLB   BANK1
    MOVLW   0b00000000
    MOVWF   TRISA
    MOVLW   0b00000000
    MOVWF   TRISB
    MOVLW   0b00011000
    MOVWF   TRISC
    MOVLW   0b00001000
    MOVWF   TRISE
    ; LATCH Outputs.
    MOVLB   BANK2
    MOVLW   0b00000000
    MOVWF   LATA
    MOVLW   0b00000000
    MOVWF   LATB
    MOVLW   0b00000000
    MOVWF   LATC
    ; ANSEL Analog.
    MOVLB   BANK3
    MOVLW   0b00000000
    MOVWF   ANSELA
    MOVLW   0b00000000
    MOVWF   ANSELB
    MOVLW   0b00000000
    MOVWF   ANSELC
    ; WPU Weak Pull-up.
    MOVLB   BANK4
    MOVLW   0b00000000
    MOVWF   WPUA
    MOVLW   0b00000000
    MOVWF   WPUB
    MOVLW   0b00000000
    MOVWF   WPUC
    MOVLW   0b00000000
    MOVWF   WPUE
    ; ODCON Open-drain.
    MOVLB   BANK5
    MOVLW   0b00000000
    MOVWF   ODCONA
    MOVLW   0b00000000
    MOVWF   ODCONB
    MOVLW   0b00000000
    MOVWF   ODCONC
    ; SRLCON Slew Rate.
    MOVLB   BANK6
    MOVLW   0b11111111
    MOVWF   SLRCONA
    MOVLW   0b11111111
    MOVWF   SLRCONB
    MOVLW   0b11111111
    MOVWF   SLRCONC
    ; INLVL Input Level.
    MOVLB   BANK7
    MOVLW   0b00000000
    MOVWF   INLVLA
    MOVLW   0b00000000
    MOVWF   INLVLB
    MOVLW   0b00000000
    MOVWF   INLVLC
    ; HIDRVB High Drive.
    MOVLB   BANK8
    MOVLW   0b00000000
    MOVWF   HIDRVB
    ; PPS Settings.
    ; PPS Write Enable.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BCF	    PPSLOCK, 0x0
    ; PPS Inputs.
    ; RC3 - MSSP.SCL.
    MOVLW   0x13
    MOVWF   SSPCLKPPS
    ; RC4 - MSSP.SCA.
    movlw   0x14
    movwf   SSPDATPPS
    ; PPS Outputs.
    MOVLB   BANK29
    ; RC3 - MSSP.SCL.
    MOVLW   0x21
    MOVWF   RC3PPS
    ; RC4 - MSSP.SDA.
    MOVLW   0x22
    MOVWF   RC4PPS
    ; PPS Write Disable.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BSF	    PPSLOCK, 0x0

    ; I2C Mater Settings.
    MOVLB   BANK4
    CLRF    SSP1BUF
    MOVLW   I2C_BAUDRATE
    MOVWF   SSP1ADD
    MOVLW   0b00000000
    MOVWF   SSP1STAT
    MOVLW   0b00001000
    MOVWF   SSP1CON1
    MOVLW   0b00000000
    MOVWF   SSP1CON2
    MOVLW   0b00000000
    MOVWF   SSP1CON3
    ; I2C Enable.
    BSF	    SSPEN

    ; C0220BiZ Initialization.
    CALL    _lcdInitialize

    ; C0220BiZ Display Strings.
    CALL    _writeStringTRONIX
    MOVLW   C0220BiZ_CONFIGURATION_SECOND_LINE + 5
    CALL    _lcdWriteInstruction
    CALL    _writeStringREADY

loop:
    BRA	    loop

; Functions.
; delay = 1 ~390us.
; delay = 255 ~98ms.
_delay:
    MOVLB   BANK0
    MOVWF   __pcstackBANK0
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  __pcstackBANK0, F
    BRA	    $-4
    RETURN

; delay = 1 ~98ms.
; delay = 255 ~25s.
_delay1:
    MOVLB   BANK0
    MOVWF   __pcstackBANK0 + 1
    MOVLW   255
    MOVWF   __pcstackBANK0
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  __pcstackBANK0, F
    BRA	    $-4
    DECFSZ  __pcstackBANK0 + 1, F
    BRA	    $-6
    RETURN

_i2cRestart:
    MOVLB   BANK4
    MOVF    SSP1CON2, W
    ANDLW   0x1F
    BTFSS   STATUS, Z
    BRA	    $-3
    BSF	    RSEN
    BTFSC   RSEN
    BRA	    $-1
    RETURN

_i2cStart:
    MOVLB   BANK4
    MOVF    SSP1CON2, W
    ANDLW   0x1F
    BTFSS   STATUS, Z
    BRA	    $-3
    BSF	    SEN
    BTFSC   SEN
    BRA	    $-1
    RETURN

_i2cStop:
    MOVLB   BANK4
    MOVF    SSP1CON2, W
    ANDLW   0x1F
    BTFSS   STATUS, Z
    BRA	    $-3
    BSF	    PEN
    BTFSC   PEN
    BRA	    $-1
    RETURN

_i2cWrite:
    MOVLB   BANK4
    MOVWF   SSP1BUF
    BTFSC   BF
    BRA	    $-1
    NOP
    NOP
    RETURN

_lcdClearDisplay:
    MOVLW   ST7036_CLEAR_DISPLAY
    CALL    _lcdWriteInstruction
    MOVLW   ST7036_CLEAR_DISPLAY_DELAY
    CALL    _delay
    RETURN

_lcdInitialize:
    MOVLW   ST7036_INITIALIZATION_DELAY
    CALL    _delay

    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_CONTINUOUS_COMMAND
    CALL    _i2cWrite
    ; Instruction Table IS2=0, IS1=1.
    MOVLW   ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1
    CALL    _i2cWrite
    MOVLW   ST7036_BIAS_SET_1_5
    CALL    _i2cWrite
    MOVLW   ST7036_CONTRAST_LSB_12
    CALL    _i2cWrite
    MOVLW   ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1
    CALL    _i2cWrite
    MOVLW   ST7036_FOLLOWER_CONTROL_ON_RAB_5
    CALL    _i2cWrite
    ; Instruction Table IS2=0, IS1=0.
    MOVLW   ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8
    CALL    _i2cWrite
    MOVLW   ST7036_DISPLAY_ON_CURSOR_ON_BLINK
    CALL    _i2cWrite
    MOVLW   ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT
    CALL    _i2cWrite
    MOVLW   ST7036_CLEAR_DISPLAY
    CALL    _i2cWrite
    CALL    _i2cStop

    MOVLW   ST7036_CLEAR_DISPLAY_DELAY
    CALL    _delay
    RETURN

_lcdWriteCharacter:
    movwf   __pcstackCOMMON
    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA
    CALL    _i2cWrite
    movf    __pcstackCOMMON, W
    CALL    _i2cWrite
    CALL    _i2cStop
    RETURN

_lcdWriteInstruction:
    movwf   __pcstackCOMMON
    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND
    CALL    _i2cWrite
    movf    __pcstackCOMMON, W
    CALL    _i2cWrite
    CALL    _i2cStop
    RETURN

_lcdWriteString:
    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_CONTINUOUS_DATA
    CALL    _i2cWrite
    MOVIW   FSR1++
    ANDLW   0xFF
    BTFSC   STATUS, Z
    BRA	    $+3
    CALL    _i2cWrite
    BRA	    $-5
    CALL    _i2cStop
    RETURN

_writeStringREADY:
    MOVLW   HIGH stringREADY + 0x80
    MOVWF   FSR1H
    MOVLW   LOW stringREADY
    MOVWF   FSR1L
    CALL    _lcdWriteString
    RETURN

_writeStringTRONIX:
    MOVLW   HIGH stringTRONIX + 0x80
    MOVWF   FSR1H
    MOVLW   LOW stringTRONIX
    MOVWF   FSR1L
    CALL    _lcdWriteString
    RETURN

; FPM Strings.
PSECT	stringtext,class=STRCODE,space=0,delta=2
stringREADY:
    DB	    'R','E','A','d','y','>',' ', 0x0

stringTRONIX:
    DB	    'T','r','o','n','i','x',' ','I','/','O','.', 0x0

    END	    resetVector
```

---
DISCLAIMER: THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.
