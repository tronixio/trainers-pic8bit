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
; PIC16F1778 - Compile with PIC-AS(v2.40).
; PIC16F1778 - @8MHz Internal Oscillator.
; -preset_vec=0000h, -pcinit=0005h, -pstringtext=3FC0h.
; Instruction ~500ns @8MHz.

; DRAFT Do not use.

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
PSECT reset_vec,class=CODE,space=0,delta=2
resetVector:
    GOTO    main

; Main.
PSECT cinit,class=CODE,space=0,delta=2
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
    ; PPS Unlock Sequence.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BCF	    PPSLOCK, 0x0
    ; PPS Inputs.
    MOVLW   0x13
    MOVWF   SSPCLKPPS ; RC3 - MSSP.SCL.
    movlw   0x14
    movwf   SSPDATPPS ; RC4 - MSSP.SCA.
    ; PPS Outputs.
    MOVLB   BANK29
    MOVLW   0x21
    MOVWF   RC3PPS ; RC3 - MSSP.SCL.
    MOVLW   0x22
    MOVWF   RC4PPS ; RC4 - MSSP.SDA
    ; PPS Lock Sequence.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BSF	    PPSLOCK, 0x0

    ; I2C Master Settings.
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

    ; C0220BiZ Initialization.
    CALL    _lcdInitialize

    ; C0220BiZ Display Strings.
    CALL    _writeStringTRONIX
    MOVLW   C0220BiZ_CONFIGURATION_SECOND_LINE
    CALL    _lcdWriteInstruction
    CALL    _writeStringREADY

loop:
    BRA	    loop

; Functions.
; delay = 1 ~390us.
; delay = 255 ~98ms.
_u8Delay:
    MOVLB   BANK0
    MOVWF   __pcstackBANK0
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  __pcstackBANK0, F
    BRA	    $-3
    RETURN

; delay = 1 ~98ms.
; delay = 255 ~25s.
_u16Delay:
    MOVLB   BANK0
    MOVWF   __pcstackBANK0 + 1
    MOVLW   255
    MOVWF   __pcstackBANK0
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  __pcstackBANK0, F
    BRA	    $-3
    DECFSZ  __pcstackBANK0 + 1, F
    BRA	    $-5
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
    BSF	    SSPEN
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
    BCF	    SSPEN
    RETURN

_i2cWrite:
    MOVLB   BANK4
    MOVWF   SSP1BUF
    BTFSC   BF
    BRA	    $-1
    ;NOP
    ;NOP
    RETURN

_lcdClearDisplay:
    MOVLW   ST7036_CLEAR_DISPLAY
    CALL    _lcdWriteInstruction
    MOVLW   ST7036_CLEAR_DISPLAY_DELAY
    CALL    _u8Delay
    RETURN

_lcdInitialize:
    MOVLW   ST7036_INITIALIZATION_DELAY
    CALL    _u8Delay

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
    CALL    _u8Delay
    RETURN

_lcdWriteCharacter:
    MOVWF   __pcstackCOMMON
    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_NO_CONTINUOUS_DATA
    CALL    _i2cWrite
    MOVF    __pcstackCOMMON, W
    CALL    _i2cWrite
    CALL    _i2cStop
    RETURN

_lcdWriteInstruction:
    MOVWF   __pcstackCOMMON
    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_NO_CONTINUOUS_COMMAND
    CALL    _i2cWrite
    MOVF    __pcstackCOMMON, W
    CALL    _i2cWrite
    CALL    _i2cStop
    RETURN

_lcdWriteString:
    CALL    _i2cStart
    MOVLW   (C0220BiZ_CONFIGURATION_I2C_ADDRESS | I2C_WRITE)
    CALL    _i2cWrite
    MOVLW   ST7036_I2C_CONTROL_CONTINUOUS_DATA
    CALL    _i2cWrite
    MOVIW   FSR0++
    BTFSC   STATUS, Z
    BRA	    $+3
    CALL    _i2cWrite
    BRA	    $-4
    CALL    _i2cStop
    RETURN

_writeStringREADY:
    MOVLW   LOW stringREADY
    MOVWF   FSR0L
    MOVLW   HIGH stringREADY + 0x80
    MOVWF   FSR0H
    CALL    _lcdWriteString
    RETURN

_writeStringTRONIX:
    MOVLW   LOW stringTRONIX
    MOVWF   FSR0L
    MOVLW   HIGH stringTRONIX + 0x80
    MOVWF   FSR0H
    CALL    _lcdWriteString
    RETURN

; PFM Strings.
PSECT stringtext,class=STRCODE,space=0,delta=2
stringREADY:
    DB	'R','e','a','d','y','>', 0x00

stringTRONIX:
    DB	'T','r','o','n','i','x',' ','I','/','O','.', 0x00

    END	    resetVector
