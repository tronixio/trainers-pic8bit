; Configuration Registers.
config FEXTOSC = OFF
config RSTOSC = HFINT32
config CLKOUTEN = OFF
config CSWEN = OFF
config FCMEN = ON
config MCLRE = ON
config PWRTE = OFF
config LPBOREN = OFF
config BOREN = OFF
config BORV = LO
config ZCD = OFF
config PPS1WAY = ON
config STVREN = ON
config WDTCPS = WDTCPS_31
config WDTE = OFF
config WDTCWS = WDTCWS_7
config WDTCCS = SC
config WRT = OFF
config SCANE = available
config LVP = ON
config CP = OFF
config CPD = OFF

#include <xc.inc>
; PIC16F18856 - Compile with PIC-AS(v2.40).
; PIC16F18856 - @4MHz Internal Oscillator.
; -preset_vec=0000h, -pintentry=0004h, -pcinit=0005h, -pstringtext=3F00h.
; Instruction ~510ns @4MHz - WDT ~XXms.

; DOGM081 SPI 3V3 - v0.1.

; Pinout.
; MCU.RC2 - DOGM.RS.
; MCU.RC3 - DOGM.SCK.
; MCU.RC4 - DOGM.SI.

; GPR BANK0.
PSECT cstackBANK0,global,class=BANK0,space=1,delta=1,noexec
;u8BANK0:   DS	1
u8delay:    DS	1

; Common RAM.
psect cstackCOMMON,global,class=COMMON,space=1,delta=1,noexec
;u8COMMON:  DS  1

; MCU Definitions.
; BANKS.
#define	BANK0	    0x0
#define	BANK1	    0x1
#define	BANK2	    0x2
#define	BANK3	    0x3
#define	BANK4	    0x4
#define	BANK5	    0x5
#define	BANK6	    0x6
#define	BANK7	    0x7
#define	BANK8	    0x8
#define	BANK9	    0x9
#define	BANK10	    0xA
#define	BANK11	    0xB
#define	BANK12	    0xC
#define	BANK13	    0xD
#define	BANK14	    0xE
#define BANK15	    0xF
#define	BANK16	    0x10
#define	BANK17	    0x11
#define	BANK18	    0x12
#define	BANK19	    0x13
#define	BANK29	    0x1D
#define	BANK30      0x1E
; SFR - Special Function Registers.
; SFR STATUS Bits.
#define	C	    0x0
#define	Z	    0x2
; SFR MSSP - SSPxCON1 Bits.
#define	BF	    0x0
#define SSPEN	0x5

; USER Definitions.
; Debug.
#define LED_DEBUG                                           LATA, 0x6
; ST7036 Commands Port.
#define ST7036_RS                                           LATC, 0x2
; ST7036 Instruction Table IS2=0, IS1=0.
#define ST7036_CLEAR_DISPLAY                                0x01
#define ST7036_RETURN_HOME                                  0x02
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_NOSHIFT       0x04
#define ST7036_ENTRY_MODE_SET_DDRAM_DECREMENT_SHIFT_RIGHT   0x05
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT       0x06
#define ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_SHIFT_LEFT    0x07
#define ST7036_DISPLAY_OFF                                  0x08
#define ST7036_DISPLAY_ON_CURSOR_OFF                        0x0C
#define ST7036_DISPLAY_ON_CURSOR_ON_NOBLINK                 0x0E
#define ST7036_DISPLAY_ON_CURSOR_ON_BLINK                   0x0F
#define ST7036_DISPLAY_CURSOR_SHIFT_LEFT                    0x10
#define ST7036_DISPLAY_CURSOR_SHIFT_RIGHT                   0x14
#define ST7036_DISPLAY_DISPLAY_SHIFT_LEFT                   0x18
#define ST7036_DISPLAY_DISPLAY_SHIFT_RIGH                   0x1C
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8          0x20
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS1      0x21
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_FONT5x8_IS2      0x22
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_FONT5x8          0x28
#define ST7036_FUNCTION_SET_4_BIT_ONE_LINE_DHFONT5x8        0x24
#define ST7036_FUNCTION_SET_4_BIT_TWO_LINE_DHFONT5x8        0x2C
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8          0x30
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1      0x31
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS2      0x32
#define ST7036_FUNCTION_SET_8_BIT_ONE_LINE_DHFONT5x8        0x34
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_FONT5x8          0x38
#define ST7036_FUNCTION_SET_8_BIT_TWO_LINE_DHFONT5x8        0x3C
#define ST7036_SET_ICON_RAM_ADDRESS                         0X40
#define ST7036_SET_CGRAM_ADDRESS                            0x40
#define ST7036_DDRAM_ADDRESS_FIRST_LINE                     0x80
; ST7036 Instruction Table IS2=0, IS1=1.
#define ST7036_BIAS_SET_1_5                                 0x14
#define ST7036_BIAS_SET_1_5_3_LINE                          0x15
#define ST7036_BIAS_SET_1_4                                 0x1C
#define ST7036_BIAS_SET_1_4_3_LINE                          0x1D
#define ST7036_POWER_ICON_OFF_BOOST_OFF_NO_CONTRAST         0x50
#define ST7036_POWER_ICON_ON_BOOST_OFF_NO_CONTRAST          0x58
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_0	    0x54
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_1	    0x55
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_2	    0x56
#define ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_3	    0x57
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_0	    0x5C
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_1	    0x5D
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_2	    0x5E
#define ST7036_POWER_ICON_ON_BOOST_ON_CONTRAST_MSB_3	    0x5F
#define ST7036_FOLLOWER_CONTROL_OFF                         0x60
#define ST7036_FOLLOWER_CONTROL_ON_RAB_0                    0x68
#define ST7036_FOLLOWER_CONTROL_ON_RAB_1                    0x69
#define ST7036_FOLLOWER_CONTROL_ON_RAB_2                    0x6A
#define ST7036_FOLLOWER_CONTROL_ON_RAB_3                    0x6B
#define ST7036_FOLLOWER_CONTROL_ON_RAB_4                    0x6C
#define ST7036_FOLLOWER_CONTROL_ON_RAB_5                    0x6D
#define ST7036_FOLLOWER_CONTROL_ON_RAB_6                    0x6E
#define ST7036_FOLLOWER_CONTROL_ON_RAB_7                    0x6F
#define ST7036_CONTRAST_LSB_0                               0x70
#define ST7036_CONTRAST_LSB_1                               0x71
#define ST7036_CONTRAST_LSB_2                               0x72
#define ST7036_CONTRAST_LSB_3                               0x73
#define ST7036_CONTRAST_LSB_4                               0x74
#define ST7036_CONTRAST_LSB_5                               0x75
#define ST7036_CONTRAST_LSB_6                               0x76
#define ST7036_CONTRAST_LSB_7                               0x77
#define ST7036_CONTRAST_LSB_8                               0x78
#define ST7036_CONTRAST_LSB_9                               0x79
#define ST7036_CONTRAST_LSB_10                              0x7A
#define ST7036_CONTRAST_LSB_11                              0x7B
#define ST7036_CONTRAST_LSB_12                              0x7C
#define ST7036_CONTRAST_LSB_13                              0x7D
#define ST7036_CONTRAST_LSB_14                              0x7E
#define ST7036_CONTRAST_LSB_15                              0x7F
#define	ST7036_INSTRUCTION_DELAY                            2
#define ST7036_CLEAR_DISPLAY_DELAY                          1
#define ST7036_INITIALIZATION_DELAY                         30
; ST7036 Instruction Table IS2=1, IS1=0.
#define ST7036_DOUBLE_HEIGHT_FONT_COM9_COM24                0x10
#define ST7036_DOUBLE_HEIGHT_FONT_COM1_COM16                0x11

; Reset Vector.
PSECT reset_vec,global,class=CODE,merge=1,delta=2
resetVector:
    GOTO    main

; ISR Vector.
PSECT intentry,global,class=CODE,space=0,delta=2
interruptVector:
    GOTO    isr

; Main.
PSECT cinit,global,class=CODE,merge=1,delta=2
main:
    ; MCU Initialization.
    ; Internal Oscillator Settings.
    MOVLB   BANK17
    MOVLW   0b00000000
    MOVWF   OSCTUNE
    MOVLW   0b00000010
    MOVWF   OSCFRQ
    BTFSS   HFOR
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
    ; TRIS Data Direction.
    MOVLW   0b00000000
    MOVWF   TRISA
    MOVLW   0b00000000
    MOVWF   TRISB
    MOVLW   0b00000000
    MOVWF   TRISC
    ; LATCH Outputs.
    MOVLW   0b00000000
    MOVWF   LATA
    MOVLW   0b00000000
    MOVWF   LATB
    MOVLW   0b00000000
    MOVWF   LATC
    ; PORTA Settings.
    MOVLB   BANK30
    MOVLW   0b00000000
    MOVWF   ANSELA
    MOVLW   0b00000000
    MOVWF   WPUA
    MOVLW   0b00000000
    MOVWF   ODCONA
    MOVLW   0b11111111
    MOVWF   SLRCONA
    MOVLW   0b00000000
    MOVWF   INLVLA
    MOVLW   0b00000000
    MOVWF   IOCAP
    MOVLW   0b00000000
    MOVWF   IOCAN
    MOVLW   0b00000000
    MOVWF   IOCAF
    ; PORTB Settings.
    MOVLW   0b00000000
    MOVWF   ANSELB
    MOVLW   0b00000000
    MOVWF   WPUB
    MOVLW   0b00000000
    MOVWF   ODCONB
    MOVLW   0b11111111
    MOVWF   SLRCONB
    MOVLW   0b00000000
    MOVWF   INLVLB
    MOVLW   0b00000000
    MOVWF   IOCBP
    MOVLW   0b00000000
    MOVWF   IOCBN
    MOVLW   0b00000000
    MOVWF   IOCBF
    ; PORTC Settings.
    MOVLW   0b00000000
    MOVWF   ANSELC
    MOVLW   0b00000000
    MOVWF   WPUC
    MOVLW   0b00000000
    MOVWF   ODCONC
    MOVLW   0b11111111
    MOVWF   SLRCONC
    MOVLW   0b00000000
    MOVWF   INLVLC
    MOVLW   0b00000000
    MOVWF   IOCCP
    MOVLW   0b00000000
    MOVWF   IOCCN
    MOVLW   0b00000000
    MOVWF   IOCCF
    ; PORTE Settings.
    MOVLW   0b00000000
    MOVWF   WPUE
    MOVLW   0b00000000
    MOVWF   INLVLE
    MOVLW   0b00000000
    MOVWF   IOCEP
    MOVLW   0b00000000
    MOVWF   IOCEN
    MOVLW   0b00000000
    MOVWF   IOCEF
    ; PPS Settings.
    ; PPS Sequence Enable.
    MOVLB   BANK29
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BCF	    PPSLOCK, 0x0
    ; PPS Ouputs Settings.
    MOVLB   BANK30
    MOVLW   0x14
    MOVWF   RC3PPS  ; MSSP1.SCK.
    MOVLW   0x15
    MOVWF   RC4PPS  ; MSSP1.MISO.

    ; MSSP Settings.
    ; MSSP1 SPI Master - Mode 0.
    MOVLB   BANK3
    CLRF    SSP1BUF
    CLRF    SSP1ADD
    CLRF    SSP1MSK
    MOVLW   0b01000000
    MOVWF   SSP1STAT
    CLRF    SSP1CON1
    CLRF    SSP1CON2
    CLRF    SSP1CON3
    ; MSSP1 Enable.
    BSF	    SSP1CON1, SSPEN

    ; DOGM081 Settings.
    CALL    dogm081_Initialization
    CALL    dogm081_writeTRONIX

loop:
    BRA	    $

; Interrupt Service Routine.
isr:
    RETFIE

; u8Delay = 1 ~775µs.
; u8Delay = 255 ~198ms.
_u8Delay:
    MOVLB   BANK0
    MOVWF   u8delay
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  u8delay, F
    BRA	    $-3
    RETURN

dogm081_Initialization:
    MOVLW   ST7036_INITIALIZATION_DELAY
    CALL    _u8Delay
    MOVLW   LOW a8st7036Initialization
    MOVWF   FSR0L
    MOVLW   HIGH a8st7036Initialization + 0x80
    MOVWF   FSR0H
    MOVIW   FSR0++
    BTFSC   STATUS, Z
    BRA	    $+3
    CALL    st7036_writeInstruction
    BRA	    $-4
    MOVLW   ST7036_CLEAR_DISPLAY_DELAY
    CALL    _u8Delay
    RETURN

dogm081_writeTRONIX:
    MOVLW   LOW a8stringTronix
    MOVWF   FSR0L
    MOVLW   HIGH a8stringTronix + 0x80
    MOVWF   FSR0H
    MOVIW   FSR0++
    BTFSC   STATUS, Z
    RETURN
    CALL    st7036_writeData
    BRA	    $-4

st7036_writeData:
    MOVLB   BANK0
    BSF	    ST7036_RS
    MOVLB   BANK3
    MOVWF   SSP1BUF
    MOVLW   ST7036_INSTRUCTION_DELAY
    DECFSZ  WREG, F
    BRA	    $-1
    RETURN

st7036_writeInstruction:
    MOVLB   BANK0
    BCF	    ST7036_RS
    MOVLB   BANK3
    MOVWF   SSP1BUF
    MOVLW   ST7036_INSTRUCTION_DELAY
    DECFSZ  WREG, F
    BRA	    $-1
    RETURN

; PFM Strings.
;PSECT	stringtext,class=CONST,space=0,delta=2
PSECT	stringtext,class=STRCODE,space=0,delta=2
a8st7036Initialization:
    DB	ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8_IS1, \
	ST7036_BIAS_SET_1_5, \
	ST7036_POWER_ICON_OFF_BOOST_ON_CONTRAST_MSB_1, \
	ST7036_FOLLOWER_CONTROL_ON_RAB_5, \
	ST7036_CONTRAST_LSB_0, \
	ST7036_FUNCTION_SET_8_BIT_ONE_LINE_FONT5x8, \
	ST7036_DISPLAY_ON_CURSOR_OFF, \
	ST7036_ENTRY_MODE_SET_DDRAM_INCREMENT_NOSHIFT, \
	ST7036_CLEAR_DISPLAY, 0x00

a8stringTronix:
    DB	'T', 'r', 'o', 'n', 'i', 'x', 'I', 'O', 0x00

    END resetVector
