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
; EUSART.

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
    MOVWF   RC4PPS ; RC4 - MSSP.SDA.
    ; PPS Lock Sequence.
    MOVLB   BANK28
    MOVLW   0x55
    MOVWF   PPSLOCK
    MOVLW   0xAA
    MOVWF   PPSLOCK
    BSF	    PPSLOCK, 0x0

loop:
    BRA	    loop

; Functions.
; delay = 1 ~390us.
; delay = 255 ~98ms.
_u8delay:
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
_u16delay:
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

; PFM Strings.
PSECT	stringtext,class=STRCODE,space=0,delta=2
stringREADY:
    DB	    0xd, 0xa, 'R','e','a','d','y','>',' '

stringTRONIX:
    DB	    0xa, 0xd, 'T','r','o','n','i','x',' ','I','/','O','.'

    END	    resetVector
