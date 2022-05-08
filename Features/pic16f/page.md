# PIC16F - Pages ACCESS.

## 0.Contents.

- [1.Code.](#1code)
- [2.Build & Load Output.](#2build--load-output)
- [3.Configuration.](#3configuration)

## 1.Code.

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
; -preset_vec=0000h, -pcinit=0005h, -ppage1=0800h, -ppage2=1000h, -ppage3=1800h.

; This is a demonstration code to show how to access the PAGES Program Space.

: LED or Oscilloscope Probe on MCU.RA0 Pin.

; GPR BANK0.
PSECT cstackBANK0,class=BANK0,space=1,delta=1
u8Delay:   DS  1

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

; Reset Vector.
PSECT reset_vec,class=CODE,space=0,delta=2
resetVect:
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
    MOVLW   0b00000000
    MOVWF   TRISC
    MOVLW   0b00000000
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

loop:
    MOVLP   HIGH _led_on
    CALL    LOW _led_on
    MOVLP   HIGH _led_off
    CALL    LOW _led_off
    BRA	    loop

; Function PAGE1.
PSECT page1,class=CODE,space=0,delta=2
_led_on:
    MOVLB   BANK2
    BSF	    LATA, 0x0
    MOVLW   255
    MOVLP   HIGH _delay
    CALL    LOW _delay
    RETURN

; Function PAGE2.
PSECT page2,class=CODE,space=0,delta=2
_led_off:
    MOVLB   BANK2
    BCF	    LATA, 0x0
    MOVLW   255
    MOVLP   HIGH _delay
    CALL    LOW _delay
    RETURN

; Function PAGE3.
PSECT page3,class=CODE,space=0,delta=2
_delay:
    MOVLB   BANK0
    MOVWF   u8Delay
    MOVLW   255
    DECFSZ  WREG, F
    BRA	    $-1
    DECFSZ  u8Delay, F
    BRA	    $-3
    RETURN

    END	    resetVect
```

## 3.Build & Load Output.

```text
Psect      | Contents            | Memory Range  | Size
-----------|---------------------|---------------|------------
reset_vec  | Reset vector        | 0000h - 0000h |  1 word   
cinit      | Initialization code | 0005h - 0051h | 4D words  
page1      |                     | 0800h - 0805h |  6 words  
page2      |                     | 1000h - 1005h |  6 words  
page3      |                     | 1800h - 1807h |  8 words  
-----------|---------------------|---------------|------------
           |                     |               |            
cstackBANK | Stack in bank 0     | 0020h - 0020h |  1 byte   
-----------|---------------------|---------------|------------
config     |                     | 8007h - 8008h |  2 words  
-----------|---------------------|---------------|------------
```

## 1.Configuration.

- Configure PIC-AS Summary output.

<p align="center"><img alt="" src="./pics/picas-00.png"></p>

- Configure PIC-AS Linker Program Sector.

<p align="center"><img alt="" src="./pics/picas-07.png"></p>

---
DISCLAIMER: THIS CODE IS PROVIDED WITHOUT ANY WARRANTY OR GUARANTEES.
USERS MAY USE THIS CODE FOR DEVELOPMENT AND EXAMPLE PURPOSES ONLY.
AUTHORS ARE NOT RESPONSIBLE FOR ANY ERRORS, OMISSIONS, OR DAMAGES THAT COULD
RESULT FROM USING THIS FIRMWARE IN WHOLE OR IN PART.
