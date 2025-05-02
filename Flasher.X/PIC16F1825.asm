						processor		16f1825
						#include		p16f1825.inc

						errorlevel -302
						radix		DEC

 __config		_CONFIG1,		_FOSC_HS & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _CLKOUTEN_OFF & _IESO_ON & _FCMEN_ON
 __config		_CONFIG2,		_WRT_OFF & _PLLEN_ON & _BORV_HI & _LVP_OFF

FCY						equ		8000000						; 8MHz
T1_FREQ					equ		FCY/8
FLASH_FREQ				equ		120							; 900rpm with 8-tooth pinion, 120Hz, -5ppm

; Timer configurations
						; PWM generator
T1CON_init				equ		(B'11'<<T1CKPS0)|(1 << TMR1ON)		; prescaler 1:8
CCP1CON_init			equ		(B'1000'<<CCP1M0)		; Compare mode: change pin from low to high on match

; etc.
OSCCON_init				equ		(B'1110'<<IRCF0)

; I/O port configurations
LATA_init				equ		B'000000'		;
WPUA_init				equ		B'001111'		; pull-up:                RA3, RA2, RA1, RA0
TRISA_active			equ		B'001111'		; input:                  RA3, RA2, RA1, RA0
ANSELA_init				equ		B'110000'		; analog input: RA5, RA4,                   

LATC_init				equ		B'000000'		; RC5=0 when CCPx relinquishs the pin.
WPUC_init				equ		B'011111'		; pull-up:           RC4, RC3, RC2, RC1, RC0
TRISC_active			equ		B'011111'		; input:             RC4, RC3, RC2, RC1, RC0
ANSELC_init				equ		B'100000'		; analog input: RC5

; prefix
; s0_ means single byte in bank 0.
;		single byte memory		s
;		double byte memory		d
;		triple byte memory		t
;		quaduple byte memory	q
;		function				f
;		array						a

; common RAM allocation
;common_ram				udata_shr		0x70

; general purpose RAM (Bank 0) allocation
;bank0_ram				udata		0x020

; general purpose RAM (Bank 1) allocation
;bank1_ram				udata		0x0A0

; general purpose RAM (Bank 3) allocation
;bank3_ram				udata		0x1A0

; general purpose RAM (Bank 5) allocation
bank5_ram				udata		0x2A0
t5_on_time				res		3		; CCPR1
t5_off_time				res		3		; CCPR1

; general purpose RAM (Bank 6) allocation
;bank6_ram				udata		0x320

;-----Bank0------------------
s0_PORTA				EQU		PORTA
s0_PORTC				EQU		PORTC
s0_PIR1					EQU		PIR1
s0_PIR2					EQU		PIR2
s0_PIR3					EQU		PIR3
s0_TMR0					EQU		TMR0
s0_TMR1L				EQU		TMR1L
s0_TMR1H				EQU		TMR1H
s0_T1CON				EQU		T1CON
s0_T1GCON				EQU		T1GCON
s0_TMR2					EQU		TMR2
s0_PR2					EQU		PR2
s0_T2CON				EQU		T2CON
s0_CPSCON0				EQU		CPSCON0
s0_CPSCON1				EQU		CPSCON1

;-----Bank1------------------
s1_TRISA				EQU		TRISA
s1_TRISC				EQU		TRISC
s1_PIE1					EQU		PIE1
s1_PIE2					EQU		PIE2
s1_PIE3					EQU		PIE3
s1_OPTION_REG			EQU		OPTION_REG
s1_PCON					EQU		PCON
s1_WDTCON				EQU		WDTCON
s1_OSCTUNE				EQU		OSCTUNE
s1_OSCCON				EQU		OSCCON
s1_OSCSTAT				EQU		OSCSTAT
s1_ADRESL				EQU		ADRESL
s1_ADRESH				EQU		ADRESH
s1_ADCON0				EQU		ADCON0
s1_ADCON1				EQU		ADCON1

;-----Bank2------------------
s2_LATA					EQU		LATA
s2_LATC					EQU		LATC
s2_CM1CON0				EQU		CM1CON0
s2_CM1CON1				EQU		CM1CON1
s2_CM2CON0				EQU		CM2CON0
s2_CM2CON1				EQU		CM2CON1
s2_CMOUT				EQU		CMOUT
s2_BORCON				EQU		BORCON
s2_FVRCON				EQU		FVRCON
s2_DACCON0				EQU		DACCON0
s2_DACCON1				EQU		DACCON1
s2_SRCON0				EQU		SRCON0
s2_SRCON1				EQU		SRCON1
s2_APFCON0				EQU		APFCON0
s2_APFCON1				EQU		APFCON1

;-----Bank3------------------
s3_ANSELA				EQU		ANSELA
s3_ANSELC				EQU		ANSELC
s3_EEADRL				EQU		EEADRL
s3_EEADRH				EQU		EEADRH
s3_EEDATL				EQU		EEDATL
s3_EEDATH				EQU		EEDATH
s3_EECON1				EQU		EECON1
s3_EECON2				EQU		EECON2
s3_RCREG				EQU		RCREG
s3_TXREG				EQU		TXREG
s3_SPBRG				EQU		SPBRG
s3_SPBRGL				EQU		SPBRGL
s3_SPBRGH				EQU		SPBRGH
s3_RCSTA				EQU		RCSTA
s3_TXSTA				EQU		TXSTA
s3_BAUDCON				EQU		BAUDCON

;-----Bank4------------------
s4_WPUA					EQU		WPUA
s4_WPUC					EQU		WPUC
s4_SSP1BUF				EQU		SSP1BUF
s4_SSPBUF				EQU		SSPBUF
s4_SSP1ADD				EQU		SSP1ADD
s4_SSPADD				EQU		SSPADD
s4_SSP1MSK				EQU		SSP1MSK
s4_SSPMSK				EQU		SSPMSK
s4_SSP1STAT				EQU		SSP1STAT
s4_SSPSTAT				EQU		SSPSTAT
s4_SSP1CON1				EQU		SSP1CON1
s4_SSPCON				EQU		SSPCON
s4_SSPCON1				EQU		SSPCON1
s4_SSP1CON2				EQU		SSP1CON2
s4_SSPCON2				EQU		SSPCON2
s4_SSP1CON3				EQU		SSP1CON3
s4_SSPCON3				EQU		SSPCON3

;-----Bank5------------------
s5_CCPR1L				EQU		CCPR1L
s5_CCPR1H				EQU		CCPR1H
s5_CCP1CON				EQU		CCP1CON
s5_PWM1CON				EQU		PWM1CON
s5_CCP1AS				EQU		CCP1AS
s5_ECCP1AS				EQU		ECCP1AS
s5_PSTR1CON				EQU		PSTR1CON
s5_CCPR2L				EQU		CCPR2L
s5_CCPR2H				EQU		CCPR2H
s5_CCP2CON				EQU		CCP2CON
s5_PWM2CON				EQU		PWM2CON
s5_CCP2AS				EQU		CCP2AS
s5_PSTR2CON				EQU		PSTR2CON
s5_CCPTMRS				EQU		CCPTMRS

;-----Bank6------------------
s6_CCPR3L				EQU		CCPR3L
s6_CCPR3H				EQU		CCPR3H
s6_CCP3CON				EQU		CCP3CON
s6_CCPR4L				EQU		CCPR4L
s6_CCPR4H				EQU		CCPR4H
s6_CCP4CON				EQU		CCP4CON

;-----Bank7------------------
s7_INLVLA				EQU		INLVLA
s7_INLVLC				EQU		INLVLC
s7_IOCAP				EQU		IOCAP
s7_IOCAN				EQU		IOCAN
s7_IOCAF				EQU		IOCAF
s7_CLKRCON				EQU		CLKRCON
s7_MDCON				EQU		MDCON
s7_MDSRC				EQU		MDSRC
s7_MDCARL				EQU		MDCARL
s7_MDCARH				EQU		MDCARH

;-----Bank8------------------
s8_TMR4					EQU		TMR4
s8_PR4					EQU		PR4
s8_T4CON				EQU		T4CON
s8_TMR6					EQU		TMR6
s8_PR6					EQU		PR6
s8_T6CON				EQU		T6CON

;		*		*		*		*		*		*		*		
;		Reset vector
;		*		*		*		*		*		*		*		
reset_vector			code	0x000
						movlb	1
						movlw	OSCCON_init
						movwf	s1_OSCCON
						goto	start

;		*		*		*		*		*		*		*		
;		Interrupt Routine
;		*		*		*		*		*		*		*		

;		the LED switch is driven by T1 cooperatively with CCP1.
interrupt_handler		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5, fluctuation 7
						clrf	PCLATH
						movlb	0
						bcf		s0_PIR1,CCP1IF

						movlb	5
						btfsc	s5_CCP1CON,CCP1M0
						goto	$+7
low_to_high:			; from low to high
						bsf		s5_CCP1CON,CCP1M0		; transit from high to low at next match

						movf	t5_on_time+2,W
						movwf	s5_CCPR1H
						movf	t5_on_time+1,W
						movwf	s5_CCPR1L
						retfie

high_to_low:			; from high to low
						bcf		s5_CCP1CON,CCP1M0		; transit from low to high at next match

						movf	t5_off_time+2,W
						movwf	s5_CCPR1H
						movf	t5_off_time+1,W
						movwf	s5_CCPR1L

						moviw	1[FSR0]
						movwf	t5_on_time+1
						moviw	2[FSR0]
						movwf	t5_on_time+2
						lsrf	t5_on_time+2,F
						rrf		t5_on_time+1,F
						lsrf	t5_on_time+2,F
						rrf		t5_on_time+1,F
						lsrf	t5_on_time+2,F
						rrf		t5_on_time+1,F
						lsrf	t5_on_time+2,F
						rrf		t5_on_time+1,F

						movf	t5_off_time+1,W
						addwf	t5_on_time+1,F
						movf	t5_off_time+2,W
						addwfc	t5_on_time+2,F

						moviw	0[FSR0]
						addwf	t5_off_time+0,F
						moviw	1[FSR0]
						addwfc	t5_off_time+1,F
						moviw	2[FSR0]
						addwfc	t5_off_time+2,F
						retfie

;		*		*		*		*		*		*		*		
;		Main routine
;		*		*		*		*		*		*		*		
start:
						; set default state when CCP3/CCP1 relinquishs the pin.
						movlb	2
						movlw	LATA_init
						movwf	s2_LATA
						movlw	LATC_init
						movwf	s2_LATC

						movlb	7
						movlw	B'111111'
						movwf	s7_INLVLA		; All input levels are ST buffer
						movwf	s7_INLVLC		; All input levels are ST buffer

						movlb	4
						movlw	WPUA_init
						movwf	s4_WPUA
						movlw	WPUC_init
						movwf	s4_WPUC

						movlb	1
						; enable pull-up
						bcf		s1_OPTION_REG,NOT_WPUEN
						; output enable
						movlw	TRISC_active
						movwf	s1_TRISC
						movlw	TRISA_active
						movwf	s1_TRISA

						movlb	3
						; input enable
						movlw	ANSELA_init
						movwf	s3_ANSELA
						movlw	ANSELC_init
						movwf	s3_ANSELC

						; clear data memory
						movlw	0x20
						movwf	FSR0H
						clrf	FSR0L				; linear data memory is starting from 0x2000
						movlw	(1024-16)/4
						movwf	0x7F
						clrw
						movwi	FSR0++
						movwi	FSR0++
						movwi	FSR0++
						movwi	FSR0++
						decfsz	0x7F,F
						goto	$-5

						movlw	0xF0
						movwf	FSR0L
						clrf	FSR0H				; common memory is starting from 0x00F0
						clrf	INDF0
						incfsz	FSR0L,F
						goto	$-2
						; FSR0H is set to zero except for initialization procedure.

						movlw	LOW lut
						movwf	FSR0L
						movlw	HIGH lut
						movwf	FSR0H

						movlb	5
						; setup CCP1 for LED driver
						movlw	1
						movwf	s5_CCPR1H
						movlw	2
						movwf	t5_on_time+2
						movlw	3
						movwf	t5_off_time+2
						clrf	s5_CCPR1L
						clrf	t5_off_time+1
						clrf	t5_on_time+1
						movlw	CCP1CON_init
						movwf	s5_CCP1CON

						movlb	0
						; start timer for LED driver
						clrf	s0_TMR1L
						clrf	s0_TMR1H
						movlw	T1CON_init
						movwf	s0_T1CON

						; interrupt enable
						bcf		s0_PIR1,CCP1IF
						bsf		INTCON,PEIE
						bsf		INTCON,GIE

						movlb	1
						bsf		s1_PIE1,CCP1IE

main_loop:				
						goto	main_loop

period_high		set		T1_FREQ/FLASH_FREQ/256
period_low		set		(T1_FREQ-FLASH_FREQ*period_high*256)/FLASH_FREQ
period_frac		set		(T1_FREQ-FLASH_FREQ*period_high*256 - FLASH_FREQ*period_low)*256/FLASH_FREQ

lut:					dw		period_frac
						dw		period_low
						dw		period_high

						end

