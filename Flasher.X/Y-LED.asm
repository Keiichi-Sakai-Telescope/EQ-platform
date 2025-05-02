						processor		16f1825
						#include		p16f1825.inc

						errorlevel -302
						radix		DEC

 __config		_CONFIG1,		_FOSC_HS & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_NSLEEP & _CLKOUTEN_OFF & _IESO_ON & _FCMEN_ON
 __config		_CONFIG2,		_WRT_OFF & _PLLEN_ON & _BORV_HI & _LVP_OFF

FCY						equ		4000000						; 4MHz
T1_FREQ					equ		FCY/4

LUT_ENTRY				macro	flash_freq_mHz
						local	dividend, divisor, period_i, period_f

dividend		= T1_FREQ*1000
divisor			= flash_freq_mHz
period_i		= dividend/divisor
period_f		= (dividend - period_i*divisor)*256/divisor

						dw		0x3F00 |   period_f
						dw		0x3F00 | ( period_i        & 0xFF)
						dw		0x3F00 | ((period_i >>  8) & 0xFF)
						dw		0x3F00 | ((period_i >> 16) & 0xFF)
						endm

; Timer configurations
						; PWM generator
T1CON_init				equ		(B'10'<<T1CKPS0)|(1 << TMR1ON)		; FCY, prescaler 1:4
CCP3CON_init			equ		(B'1001'<<CCP3M0)		; Compare mode: change pin from high to low on match

; etc.
OSCCON_init				equ		(B'1111'<<IRCF0)

; I/O port configurations
LATA_init				equ		B'000100'		;
WPUA_init				equ		B'001011'		; pull-up:                RA3, RA2, RA1, RA0
TRISA_active			equ		B'111011'		; input:                  RA3, RA2, RA1, RA0
ANSELA_init				equ		B'110100'		; analog input: RA5, RA4,                   

LATC_init				equ		B'000001'		; RC5=0 when CCPx relinquishs the pin.
WPUC_init				equ		B'000000'		; pull-up:                                  
TRISC_active			equ		B'111110'		; input:        RC5, RC4, RC3, RC2, RC1
ANSELC_init				equ		B'111110'		; analog input:                RC2, RC1

ADCON0_init				equ		(5<<CHS0)|(1<<ADON)			; AN5(RC1)
ADCON1_init				equ		(5<<ADCS0)|(0<<ADPREF0)|(0<<ADFM); Fosc/16, Vref+=VDD, flush left

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
bank1_ram				udata	0x0A0
s1_last_offset			res		1		; from 0x00 to 0x3C with step 4

; general purpose RAM (Bank 3) allocation
;bank3_ram				udata	0x1A0

; general purpose RAM (Bank 6) allocation
bank6_ram				udata	0x320
t6_on_time_end			res		3		; CCPR3
t6_off_time_end			res		3		; CCPR3
t6_on_duration			res		3		; CCPR3

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

;		the LED switch is driven by T1 cooperatively with CCP3.
interrupt_handler		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						clrf	PCLATH
						movlb	0
						bcf		s0_PIR3,CCP3IF

						movlb	6
						btfss	s6_CCP3CON,CCP3M0
						bra		led_turned_off

led_turned_on:			; parepare turn off
						bcf		s6_CCP3CON,CCP3M0
						movf	t6_on_time_end+2,W
						movwf	s6_CCPR3H
						movf	t6_on_time_end+1,W
						movwf	s6_CCPR3L			; minimul on duration is 16 Tcy
						retfie

led_turned_off:			; parepare turn on
						bsf		s6_CCP3CON,CCP3M0
						movf	t6_off_time_end+2,W
						movwf	s6_CCPR3H
						movf	t6_off_time_end+1,W
						movwf	s6_CCPR3L

						movlb	1
						lsrf	s1_ADRESH,W
						lsrf	WREG,F
						andlw	0xFE
						subwf	s1_last_offset,W
						; unchanged when W is between -4 and 2
						addlw	4
						addlw	-6
						btfsc	STATUS,C
						call	f1_read_offset

						movf	s1_last_offset,W
						addwf	FSR0L,F
						clrw
						addwfc	FSR0H,F

						bsf		s1_ADCON0,GO

						movlb	6
						; calculate on duration
						moviw	1[FSR0]
						movwf	t6_on_duration+1
						moviw	2[FSR0]
						movwf	t6_on_duration+2
						moviw	3[FSR0]

						; on time is 1/16 of off time
						lsrf	WREG,W
						rrf		t6_on_duration+2,F
						rrf		t6_on_duration+1,F
						lsrf	t6_on_duration+2,F
						rrf		t6_on_duration+1,F
						lsrf	t6_on_duration+2,F
						rrf		t6_on_duration+1,F
						lsrf	t6_on_duration+2,F
						rrf		t6_on_duration+1,F

						; update end time of on duration
						movf	t6_off_time_end+1,W
						addwf	t6_on_duration+1,W
						movwf	t6_on_time_end+1
						movf	t6_off_time_end+2,W
						addwfc	t6_on_duration+2,W
						movwf	t6_on_time_end+2
	if 0
						lsrf	t6_on_duration+2,F
						rrf		t6_on_duration+1,F
						movf	t6_on_duration+1,W
						addwf	t6_on_time_end+1,F
						movf	t6_on_duration+2,W
						addwfc	t6_on_time_end+2,F
	endif

						; update end time of off duration
						moviw	0[FSR0]
						addwf	t6_off_time_end+0,F
						moviw	1[FSR0]
						addwfc	t6_off_time_end+1,F
						moviw	2[FSR0]
						addwfc	t6_off_time_end+2,F
						retfie

f1_read_offset:			lsrf	s1_ADRESH,W
						lsrf	WREG,F
						andlw	0xFC
						movwf	s1_last_offset
						return

;		*		*		*		*		*		*		*		
;		Main routine
;		*		*		*		*		*		*		*		
start:					movlb	0
						clrf	s0_T1CON

						; clear data memory
						movlw	0x29
						movwf	FSR0H
						movlw	0xAF
						movwf	FSR0L
						clrw
						movwi	FSR0--
						btfsc	FSR0H,1
						bra		$-2

						movlb	1
						; analyze reset
						btfss	s1_PCON,NOT_POR
						bra		cold_boot

						bcf		s1_PCON,NOT_POR

						clrf	INTCON
						sleep				; method to wake-up is assertion of MCLR, POR or BOR

cold_boot:				bsf		s1_PCON,NOT_POR

						; set default state when CCP3 relinquishs the pin.
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

						movlw	ADCON0_init
						movwf	s1_ADCON0
						movlw	ADCON1_init
						movwf	s1_ADCON1

						movlb	3
						; input enable
						movlw	ANSELA_init
						movwf	s3_ANSELA
						movlw	ANSELC_init
						movwf	s3_ANSELC

						movlw	0xF0
						movwf	FSR0L
						clrf	FSR0H				; common memory is starting from 0x00F0
						clrf	INDF0
						incfsz	FSR0L,F
						goto	$-2

						movlw	LOW lut
						movwf	FSR0L
						movlw	HIGH lut
						movwf	FSR0H
						; FSR0 is pointing lut

						movlb	6
						; setup CCP3 for LED driver
						movlw	1				; first interrupt occurs at 256 T1CLK later
						movwf	s6_CCPR3H
						movlw	2
						movwf	t6_on_time_end+2
						movlw	3
						movwf	t6_off_time_end+2
						clrf	s6_CCPR3L
						clrf	t6_off_time_end+1
						clrf	t6_on_time_end+1
						movlw	CCP3CON_init
						movwf	s6_CCP3CON

						movlb	0
						; start timer for LED driver
						clrf	s0_TMR1L
						clrf	s0_TMR1H
						movlw	T1CON_init
						movwf	s0_T1CON

						; interrupt enable
						bcf		s0_PIR3,CCP3IF
						bsf		INTCON,PEIE
						bsf		INTCON,GIE

						movlb	1
						bsf		s1_PIE3,CCP3IE
						bsf		s1_ADCON0,GO

						btfsc	s1_ADCON0,GO
						bra		$-1
						call	f1_read_offset

main_loop:				nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						goto	main_loop

lut:					
	if 0
						LUT_ENTRY		1800*1000/60		; 1800rpm
						LUT_ENTRY		2700*1000/60		; 2700rpm, 900rpm with 3-slot
						LUT_ENTRY		3000*1000/60		; 3000rpm
						LUT_ENTRY		4050*1000/60		; 4050rpm, 1350rpm with 3-slot
						LUT_ENTRY		4500*1000/60		; 4500rpm, 563rpm with 8T, 1500rpm with 3-slot
						LUT_ENTRY		7200*1000/60		; 7200rpm, 900rpm with 8T, 800rpm with 9T
						LUT_ENTRY		8100*1000/60		; 8100rpm, 900rpm with 9T
						LUT_ENTRY		9000*1000/60		; 9000rpm, 900rpm with 10T, 1000rpm with 9T
						LUT_ENTRY		10800*1000/60		; 10800rpm, 900rpm with 12T, 1200rpm with 9T, 1350rpm with 8T
						LUT_ENTRY		12000*1000/60		; 12000rpm, 1500rpm with 8T
						LUT_ENTRY		12150*1000/60		; 12150rpm, 1350rpm with 9T
						LUT_ENTRY		13500*1000/60		; 13500rpm, 1350rpm with 10T, 1500rpm with 9T
						LUT_ENTRY		14400*1000/60		; 14400rpm, 900rpm with 16T, 1600rpm with 9T, 1800rpm with 8T
						LUT_ENTRY		15000*1000/60		; 15000rpm, 1500rpm with 10T
						LUT_ENTRY		16200*1000/60		; 16200rpm, 1350rpm with 12T
						LUT_ENTRY		18000*1000/60		; 18000rpm, 1500rpm with 12T, 1800rpm with 10T
	else
;		              1T              3T       8T       9T      10T      12T      14T
		LUT_ENTRY    900*1000/60 ;   300.00   112.50   100.00    90.00    75.00    64.29
		LUT_ENTRY   1125*1000/60 ;   375.00   140.63   125.00   112.50    93.75    80.36
		LUT_ENTRY   1440*1000/60 ;   480.00   180.00   160.00   144.00   120.00   102.86
		LUT_ENTRY   1800*1000/60 ;   600.00   225.00   200.00   180.00   150.00   128.57
		LUT_ENTRY   2250*1000/60 ;   750.00   281.25   250.00   225.00   187.50   160.71
		LUT_ENTRY   2880*1000/60 ;   960.00   360.00   320.00   288.00   240.00   205.71
		LUT_ENTRY   3600*1000/60 ;  1200.00   450.00   400.00   360.00   300.00   257.14
		LUT_ENTRY   4500*1000/60 ;  1500.00   562.50   500.00   450.00   375.00   321.43
		LUT_ENTRY   5760*1000/60 ;  1920.00   720.00   640.00   576.00   480.00   411.43
		LUT_ENTRY   7200*1000/60 ;  2400.00   900.00   800.00   720.00   600.00   514.29
		LUT_ENTRY   9000*1000/60 ;  3000.00  1125.00  1000.00   900.00   750.00   642.86
		LUT_ENTRY  11520*1000/60 ;  3840.00  1440.00  1280.00  1152.00   960.00   822.86
		LUT_ENTRY  14400*1000/60 ;  4800.00  1800.00  1600.00  1440.00  1200.00  1028.57
		LUT_ENTRY  18000*1000/60 ;  6000.00  2250.00  2000.00  1800.00  1500.00  1285.71
		LUT_ENTRY  23040*1000/60 ;  7680.00  2880.00  2560.00  2304.00  1920.00  1645.71
		LUT_ENTRY  28800*1000/60 ;  9600.00  3600.00  3200.00  2880.00  2400.00  2057.14
	endif

						end

