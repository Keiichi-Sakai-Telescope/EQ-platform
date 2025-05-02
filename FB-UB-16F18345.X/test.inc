
;								PPS			CLC/test case
TEST_NONE				equ		B'00000' + 0x0000
TEST_DUTY_UPDATE		equ		B'00000' + 0x0100
TEST_CALIBRATION		equ		B'00000' + 0x0200
TEST_LED_DRIVER			equ		B'00000' + 0x0300
TEST_NVM				equ		B'00000' + 0x0400
TEST_CPU_USAGE			equ		B'00000' + 0x0500
TEST_CLKR				equ		B'11110'				; Rxy source is CLKR
TEST_CLC1OUT			equ		B'00100'				; Rxy source is CLC1OUT
TEST_CCP1_MATCH			equ		B'00100' + 0x0C00		; Rxy source is CLC1OUT
TEST_PR6_MATCH			equ		B'00100' + 0x2300		; Rxy source is CLC1OUT
TEST_PR2_MATCH			equ		B'00100' + 0x1A00		; Rxy source is CLC1OUT
TEST_TMR0_OVERFLOW		equ		B'00100' + 0x1800		; Rxy source is CLC1OUT
TEST_TMR1_OVERFLOW		equ		B'00100' + 0x1900		; Rxy source is CLC1OUT
TEST_TMR3_OVERFLOW		equ		B'00100' + 0x2000		; Rxy source is CLC1OUT
TEST_TMR5_OVERFLOW		equ		B'00100' + 0x2200		; Rxy source is CLC1OUT
TEST_HFINTOSC			equ		B'00100' + 0x1E00		; Rxy source is CLC1OUT
TEST_LFINTOSC			equ		B'00100' + 0x1D00		; Rxy source is CLC1OUT
TEST_CCP4				equ		B'01111'				; Rxy source is CCP4
TEST_CCP3				equ		B'01110'				; Rxy source is CCP3
TEST_CCP2				equ		B'01101'				; Rxy source is CCP2
TEST_CCP1				equ		B'01100'				; Rxy source is CCP1
TEST_HB1_HS				equ		B'01000'				; Rxy source is CWG1A
TEST_HB1_LS				equ		B'01001'				; Rxy source is CWG1B
TEST_HB2_LS				equ		B'10110'				; Rxy source is C1
TEST_UART_CEMF			equ		B'10100' + 0x0000		; Rxy source is TX/CK
TEST_UART_P				equ		B'10100' + 0x0100		; Rxy source is TX/CK
TEST_UART_I				equ		B'10100' + 0x0200		; Rxy source is TX/CK

TEST_TARGET				set		TEST_NONE

TEST_PIN_INIT			macro	tris_adrs, tris_bit
PPS_RC4_init			set		TEST_TARGET & 0xFF
		if (PPS_RC4_init == (TEST_CLKR & 0xFF))			; Rxy source is CLKR
						movlb	6
						bcf		s6_SLRCONC,SLRC4
						movlb	7
						movlw	CLKRCON_init
						movwf	s7_CLKRCON
		endif
		if (PPS_RC4_init == (TEST_CLC1OUT & 0xFF))		; Rxy source is CLC1OUT
						movlb	30
						movlw	CLC1CON_init
						movwf	s30_CLC1CON
						movlw	CLC1POL_init
						movwf	s30_CLC1POL
						movlw	TEST_TARGET >> 8
						movwf	s30_CLC1SEL0
						movwf	s30_CLC1SEL1
						movwf	s30_CLC1SEL2
						movwf	s30_CLC1SEL3
						movlw	(1<<LC1G1D1T)
						movwf	s30_CLC1GLS0
						clrf	s30_CLC1GLS1
						clrf	s30_CLC1GLS2
						clrf	s30_CLC1GLS3
		endif
		if PPS_RC4_init == 0
						movlw	LOW  LATC
						movwf	FSR1L
						movlw	HIGH LATC
						movwf	FSR1H
						bcf		INDF1,LATC4
		else
						movlb	29
						movlw	PPS_RC4_init
						movwf	s29_RC4PPS
		endif
		if (PPS_RC4_init == (TEST_UART_CEMF & 0xFF))	; Rxy source is TX/CK
BAUD_RATE				set		38400
						movlb	3
						bcf		s3_ANSELC,ANSELC4
						movlw	(1<<BRG16)
						movwf	s3_BAUD1CON
						movlw	LOW ((FCY + BAUD_RATE/2)/BAUD_RATE - 1)
						movwf	s3_SP1BRGL
						movlw	HIGH ((FCY + BAUD_RATE/2)/BAUD_RATE - 1)
						movwf	s3_SP1BRGH
						movlw	(1<<SPEN)
						movwf	s3_RC1STA
						movlw	(1<<TXEN)|(1<<BRGH)
						movwf	s3_TX1STA
						movlw	LOW  TX1REG
						movwf	FSR1L
						movlw	HIGH TX1REG
						movwf	FSR1H
		endif
		if TEST_TARGET != TEST_NONE
						movlb	1
						bcf		tris_adrs,tris_bit
		endif
	endm

TEST_PIN_ON				macro	test_name
		if test_name == TEST_TARGET
						bcf		INDF1,LATC4
		endif
	endm

TEST_PIN_OFF			macro	test_name
		if test_name == TEST_TARGET
						bsf		INDF1,LATC4
		endif
	endm

