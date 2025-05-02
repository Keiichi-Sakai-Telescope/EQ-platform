
;								PPS			CLC/test case
TEST_NONE				equ		0x00 + 0x0000
TEST_DUTY_UPDATE		equ		0x00 + 0x0100
TEST_CALIBRATION		equ		0x00 + 0x0200
TEST_LED_DRIVER			equ		0x00 + 0x0300
TEST_NVM				equ		0x00 + 0x0400
TEST_CPU_USAGE			equ		0x00 + 0x0500
TEST_VOLTAGE_OUT		equ		0x00 + 0x0600
TEST_CLKR				equ		0x1A				; Rxy source is CLKR
TEST_CLC1OUT			equ		0x01				; Rxy source is CLC1OUT
TEST_CCP1_MATCH			equ		0x01 + 0x0C00		; Rxy source is CLC1OUT
TEST_PR6_MATCH			equ		0x01 + 0x2300		; Rxy source is CLC1OUT
TEST_PR2_MATCH			equ		0x01 + 0x1A00		; Rxy source is CLC1OUT
TEST_TMR0_OVERFLOW		equ		0x01 + 0x1800		; Rxy source is CLC1OUT
TEST_TMR1_OVERFLOW		equ		0x01 + 0x1900		; Rxy source is CLC1OUT
TEST_TMR3_OVERFLOW		equ		0x01 + 0x2000		; Rxy source is CLC1OUT
TEST_TMR5_OVERFLOW		equ		0x01 + 0x2200		; Rxy source is CLC1OUT
TEST_HFINTOSC			equ		0x01 + 0x1E00		; Rxy source is CLC1OUT
TEST_LFINTOSC			equ		0x01 + 0x1D00		; Rxy source is CLC1OUT
TEST_CCP4				equ		0x01 + 0x1600		; Rxy source is CLC1OUT
TEST_CCP3				equ		0x01 + 0x1500		; Rxy source is CLC1OUT
TEST_CCP2				equ		0x01 + 0x1400		; Rxy source is CLC1OUT
TEST_CCP1				equ		0x01 + 0x1300		; Rxy source is CLC1OUT
TEST_HB1_HS				equ		0x05				; Rxy source is CWG1A
TEST_HB1_LS				equ		0x06				; Rxy source is CWG1B
TEST_HB2_LS				equ		0x12				; Rxy source is C1
TEST_UART_CEMF			equ		0x10 + 0x0000		; Rxy source is TX/CK
TEST_UART_P				equ		0x10 + 0x0100		; Rxy source is TX/CK
TEST_UART_I				equ		0x10 + 0x0200		; Rxy source is TX/CK
TEST_HB1_LS_CLAMP		equ		0x05				; Rxy source is CWG1A
TEST_HB1_HS_OD			equ		0x1C				; Rxy source is CWG2A

TEST_TARGET				set		TEST_NONE

TEST_PIN_INIT			macro
PPS_RA2_init			set		TEST_TARGET & 0xFF
		if (PPS_RA2_init == (TEST_CLKR & 0xFF))			; Rxy source is CLKR
						movlb	62
						bcf		s62_SLRCONA,SLRA2
						movlb	17
						movlw	CLKRCON_init
						movwf	s17_CLKRCON
		endif
		if (PPS_RA2_init == (TEST_CLC1OUT & 0xFF))		; Rxy source is CLC1OUT
						movlb	62
						bcf		s62_SLRCONA,SLRA2
						movlb	60
						movlw	CLC1CON_init
						movwf	s60_CLC1CON
						movlw	CLC1POL_init
						movwf	s60_CLC1POL
						movlw	TEST_TARGET >> 8
						movwf	s60_CLC1SEL0
						movwf	s60_CLC1SEL1
						movwf	s60_CLC1SEL2
						movwf	s60_CLC1SEL3
						movlw	(1<<LC1G1D1T)
						movwf	s60_CLC1GLS0
						clrf	s60_CLC1GLS1
						clrf	s60_CLC1GLS2
						clrf	s60_CLC1GLS3
		endif
		if PPS_RA2_init == 0
			if (TEST_TARGET != TEST_VOLTAGE_OUT)
						movlw	LOW  LATA
						movwf	FSR1L
						movlw	HIGH LATA
						movwf	FSR1H
						bcf		INDF1,LATA2
			else
						movlb	18
						movlw	(1<<DAC1EN)|(1<<DAC1OE1)|(B'00'<<DAC1PSS0)
						movwf	s18_DAC1CON0
						movlw	LOW  DAC1CON1
						movwf	FSR1L
						movlw	HIGH DAC1CON1
						movwf	FSR1H
						clrf	INDF1
			endif
		else
						movlb	62
						movlw	PPS_RA2_init
						movwf	s62_RA2PPS
		endif
		if (PPS_RA2_init == (TEST_UART_CEMF & 0xFF))	; Rxy source is TX/CK
BAUD_RATE				set		38400
						movlb	62
						bcf		s62_SLRCONA,SLRA2
						bcf		s62_ANSELA,ANSA2
						movlb	2
						movlw	(1<<BRG16)
						movwf	s2_BAUD1CON
						movlw	LOW ((FCY + BAUD_RATE/2)/BAUD_RATE - 1)
						movwf	s2_SP1BRGL
						movlw	HIGH ((FCY + BAUD_RATE/2)/BAUD_RATE - 1)
						movwf	s2_SP1BRGH
						movlw	(1<<SPEN)
						movwf	s2_RC1STA
						movlw	(1<<TXEN)|(1<<BRGH)
						movwf	s2_TX1STA
						movlw	LOW  TX1REG
						movwf	FSR1L
						movlw	HIGH TX1REG
						movwf	FSR1H
		endif
	endm

TEST_PIN_ON				macro	test_name
		if test_name == TEST_TARGET
						bsf		INDF1,LATA2
		endif
	endm

TEST_PIN_OFF			macro	test_name
		if test_name == TEST_TARGET
						bcf		INDF1,LATA2
		endif
	endm

