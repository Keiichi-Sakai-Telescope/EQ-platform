
;								test case/DSM/PPS
TEST_NONE				equ		0x000000
TEST_DUTY_UPDATE		equ		0x010000
TEST_CALIBRATION		equ		0x020000
TEST_LED_DRIVER			equ		0x030000
TEST_NVM				equ		0x040000
TEST_CPU_USAGE			equ		0x050000
TEST_CLKR				equ		0x003111 		; Rxy source is DSM, MDSRC is MDBIT, MDCAR is CLKREF
TEST_HFINTPSC			equ		0x002111 		; Rxy source is DSM, MDSRC is MDBIT, MDCAR is HFINTOSC
TEST_FOSC				equ		0x001111 		; Rxy source is DSM, MDSRC is MDBIT, MDCAR is FOSC

TEST_PWM3				equ		0x000007		; Rxy source is PWM3
TEST_PWM4				equ		0x000008		; Rxy source is PWM4
TEST_CCP2				equ		0x000311		; Rxy source is DSM, MDSRC is CCP2
TEST_CCP1				equ		0x000211		; Rxy source is DSM, MDSRC is CCP1
TEST_CMP1				equ		0x00000B		; Rxy source is CMP1
TEST_CMP2				equ		0x00000C		; Rxy source is CMP2
TEST_UART_CEMF			equ		0x000911		; Rxy source is DSM, MDSRC is TX/CK
TEST_UART_P				equ		0x010911		; Rxy source is DSM, MDSRC is TX/CK
TEST_UART_I				equ		0x020911		; Rxy source is DSM, MDSRC is TX/CK

TEST_TARGET				set		TEST_NONE

TEST_LED_INIT			macro
						local	PPS_RA2_init, DSM_MDSRC_init, DSM_MDCAR_init, test_case
PPS_RA2_init			set		 TEST_TARGET        & 0xFF
DSM_MDSRC_init			set		(TEST_TARGET >>  8) & 0x0F
DSM_MDCAR_init			set		(TEST_TARGET >> 12) & 0x0F
test_case				set		(TEST_TARGET >> 16)
		if (DSM_MDSRC_init != 0)	; use DSM to re-route signal
						movlb	14
						movlw	PPS_DSM_init
						movwf	s14_MDSRCPPS,BANKED
						movwf	s14_MDCARLPPS,BANKED
						movwf	s14_MDCARHPPS,BANKED
						movlb	15
						movlw	MDCON0_init
						movwf	s15_MDCON0,BANKED
						movlw	MDCON1_init
						movwf	s15_MDCON1,BANKED
						movlw	DSM_MDSRC_init
						movwf	s15_MDSRC,BANKED
			if (DSM_MDCAR_init != 0)
						movlw	DSM_MDCAR_init
						movwf	s15_MDCARL,BANKED
						movwf	s15_MDCARH,BANKED
			else
			endif
		endif
		if (TEST_TARGET == TEST_CLKR)
						movlb	15
						movlw	CLKRCON_init
						movwf	s15_CLKRCON,BANKED
		endif
		if (PPS_RA2_init == 0)
						lfsr	1,LATA
						bcf		INDF1,LATA2
		else
						movlb	14
						movlw	PPS_RA2_init
						movwf	s14_RA2PPS,BANKED
		endif
						bcf		s15a_TRISA,TRISA2
		if (PPS_RA2_init == (TEST_UART_CEMF & 0xFFFF))
BAUD_RATE				set		38400
						movlb	6
						bcf		s6_SLRCONA,SLRA2
						movlb	3
						bcf		s3_ANSELA,ANSA2
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
						lfsr	1,TX1REG
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

