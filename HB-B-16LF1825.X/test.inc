
TEST_NONE				equ		0
TEST_DUTY_UPDATE		equ		1
TEST_CALIBRATION		equ		2
TEST_NVM				equ		3
TEST_CPU_USAGE			equ		4

TEST_TARGET				set		TEST_NONE

TEST_LED_INIT			macro	tris_adrs,tris_bit
		if TEST_TARGET != TEST_NONE
						movlw	LOW  (tris_adrs)
						movwf	FSR1L
						movlw	HIGH (tris_adrs)
						movwf	FSR1H
						bcf		INDF1,tris_bit

						movlw	LOW  SRCON0
						movwf	FSR1L
						movlw	HIGH SRCON0
						movwf	FSR1H
						BANKSEL	SRCON0
						movlw	SRCON0_FORCE_LED_OFF
						movwf	SRCON0		; turn off LED
		endif
	endm

TEST_LED_ON				macro	test_name
		if (test_name) == TEST_TARGET
			if (SRCON0_FORCE_LED_ON & (1<<SRPR))
						bsf		INDF1,SRPR		; turn on LED
			else
						bsf		INDF1,SRPS		; turn on LED
			endif
		endif
	endm

TEST_LED_OFF			macro	test_name
		if (test_name) == TEST_TARGET
			if (SRCON0_FORCE_LED_OFF & (1<<SRPS))
						bsf		INDF1,SRPS		; turn off LED
			else
						bsf		INDF1,SRPR		; turn off LED
			endif
		endif
	endm

