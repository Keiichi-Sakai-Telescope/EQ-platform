						#include		mpasmx.inc

						errorlevel -302
						radix	DEC

FCY						equ		4000000	; 4MHz, instruction cycle
PWM_PERIOD				equ		200		; PWM frequency = FCY/PWM_PERIOD, min: 200, max: 256
OPEN_LOOP_PERIOD		equ		62500					; fixed settings

 __config	_CONFIG1,	_FOSC_HS & _WDTE_ON & _MCLRE_OFF & _BOREN_NSLEEP
 __config	_CONFIG2,	_PLLEN_ON & _BORV_LO & _LVP_OFF & _STVREN_ON

LOW_BATT_ERROR			set		2700	; [mV]
LOW_BATT_WARNING		set		3000	; [mV]

POLLING_FREQUENCY		equ		4

SIDEREAL_TIME_0p16		equ		65536		; [0.5, 1.5)
SOLAR_TIME_0p16			equ		65715		; [0.5, 1.5)
LUNAR_TIME_0p16			equ		68019		; [0.5, 1.5)

DECIDE_PRESCALER		macro	MOTOR_100TH_RPM
						local	sampling_rpm, threshold_8th

sampling_rpm = (MOTOR_100TH_RPM)*2	; double sampling / wave
		if (MODE_NORMAL_SPEED == SAMPLING_SINGLE) || (MODE_NORMAL_SPEED == SAMPLING_MOVING_AVERAGE) || (MODE_DOUBLE_SPEED == SAMPLING_HALF)
sampling_rpm = (MOTOR_100TH_RPM)	; single sampling / wave
		endif
		if (MODE_NORMAL_SPEED == SAMPLING_HALF)
sampling_rpm = (MOTOR_100TH_RPM)/2	; half sampling / wave
		endif

; calculate threshold by the slowest speed
; FCY*1000/65536 * LUNAR_TIME_0p16/65536 / 4 + 1
threshold_8th = ( ( (FCY*(1000/4)) / 65536) * (LUNAR_TIME_0p16) ) / 65536 + 1

		if ((sampling_rpm) <= threshold_8th)
T1_PRESCALER			equ		3	; 1:8
		else
			if ((sampling_rpm) <= (threshold_8th*2))
T1_PRESCALER			equ		2	; 1:4
			else
				if ((sampling_rpm) <= (threshold_8th*4))
T1_PRESCALER			equ		1	; 1:2
				else
T1_PRESCALER			equ		0	; 1:1
				endif
			endif
		endif
	endm
#define	FCY_SCALED	(FCY >> T1_PRESCALER)

; sampling period[clock] = FCY_SCALED[clock/s]*10/MOTOR_RPM
LUT_ENTRIES				macro	MOTOR_100TH_RPM
						local	dividend, divisor, period_i, period_f, fraction

dividend = FCY_SCALED*(1000 >> 3)
	if (MODE_NORMAL_SPEED == SAMPLING_SINGLE) || (MODE_NORMAL_SPEED == SAMPLING_MOVING_AVERAGE)
divisor  = (MOTOR_100TH_RPM)	; normal sampling
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_DOUBLE)
divisor  = (MOTOR_100TH_RPM)*2	; double sampling
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_HALF)
divisor  = (MOTOR_100TH_RPM)/2	; half sampling
	endif
period_i = dividend/divisor
period_f = ((dividend - period_i*divisor) << 3)*32/divisor
period_i = (period_i << 3) + (period_f >> 5)
period_f = period_f & 0x1F

fraction = period_f
		dw		0x3F00 | (LOW period_i)
		dw		0x3F00 | (HIGH period_i)
		while fraction != 0
fraction += period_f
				if fraction >= 32
fraction -= 32
						dw		0x3F00 | (LOW (period_i + 1))
						dw		0x3F00 | (HIGH (period_i + 1))
				else
						dw		0x3F00 | (LOW period_i)
						dw		0x3F00 | (HIGH period_i)
				endif
		endw
						endm

LUT_ENTRY				macro	MOTOR_10TH_RPM
						local	period_i

	if (MODE_NORMAL_SPEED == SAMPLING_SINGLE) || (MODE_NORMAL_SPEED == SAMPLING_MOVING_AVERAGE)
period_i = (FCY_SCALED*100 + (MOTOR_10TH_RPM)/2)/(MOTOR_10TH_RPM)	; normal sampling
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_DOUBLE)
period_i = (FCY_SCALED*50 + (MOTOR_10TH_RPM)/2)/(MOTOR_10TH_RPM)	; double sampling
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_HALF)
period_i = (FCY_SCALED*200 + (MOTOR_10TH_RPM)/2)/(MOTOR_10TH_RPM)	; half sampling
	endif

						dw		0x3F00 | (LOW period_i)
						dw		0x3F00 | (HIGH period_i)
						endm

_CYCLE_TIME_L			equ		0						; cycle time is [T1CK] in 16p0
_CYCLE_TIME_H			equ		1
_SIZEOF_LUT_ENTRY		equ		2						; fixed. don't change

; gain
;P_GAIN					proportional gain(amplitude)
;I_TIME_8p8				integration time(amplitude) where the range is from 2.0 to 256.0
; proportional_error = P_GAIN*(target_voltage - cemf_voltage)
; duty_ratio = (proportional_error + integral_error) / supplied_voltage
;	integral_error += proportional_error/I_TIME_8p8

; sampling mode
;MODE_NORMAL_SPEED		sampling mode at normal speed
;MODE_DOUBLE_SPEED		sampling mode at double speed
;	sampling modes are following.
SAMPLING_SINGLE			equ		0		; single sampling
SAMPLING_MOVING_AVERAGE	equ		1		; single sampling and averaging with previous sample
SAMPLING_DOUBLE			equ		2		; double sampling with phase control
SAMPLING_HALF			equ		3		; half sampling

lut						code
pfm_lut_start:
						#include ../motor.inc
pfm_lut_end:
OPEN_LOOP_FREQ			equ		(FCY_SCALED/OPEN_LOOP_PERIOD)
						LUT_ENTRY	OPEN_LOOP_FREQ*100

; Vin*duty = Rcoil*Is + target_voltage
; duty = (Rcoil*Is + target_voltage)/Vin
STARTUP_VOLTAGE_15p1	equ		MOTOR_RESISTANCE*STARTUP_CURRENT*3/2*2/1000	; IR-drop, 2LSB=1mV	[V]

; target_voltage[LSB] = FCY_SCALED/period*10 / MOTOR_SPEED_CONSTANT * 500[LSB/V]
;                     = FCY_SCALED*5000/MOTOR_SPEED_CONSTANT / period
; FCY_SCALED*5000/MOTOR_SPEED_CONSTANT = period*target_voltage

MEAN_MOTOR_SPEED_CONSTANT   = (MOTOR_SPEED_CONSTANT*PEAK_WAVE_HEIGHT + CENTER_WAVE_HEIGHT/2)/CENTER_WAVE_HEIGHT
; MOTOR_CONSTANT_init = FCY_SCALED*10*500[LSB/V]/MEAN_MOTOR_SPEED_CONSTANT
MOTOR_CONSTANT_init = (FCY_SCALED*10)                                                /MEAN_MOTOR_SPEED_CONSTANT
MOTOR_CONSTANT_init = (FCY_SCALED*10 - MOTOR_CONSTANT_init*MEAN_MOTOR_SPEED_CONSTANT)*500/MEAN_MOTOR_SPEED_CONSTANT + MOTOR_CONSTANT_init*500

						; Test
SRCON0_FORCE_LED_ON		equ		(1<<SRLEN)|(1<<SRQEN)|(1<<SRPS)	; LED is forced turnng ON.
SRCON0_RELEASE_LED		equ		(0<<SRLEN)|(1<<SRQEN)|(1<<SRPS)	; LED is released.
SRCON0_FORCE_LED_OFF	equ		(1<<SRLEN)|(1<<SRQEN)|(1<<SRPR)	; LED is forced turnng OFF.
						#include ../HB-B-16F1825.X/test.inc

	if (MODE_NORMAL_SPEED == SAMPLING_SINGLE)
MULT_PERIOD_1X_24p8		equ		256		; 1.0
CALC_SAMPLES_1X			equ		0		; no calculation
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_MOVING_AVERAGE)
MULT_PERIOD_1X_24p8		equ		256		; 1.0
CALC_SAMPLES_1X			equ		1		; averaging
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_DOUBLE)
MULT_PERIOD_1X_24p8		equ		128		; 0.5
CALC_SAMPLES_1X			equ		2		; phase control
	endif
	if (MODE_NORMAL_SPEED == SAMPLING_HALF)
MULT_PERIOD_1X_24p8		equ		512		; 2.0
CALC_SAMPLES_1X			equ		0		; no calculation
	endif
MULT_VOLTAGE_1X_24p8	equ		MULT_PERIOD_1X_24p8
MULT_ITIME_1X_24p8		equ		65536/MULT_PERIOD_1X_24p8

	if (MODE_DOUBLE_SPEED == SAMPLING_SINGLE)
MULT_PERIOD_2X_24p8		equ		256/2	; 0.5
CALC_SAMPLES_2X			equ		0		; no calculation
	endif
	if (MODE_DOUBLE_SPEED == SAMPLING_MOVING_AVERAGE)
MULT_PERIOD_2X_24p8		equ		256/2	; 0.5
CALC_SAMPLES_2X			equ		1		; averaging
	endif
	if (MODE_DOUBLE_SPEED == SAMPLING_DOUBLE)
MULT_PERIOD_2X_24p8		equ		128/2	; 0.25
CALC_SAMPLES_2X			equ		2		; phase control
	endif
	if (MODE_DOUBLE_SPEED == SAMPLING_HALF)
MULT_PERIOD_2X_24p8		equ		512/2	; 1.0
CALC_SAMPLES_2X			equ		0		; no calculation
	endif
MULT_VOLTAGE_2X_24p8	equ		2*MULT_PERIOD_2X_24p8
MULT_ITIME_2X_24p8		equ		65536/MULT_PERIOD_2X_24p8

; Timer configurations
						; Timer assignment
						;        CCP4  CCP3 ECCP2 ECCP1
						; 16bit   *T1    T1    T1    T1
						;  8bit    T2   *T6   *T2   *T2
CCPTMRS_init			equ		(B'00'<<C4TSEL0)|(B'10'<<C3TSEL0)|(B'00'<<C2TSEL0)|(B'00'<<C1TSEL0)

						; Trigger for A/D conversion of CEMF
T1CON_init				equ		(T1_PRESCALER<<T1CKPS0)|(1<<TMR1ON)
CCP4CON_init			equ		(B'1011'<<CCP4M0)		; special event trigger

						; PWM generator
T2CON_init				equ		(B'000'<<T2CKPS0)|(1<<TMR2ON)	; FCY, pre 1:1, post 1:1
T2PR_init				equ		PWM_PERIOD - 1
CCP1CON_init			equ		B'10001110'				; PWM HB, P1A: active L, P1B: active H
PWM1CON_init			equ		(1<<P1RSEN)|1			; delay time is 1*Tcy = 250ns
CCP1AS_init				equ		(B'001'<<CCP1AS0)|(B'01'<<PSS1AC0)|(B'00'<<PSS1BD0)		; by CM1
CM1CON0_init			equ		(0<<C1ON)|(1<<C1POL)|(1<<C1SP)|(0<<C1HYS)
CM2CON0_init			equ		(1<<C2ON)|(1<<C2POL)|(1<<C2SP)|(1<<C2HYS)
CM2CON1_init			equ		B'00110010'		; RC2/AN6 -> C12IN2-
WPUC_CEMF_SENSE			equ		WPUC2
TRISC_CEMF_SENSE		equ		TRISC2

						; Trigger for A/D conversion of FVR
FVR_SETTLING_TIME		set		10
T4CON_init				equ		(B'01'<<T4CKPS0)|(1<<TMR4ON)|((CEMF_SETTLING_TIME&0x0F00)>>5)	; FCY/4
TMR4_init				equ		~CEMF_SETTLING_TIME & 0x00FF
T4PR_init				equ		0xFF

						; Constant brightness LED driver
T6CON_init				equ		(B'01'<<T6CKPS0)|(1<<TMR6ON) ; FCY/4, pre 1:4, post 1:1
T6PR_init				equ		255
CCP3CON_init			equ		(B'1100'<<CCP3M0)		; Normal PWM mode
CCPR3_init				equ		0
OPTION_REG_init			equ		(1<<TMR0CS)|(1<<TMR0SE)|(1<<PSA)	; Timer0 falling edge on T0CKI without prescaler
IOCA_LED_ANODE			equ		IOCAP2					; anode of LED on RA2
TRISA_LED_ANODE			equ		TRISA2

; etc.
OSCCON_sleep			equ		(B'1111'<<IRCF0)|(B'11'<<SCS0)	; HFINTOSC 16MHz, use INTOSC
OSCCON_backup			equ		(B'1111'<<IRCF0)|(B'00'<<SCS0)	; HFINTOSC 16MHz, use EXTOSC
WDTCON_active			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON_sleep			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON_calibration		equ		(B'01010'<<WDTPS0)		; 1s nominal, 750ms minimum, 1250ms maximum
PCON_init				equ		(1<<NOT_BOR)|(1<<NOT_POR)|(1<<NOT_RI)|(1<<NOT_RMCLR)
PCON_cold_boot			equ		(1<<NOT_BOR)|(1<<NOT_RMCLR)
INTCON_sleep			equ		(1<<PEIE)
INTCON_active			equ		(1<<PEIE)|(1<<GIE)

; ADC and FVR configurations
FVRCON_init				equ		B'11000010'		; CDAFVR = 00 for OFF, ADFVR = 10 for 2.048V
ADCON0_MUX_FVR			equ		(31<<CHS0)|(1<<ADON)		; AN31(FVR output)
ADCON0_MUX_MOTOR		equ		( 6<<CHS0)|(1<<ADON)		; AN6(motor, RC2 pin)
ADCON0_MUX_LED			equ		( 2<<CHS0)|(1<<ADON)		; AN2(LED, RA2 pin)
ADCON1_VDD_RATIOMETRIC	equ		(5<<ADCS0)|(0<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=VDD, flush right
ADCON1_FVR_ABSOLUTE		equ		(5<<ADCS0)|(3<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=FVR, flush right

; I/O port configurations
LATA_init				equ		B'000000'		; digital output:                RA2
WPUA_init				equ		B'001011'		; pull-up:                  RA3,      RA1, RA0
TRISA_active			equ		B'001011'		; input:                    RA3,      RA1, RA0
ANSELA_init				equ		B'110100'		; digital input:            RA3,      RA1, RA0 
ANSELA_LED_ANODE		equ		ANSA2			; anode of LED on RA2

LATC_init				equ		B'00101000'		; digital output: RC5, RC4, RC3
WPUC_init				equ		B'00000011'		; pull-up:                            RC1, RC0
TRISC_active			equ		B'00000111'		; input:                         RC2, RC1, RC0
ANSELC_init				equ		B'11111100'		; digital input:                      RC1, RC0

; bit position of notification_flag
BIT_CLOSED_LOOP_CTRL	equ		0	;-----------+
BIT_OPEN_LOOP_CTRL		equ		1	;----------+|
BIT_DOUBLE_VOLTAGE		equ		2	;---------+||
BIT_ADVANCE_LUT_INDEX	equ		3	;--------+|||
BIT_OUT_OF_CTRL			equ		4	;-------+||||
BIT_POLLING				equ		5	;------+|||||
									;-----+||||||
									;----+|||||||
;									     ||||||||
CTRL_MOTOR_BREAK_IN		equ			   B'00000010'
CTRL_MOTOR_2x			equ			   B'00001101'
CTRL_MOTOR_1x			equ			   B'00001001'
CTRL_MOTOR_STOP			equ			   B'00010000'
CTRL_MOTOR_FREE			equ			   B'00011000'

; bit position of sw_state, sw_released, sw_pressed, sw_toggle
BIT_SW_LIMIT			equ		0		; RA0	limit SW			edge (pressing)
BIT_SW_ORIGIN			equ		1		; RA1	origin SW			level
BIT_SW_RESET			equ		3		; RA3	power SW			alternate
BIT_SW_STOP				equ		4		; RC0	stop SW				level
BIT_SW_DOUBLE_SPEED		equ		5		; RC1	double speed SW		level

; common RAM allocation
common_ram				udata_shr		0x70
pulse_width_L			res		1		; 6p10	[V/V]
pulse_width_H			res		1
notification_flag		res		1		; flags for notification
cemf_sampling_period_L	res		1		; T1_LSB	[s]
cemf_sampling_period_H	res		1
lut_address_L			res		1
lut_address_H			res		1
lut_lookup_time			res		1		; [s]
motor_constant_LL		res		1		; ticks*ADRES_LSB	[s*V]
motor_constant_LH		res		1
motor_constant_HL		res		1
motor_constant_HH		res		1

; prefix for a variable
; s0_ means single byte in bank 0.
;		single byte memory		s
;		double byte memory		d
;		triple byte memory		t
;		quadruple byte memory	q
;		array					a

; prefix for a function
; f1_ means a function returning BSR with 1.
; caller saved registers.
;	W
;	STATUS
;	BSR
;	FSR0L, FSR0H
;	PCLATH

;-----Bank0------------------
s0_PORTA				EQU		PORTA
s0_PORTC				EQU		PORTC
s0_PIR1					EQU		PIR1
s0_PIR2					EQU		PIR2
s0_PIR3					EQU		PIR3
s0_TMR0					EQU		TMR0
s0_T1TMRL				EQU		TMR1L
s0_T1TMRH				EQU		TMR1H
s0_T1CON				EQU		T1CON
s0_T1GCON				EQU		T1GCON
s0_T2TMR				EQU		TMR2
s0_T2PR					EQU		PR2
s0_T2CON				EQU		T2CON
s0_CPSCON0				EQU		CPSCON0
s0_CPSCON1				EQU		CPSCON1

; general purpose RAM (Bank 0) allocation
bank0_ram				udata	0x20

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

; general purpose RAM (Bank 1) allocation
bank1_ram				udata	0xA0
d1_lut_address_com		res		2

bss_start:
; Switch
d1_sw_transient			res		2		; transient state of SW
s1_sw_state				res		1		; current state {0, 0, RC1, RC0, RA3, RA2, RA1, RA0}
s1_sw_press				res		1
s1_sw_release			res		1
s1_sw_released			res		1		; changed state of SW
s1_sw_toggle			res		1

t1_elapsed_time			res		3		; ticks	[s], to generate 4Hz

s1_multiplier_bits		res		1
s1_samples				res		1

s1_notification_change	res		1		; flags for notification

; battery voltage
s1_low_batt_flag		res		1
t1_adres_FVR_work		res		3
; adres_FVR = 8192*1024/Vin[mV], 1/Vin[mV] = adres_FVR/8192/1024
d1_adres_FVR_m7p23		res		2		; m7p23	 8388608LSB=1/1mV		[1/V]

; PI controller for motor
t1_target_voltage_15p9	res		3		; 15p9	512LSB=1mV	[V]
t1_target_ratio_6p18	res		3		; 6p18	[V/V]
t1_startup_ratio_6p18	res		3		; 6p18	[V/V]

t1_proportional_6p18	res		3		; 6p18	[V/V]
t1_integral_6p18		res		3		; 6p18	[V/V]

s1_sampling_phase		res		1
t1_current_cemf_6p18	res		3		; 6p18	[V/V]
t1_previous_cemf_6p18	res		3		; 6p18	[V/V]

t1_sampling_period		res		3		; ticks	[s]
q1_motor_constant_work	res		4		; ticks*ADRES_LSB	[s*V]

d1_tracking_period_0p16	res		2

; open loop controller for motor
s1_breaking_in_time		res		1

s1_sampled_waves		res		1
s1_sampling_flag		res		1

d1_cemf_mean_14p2		res		2		; 4LSB=1mV [V]
d1_cemf_range_14p2		res		2		; 4LSB=1mV [V]
d1_cemf_min_14p2		res		2		; 4LSB=1mV [V]
d1_cemf_work_14p2		res		2		; 4LSB=1mV [V]

; constant current controller for LED
d1_led_current			res		2

bss_end:

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
s3_SPBRGL				EQU		SPBRGL
s3_SPBRGH				EQU		SPBRGH
s3_RCSTA				EQU		RCSTA
s3_TXSTA				EQU		TXSTA
s3_BAUDCON				EQU		BAUDCON

;-----Bank4------------------
s4_WPUA					EQU		WPUA
s4_WPUC					EQU		WPUC
s4_SSP1BUF				EQU		SSP1BUF
s4_SSP1ADD				EQU		SSP1ADD
s4_SSP1MSK				EQU		SSP1MSK
s4_SSP1STAT				EQU		SSP1STAT
s4_SSP1CON1				EQU		SSP1CON1
s4_SSP1CON2				EQU		SSP1CON2
s4_SSP1CON3				EQU		SSP1CON3

;-----Bank5------------------
s5_CCPR1L				EQU		CCPR1L
s5_CCPR1H				EQU		CCPR1H
s5_CCP1CON				EQU		CCP1CON
s5_PWM1CON				EQU		PWM1CON
s5_CCP1AS				EQU		CCP1AS
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
s8_T4TMR				EQU		TMR4
s8_T4PR					EQU		PR4
s8_T4CON				EQU		T4CON
s8_T6TMR				EQU		TMR6
s8_T6PR					EQU		PR6
s8_T6CON				EQU		T6CON

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Reset vector																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
reset_vector			code	0x000
						movlb	1
						; speed-up HFINTOSC for backup.
						movlw	OSCCON_backup
						movwf	s1_OSCCON
						goto	common_entry_point

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Interrupt Routine																		;
;		*		*		*		*		*		*		*		*		*		*		*		;

; interrupt handler is consist of pulse generator and voltage sampler.
;		the pulse generator is driven by T2 cooperatively with CCP1.
;		the voltage sampler is driven by T1 cooperatively with CCP4.
interrupt_vector		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						TEST_LED_ON	TEST_CPU_USAGE
						movlb	0
						btfsc	s0_PIR2,OSFIF
						bra		__intr_osc_fail
						btfsc	s0_PIR3,CCP4IF
						bra		__intr_motor
						reset

idle_loop:				movlp	HIGH __idle_loop
						movlw	LOW  __idle_loop
__idle_loop:			; wait for interrupt
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						nop
						movwf	PCL		; the lowest power branch instruction
						nop

interrupt_handler		code	0x020
__intr_osc_fail:		; clear fail-safe clock operation flag
						bcf		s0_PIR2,OSFIF

						call 	f5_pwm_pause

						movlb	1
						; switch to backup osc.
						movlw	OSCCON_backup
						movwf	s1_OSCCON
						TEST_LED_OFF	TEST_CPU_USAGE
						retfie

__intr_motor:
						bcf		s0_PIR3,CCP4IF

						; prepare FVR, and ADC to acquire VDD voltage
						call	f1_prepare_adc_and_fvr
						bsf		s1_ADCON0,GO			; start 1st A/D conversion

						; read keys and remove chattering
						call	f1_read_keys			; 35
						clrwdt
						call	f1_update_adc_result	; wait for 1st ADC result
						bsf		s1_ADCON0,GO			; start 2nd A/D conversion

						call	f1_get_notification		; 64
						xorwf	notification_flag,W
						movwf	s1_notification_change
						xorwf	notification_flag,F
						call	f1_update_adc_result	; wait for 2nd ADC result
						bsf		s1_ADCON0,GO			; start 3rd A/D conversion

						; maintain 4Hz flag
						call	f1_maintain_4hz			; 21
						call	f1_update_adc_result	; wait for 3rd ADC result
						bsf		s1_ADCON0,GO			; start 4th A/D conversion

						; set default period into sampling_interval
						call	f6_load_default_period	; 16
						call	f1_update_adc_result	; wait for 4th ADC result

						; turn-OFF ADC
						call	f1_finish_adc_and_fvr
						TEST_LED_ON	TEST_DUTY_UPDATE

						btfsc	s1_notification_change,BIT_OPEN_LOOP_CTRL
						call	f1_init_open_loop_ctrl

						btfsc	s1_notification_change,BIT_CLOSED_LOOP_CTRL
						call	f1_init_closed_loop_ctrl

						btfsc	notification_flag,BIT_OUT_OF_CTRL
						call	f5_pwm_pause

						btfsc	notification_flag,BIT_OPEN_LOOP_CTRL
						call	f5_open_loop_control

						btfsc	notification_flag,BIT_CLOSED_LOOP_CTRL
						call	f5_closed_loop_control

						TEST_LED_OFF TEST_DUTY_UPDATE

						; Advance lut index during last A/D conversion
						btfsc	notification_flag,BIT_ADVANCE_LUT_INDEX
						call	f1_advance_lut_index
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						call	f1_advance_lut_index

						btfsc	notification_flag,BIT_CLOSED_LOOP_CTRL
						call	f1_closed_loop_integration

						movlb	1
						; battery voltage check
						btfss	notification_flag,BIT_POLLING
						bra		__skip_LED_control

						movlw	LOW (8192*1024/LOW_BATT_ERROR)
						btfss	s1_low_batt_flag,0
						movlw	LOW (8192*1024/LOW_BATT_WARNING)
						subwf	d1_adres_FVR_m7p23+0,W
						movlw	HIGH (8192*1024/LOW_BATT_ERROR)
						btfss	s1_low_batt_flag,0
						movlw	HIGH (8192*1024/LOW_BATT_WARNING)
						subwfb	d1_adres_FVR_m7p23+1,W
						rlf		s1_low_batt_flag,F

__skip_LED_control:
						btfsc	s1_low_batt_flag,0
						bra		__led_turn_off

						btfsc	s1_OSCSTAT,HFIOFR
						bra		__led_turn_off			; HFINTOSC is working
						btfsc	s1_OSCSTAT,MFIOFR
						bra		__led_turn_off			; MFINTOSC is working
						btfss	s1_OSCSTAT,LFIOFR
						bra		__led_turn_off			; LFINTOSC is not working

__led_turn_on:
		if TEST_TARGET == TEST_NONE
						; prepare for LED brightness control, set MUX to LED
						movlw	ADCON0_MUX_LED
						movwf	s1_ADCON0
						movlw	ADCON1_VDD_RATIOMETRIC
						movwf	s1_ADCON1

						movlb	2
						movlw	SRCON0_RELEASE_LED
						movwf	s2_SRCON0		; LED is controlled by PWM

						movlb	6
						movf	s6_CCPR3L,W
						btfsc	STATUS,Z
						incf	s6_CCPR3L,F		; reduce LED ON duty to generate pulse edge

						movlb	3
						bcf		s3_ANSELA,ANSELA_LED_ANODE

						movlb	7
						bsf		s7_IOCAP,IOCA_LED_ANODE	; interrupt on change (positive edge)
						bcf		s7_IOCAF,IOCA_LED_ANODE
						btfss	s7_IOCAF,IOCA_LED_ANODE
						bra		$-1

						; prepare for LED brightness control, set trigger source
						movlb	2
						bsf		s2_SRCON0,SRLEN		; turn on LED, start setup time.

						movlw	3
						decfsz	WREG,F
						bra		$-1

						movlb	1
						bsf		s1_ADCON0,GO	; setup time 4.25us, hold time 0.75us

						movlb	2
						movlw	SRCON0_RELEASE_LED
						movwf	s2_SRCON0		; turn off LED, end hold time.

						movlb	3
						bsf		s3_ANSELA,ANSELA_LED_ANODE

						movlb	1
; estimate LED current and control constant brightness
LED_WIDTH_CORRECTION	equ		45
LED_PERIOD_CORRECTION	equ		15
; Duty ratio = CCPR3/PR6
;		PR6	  = (1023 - LED_PERIOD_CORRECTION) - adres_LED
;		CCPR3 = (adres_FVR - LED_WIDTH_CORRECTION)/16
						movlw	LOW (1023 - LED_PERIOD_CORRECTION)
						movwf	d1_led_current+0
						movlw	HIGH (1023 - LED_PERIOD_CORRECTION)
						movwf	d1_led_current+1

						movlw	LOW (2*LED_WIDTH_CORRECTION*64/16)
						subwf	d1_adres_FVR_m7p23+0,W
						movwf	t1_adres_FVR_work+0
						movlw	HIGH (2*LED_WIDTH_CORRECTION*64/16)
						subwfb	d1_adres_FVR_m7p23+1,W
						movwf	t1_adres_FVR_work+1
						clrf	t1_adres_FVR_work+2

						; wait for result
						btfsc	s1_ADCON0,GO
						bra		$-1

						; read LED current
						movf	s1_ADRESL,W
						subwf	d1_led_current+0,F
						movf	s1_ADRESH,W
						subwfb	d1_led_current+1,F

						; Turn-OFF ADC
						clrf	s1_ADCON0

						;                                                                adres_FVR_work
						;  0, 0, 0, 0, 0, 0, 0, 0     0, 0,d9,d8,d7,d6,d5,d4    d3,d2,d1,d0, 0, 0, 0, 0
						;                                        led_current
						;  0, 0, 0, 0, 0, 0,d9,d8    d7,d6,d5,d4,d3,d2,d1,d0
						;                            pulse_width_H              pulse_width_L
						;                             0, 0, 0, 0, 0, 0, 0, 0     0, 1, 0, 0, 0, 0, 0, 0
						;                                                           ^ sentinel
						clrf	pulse_width_H
						clrf	pulse_width_L
						bsf		pulse_width_L,6
						bra		__led_div_begin

__led_div_pos:			movf	d1_led_current+0,W
						subwf	t1_adres_FVR_work+1,F
						movf	d1_led_current+1,W
						subwfb	t1_adres_FVR_work+2,F
__led_div_common:		btfsc	STATUS,C				; output is positive logic
						bsf		pulse_width_L,5
						lslf	pulse_width_L,F
						rlf		pulse_width_H,F
						btfsc	STATUS,C
						bra		__led_div_end
__led_div_begin:		lslf	t1_adres_FVR_work+0,F
						rlf		t1_adres_FVR_work+1,F
						rlf		t1_adres_FVR_work+2,F
						btfsc	pulse_width_L,6			; output is positive logic
						bra		__led_div_pos
__led_div_neg:			movf	d1_led_current+0,W
						addwf	t1_adres_FVR_work+1,F
						movf	d1_led_current+1,W
						addwfc	t1_adres_FVR_work+2,F
						bra		__led_div_common

__led_div_end:			; update duty register
						movlb	6
						lsrf	pulse_width_L,W
						lsrf	WREG,W
						iorlw	CCP3CON_init	; PxM1,PxM0,d1,d0,CCPxM3,CCPxM2,CCPxM1,CCPxM0
						movwf	s6_CCP3CON
						movf	pulse_width_H,W
						movwf	s6_CCPR3L
		endif
						bra		__led_ctrl_end

__led_turn_off:			; halt LED brightness controll
		if TEST_TARGET == TEST_NONE
						movlb	2
						movlw	SRCON0_FORCE_LED_OFF
						movwf	s2_SRCON0		; turn off LED
		endif
						; fall through

__led_ctrl_end:			TEST_LED_OFF	TEST_CPU_USAGE
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Subroutine for Interrupt Routine														;
;		*		*		*		*		*		*		*		*		*		*		*		;
f1_prepare_adc_and_fvr:
						movlb	2
						; power-up fixed voltage regulator for ADC
						movlw	FVRCON_init
						movwf	s2_FVRCON

						movlb	1
						movlw	ADCON1_VDD_RATIOMETRIC
						movwf	s1_ADCON1

						; configure ADC for input voltage acquisition
						movlw	ADCON0_MUX_FVR
						movwf	s1_ADCON0

						clrf	d1_adres_FVR_m7p23+0
						clrf	d1_adres_FVR_m7p23+1

						movlw	FVR_SETTLING_TIME*(FCY/1000000)/3
						decfsz	WREG,W
						bra		$-1
						return

f1_update_adc_result:
						movlb	1
						; wait for completion of A/D convertion
						btfsc	s1_ADCON0,GO
						bra		$-1

						movf	s1_ADRESL,W
						addwf	d1_adres_FVR_m7p23+0,F
						movf	s1_ADRESH,W
						addwfc	d1_adres_FVR_m7p23+1,F
						return

f1_finish_adc_and_fvr:
						movlb	2
						; turn-OFF FVR
						clrf	s2_FVRCON

						movlb	1
						; turn-OFF ADC
						clrf	s1_ADCON0
						return

f6_load_default_period:	movlb	6
						clrf	s6_CCP4CON
						movlw	LOW  (OPEN_LOOP_PERIOD - 1)
						movwf	s6_CCPR4L
						movlw	HIGH (OPEN_LOOP_PERIOD - 1)
						movwf	s6_CCPR4H
						movlw	CCP4CON_init
						movwf	s6_CCP4CON					;  8

						movlw	LOW  (OPEN_LOOP_PERIOD)
						movwf	cemf_sampling_period_L
						movlw	HIGH (OPEN_LOOP_PERIOD)
						movwf	cemf_sampling_period_H
						return								;  6	14

; PWM module is temporary shutdown
;	 input: none
;	output: duty ratio register in PWM module with duty=0%
f5_pwm_pause:			movlb	2
						; disconnect power from motor
						bsf		s2_CM1CON0,C1POL

						movlb	0
						; stop PWM timer
						bcf		s0_T2CON,TMR2ON
						movf	s0_T2PR,W
						movwf	s0_T2TMR

						movlb	2
						; resume half-bridge operation at next cycle
						bcf		s2_CM1CON0,C1POL	; re-connect motor

						movlb	5
						; set duty ratio to zero
						clrf	s5_CCPR1L
						clrf	s5_CCPR2L
						movlw	CCP1CON_init	; PxM1,PxM0,d1,d0,CCPxM3,CCPxM2,CCPxM1,CCPxM0
						movwf	s5_CCP1CON
						movwf	s5_CCP2CON
						return

; closed_loop_control updates pulse_width to drive motor with constant speed
f1_init_closed_loop_ctrl:
						movlb	1
						clrf	t1_current_cemf_6p18+0
						clrf	t1_current_cemf_6p18+1
						clrf	t1_current_cemf_6p18+2
						; fall through
f1_clear_integral:		movlb	1
						clrf	t1_integral_6p18+0
						clrf	t1_integral_6p18+1
						clrf	t1_integral_6p18+2
						return

f5_closed_loop_control:
						movlb	2
						; power-up Comparator 2 for CEMF acquisition
						movlw	CM2CON0_init
						movwf	s2_CM2CON0
						movlw	CM2CON1_init
						movwf	s2_CM2CON1

						; load lut entry into sampling_interval
						call	f6_load_lut_entry

						movlb	5
						; save duty
						movf	s5_CCPR1L,W
						movwf	pulse_width_L
						movf	s5_CCP1CON,W
						movwf	pulse_width_H

						movlb	0
						bcf		s0_T2CON,TMR2ON

						movlb	5
						; set duty to 0%
						clrf	s5_CCPR1L
						clrf	s5_CCPR2L
						movlw	CCP1CON_init	; PxM1,PxM0,d1,d0,CCPxM3,CCPxM2,CCPxM1,CCPxM0
						movwf	s5_CCP1CON
						movwf	s5_CCP2CON

						movlb	0
						bsf		s0_T2CON,TMR2ON
						movf	s0_T2PR,W
						movwf	s0_T2TMR					; force update duty register

						nop

						; stop PWM timer
						bcf		s0_T2CON,TMR2ON
						movwf	s0_T2TMR

						movlb	5
						; restore duty
						movf	pulse_width_L,W
						movwf	s5_CCPR1L
						movwf	s5_CCPR2L
						movf	pulse_width_H,W
						movwf	s5_CCP1CON
						movwf	s5_CCP2CON

						movlb	2
						; disconnect power from motor
						bsf		s2_CM1CON0,C1POL

						movlb	4
						; turn on bias resister
						bsf		s4_WPUC,WPUC_CEMF_SENSE

						movlb	2
						; wait for end of slow decay discharge
						btfss	s2_CM2CON0,C2OUT
						bra		$-1
						btfss	s2_CM2CON0,C2OUT
						bra		$-1
						btfss	s2_CM2CON0,C2OUT
						bra		$-1

						movlb	4
						; turn off bias resister
						bcf		s4_WPUC,WPUC_CEMF_SENSE

						movlb	2
						; shutdown Comparator 2
						clrf	s2_CM2CON0

						; resume half-bridge operation at next cycle
						bcf		s2_CM1CON0,C1POL	; re-connect motor

						; power-up fixed voltage regulator for ADC
						movlw	FVRCON_init
						movwf	s2_FVRCON

						movlb	8
						; enter settling time of CEMF
						movlw	T4PR_init
						movwf	s8_T4PR
						movlw	TMR4_init
						movwf	s8_T4TMR
						movlw	T4CON_init
						movwf	s8_T4CON

						movlb	0
						bcf		s0_PIR3,TMR4IF

						movlb	1
						; configure ADC for CEMF acquisition
						movlw	ADCON0_MUX_MOTOR
						movwf	s1_ADCON0
						movlw	ADCON1_FVR_ABSOLUTE
						movwf	s1_ADCON1

						; calculate target_voltage from sampling_interval
						call	f1_calc_target_voltage

						; calculate target ratio from target voltage
						call	f1_calc_target_ratio

						TEST_LED_OFF	TEST_DUTY_UPDATE

						; proportional_voltage = P_GAIN*target voltage - P_GAIN*ADRES
						movf	t1_target_ratio_6p18+0,W
						movwf	t1_proportional_6p18+0
						movf	t1_target_ratio_6p18+1,W
						movwf	t1_proportional_6p18+1
						movf	t1_target_ratio_6p18+2,W
						movwf	t1_proportional_6p18+2

	if (CALC_SAMPLES_1X != 0) || (CALC_SAMPLES_2X != 0)
		if (CALC_SAMPLES_1X == 0)
			if (CALC_SAMPLES_2X == 1)	; averaging
						clrf	s1_sampling_phase
			endif
			if (CALC_SAMPLES_2X == 2)	; phase control
						comf	s1_sampling_phase,F
			endif
		endif
		if (CALC_SAMPLES_1X == 1)		; averaging
			if (CALC_SAMPLES_2X == 2)	; phase control
						comf	s1_sampling_phase,F
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
			endif
						clrf	s1_sampling_phase
		endif
		if (CALC_SAMPLES_1X == 2)		; phase control
						comf	s1_sampling_phase,F
			if (CALC_SAMPLES_2X == 1)	; averaging
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						clrf	s1_sampling_phase
			endif
		endif
						clrf	t1_previous_cemf_6p18+0
						movf	t1_current_cemf_6p18+1,W
						movwf	t1_previous_cemf_6p18+1
						movf	t1_current_cemf_6p18+2,W
						movwf	t1_previous_cemf_6p18+2
	endif
						clrf	t1_current_cemf_6p18+0
						clrf	t1_current_cemf_6p18+1
						clrf	t1_current_cemf_6p18+2

						clrf	t1_adres_FVR_work+1
						clrf	t1_adres_FVR_work+2

						movf	t1_integral_6p18+1,W
						movwf	pulse_width_L
						movf	t1_integral_6p18+2,W
						movwf	pulse_width_H

						movlb	0
						btfss	s0_PIR3,TMR4IF
						bra		$-1

						movlb	8
						; turn-OFF Timer 4
						clrf	s8_T4CON

						movlb	1
						movlw	(P_GAIN*PWM_PERIOD+128)/256-1
						movwf	s1_samples
__cemf_sampling_lp:
						bsf		s1_ADCON0,GO

						movf	t1_target_ratio_6p18+0,W
						addwf	t1_proportional_6p18+0,F
						movf	t1_target_ratio_6p18+1,W
						addwfc	t1_proportional_6p18+1,F
						movf	t1_target_ratio_6p18+2,W
						addwfc	t1_proportional_6p18+2,F

						TEST_LED_ON	TEST_DUTY_UPDATE

						; wait for of A/D convertion completion
						btfsc	s1_ADCON0,GO
						bra		$-1

						movf	s1_ADRESL,W
						addwf	t1_adres_FVR_work+1,F
						movf	s1_ADRESH,W
						addwfc	t1_adres_FVR_work+2,F
						TEST_LED_OFF	TEST_DUTY_UPDATE

						decfsz	s1_samples,F
						bra		__cemf_sampling_lp

						; last conversion start
						bsf		s1_ADCON0,GO
						nop

						movlb	0
						; restart PWM generator after ADC sampling end
						bsf		s0_T2CON,TMR2ON
						TEST_LED_ON	TEST_DUTY_UPDATE

						movlb	1
						; wait for last A/D convertion completion
						btfsc	s1_ADCON0,GO
						bra		$-1							; 45

						movf	s1_ADRESL,W
						addwf	t1_adres_FVR_work+1,F
						movf	s1_ADRESH,W
						addwfc	t1_adres_FVR_work+2,F		; 4

						; turn-OFF ADC
						clrf	s1_ADCON0

						movlb	2
						; shutdown FVR
						clrf	s2_FVRCON

						movlb	1
						; current_cemf(16bit) = control_voltage(mid 16bit) * FVR(12bit)
						;       10p22 >> 12               17m1            m7p23
BIT_POS					set		1
	while BIT_POS < 12
						movf	t1_adres_FVR_work+1,W
						btfsc	d1_adres_FVR_m7p23+(BIT_POS/8),(BIT_POS%8)
						addwf	t1_current_cemf_6p18+1,F
						movf	t1_adres_FVR_work+2,W
						btfsc	d1_adres_FVR_m7p23+(BIT_POS/8),(BIT_POS%8)
						addwfc	t1_current_cemf_6p18+2,F
						lsrf	t1_current_cemf_6p18+2,F
						rrf		t1_current_cemf_6p18+1,F
BIT_POS					set		BIT_POS + 1
	endw													; 92 = 4+8*11

						; proportional = target - current_cemf
						movf	t1_current_cemf_6p18+1,W
						subwf	t1_proportional_6p18+1,F
						movf	t1_current_cemf_6p18+2,W
						subwfb	t1_proportional_6p18+2,F	; 4

	if (CALC_SAMPLES_1X != 0) || (CALC_SAMPLES_2X != 0)
		if (CALC_SAMPLES_1X == 0)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		__pi_calc_pi
		endif
		if (CALC_SAMPLES_2X == 0)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		__pi_calc_pi
		endif
						; differencial = (previous - current)/2
						movf	t1_current_cemf_6p18+1,W
						subwf	t1_previous_cemf_6p18+1,F
						movf	t1_current_cemf_6p18+2,W
						subwfb	t1_previous_cemf_6p18+2,F

						asrf	t1_previous_cemf_6p18+2,F
						rrf		t1_previous_cemf_6p18+1,F

						; proportional = (target - current) - (previous - current)/2 = target -   current/2 - previous/2
						; proportional = (target - current) + (previous - current)/2 = target - 3*current/2 + previous/2
						asrf	s1_sampling_phase,W			; 0x00 or 0xFF
						xorwf	t1_previous_cemf_6p18+1,F
						xorwf	t1_previous_cemf_6p18+2,F

						movf	t1_previous_cemf_6p18+1,W
						addwfc	t1_proportional_6p18+1,F
						movf	t1_previous_cemf_6p18+2,W
						addwfc	t1_proportional_6p18+2,F	; 15
						; integral += target -   cemf(1)/2 - cemf(0)/2
						; integral += target - 3*cemf(2)/2 + cemf(1)/2
						; integral += target -   cemf(3)/2 - cemf(2)/2
						; integral += target - 3*cemf(4)/2 + cemf(3)/2
__pi_calc_pi:
	endif
						; control = integral + proportional
						movf	t1_proportional_6p18+1,W
						addwf	pulse_width_L,F
						movf	t1_proportional_6p18+2,W
						addwfc	pulse_width_H,F				; 4

						btfsc	pulse_width_H,7
						goto	f5_pwm_pause				; 2	call and return

						; limit positive duty
						movf	t1_startup_ratio_6p18+1,W
						subwf	pulse_width_L,W
						movf	t1_startup_ratio_6p18+2,W
						subwfb	pulse_width_H,W

						movf	t1_startup_ratio_6p18+1,W
						btfsc	STATUS,C
						movwf	pulse_width_L
						movf	t1_startup_ratio_6p18+2,W
						btfsc	STATUS,C
						movwf	pulse_width_H				; 10	176

f5_pwm_by_duty_ratio:	movlb	5
						; saturate calculation
						comf	pulse_width_H,W				; expect B'111111xx' 
						addlw	0x04
						subwfb	WREG,W						; W = -1 +C
						iorwf	pulse_width_L,F
						iorwf	pulse_width_H,F				; +6	182

						; change from 6p10 to 0p16
						; pulse_width_H         pulse_width_L
						; 0,0,0,0,0,0,d9,d8     d7,d6,d5,d4,d3,d2,d1,d0
						swapf	pulse_width_L,W
						andlw	0x30			;		0,0,d1,d0,0,0,0,0
						iorlw	CCP1CON_init	; PxM1,PxM0,d1,d0,CCPxM3,CCPxM2,CCPxM1,CCPxM0
						movwf	s5_CCP1CON
						movwf	s5_CCP2CON
						lsrf	pulse_width_H,F
						rrf		pulse_width_L,F
						lsrf	pulse_width_H,F
						rrf		pulse_width_L,W
						movwf	s5_CCPR1L 
						movwf	s5_CCPR2L
						return								; +13	195

						; integration
f1_closed_loop_integration:
						movlb	1
	if (MULT_ITIME_1X_24p8 == 128)
I_GAIN_fxp1p8			set		((512<<8)/I_TIME_8p8)&0x1FF
		if (256 <= MULT_ITIME_2X_24p8)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		__integral_1x
						asrf	t1_proportional_6p18+2,F
						rrf		t1_proportional_6p18+1,F
						rrf		t1_proportional_6p18+0,F
			if (512 <= MULT_ITIME_2X_24p8)
						asrf	t1_proportional_6p18+2,F
						rrf		t1_proportional_6p18+1,F
						rrf		t1_proportional_6p18+0,F
			endif
__integral_1x:
		endif
	endif
	if (MULT_ITIME_1X_24p8 == 256)
		if (MULT_ITIME_2X_24p8 == 128)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		$+4
						lslf	t1_proportional_6p18+0,F
						rlf		t1_proportional_6p18+1,F
						rlf		t1_proportional_6p18+2,F
		endif
I_GAIN_fxp1p8			set		((256<<8)/I_TIME_8p8)&0x1FF
		if (MULT_ITIME_2X_24p8 == 512)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		$+4
						asrf	t1_proportional_6p18+2,F
						rrf		t1_proportional_6p18+1,F
						rrf		t1_proportional_6p18+0,F
		endif
	endif
	if (MULT_ITIME_1X_24p8 == 512)
		if (MULT_ITIME_2X_24p8 <= 256)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		__integral_1x
						lslf	t1_proportional_6p18+0,F
						rlf		t1_proportional_6p18+1,F
						rlf		t1_proportional_6p18+2,F
			if (MULT_ITIME_2X_24p8 <= 128)
						lslf	t1_proportional_6p18+0,F
						rlf		t1_proportional_6p18+1,F
						rlf		t1_proportional_6p18+2,F
			endif
__integral_1x:
		endif
I_GAIN_fxp1p8			set		((128<<8)/I_TIME_8p8)&0x1FF
	endif
	while I_GAIN_fxp1p8
		if (I_GAIN_fxp1p8 & 0x100)
						movf	t1_proportional_6p18+0,W
						addwf	t1_integral_6p18+0,F
						movf	t1_proportional_6p18+1,W
						addwfc	t1_integral_6p18+1,F
						movf	t1_proportional_6p18+2,W
						addwfc	t1_integral_6p18+2,F
		endif
I_GAIN_fxp1p8			set		(I_GAIN_fxp1p8<<1)&0x1FF
		if (I_GAIN_fxp1p8 != 0)
						asrf	t1_proportional_6p18+2,F
						rrf		t1_proportional_6p18+1,F
						rrf		t1_proportional_6p18+0,F
		endif
	endw

						; if integral voltage is negative, set to 0
						btfsc	t1_integral_6p18+2,7
						goto	f1_clear_integral		; call and return

						; limit positive duty
						movf	t1_startup_ratio_6p18+1,W
						subwf	t1_integral_6p18+1,W
						movf	t1_startup_ratio_6p18+2,W
						subwfb	t1_integral_6p18+2,W
						btfss	STATUS,C
						return
						clrf	t1_integral_6p18+0
						movf	t1_startup_ratio_6p18+1,W
						movwf	t1_integral_6p18+1
						movf	t1_startup_ratio_6p18+2,W
						movwf	t1_integral_6p18+2
						return

; open_loop_control updates pulse_width to drive motor with constant voltage
f1_init_open_loop_ctrl: movlb	2
						; connect
						bcf		s2_CM1CON0,C1POL

						movlb	0
						; restart PWM timer
						btfss	s0_T2CON,TMR2ON
						bsf		s0_T2CON,TMR2ON

						movlb	1
						movlw	BREAKING_IN_DURATION*POLLING_FREQUENCY-1
						movwf	s1_breaking_in_time
						movlw	OPEN_LOOP_FREQ
						movwf	s1_sampled_waves
						clrf	t1_integral_6p18+0
						clrf	t1_integral_6p18+1
						return

f5_open_loop_control:	movlb	1
						; slewing
OPEN_LOOP_VOLTAGE_13p3	equ		PWM_PERIOD*OPEN_LOOP_VOLTAGE_mV/32/OPEN_LOOP_FREQ
						; open_loop_voltage *= pwm_period
						;         16p8 >> 5           0p8
						movf	s1_sampled_waves,F
						bz		$+6
						decf	s1_sampled_waves,F
						movlw	LOW  OPEN_LOOP_VOLTAGE_13p3
						addwf	t1_integral_6p18+0,F
						movlw	HIGH OPEN_LOOP_VOLTAGE_13p3
						addwfc	t1_integral_6p18+1,F

						; startup_ratio = open_loop_voltage(16bit) * FVR(13bit)
						;    6p26 >> 16                13p3        m7p23
						movf	d1_adres_FVR_m7p23+0,W
						movwf	t1_adres_FVR_work+0
						movf	d1_adres_FVR_m7p23+1,W
						movwf	t1_adres_FVR_work+1

						clrf	pulse_width_L
						clrf	pulse_width_H

						movlw	0x10
						movwf	s1_multiplier_bits
__calc_open_loop_ratio:
						lsrf	t1_adres_FVR_work+1,F
						rrf		t1_adres_FVR_work+0,F

						btfss	STATUS,C
						bra		$+5
						movf	t1_integral_6p18+0,W
						addwf	pulse_width_L,F
						movf	t1_integral_6p18+1,W
						addwfc	pulse_width_H,F

						lsrf	pulse_width_H,F
						rrf		pulse_width_L,F

						decfsz	s1_multiplier_bits,F
						bra		__calc_open_loop_ratio

						movlb	0
						movlw	(256-T2PR_init+30)
						addwf	s0_T2TMR,W
						btfsc	STATUS,C
						bra		$-3

						movlb	1
						btfsc	notification_flag,BIT_POLLING
						decfsz	s1_breaking_in_time,F
						goto	f5_pwm_by_duty_ratio	; break-in, call and return
						; fall through

__calib_begin:
						movlb	5
						; set duty to 0%
						clrf	s5_CCPR1L
						clrf	s5_CCPR2L
						movlw	CCP1CON_init	; PxM1,PxM0,d1,d0,CCPxM3,CCPxM2,CCPxM1,CCPxM0
						movwf	s5_CCP1CON
						movwf	s5_CCP2CON

						movlb	0
						movlw	T2PR_init
						movwf	s0_T2TMR					; force update duty register

CALIB_ADC_INTERVAL		equ		18
END_OF_CALIB_VOLTAGE	equ		256
CALIB_WAVES				equ		24
CALIB_COEFFICIENT_8p8	equ		256*FCY_SCALED/1000000*CENTER_WAVE_HEIGHT/CALIB_WAVES*CALIB_ADC_INTERVAL/MEAN_WAVE_HEIGHT
						; since measured value is an arithmetic average, parameter is corrected from it to center.

						; set Timer to discharge time
						clrf	s0_T1CON
						movlw	HIGH (0x10000-(500-CALIB_ADC_INTERVAL)*(FCY_SCALED/1000000))
						movwf	s0_T1TMRH
						movlw	LOW  (0x10000-(500-CALIB_ADC_INTERVAL)*(FCY_SCALED/1000000))
						movwf	s0_T1TMRL
						movlw	T1CON_init
						movwf	s0_T1CON			; start 500us later

						movlb	6
						; set CCP to A/D conversion interval
						clrf	s6_CCP4CON
						movlw	LOW	 (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s6_CCPR4L
						movlw	HIGH (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s6_CCPR4H
						movlw	CCP4CON_init
						movwf	s6_CCP4CON

						movlb	2
						; halt half-bridge operation
						bsf		s2_CM1CON0,C1POL

						; halt LED brightness controll
	if TEST_TARGET == TEST_NONE
						movlw	SRCON0_FORCE_LED_OFF
						movwf	s2_SRCON0		; turn off LED
	endif
						; power-up fixed voltage regulator for ADC
						movlw	FVRCON_init
						movwf	s2_FVRCON

						movlb	1
						; change WDT period
						movlw	WDTCON_calibration
						movwf	s1_WDTCON

						; set ADC to basic mode using CCP trigger
						movlw	ADCON0_MUX_MOTOR
						movwf	s1_ADCON0
						movlw	ADCON1_FVR_ABSOLUTE
						movwf	s1_ADCON1

						; clear data memory used to hold c-emf values
						clrf	FSR0L
						movlw	0x21
						movwf	FSR0H
						clrf	INDF0
						incfsz	FSR0L,F			; with wrap-round
						bra		$-2

						; initialize variables
						clrf	d1_cemf_mean_14p2+0
						movlw	HIGH (END_OF_CALIB_VOLTAGE << 2)
						movwf	d1_cemf_mean_14p2+1

						clrf	d1_cemf_min_14p2+0
						clrf	d1_cemf_min_14p2+1

						clrf	d1_cemf_range_14p2+0
						clrf	d1_cemf_range_14p2+1

						clrf	s1_sampled_waves
						clrf	s1_sampling_flag

						clrf	q1_motor_constant_work+0
						clrf	q1_motor_constant_work+1
						clrf	q1_motor_constant_work+2
						clrf	q1_motor_constant_work+3

						bra		__calib_lp

__calib_lp_decreasing:	; decreasing in CEMF voltage	; cemf_mean < cemf_min
						TEST_LED_ON TEST_CALIBRATION
						bsf		s1_sampling_flag,1		; set bit1 as minimum update

						clrwdt
						movf	q1_motor_constant_work+0,W
						movwi	0[FSR0]
						movf	q1_motor_constant_work+1,W
						movwi	1[FSR0]
						movf	q1_motor_constant_work+2,W
						movwi	2[FSR0]
						movf	q1_motor_constant_work+3,W
						movwi	3[FSR0]						; 10/	44/

__calib_lp_update:		movf	d1_cemf_work_14p2+0,W
						addwf	d1_cemf_min_14p2+0,F
						movf	d1_cemf_work_14p2+1,W
						addwfc	d1_cemf_min_14p2+1,F	; 4		48/52

__calib_lp:				; wait for start of A/D convertion
						btfss	s1_ADCON0,GO
						bra		$-1

						; IIR LPF, time constant is 8 periods
						movf	d1_cemf_range_14p2+0,W
						subwf	d1_cemf_mean_14p2+0,F
						movf	d1_cemf_range_14p2+1,W
						subwfb	d1_cemf_mean_14p2+1,F	; 6		56/60

						; wait for completion of A/D convertion
						btfsc	s1_ADCON0,GO				;		0
						bra		$-1

						movf	s1_ADRESL,W
						addwf	d1_cemf_mean_14p2+0,F
						movf	s1_ADRESH,W
						addwfc	d1_cemf_mean_14p2+1,F

						movf	d1_cemf_mean_14p2+1,W
						addlw	HIGH (0x10000 - (END_OF_CALIB_VOLTAGE << 2))
						btfss	STATUS,C
						bra		__calib_end

						movf	s1_ADRESL,W
						addwf	q1_motor_constant_work+0,F
						movf	s1_ADRESH,W
						addwfc	q1_motor_constant_work+1,F
						clrw
						addwfc	q1_motor_constant_work+2,F
						addwfc	q1_motor_constant_work+3,F	; 17

						; max-min is kept within 1/8 of mean value
						lsrf	d1_cemf_mean_14p2+1,W
						movwf	d1_cemf_range_14p2+1
						rrf		d1_cemf_mean_14p2+0,W
						movwf	d1_cemf_range_14p2+0
						lsrf	d1_cemf_range_14p2+1,F
						rrf		d1_cemf_range_14p2+0,F
						lsrf	d1_cemf_range_14p2+1,F
						rrf		d1_cemf_range_14p2+0,F	;  8

						; test for minimum
						movf	d1_cemf_min_14p2+0,W
						subwf	d1_cemf_mean_14p2+0,W
						movwf	d1_cemf_work_14p2+0
						movf	d1_cemf_min_14p2+1,W
						subwfb	d1_cemf_mean_14p2+1,W
						movwf	d1_cemf_work_14p2+1
						btfss	STATUS,C
						bra		__calib_lp_decreasing			;  9/ 8	34/33

						; test for maximum
						movf	d1_cemf_range_14p2+0,W
						subwf	d1_cemf_work_14p2+0,F
						movf	d1_cemf_range_14p2+1,W
						subwfb	d1_cemf_work_14p2+1,F
						btfss	STATUS,C
						bra		__calib_lp					;  /6	 /39

__calib_lp_increasing:	; increasing in CEMF voltage		; cemf_max < cemf_mean
						TEST_LED_OFF TEST_CALIBRATION
						btfss	s1_sampling_flag,1
						bra		__calib_lp_update
						bcf		s1_sampling_flag,1		; clear bit1 as maximum update

						movlw	0x04
						addwf	FSR0L,F
						btfsc	STATUS,C
						incf	s1_sampled_waves,F
						bra		__calib_lp_update				;  /9	 /48

__calib_end:
						movlb	6
						; Turn-OFF periodic timer
						clrf	s6_CCP4CON
						movlb	0
						clrf	s0_T1CON

						movlb	2
						; shutdown FVR
						clrf	s2_FVRCON

						movlb	1
						; Turn-OFF ADC
						clrf	s1_ADCON0

						movf	s1_sampled_waves,F
						btfsc	STATUS,Z
						call	set_factory_default		; call but never return

						clrwdt

						; Equivalent Time Sampling
						decf	FSR0L,F						; wrap-around
						; load the latest snap shot
						moviw	FSR0--
						movwf	q1_motor_constant_work+3
						moviw	FSR0--
						movwf	q1_motor_constant_work+2
						moviw	FSR0--
						movwf	q1_motor_constant_work+1
						movf	INDF0,W
						movwf	q1_motor_constant_work+0

						movlw	-4*CALIB_WAVES
						addwf	FSR0L,F						; wrap-around
						; get motor_constant samples by subtraction
						moviw	FSR0++
						subwf	q1_motor_constant_work+0,F
						moviw	FSR0++
						subwfb	q1_motor_constant_work+1,F
						moviw	FSR0++
						subwfb	q1_motor_constant_work+2,F
						moviw	FSR0++
						subwfb	q1_motor_constant_work+3,F

						clrf	motor_constant_LL
						clrf	motor_constant_LH
						clrf	motor_constant_HL
						clrf	motor_constant_HH

						movlw	LOW  CALIB_COEFFICIENT_8p8
						movwf	d1_cemf_work_14p2+0
						movlw	HIGH CALIB_COEFFICIENT_8p8
						movwf	d1_cemf_work_14p2+1

						movlw	0x08
						movwf	s1_sampled_waves
__calc_motor_const_lp1:	lsrf	d1_cemf_work_14p2+0,F
						btfss	STATUS,C
						bra		$+9
						movf	q1_motor_constant_work+0,W
						addwf	motor_constant_LL,F
						movf	q1_motor_constant_work+1,W
						addwfc	motor_constant_LH,F
						movf	q1_motor_constant_work+2,W
						addwfc	motor_constant_HL,F
						movf	q1_motor_constant_work+3,W
						addwfc	motor_constant_HH,F

						lsrf	motor_constant_HH,F
						rrf		motor_constant_HL,F
						rrf		motor_constant_LH,F
						rrf		motor_constant_LL,F

						decfsz	s1_sampled_waves,F
						bra		__calc_motor_const_lp1

						movlw	0x08
						movwf	s1_sampled_waves
__calc_motor_const_lp2:	lsrf	d1_cemf_work_14p2+1,F
						btfss	STATUS,C
						bra		$+9
						movf	q1_motor_constant_work+0,W
						addwf	motor_constant_LL,F
						movf	q1_motor_constant_work+1,W
						addwfc	motor_constant_LH,F
						movf	q1_motor_constant_work+2,W
						addwfc	motor_constant_HL,F
						movf	q1_motor_constant_work+3,W
						addwfc	motor_constant_HH,F

						lslf	q1_motor_constant_work+0,F
						rlf		q1_motor_constant_work+1,F
						rlf		q1_motor_constant_work+2,F
						rlf		q1_motor_constant_work+3,F

						decfsz	s1_sampled_waves,F
						bra		__calc_motor_const_lp2

						call	update_parameter	; call but never return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Common entry point (cold boot, wake up from sleep, fall into sleep)						;
;		*		*		*		*		*		*		*		*		*		*		*		;
common_entry_point:
						movlb	4
						; enable pull-up
						movlw	WPUA_init
						movwf	s4_WPUA
						movlw	WPUC_init
						movwf	s4_WPUC

						movlb	1
						movlw	OPTION_REG_init	
						movwf	s1_OPTION_REG

						; analyze reset
						movf	s1_PCON,W
						movwf	notification_flag
						movlw	PCON_init
						movwf	s1_PCON

						TEST_LED_INIT	s1_TRISA,TRISA_LED_ANODE
						TEST_LED_ON	TEST_CPU_USAGE

						; clear data memory
						movlw	bss_start
						movwf	FSR0L
						clrf	FSR0H
						movlw	(bss_end-bss_start)

						clrf	INDF0
						incf	FSR0L,F						; wrap-around
						decfsz	WREG,F
						bra		$-3

						movlb	0
						clrf	s0_T1CON

						movlb	7
						movlw	B'111111'
						movwf	s7_INLVLA		; All input levels are ST buffer
						movwf	s7_INLVLC		; All input levels are ST buffer
						bcf		s7_INLVLA,TRISA_LED_ANODE

						movlb	3
						; enable input buffer on PORTA
						movlw	ANSELA_init
						movwf	s3_ANSELA

						movlb	2
						; set default state when the port is cofigured for generic use.
						movlw	LATA_init
						movwf	s2_LATA
						movlw	LATC_init
						movwf	s2_LATC

						btfss	notification_flag,NOT_POR
						bra		cold_boot					; Power on reset

						movlb	0
						; wait for RA3/MCLR release when debugger is attached
						movf	s0_PORTA,W
						andlw	(1<<BIT_SW_RESET)|(1<<BIT_SW_ORIGIN)|(1<<BIT_SW_LIMIT)
						btfsc	STATUS,Z
						bra		$-3

						; don't wake-up by RA0 or RA1 when debugger is attached.
						andlw	(1<<BIT_SW_ORIGIN)|(1<<BIT_SW_LIMIT)
						movlw	LOW (~((1<<BIT_SW_RESET)|(1<<BIT_SW_ORIGIN)))
						btfsc	STATUS,Z
						movlw	LOW (~(1<<BIT_SW_RESET))

						movlb	3
						movwf	s3_ANSELA
						movlb	4
						comf	WREG,W
						movwf	s4_WPUA
						clrf	s4_WPUC			; All pullups on PORTC are disabled

						call	f1_save_lut_index

						; change WDT period
						movlw	WDTCON_sleep
						movwf	s1_WDTCON

						; speed-down HFINTOSC for sleep
						movlw	OSCCON_sleep
						movwf	s1_OSCCON

__sleep_lp:				movlb	0
						movf	s0_PORTA,W				; negative level

						movlb	3
						btfss	WREG,BIT_SW_RESET		; is Power button pressed ?
						bra		__warm_reset
						btfss	s3_ANSELA,BIT_SW_ORIGIN	; is buffer for origin SW active ?
						btfsc	WREG,BIT_SW_ORIGIN		; is origin SW pressed ?
						bra		__short_sleep

						bsf		s3_ANSELA,BIT_SW_ORIGIN
						movlb	4
						bcf		s4_WPUA,BIT_SW_ORIGIN
						call	f1_set_idx_to_lut_start
						call	f1_save_lut_index

__short_sleep:			TEST_LED_OFF	TEST_CPU_USAGE
						sleep				; method to wake-up is assertion of MCLR, POR or WDT
						TEST_LED_ON	TEST_CPU_USAGE
						bra		__sleep_lp

__warm_reset:			movlb	1
						movlw	PCON_cold_boot	; merging into cold boot sequence 
						movwf	s1_PCON
						reset					; due to unstable condition while sleeping

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Main routine																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
cold_boot:				clrf	notification_flag

						; restore lut_index from DATA EEPROM
						call	f1_restore_lut_index

						movlb	2
						; disconnect power from motor
						movlw	CM1CON0_init
						movwf	s2_CM1CON0

						; SR-latch for LED control
						movlw	SRCON0_FORCE_LED_OFF
						movwf	s2_SRCON0

						movlb	5
						; timer assignment
						movlw	CCPTMRS_init
						movwf	s5_CCPTMRS

						; setup PWM for motor control
						clrf	s5_CCPR1L
						clrf	s5_CCPR1H
						clrf	s5_CCPR2L
						clrf	s5_CCPR2H
						movlw	CCP1CON_init
						movwf	s5_CCP1CON
						movwf	s5_CCP2CON

						; halfbridge
						movlw	PWM1CON_init
						movwf	s5_PWM1CON
						movwf	s5_PWM2CON
						movlw	CCP1AS_init
						movwf	s5_CCP1AS
						movwf	s5_CCP2AS

						movlb	6
						; setup PWM for LED brightness control
						movlw	CCPR3_init
						movwf	s6_CCPR3L
						movwf	s6_CCPR3H
						movlw	CCP3CON_init
						movwf	s6_CCP3CON

						movlb	3
						; enable input buffer
						movlw	ANSELC_init
						movwf	s3_ANSELC

						movlb	0
						; start timer for ADC trigger
						clrf	s0_T1TMRL
						clrf	s0_T1TMRH
						movlw	T1CON_init
						movwf	s0_T1CON

						; start PWM controller for motor
						movlw	T2PR_init
						movwf	s0_T2PR
						movwf	s0_T2TMR
						movlw	T2CON_init
						movwf	s0_T2CON

						movlb	8
						; start PWM controller for LED
						movlw	T6PR_init
						movwf	s8_T6PR
						movwf	s8_T6TMR
						movlw	T6CON_init
						movwf	s8_T6CON

						; setup CCP4 for ADC trigger of motor
						call	f6_load_default_period

						movlb	0
						; initial key state
						swapf	s0_PORTC,W
						comf	WREG,W
						andlw	(1<<BIT_SW_STOP)|(1<<BIT_SW_DOUBLE_SPEED)
						iorlw	(1<<BIT_SW_RESET)

						movlb	1
						movwf	s1_sw_state
						movwf	d1_sw_transient+0
						movwf	d1_sw_transient+1

						; set multiplier for tracking
						movlw	( SIDEREAL_TIME_0p16        & 0xFF)
						btfsc	s1_sw_state,BIT_SW_DOUBLE_SPEED
						movlw	( SOLAR_TIME_0p16           & 0xFF)
						btfsc	s1_sw_state,BIT_SW_STOP
						movlw	( LUNAR_TIME_0p16           & 0xFF)
						movwf	d1_tracking_period_0p16+0

						movlw	((SIDEREAL_TIME_0p16 >> 8 ) & 0xFF)
						btfsc	s1_sw_state,BIT_SW_DOUBLE_SPEED
						movlw	((SOLAR_TIME_0p16 >> 8 )    & 0xFF)
						btfsc	s1_sw_state,BIT_SW_STOP
						movlw	((LUNAR_TIME_0p16 >> 8 )    & 0xFF)
						movwf	d1_tracking_period_0p16+1

						; change WDT period
						movlw	WDTCON_active
						movwf	s1_WDTCON

						; output enable
						movlw	TRISA_active
						movwf	s1_TRISA
						movlw	TRISC_active
						movwf	s1_TRISC

						; interrupt enable
						bsf		s1_PIE2,OSFIE
						bsf		s1_PIE3,CCP4IE

						movlw	INTCON_active
						movwf	INTCON
						goto	idle_loop

f1_get_notification:	movlb	1
						; if reset button has released, MCU sleep
						btfsc	s1_sw_toggle,BIT_SW_RESET
						bra		$+3
						btfss	s1_sw_state,BIT_SW_RESET
						call	__prepare_for_sleep	; toggle == 0 and state == 0, call but never return

						; if origin flag is set, index is reset to lut_start
						btfsc	s1_sw_state,BIT_SW_ORIGIN	; 6
						call	f1_set_idx_to_lut_start		; 17

	if (REWIND_TIMEOUT != 0) && (LUT_UPDATE_PERIOD != 0)
						; wait for timeout
						movlw	LOW  (pfm_lut_end + _SIZEOF_LUT_ENTRY*REWIND_TIMEOUT/LUT_UPDATE_PERIOD)
						subwf	lut_address_L,W
						movlw	HIGH (pfm_lut_end + _SIZEOF_LUT_ENTRY*REWIND_TIMEOUT/LUT_UPDATE_PERIOD)
						subwfb	lut_address_H,W
						btfsc	STATUS,C
						call	__prepare_for_sleep	; timeout, call but never return

						; motor is freed when lut_address >= lut_end
						movlw	LOW pfm_lut_end
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_end
						subwfb	lut_address_H,W
						btfsc	STATUS,C
						retlw	CTRL_MOTOR_FREE

						; if limit flag is set, index is set to lut_end
						btfsc	s1_sw_state,BIT_SW_LIMIT	; 13
						call	f1_set_idx_to_lut_end		; 17
	else
						; index is wrap-round to lut_start when lut_address >= lut_end
						movlw	LOW pfm_lut_end
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_end
						subwfb	lut_address_H,W
						btfsc	STATUS,C					; 5
						call	f1_set_idx_to_lut_start		; 17
	endif
						; limit sw or stop sw is pushed
						btfss	s1_sw_state,BIT_SW_LIMIT
						btfsc	s1_sw_state,BIT_SW_STOP
						retlw	CTRL_MOTOR_STOP

						; power sw is pushed
						btfss	s1_sw_toggle,BIT_SW_RESET
						retlw	CTRL_MOTOR_BREAK_IN

						; twice speed sw is pushed
						btfsc	s1_sw_state,BIT_SW_DOUBLE_SPEED
						retlw	CTRL_MOTOR_2x	; set motor speed to 2x

						; none of sw is pushed.
						retlw	CTRL_MOTOR_1x	; set motor speed to 1x	; 9 62/54

__prepare_for_sleep:	; interrupt disable
						bcf		INTCON,GIE

						movlb	2
						; disconnect power from motor
						bsf		s2_CM1CON0,C1POL

						call	wait_1us
						TEST_LED_OFF	TEST_CPU_USAGE
						reset

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Key Handling																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
f1_read_keys:			movlb	1
						movf	s1_sw_state,W
						movwf	s1_sw_released

						; prepare to reduce chattering of SWs
						movf	d1_sw_transient+1,W
						movwf	s1_sw_press
						movwf	s1_sw_release

						movf	d1_sw_transient+0,W
						andwf	s1_sw_press,F
						iorwf	s1_sw_release,F
						movwf	d1_sw_transient+1			; 10

						movlb	0
						; read SWs
						swapf	s0_PORTC,W
						andlw	(1<<BIT_SW_STOP)|(1<<BIT_SW_DOUBLE_SPEED)
						iorwf	s0_PORTA,W			; {0, 0, RC1, RC0, RA3, RA2, RA1, RA0}

						movlb	1
						; check debugger
						btfsc	WREG,RA0
						bra		$+3
						btfss	WREG,RA1
						iorlw	(1<<RA0)|(1<<RA1)	; if both RA0 and RA1 are ON, set both to OFF

						; update states of SWs
						xorlw	0xFF				; invert logic (from negative to positive)
						andwf	s1_sw_press,F
						iorwf	s1_sw_release,F
						movwf	d1_sw_transient+0			; 13

						; reduce chattering of SWs
						movf	s1_sw_press,W
						iorwf	s1_sw_state,F		; set by triple 1 reads
						movf	s1_sw_release,W
						andwf	s1_sw_state,F		; clear by triple 0 reads

						; event generation
						comf	s1_sw_state,W
						andwf	s1_sw_released,F	; released SW(1: released)

						movf	s1_sw_released,W
						xorwf	s1_sw_toggle,F
wait_1us:				return								; 10	33

f1_maintain_4hz:		movlb	1
						bcf		notification_flag,BIT_POLLING
						movf	cemf_sampling_period_L,W
						subwf	t1_elapsed_time+0,F
						movf	cemf_sampling_period_H,W
						subwfb	t1_elapsed_time+1,F
						clrw
						subwfb	t1_elapsed_time+2,F
						btfsc	STATUS,C
						return								; 10

						bsf		notification_flag,BIT_POLLING
						movlw	( FCY_SCALED/POLLING_FREQUENCY        & 0xFF)
						addwf	t1_elapsed_time+0,F
						movlw	((FCY_SCALED/POLLING_FREQUENCY >>  8) & 0xFF)
						addwfc	t1_elapsed_time+1,F
						movlw	((FCY_SCALED/POLLING_FREQUENCY >> 16) & 0xFF)
						addwfc	t1_elapsed_time+2,F
						return								;  9	19

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Data EEPROM																				;
;		*		*		*		*		*		*		*		*		*		*		*		;
f1_restore_lut_index:
						movlb	3
						bcf		s3_EECON1,EEPGD			; Select Data EEPROM
						bcf		s3_EECON1,CFGS			; Do not select Configuration Space

						; read motor parameter from EEPROM
						movlw	LOW dfm_motor_constant
						movwf	s3_EEADRL

						call	__read_data_eeprom
						movwf	motor_constant_LL
						call	__read_data_eeprom
						movwf	motor_constant_LH
						call	__read_data_eeprom
						movwf	motor_constant_HL
						call	__read_data_eeprom
						movwf	motor_constant_HH

						; read lut address from EEPROM
						movlw	LOW dfm_lut_address
						movwf	s3_EEADRL

						call	__read_data_eeprom
						movwf	lut_address_L
						call	__read_data_eeprom
						movwf	lut_address_H
						call	__read_data_eeprom
						movwf	lut_lookup_time

						; upper range check
						movlw	LOW pfm_lut_end
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_end
						subwfb	lut_address_H,W
						btfsc	STATUS,C
						bra		f1_set_idx_to_lut_start

						; lower range check
						movlw	LOW pfm_lut_start
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_start
						subwfb	lut_address_H,W
						btfss	STATUS,C
						bra		f1_set_idx_to_lut_start

						; upper range check
						decf	lut_lookup_time,W
						addlw	-POLLING_FREQUENCY*LUT_UPDATE_PERIOD
						btfss	STATUS,C
						bra		__range_ok

f1_set_idx_to_lut_start: movlw	HIGH pfm_lut_start
						movwf	lut_address_H
						movlw	LOW pfm_lut_start
						movwf	lut_address_L
						bra		__reset_lut_counter			;  6

f1_set_idx_to_lut_end:	movlw	HIGH pfm_lut_end
						movwf	lut_address_H
						movlw	LOW pfm_lut_end
						movwf	lut_address_L
						bra		__reset_lut_counter			;  6

f1_advance_lut_index:
		if (LUT_UPDATE_PERIOD != 0)
						btfsc	notification_flag,BIT_POLLING
						decfsz	lut_lookup_time,F
						bra		__range_ok
		endif
						movlw	LOW _SIZEOF_LUT_ENTRY
						addwf	lut_address_L,F
						movlw	HIGH _SIZEOF_LUT_ENTRY
						addwfc	lut_address_H,F			; lut_address ++

__reset_lut_counter:	movlw	POLLING_FREQUENCY*LUT_UPDATE_PERIOD
						movwf	lut_lookup_time				;  2

__range_ok:				movlb	1
						comf	lut_address_L,W
						movwf	d1_lut_address_com+0
						comf	lut_address_H,W
						movwf	d1_lut_address_com+1
						return								;  7	15

; save lut index into latest entry in EEPROM
f1_save_lut_index:		movlb	1
						comf	lut_address_L,W
						xorwf	d1_lut_address_com+0,W
						btfss	STATUS,Z
						return
						comf	lut_address_H,W
						xorwf	d1_lut_address_com+1,W
						btfss	STATUS,Z
						return

						movlb	3
						bcf		s3_EECON1,EEPGD			; Select Data EEPROM
						bcf		s3_EECON1,CFGS			; Do not select Configuration Space
						movlw	LOW dfm_lut_address
						movwf	s3_EEADRL

						; write lower byte
						movf	lut_address_L,W
						call	__write_data_eeprom

						; write upper byte
						movf	lut_address_H,W
						call	__write_data_eeprom

						; write counter byte
						movf	lut_lookup_time,W
						call	__write_data_eeprom

						movlb	1
						; succesfully wrote
						movf	lut_address_L,W
						movwf	d1_lut_address_com+0
						movf	lut_address_H,W
						movwf	d1_lut_address_com+1
						return

; load factory default
set_factory_default:
						movlw	( MOTOR_CONSTANT_init        & 0xFF)
						movwf	motor_constant_LL
						movlw	((MOTOR_CONSTANT_init >>  8) & 0xFF)
						movwf	motor_constant_LH
						movlw	((MOTOR_CONSTANT_init >> 16) & 0xFF)
						movwf	motor_constant_HL
						movlw	((MOTOR_CONSTANT_init >> 24) & 0xFF)
						movwf	motor_constant_HH
; update motor parameter
update_parameter:
						movlb	3
						bcf		s3_EECON1,EEPGD			; Select Data EEPROM
						bcf		s3_EECON1,CFGS			; Do not select Configuration Space
						movlw	LOW dfm_motor_constant
						movwf	s3_EEADRL

						; program parameter area
						movf	motor_constant_LL,W
						call	__write_data_eeprom
						movf	motor_constant_LH,W
						call	__write_data_eeprom
						movf	motor_constant_HL,W
						call	__write_data_eeprom
						movf	motor_constant_HH,W
						call	__write_data_eeprom
						reset

;		EEADR	address of data
;		BSR		3
__read_data_eeprom:		bsf		s3_EECON1,RD
						movf	s3_EEDATL,W
						incf	s3_EEADRL,F
						return

;		WREG	data to be written
;		EEADR	address of data
;		BSR		3
__write_data_eeprom:	clrwdt
						bsf		s3_EECON1,RD
						xorwf	s3_EEDATL,F
						btfsc	STATUS,Z
						bra		__skip_programming
						movwf	s3_EEDATL

						TEST_LED_ON TEST_NVM

						bsf		s3_EECON1,WREN
						movlw	0x55				; Load 55h to get ready for unlock sequence
						movwf	s3_EECON2			; First step is to load 55h into NVMCON2
						movlw	0xAA				; Second step is to load AAh into W
						movwf	s3_EECON2			; Third step is to load AAh into NVMCON2
						bsf		s3_EECON1,WR		; Final step is to set WR bit
						bcf		s3_EECON1,WREN

						btfsc	s3_EECON1,WR
						bra		$-1

						TEST_LED_OFF TEST_NVM

__skip_programming:		incf	s3_EEADRL,F
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Code Flash																				;
;		*		*		*		*		*		*		*		*		*		*		*		;
f6_load_lut_entry:		movlb	1
						; load lut entry as interval time from code flash
						movf	lut_address_L,W
						movwf	FSR0L
						movf	lut_address_H,W
						movwf	FSR0H

						clrf	cemf_sampling_period_L
						clrf	cemf_sampling_period_H

						bsf		STATUS,C
						rrf		d1_tracking_period_0p16+0,W
						movwf	s1_multiplier_bits
__lut_load_lp1:			btfss	STATUS,C
						bra		$+5
						moviw	_CYCLE_TIME_L[FSR0]
						addwf	cemf_sampling_period_L,F
						moviw	_CYCLE_TIME_H[FSR0]
						addwfc	cemf_sampling_period_H,F

						rrf		cemf_sampling_period_H,F
						rrf		cemf_sampling_period_L,F

						lsrf	s1_multiplier_bits,F
						btfss	STATUS,Z
						bra		__lut_load_lp1

						bsf		STATUS,C
						rrf		d1_tracking_period_0p16+1,W
						movwf	s1_multiplier_bits
__lut_load_lp2:			btfss	STATUS,C
						bra		$+5
						moviw	_CYCLE_TIME_L[FSR0]
						addwf	cemf_sampling_period_L,F
						moviw	_CYCLE_TIME_H[FSR0]
						addwfc	cemf_sampling_period_H,F

						rrf		cemf_sampling_period_H,F
						rrf		cemf_sampling_period_L,F

						lsrf	s1_multiplier_bits,F
						btfss	STATUS,Z
						bra		__lut_load_lp2

						moviw	_CYCLE_TIME_L[FSR0]
						btfss	d1_tracking_period_0p16+1,7
						addwf	cemf_sampling_period_L,F
						moviw	_CYCLE_TIME_H[FSR0]
						btfss	d1_tracking_period_0p16+1,7
						addwfc	cemf_sampling_period_H,F

	if (MULT_PERIOD_2X_24p8/MULT_PERIOD_1X_24p8 == 2)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						lslf	cemf_sampling_period_L,F
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rlf		cemf_sampling_period_H,F
	endif
	if (MULT_PERIOD_1X_24p8/MULT_PERIOD_2X_24p8 == 2)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						lsrf	cemf_sampling_period_H,F
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rrf		cemf_sampling_period_L,F
	endif

						movlb	6
						; load sampling start time into CCPR4
						clrf	s6_CCP4CON
						movlw	0xFF
						addwf	cemf_sampling_period_L,W
						movwf	s6_CCPR4L
						movlw	0xFF
						addwfc	cemf_sampling_period_H,W
						movwf	s6_CCPR4H
						movlw	CCP4CON_init
						movwf	s6_CCP4CON
						return

f1_calc_target_voltage:	movlb	1
						; calculate target voltage
						movf	motor_constant_LL,W
						movwf	q1_motor_constant_work+0		; bit position  7 -  0			32p0
						movf	motor_constant_LH,W
						movwf	q1_motor_constant_work+1		; bit position 15 -  8
						movf	motor_constant_HL,W
						movwf	q1_motor_constant_work+2		; bit position 23 - 16
						movf	motor_constant_HH,W
						movwf	q1_motor_constant_work+3		; bit position 31 - 24

						; target_voltage = motor_constant/sampling_period
						;                                                      motor_constant
						;  X X X X X X X X  X X X X X X X X  X X X X X X X X  X X X X X X X X
						;                                  sampling_period
						;                Y  Y Y Y Y Y Y Y Y  Y Y Y Y Y Y Y  
						;                                                          target_voltage
						;                                        0 0 0 0 Q Q Q Q  Q Q Q Q Q Q.Q Q
						;                                        0 0 0 0 0 0 0 0  0 0 0 1 0 0 0 0
						;                                         sentinel for 1x speed ^
						; shift sampling_period by 1bit left
						lslf	cemf_sampling_period_L,W
						movwf	t1_sampling_period+0
						rlf		cemf_sampling_period_H,W
						movwf	t1_sampling_period+1
						clrf	t1_sampling_period+2
						rlf		t1_sampling_period+2,F

	if (MULT_VOLTAGE_1X_24p8 == 128)
						movlw	(1<<5)							; result is 11bit at 1x speed
		if (MULT_VOLTAGE_2X_24p8 == 256)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						movlw	(1<<4)							; result is 12bit at 2x speed
		endif
		if (MULT_VOLTAGE_2X_24p8 == 512)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						movlw	(1<<4)							; result is 12bit at 2x speed
		endif
	endif
	if (MULT_VOLTAGE_1X_24p8 == 256)
						movlw	(1<<4)							; result is 12bit at 1x speed
		if (MULT_VOLTAGE_2X_24p8 == 128)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						movlw	(1<<5)							; result is 11bit at 2x speed
		endif
		if (MULT_VOLTAGE_2X_24p8 == 512)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						movlw	(1<<3)							; result is 13bit at 2x speed
		endif
	endif
	if (MULT_VOLTAGE_1X_24p8 == 512)
						movlw	(1<<3)							; result is 13bit at 1x speed
		if (MULT_VOLTAGE_2X_24p8 == 128)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						movlw	(1<<5)							; result is 11bit at 2x speed
		endif
		if (MULT_VOLTAGE_2X_24p8 == 256)
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						movlw	(1<<4)							; result is 12bit at 2x speed
		endif
	endif
						clrf	t1_target_voltage_15p9+0
						movwf	t1_target_voltage_15p9+1
						clrf	t1_target_voltage_15p9+2		; 20

__pi_ctrl_div_pos:		movf	t1_sampling_period+0,W
						subwf	q1_motor_constant_work+1,F
						movf	t1_sampling_period+1,W
						subwfb	q1_motor_constant_work+2,F
						movf	t1_sampling_period+2,W
						subwfb	q1_motor_constant_work+3,F		; 6
__pi_ctrl_div_common:	rlf		t1_target_voltage_15p9+1,F
						rlf		t1_target_voltage_15p9+2,F
						btfsc	STATUS,C
						return
						lslf	q1_motor_constant_work+0,F
						rlf		q1_motor_constant_work+1,F
						rlf		q1_motor_constant_work+2,F
						rlf		q1_motor_constant_work+3,F
						btfsc	t1_target_voltage_15p9+1,0
						bra		__pi_ctrl_div_pos					; 11	20+(6+11)*12=224
__pi_ctrl_div_neg:		movf	t1_sampling_period+0,W			; 10	20+(8+10)*12=236
						addwf	q1_motor_constant_work+1,F
						movf	t1_sampling_period+1,W
						addwfc	q1_motor_constant_work+2,F
						movf	t1_sampling_period+2,W
						addwfc	q1_motor_constant_work+3,F
						bra		__pi_ctrl_div_common				; 8

f1_calc_target_ratio:	movlb	1
						; calculate target voltage ratio
						; target_ratio = target_voltage(13bit) * FVR(13bit)
						;   8p32 >> 14             15p9        m7p23
						clrf	t1_target_ratio_6p18+0
						clrf	t1_target_ratio_6p18+1
						clrf	t1_target_ratio_6p18+2

						; startup_ratio = startup_voltage(13bit) * FVR(13bit)
						;    8p32 >> 14              15p9        m7p23
						clrf	t1_startup_ratio_6p18+1
						clrf	t1_startup_ratio_6p18+2

						movlw	0x0D
						movwf	s1_multiplier_bits
						movf	d1_adres_FVR_m7p23+0,W
						movwf	t1_adres_FVR_work+0
						movf	d1_adres_FVR_m7p23+1,W
						movwf	t1_adres_FVR_work+1			; 11

__calc_target_ratio_lp:	btfss	t1_adres_FVR_work+0,0
						bra		$+9

						movf	t1_target_voltage_15p9+1,W
						addwf	t1_target_ratio_6p18+1,F
						movf	t1_target_voltage_15p9+2,W
						addwfc	t1_target_ratio_6p18+2,F

						movlw	LOW STARTUP_VOLTAGE_15p1
						addwf	t1_startup_ratio_6p18+1,F
						movlw	HIGH STARTUP_VOLTAGE_15p1
						addwfc	t1_startup_ratio_6p18+2,F

						lsrf	t1_target_ratio_6p18+2,F
						rrf		t1_target_ratio_6p18+1,F
						rrf		t1_target_ratio_6p18+0,F

						lsrf	t1_startup_ratio_6p18+2,F
						rrf		t1_startup_ratio_6p18+1,F

						lsrf	t1_adres_FVR_work+1,F
						rrf		t1_adres_FVR_work+0,F

						decfsz	s1_multiplier_bits,F
						bra		__calc_target_ratio_lp			; 20

						; startup_voltage = target_voltage + IR-drop
						movf	t1_target_ratio_6p18+1,W
						addwf	t1_startup_ratio_6p18+1,F
						movf	t1_target_ratio_6p18+2,W
						addwfc	t1_startup_ratio_6p18+2,F

						lsrf	t1_target_ratio_6p18+2,F
						rrf		t1_target_ratio_6p18+1,F
						rrf		t1_target_ratio_6p18+0,F

						lsrf	t1_startup_ratio_6p18+2,F
						rrf		t1_startup_ratio_6p18+1,F
						return								; 11		11+20*13+11=282

eeprom					code	0xF000
dfm_lut_address:		de		LOW  pfm_lut_start
						de		HIGH pfm_lut_start
						de		POLLING_FREQUENCY*LUT_UPDATE_PERIOD
dfm_motor_constant:		de		( MOTOR_CONSTANT_init        & 0xFF)	; 32p0
						de		((MOTOR_CONSTANT_init >>  8) & 0xFF)
						de		((MOTOR_CONSTANT_init >> 16) & 0xFF)
						de		((MOTOR_CONSTANT_init >> 24) & 0xFF)

						end
