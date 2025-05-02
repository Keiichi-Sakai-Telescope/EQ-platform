						#include		mpasmx.inc

						errorlevel -302
						radix	DEC

FCY						equ		4000000	; 4MHz, instruction cycle
PWM_PERIOD				equ		200		; PWM frequency = FCY/PWM_PERIOD, min: 200, max: 256
OPEN_LOOP_PERIOD		equ		62500					; fixed settings
ADC_INTERVAL			equ		15

 __config	_CONFIG1,	_FEXTOSC_HS & _RSTOSC_HFINT1
 __config	_CONFIG2,	_MCLRE_OFF & _PPS1WAY_OFF & _WDTE_ON & _BOREN_SLEEP & _BORV_LOW & _LPBOREN_OFF
 __config	_CONFIG3,	_LVP_OFF

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
CLKRCON_init			equ		(1<<CLKREN)|(B'10'<<CLKRDC0)|(B'000'<<CLKRDIV0)	; Fcy
CLC1CON_init			equ		(1<<LC1EN)|(B'010'<<LC1MODE0)	; 4-input AND
CLC1POL_init			equ		B'1110'
						#include test.inc

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
						;        PWM6  PWM5  CCP4  CCP3  CCP2  CCP1
						; 16bit                T3   *T3    T1   *T1
						;  8bit   *T6    T6    T4    T4   *T2    T2
CCPTMRS_init			equ		(B'10'<<C4TSEL0)|(B'10'<<C3TSEL0)|(B'01'<<C2TSEL0)|(B'01'<<C1TSEL0)
PWMTMRS_init			equ		(B'11'<<P6TSEL0)|(B'11'<<P5TSEL0)

						; Trigger for A/D conversion of CEMF
T1CON_init				equ		(T1_PRESCALER<<T1CKPS0)|(1<<TMR1ON)
CCP1CON_init			equ		(1<<CCP1EN)|(B'1011'<<CCP1MODE0)	; 0-1-0 pulse, clear on match

						; PWM generator
T2CON_init				equ		(B'000'<<T2CKPS0)|(1<<TMR2ON)	; FCY, pre 1:1, post 1:1
T2PR_init				equ		PWM_PERIOD - 1
CCP2CON_init			equ		(1<<CCP2EN)|(B'1111'<<CCP2MODE0)	; Right-aligned, PWM
CWG1CON0_init			equ		(1<<EN)|(B'100'<<MODE0)	; PWM HB
CWG1CON1_init			equ		(0<<POLB)|(1<<POLA)		; POLA: active L, POLB: active H
CWG1DAT_init			equ		B'0100'					; CCP2
CWG1AS0_init			equ		(1<<REN)|(B'10'<<LSBD0)|(B'11'<<LSAC0)	; A=H, B=L
CWG1AS1_init			equ		(1<<AS1E)				; by C1
CWG1DBR_init			equ		2						; delay time of rising is 2*Tosc = 125ns
CWG1DBF_init			equ		3						; delay time of falling is 3*Tosc = 188ns
PPS_HB1_HS_init			equ		B'01000'				; Rxy source is CWG1A
PPS_HB1_LS_init			equ		B'01001'				; Rxy source is CWG1B
PPS_HB2_LS_init			equ		B'10110'				; Rxy source is C1
CM1CON0_init			equ		(0<<C1ON)|(0<<C1POL)|(1<<C1SP)|(0<<C1HYS)
WPUC_CEMF_SENSE			equ		WPUC3
TRISC_CEMF_SENSE		equ		TRISC3
ANSELC_CEMF_SENSE		equ		ANSC3
PORTC_CEMF_SENSE		equ		RC3

						; Trigger for A/D conversion of FVR
FVR_SETTLING_TIME		set		25
T3CON_init				equ		(1<<TMR3ON)						; FCY
CCP3CON_init			equ		(1<<CCP3EN)|(B'1011'<<CCP3MODE0)	; 0-1-0 pulse, clear on match

						; Constant brightness LED driver
T6CON_init				equ		(B'01'<<T6CKPS0)|(1<<TMR6ON) ; FCY/4, pre 1:4, post 1:1
T6PR_init				equ		255
PWM6CON_init			equ		(1<<PWM6EN)
PWM6DCH_init			equ		(256-6*(FCY/1000000)/4)	;  minimum settling time 5.25us
PPS_LED_CATHODE_init	equ		B'00011'				; Rxy source is PWM6
TRISC_LED_CATHODE		equ		TRISC4

; etc.
OSCFRQ_backup			equ		(B'0110'<<HFFRQ0)		; 16MHz
OSCFRQ_sleep			equ		(B'0000'<<HFFRQ0)		; 1MHz
OSCCON1_backup			equ		(B'110'<<NOSC0)			; HFINTOSC, 1:1
OSCCON1_sleep			equ		(B'110'<<NOSC0)			; HFINTOSC, 1:1
OSCCON1_active			equ		(B'001'<<NOSC0)			; EXTOSC with 4x PLL, 1:1
WDTCON_active			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON_sleep			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON_calibration		equ		(B'01010'<<WDTPS0)		; 1s nominal, 750ms minimum, 1250ms maximum
PCON_init				equ		(1<<NOT_BOR)|(1<<NOT_POR)|(1<<NOT_RI)|(1<<NOT_RMCLR)
PCON_cold_boot			equ		(1<<NOT_BOR)|(1<<NOT_RMCLR)
INTCON_sleep			equ		(1<<PEIE)
INTCON_active			equ		(1<<PEIE)|(1<<GIE)
VREGCON_active			equ		B'11'					; Main regulator in LP mode
VREGCON_sleep			equ		B'11'					; Main regulator in LP mode
SIDEREAL_TIME_0p16		equ		65536		; [0.5, 1.5)
SOLAR_TIME_0p16			equ		65715		; [0.5, 1.5)
LUNAR_TIME_0p16			equ		68019		; [0.5, 1.5)

PMD0_init				equ		(0<<SYSCMD)|(1<<FVRMD)|(0<<NVMMD)|(1<<CLKRMD)|(1<<IOCMD)
PMD1_init				equ		(1<<NCOMD)|(0<<TMR6MD)|(1<<TMR5MD)|(1<<TMR4MD)|(1<<TMR3MD)|(0<<TMR2MD)|(0<<TMR1MD)|(1<<TMR0MD)
PMD2_init				equ		(1<<DACMD)|(1<<ADCMD)|(1<<CMP2MD)|(0<<CMP1MD)
PMD3_init				equ		(1<<CWG2MD)|(0<<CWG1MD)|(0<<PWM6MD)|(1<<PWM5MD)|(1<<CCP4MD)|(1<<CCP3MD)|(0<<CCP2MD)|(0<<CCP1MD)
PMD4_init				equ		(1<<UART1MD)|(1<<MSSP2MD)|(1<<MSSP1MD)
PMD5_init				equ		(1<<CLC4MD)|(1<<CLC3MD)|(1<<CLC2MD)|(1<<CLC1MD)|(1<<DSMMD)

; ADC and FVR configurations
FVRCON_init				equ		B'11000010'		; CDAFVR = 00 for OFF, ADFVR = 10 for 2.048V
ADCON0_MUX_FVR			equ		(B'111111'<<CHS0)|(1<<ADON)		; FVR(FVR output)
ADCON0_MUX_MOTOR		equ		(B'010011'<<CHS0)|(1<<ADON)		; ANC3(motor, RC3 pin)
ADCON0_MUX_LED			equ		(B'010101'<<CHS0)|(1<<ADON)		; ANC5(LED, RC5 pin)
ADCON1_VDD_RATIOMETRIC	equ		(5<<ADCS0)|(0<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=VDD, flush right
ADACT_CCP1				equ		B'01100'		; Timer1-CCP1 match
ADACT_CCP3				equ		B'01110'		; Timer3-CCP3 match
ADACT_LED				equ		B'00010'		; Timer6-PR6 match

; I/O port configurations
LATA_init				equ		B'000000'		; digital output:                RA2          
WPUA_init				equ		B'001011'		; pull-up:                  RA3,      RA1, RA0
TRISA_active			equ		B'001011'		; input:                    RA3,      RA1, RA0
ANSELA_init				equ		B'110100'		; digital input:            RA3,      RA1, RA0 
SLRCONA_init			equ		B'111111'		; high-speed:                                 
ODCONA_init				equ		B'000000'		; open drain:                                 
INLVLA_init				equ		B'111111'		; ST input:       RA5, RA4, RA3, RA2, RA1, RA0

LATB_init				equ		B'10000000'		; digital output: RB7, RB6, RB5, RB4                    
WPUB_init				equ		B'00000000'		; pull-up:                                              
TRISB_active			equ		B'00000000'		; input:                                                
ANSELB_init				equ		B'11110000'		; digital input:                                        
SLRCONB_init			equ		B'11100000'		; high-speed:                    RB4                    
ODCONB_init				equ		B'00010000'		; open drain:                    RB4                    
INLVLB_init				equ		B'11110000'		; ST input:       RB7, RB6, RB5, RB4

LATC_init				equ		B'11010000'		; digital output: RC7, RC6,      RC4,      RC2          
WPUC_init				equ		B'00000011'		; pull-up:                                      RC1, RC0
TRISC_active			equ		B'00101011'		; input:                    RC5,      RC3,      RC1, RC0
ANSELC_init				equ		B'11110100'		; digital input:                      RC3,      RC1, RC0
SLRCONC_init			equ		B'11111011'		; high-speed:                              RC2          
ODCONC_init				equ		B'00000100'		; open drain:                              RC2          
INLVLC_init				equ		B'11110111'		; ST input:       RC7, RC6, RC5, RC4,      RC2, RC1, RC0

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
s0_PORTB				EQU		PORTB
s0_PORTC				EQU		PORTC
s0_PIR0					EQU		PIR0
s0_PIR1					EQU		PIR1
s0_PIR2					EQU		PIR2
s0_PIR3					EQU		PIR3
s0_PIR4					EQU		PIR4
s0_TMR0					EQU		TMR0
s0_TMR0L				EQU		TMR0L
s0_T0PR					EQU		PR0
s0_TMR0H				EQU		TMR0H
s0_T0CON0				EQU		T0CON0
s0_T0CON1				EQU		T0CON1
s0_T1TMRL				EQU		TMR1L
s0_T1TMRH				EQU		TMR1H
s0_T1CON				EQU		T1CON
s0_T1GCON				EQU		T1GCON
s0_T2TMR				EQU		TMR2
s0_T2PR					EQU		PR2
s0_T2CON				EQU		T2CON

; general purpose RAM (Bank 0) allocation
bank0_ram				udata	0x20

;-----Bank1------------------
s1_TRISA				EQU		TRISA
s1_TRISB				EQU		TRISB
s1_TRISC				EQU		TRISC
s1_PIE0					EQU		PIE0
s1_PIE1					EQU		PIE1
s1_PIE2					EQU		PIE2
s1_PIE3					EQU		PIE3
s1_PIE4					EQU		PIE4
s1_WDTCON				EQU		WDTCON
s1_ADRESL				EQU		ADRESL
s1_ADRESH				EQU		ADRESH
s1_ADCON0				EQU		ADCON0
s1_ADCON1				EQU		ADCON1
s1_ADACT				EQU		ADACT

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
s2_LATB					EQU		LATB
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

;-----Bank3------------------
s3_ANSELA				EQU		ANSELA
s3_ANSELB				EQU		ANSELB
s3_ANSELC				EQU		ANSELC
s3_VREGCON				EQU		VREGCON
s3_RC1REG				EQU		RC1REG
s3_TX1REG				EQU		TX1REG
s3_SP1BRGL				EQU		SP1BRGL
s3_SP1BRGH				EQU		SP1BRGH
s3_RC1STA				EQU		RC1STA
s3_TX1STA				EQU		TX1STA
s3_BAUD1CON				EQU		BAUD1CON

;-----Bank4------------------
s4_WPUA					EQU		WPUA
s4_WPUB					EQU		WPUB
s4_WPUC					EQU		WPUC
s4_SSP1BUF				EQU		SSP1BUF
s4_SSP1ADD				EQU		SSP1ADD
s4_SSP1MSK				EQU		SSP1MSK
s4_SSP1STAT				EQU		SSP1STAT
s4_SSP1CON1				EQU		SSP1CON1
s4_SSP1CON2				EQU		SSP1CON2
s4_SSP1CON3				EQU		SSP1CON3
s4_SSP2BUF				EQU		SSP2BUF
s4_SSP2ADD				EQU		SSP2ADD
s4_SSP2MSK				EQU		SSP2MSK
s4_SSP2STAT				EQU		SSP2STAT
s4_SSP2CON1				EQU		SSP2CON1
s4_SSP2CON2				EQU		SSP2CON2
s4_SSP2CON3				EQU		SSP2CON3

;-----Bank5------------------
s5_ODCONA				EQU		ODCONA
s5_ODCONB				EQU		ODCONB
s5_ODCONC				EQU		ODCONC
s5_CCPR1L				EQU		CCPR1L
s5_CCPR1H				EQU		CCPR1H
s5_CCP1CON				EQU		CCP1CON
s5_CCP1CAP				EQU		CCP1CAP
s5_CCPR2L				EQU		CCPR2L
s5_CCPR2H				EQU		CCPR2H
s5_CCP2CON				EQU		CCP2CON
s5_CCP2CAP				EQU		CCP2CAP
s5_CCPTMRS				EQU		CCPTMRS

;-----Bank6------------------
s6_SLRCONA				EQU		SLRCONA
s6_SLRCONB				EQU		SLRCONB
s6_SLRCONC				EQU		SLRCONC
s6_CCPR3L				EQU		CCPR3L
s6_CCPR3H				EQU		CCPR3H
s6_CCP3CON				EQU		CCP3CON
s6_CCP3CAP				EQU		CCP3CAP
s6_CCPR4L				EQU		CCPR4L
s6_CCPR4H				EQU		CCPR4H
s6_CCP4CON				EQU		CCP4CON
s6_CCP4CAP				EQU		CCP4CAP

;-----Bank7------------------
s7_INLVLA				EQU		INLVLA
s7_INLVLB				EQU		INLVLB
s7_INLVLC				EQU		INLVLC
s7_IOCAP				EQU		IOCAP
s7_IOCAN				EQU		IOCAN
s7_IOCAF				EQU		IOCAF
s7_IOCBP				EQU		IOCBP
s7_IOCBN				EQU		IOCBN
s7_IOCBF				EQU		IOCBF
s7_IOCCP				EQU		IOCCP
s7_IOCCN				EQU		IOCCN
s7_IOCCF				EQU		IOCCF
s7_CLKRCON				EQU		CLKRCON
s7_MDCON				EQU		MDCON
s7_MDSRC				EQU		MDSRC
s7_MDCARL				EQU		MDCARL
s7_MDCARH				EQU		MDCARH

;-----Bank8------------------
s8_T3TMRL				EQU		TMR3L
s8_T3TMRH				EQU		TMR3H
s8_T3CON				EQU		T3CON
s8_T3GCON				EQU		T3GCON
s8_T4TMR				EQU		TMR4
s8_T4PR					EQU		PR4
s8_T4CON				EQU		T4CON
s8_T5TMRL				EQU		TMR5L
s8_T5TMRH				EQU		TMR5H
s8_T5CON				EQU		T5CON
s8_T5GCON				EQU		T5GCON
s8_T6TMR				EQU		TMR6
s8_T6PR					EQU		PR6
s8_T6CON				EQU		T6CON

;-----Bank9------------------
s9_NCO1ACCL				EQU		NCO1ACCL
s9_NCO1ACCH				EQU		NCO1ACCH
s9_NCO1ACCU				EQU		NCO1ACCU
s9_NCO1INCL				EQU		NCO1INCL
s9_NCO1INCH				EQU		NCO1INCH
s9_NCO1INCU				EQU		NCO1INCU
s9_NCO1CON				EQU		NCO1CON
s9_NCO1CLK				EQU		NCO1CLK

;-----Bank12------------------
s12_PWM5DCL				EQU		PWM5DCL
s12_PWM5DCH				EQU		PWM5DCH
s12_PWM5CON				EQU		PWM5CON
s12_PWM6DCL				EQU		PWM6DCL
s12_PWM6DCH				EQU		PWM6DCH
s12_PWM6CON				EQU		PWM6CON
s12_PWMTMRS				EQU		PWMTMRS

;-----Bank13------------------
s13_CWG1CLKCON			EQU		CWG1CLKCON
s13_CWG1DAT				EQU		CWG1DAT
s13_CWG1DBR				EQU		CWG1DBR
s13_CWG1DBF				EQU		CWG1DBF
s13_CWG1CON0			EQU		CWG1CON0
s13_CWG1CON1			EQU		CWG1CON1
s13_CWG1AS0				EQU		CWG1AS0
s13_CWG1AS1				EQU		CWG1AS1
s13_CWG1STR				EQU		CWG1STR

;-----Bank14------------------
s14_CWG2CLKCON			EQU		CWG2CLKCON
s14_CWG2DAT				EQU		CWG2DAT
s14_CWG2DBR				EQU		CWG2DBR
s14_CWG2DBF				EQU		CWG2DBF
s14_CWG2CON0			EQU		CWG2CON0
s14_CWG2CON1			EQU		CWG2CON1
s14_CWG2AS0				EQU		CWG2AS0
s14_CWG2AS1				EQU		CWG2AS1
s14_CWG2STR				EQU		CWG2STR

;-----Bank17------------------
s17_NVMADRL				EQU		NVMADRL
s17_NVMADRH				EQU		NVMADRH
s17_NVMDATL				EQU		NVMDATL
s17_NVMDATH				EQU		NVMDATH
s17_NVMCON1				EQU		NVMCON1
s17_NVMCON2				EQU		NVMCON2
s17_PCON0				EQU		PCON0

;-----Bank18------------------
s18_PMD0				EQU		PMD0
s18_PMD1				EQU		PMD1
s18_PMD2				EQU		PMD2
s18_PMD3				EQU		PMD3
s18_PMD4				EQU		PMD4
s18_PMD5				EQU		PMD5
s18_CPUDOZE				EQU		CPUDOZE
s18_OSCCON1				EQU		OSCCON1
s18_OSCCON2				EQU		OSCCON2
s18_OSCCON3				EQU		OSCCON3
s18_OSCSTAT1			EQU		OSCSTAT1
s18_OSCEN				EQU		OSCEN
s18_OSCTUNE				EQU		OSCTUNE
s18_OSCFRQ				EQU		OSCFRQ

;-----Bank28------------------
s28_PPSLOCK				EQU		PPSLOCK
s28_INTPPS				EQU		INTPPS
s28_T0CKIPPS			EQU		T0CKIPPS
s28_T1CKIPPS			EQU		T1CKIPPS
s28_T1GPPS				EQU		T1GPPS
s28_CCP1PPS				EQU		CCP1PPS
s28_CCP2PPS				EQU		CCP2PPS
s28_CCP3PPS				EQU		CCP3PPS
s28_CCP4PPS				EQU		CCP4PPS
s28_CWG1PPS				EQU		CWG1PPS
s28_CWG2PPS				EQU		CWG2PPS
s28_MDCIN1PPS			EQU		MDCIN1PPS
s28_MDCIN2PPS			EQU		MDCIN2PPS
s28_MDMINPPS			EQU		MDMINPPS
s28_SSP2CLKPPS			EQU		SSP2CLKPPS
s28_SSP2DATPPS			EQU		SSP2DATPPS
s28_SSP2SSPPS			EQU		SSP2SSPPS
s28_SSP1CLKPPS			EQU		SSP1CLKPPS
s28_SSP1DATPPS			EQU		SSP1DATPPS
s28_SSP1SSPPS			EQU		SSP1SSPPS
s28_RXPPS				EQU		RXPPS
s28_TXPPS				EQU		TXPPS
s28_CLCIN0PPS			EQU		CLCIN0PPS
s28_CLCIN1PPS			EQU		CLCIN1PPS
s28_CLCIN2PPS			EQU		CLCIN2PPS
s28_CLCIN3PPS			EQU		CLCIN3PPS
s28_T3CKIPPS			EQU		T3CKIPPS
s28_T3GPPS				EQU		T3GPPS
s28_T5CKIPPS			EQU		T5CKIPPS
s28_T5GPPS				EQU		T5GPPS

;-----Bank29------------------
s29_RA0PPS				EQU		RA0PPS
s29_RA1PPS				EQU		RA1PPS
s29_RA2PPS				EQU		RA2PPS
s29_RA4PPS				EQU		RA4PPS
s29_RA5PPS				EQU		RA5PPS
s29_RB4PPS				EQU		RB4PPS
s29_RB5PPS				EQU		RB5PPS
s29_RB6PPS				EQU		RB6PPS
s29_RB7PPS				EQU		RB7PPS
s29_RC0PPS				EQU		RC0PPS
s29_RC1PPS				EQU		RC1PPS
s29_RC2PPS				EQU		RC2PPS
s29_RC3PPS				EQU		RC3PPS
s29_RC4PPS				EQU		RC4PPS
s29_RC5PPS				EQU		RC5PPS
s29_RC6PPS				EQU		RC6PPS
s29_RC7PPS				EQU		RC7PPS

;-----Bank30------------------
s30_CLCDATA				EQU		CLCDATA
s30_CLC1CON				EQU		CLC1CON
s30_CLC1POL				EQU		CLC1POL
s30_CLC1SEL0			EQU		CLC1SEL0
s30_CLC1SEL1			EQU		CLC1SEL1
s30_CLC1SEL2			EQU		CLC1SEL2
s30_CLC1SEL3			EQU		CLC1SEL3
s30_CLC1GLS0			EQU		CLC1GLS0
s30_CLC1GLS1			EQU		CLC1GLS1
s30_CLC1GLS2			EQU		CLC1GLS2
s30_CLC1GLS3			EQU		CLC1GLS3
s30_CLC2CON				EQU		CLC2CON
s30_CLC2POL				EQU		CLC2POL
s30_CLC2SEL0			EQU		CLC2SEL0
s30_CLC2SEL1			EQU		CLC2SEL1
s30_CLC2SEL2			EQU		CLC2SEL2
s30_CLC2SEL3			EQU		CLC2SEL3
s30_CLC2GLS0			EQU		CLC2GLS0
s30_CLC2GLS1			EQU		CLC2GLS1
s30_CLC2GLS2			EQU		CLC2GLS2
s30_CLC2GLS3			EQU		CLC2GLS3
s30_CLC3CON				EQU		CLC3CON
s30_CLC3POL				EQU		CLC3POL
s30_CLC3SEL0			EQU		CLC3SEL0
s30_CLC3SEL1			EQU		CLC3SEL1
s30_CLC3SEL2			EQU		CLC3SEL2
s30_CLC3SEL3			EQU		CLC3SEL3
s30_CLC3GLS0			EQU		CLC3GLS0
s30_CLC3GLS1			EQU		CLC3GLS1
s30_CLC3GLS2			EQU		CLC3GLS2
s30_CLC3GLS3			EQU		CLC3GLS3
s30_CLC4CON				EQU		CLC4CON
s30_CLC4POL				EQU		CLC4POL
s30_CLC4SEL0			EQU		CLC4SEL0
s30_CLC4SEL1			EQU		CLC4SEL1
s30_CLC4SEL2			EQU		CLC4SEL2
s30_CLC4SEL3			EQU		CLC4SEL3
s30_CLC4GLS0			EQU		CLC4GLS0
s30_CLC4GLS1			EQU		CLC4GLS1
s30_CLC4GLS2			EQU		CLC4GLS2
s30_CLC4GLS3			EQU		CLC4GLS3

;-----Bank31------------------
s31_STATUS_SHAD			EQU		STATUS_SHAD
s31_WREG_SHAD			EQU		WREG_SHAD
s31_BSR_SHAD			EQU		BSR_SHAD
s31_PCLATH_SHAD			EQU		PCLATH_SHAD
s31_FSR0L_SHAD			EQU		FSR0L_SHAD
s31_FSR0H_SHAD			EQU		FSR0H_SHAD
s31_FSR1L_SHAD			EQU		FSR1L_SHAD
s31_FSR1H_SHAD			EQU		FSR1H_SHAD
s31_STKPTR				EQU		STKPTR
s31_TOSL				EQU		TOSL
s31_TOSH				EQU		TOSH

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Reset vector																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
reset_vector			code	0x000
						movlb	3
						movlw	VREGCON_active
						movwf	s3_VREGCON
						goto	common_entry_point

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Interrupt Routine																		;
;		*		*		*		*		*		*		*		*		*		*		*		;

; interrupt handler is consist of pulse generator and voltage sampler.
;		the pulse generator is driven by T2 cooperatively with CCP2.
;		the voltage sampler is driven by T1 cooperatively with CCP1.
interrupt_handler		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						movlb	0
						btfsc	s0_PIR1,OSFIF
						bra		__intr_osc_fail
						btfsc	s0_PIR4,CCP1IF
						bra		__intr_motor
						reset

__intr_osc_fail:		; clear fail-safe clock operation flag
						bcf		s0_PIR1,OSFIF

						call 	f5_pwm_pause

						movlb	18
						; switch to backup osc.
						movlw	OSCFRQ_backup
						movwf	s18_OSCFRQ
						movlw	OSCCON1_backup
						movwf	s18_OSCCON1
						retfie

__intr_motor:
						bcf		s0_PIR4,CCP1IF

						movlb	18
						; 3.6V,  	6-Speed HE,	no load
						;  900rpm	  15mA,		1.24mA
						; 1800rpm	  25mA,		1.28mA
						;    0rpm	1207uA
						;    OFF	   2uA
						bcf		s18_PMD0,FVRMD
						bcf		s18_PMD1,TMR3MD
						bcf		s18_PMD2,ADCMD
						bcf		s18_PMD3,CCP3MD

						; prepare FVR, ADC, and T3 to acquire VDD voltage
						call	f0_prepare_adc_and_fvr

						; read keys and remove chattering
						call	f1_read_keys			; 35
						call	f1_update_adc_result	; wait for 1st ADC result

						call	f1_get_notification		; 64
						xorwf	notification_flag,W
						movwf	s1_notification_change
						xorwf	notification_flag,F
						call	f1_update_adc_result	; wait for 2nd ADC result

						; maintain 4Hz flag
						call	f1_maintain_4hz			; 21
						call	f1_update_adc_result	; wait for 3rd ADC result

						; set default period into sampling_interval
						call	f5_load_default_period	; 16
						call	f1_update_adc_result	; wait for 4th ADC result

						; turn-OFF ADC
						call	f1_finish_adc_and_fvr
						TEST_PIN_ON	TEST_DUTY_UPDATE

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

						TEST_PIN_OFF TEST_DUTY_UPDATE

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

						movlb	18
	if 0
						movlw	OSCCON1_active
						xorwf	s18_OSCCON2,W
						btfss	STATUS,Z
						bra		__led_turn_off
	else
						btfss	s18_OSCSTAT1,EXTOR
						bra		__led_turn_off			; External OSC is not working
						btfsc	s18_OSCSTAT1,HFOR
						bra		__led_turn_off			; HFINTOSC is working
						btfss	s18_OSCSTAT1,LFOR
						bra		__led_turn_off			; LFINTOSC is not working
						btfsc	s18_OSCSTAT1,SOR
						bra		__led_turn_off			; Secondary OSC is working
						btfsc	s18_OSCSTAT1,ADOR
						bra		__led_turn_off			; ADOSC is working
						btfss	s18_OSCSTAT1,PLLR
						bra		__led_turn_off			; PLL is not working
	endif

__led_turn_on:
		if TEST_TARGET == TEST_NONE
						movlb	1
						; prepare for LED brightness control, set MUX to LED
						bcf		s1_TRISC,TRISC_LED_CATHODE		; LED is controlled by PWM

						movlw	ADACT_LED
						movwf	s1_ADACT
						movlw	ADCON0_MUX_LED
						movwf	s1_ADCON0

						; prepare for LED brightness control, set trigger source
						movlw	(5*FCY/1000000/4)
						bcf		s1_ADCON0,GO
						decfsz	WREG,W
						bra		$-2

; read LED current and control constant brightness
;	Max LED current = 4V/1k = 4mA, duty ratio = 0.25/4 = 64/1024
						; Average Iled = 0.26mA, Iled*1000 = 256 at 100% duty
						; adres_FVR_work = adres_FVR_m7p23 << 3
						lslf	d1_adres_FVR_m7p23+0,W
						movwf	t1_adres_FVR_work+0
						rlf		d1_adres_FVR_m7p23+1,W
						movwf	t1_adres_FVR_work+1
						lslf	t1_adres_FVR_work+0,F
						rlf		t1_adres_FVR_work+1,F
						lslf	t1_adres_FVR_work+0,F
						rlf		t1_adres_FVR_work+1,F
						lslf	t1_adres_FVR_work+0,F
						rlf		t1_adres_FVR_work+1,F
						clrf	t1_adres_FVR_work+2
						; adres_FVR_work = 131072*1024/Vin[mV]

						movlb	0
						bcf		s0_PIR1,ADIF
						movlb	1
						bsf		s1_PIE1,ADIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s1_PIE1,ADIE

						; read LED current
						comf	s1_ADRESL,W
						movwf	d1_led_current+0
						comf	s1_ADRESH,W
						andlw	0x03
						movwf	d1_led_current+1

						; Turn-OFF ADC
						clrf	s1_ADCON0
						clrf	s1_ADACT
						; led_current = 1023 - adres_LED = 1024*Iled[mA]*1000/Vin[mV]

						; Iled		0.26mA			simplify
						; dividend	131072*1024/Vin	512			adres_FVR_work
						; divider	1024*256/Vin	1			led_current
						; quotient	65535+1			65535+1		pulse_width
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

__led_div_pos:			movf	d1_led_current+0,W
						subwf	t1_adres_FVR_work+1,F
						movf	d1_led_current+1,W
						subwfb	t1_adres_FVR_work+2,F
__led_div_common:		btfss	STATUS,C				; output is negative logic
						bsf		pulse_width_L,5
						lslf	pulse_width_L,F
						rlf		pulse_width_H,F
						btfsc	STATUS,C
						bra		__led_div_end
						lslf	t1_adres_FVR_work+0,F
						rlf		t1_adres_FVR_work+1,F
						rlf		t1_adres_FVR_work+2,F
						btfss	pulse_width_L,6			; output is negative logic
						bra		__led_div_pos
__led_div_neg:			movf	d1_led_current+0,W
						addwf	t1_adres_FVR_work+1,F
						movf	d1_led_current+1,W
						addwfc	t1_adres_FVR_work+2,F
						bra		__led_div_common

__led_div_end:			; update duty register
						movlb	12
						movf	pulse_width_L,W
						movwf	s12_PWM6DCL
						movf	pulse_width_H,W
						movwf	s12_PWM6DCH
		endif
						bra		__led_ctrl_end

__led_turn_off:			; halt LED brightness controll
		if TEST_TARGET == TEST_NONE
						movlb	1
						bsf		s1_TRISC,TRISC_LED_CATHODE
		endif
						; fall through

__led_ctrl_end:			movlb	18
						; shutdown ADC, FVR, CCP3, and Timer 3
						bsf		s18_PMD0,FVRMD
						bsf		s18_PMD1,TMR3MD
						bsf		s18_PMD2,ADCMD
						bsf		s18_PMD3,CCP3MD
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Subroutine for Interrupt Routine														;
;		*		*		*		*		*		*		*		*		*		*		*		;
f0_prepare_adc_and_fvr:
						movlb	2
						; power-up fixed voltage regulator for ADC
						movlw	FVRCON_init
						movwf	s2_FVRCON

						movlb	8
						; enter settling time of FVR
						clrf	s8_T3CON
						movlw	LOW  (0x10000 - (FVR_SETTLING_TIME-ADC_INTERVAL)*(FCY/1000000))
						movwf	s8_T3TMRL
						movlw	HIGH (0x10000 - (FVR_SETTLING_TIME-ADC_INTERVAL)*(FCY/1000000))
						movwf	s8_T3TMRH
						movlw	T3CON_init
						movwf	s8_T3CON

						; common settings
						movlb	6
						clrf	s6_CCP3CON
						movlw	LOW	 (ADC_INTERVAL*(FCY/1000000))
						movwf	s6_CCPR3L
						movlw	HIGH (ADC_INTERVAL*(FCY/1000000))
						movwf	s6_CCPR3H
						movlw	CCP3CON_init
						movwf	s6_CCP3CON

						movlb	1
						movlw	ADCON1_VDD_RATIOMETRIC
						movwf	s1_ADCON1

						; configure ADC for input voltage acquisition
						movlw	ADACT_CCP3
						movwf	s1_ADACT
						movlw	ADCON0_MUX_FVR
						movwf	s1_ADCON0

						clrf	d1_adres_FVR_m7p23+0
						clrf	d1_adres_FVR_m7p23+1

						bsf		s1_PIE1,ADIE

						movlb	0
						bcf		s0_PIR1,ADIF
						return

f1_update_adc_result:
						movlb	0
						TEST_PIN_OFF	TEST_CPU_USAGE
						; wait for completion of A/D convertion
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s0_PIR1,ADIF

						movlb	1
						movf	s1_ADRESL,W
						addwf	d1_adres_FVR_m7p23+0,F
						movf	s1_ADRESH,W
						addwfc	d1_adres_FVR_m7p23+1,F
						return

f1_finish_adc_and_fvr:
						movlb	2
						; turn-OFF FVR
						clrf	s2_FVRCON

						movlb	6
						; turn-OFF CCP3
						clrf	s6_CCP3CON

						movlb	8
						; turn-OFF Timer 3
						clrf	s8_T3CON

						movlb	1
						bcf		s1_PIE1,ADIE
						; turn-OFF ADC
						clrf	s1_ADCON0
						clrf	s1_ADACT
						return

f5_load_default_period:	movlb	5
						clrf	s5_CCP1CON
						movlw	LOW  (OPEN_LOOP_PERIOD - 1)
						movwf	s5_CCPR1L
						movlw	HIGH (OPEN_LOOP_PERIOD - 1)
						movwf	s5_CCPR1H
						movlw	CCP1CON_init
						movwf	s5_CCP1CON					;  8

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
						bcf		s2_CM1CON0,C1POL

						movlb	0
						; stop PWM timer
						bcf		s0_T2CON,TMR2ON
						movf	s0_T2PR,W
						movwf	s0_T2TMR

						movlb	2
						; resume half-bridge operation at next cycle
						bsf		s2_CM1CON0,C1POL	; turn-ON HB2 LS FET

						movlb	5
						; set duty ratio to zero
						clrf	s5_CCPR2L
						clrf	s5_CCPR2H
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
						; load lut entry into sampling_interval
						call	f5_load_lut_entry

						; save duty
						movf	s5_CCPR2L,W
						movwf	pulse_width_L
						movf	s5_CCPR2H,W
						movwf	pulse_width_H

						; set duty to 0%
						clrf	s5_CCPR2L
						clrf	s5_CCPR2H

						movlb	0
						movf	s0_T2PR,W
						movwf	s0_T2TMR					; force update duty register

						bsf		pulse_width_L,0

						; stop PWM timer
						bcf		s0_T2CON,TMR2ON
						movwf	s0_T2TMR

						movlb	5
						; restore duty
						movf	pulse_width_L,W
						movwf	s5_CCPR2L
						movf	pulse_width_H,W
						movwf	s5_CCPR2H

						movlb	2
						; disconnect power from motor
						bcf		s2_CM1CON0,C1POL

						movlb	4
						; turn on bias resister
						bsf		s4_WPUC,WPUC_CEMF_SENSE

						movlb	3
						; turn on input buffer
						bcf		s3_ANSELC,ANSELC_CEMF_SENSE

						movlb	0
						; wait for end of fast decay discharge
						btfss	s0_PORTC,PORTC_CEMF_SENSE
						bra		$-1
						btfss	s0_PORTC,PORTC_CEMF_SENSE
						bra		$-1
						btfss	s0_PORTC,PORTC_CEMF_SENSE
						bra		$-1

						movlb	4
						; turn off bias resister
						bcf		s4_WPUC,WPUC_CEMF_SENSE

						movlb	3
						; turn off input buffer
						bsf		s3_ANSELC,ANSELC_CEMF_SENSE

						movlb	2
						; resume half-bridge operation at next cycle
						bsf		s2_CM1CON0,C1POL	; turn-ON HB2 LS FET

						movlb	8
						; enter settling time of CEMF
						clrf	s8_T3CON
						movlw	LOW  (0x10000-(CEMF_SETTLING_TIME-ADC_INTERVAL)*(FCY/1000000))
						movwf	s8_T3TMRL
						movlw	HIGH (0x10000-(CEMF_SETTLING_TIME-ADC_INTERVAL)*(FCY/1000000))
						movwf	s8_T3TMRH
						movlw	T3CON_init
						movwf	s8_T3CON

						movlb	6
						movlw	CCP3CON_init
						movwf	s6_CCP3CON

						movlb	1
						; configure ADC for CEMF acquisition
						movlw	ADACT_CCP3
						movwf	s1_ADACT
						movlw	ADCON0_MUX_MOTOR
						movwf	s1_ADCON0

						; calculate target_voltage from sampling_interval
						call	f1_calc_target_voltage

						; calculate target ratio from target voltage
						call	f1_calc_target_ratio

						TEST_PIN_OFF	TEST_DUTY_UPDATE

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

						movf	t1_integral_6p18+1,W
						movwf	pulse_width_L
						movf	t1_integral_6p18+2,W
						movwf	pulse_width_H

						bsf		s1_PIE1,ADIE
						movlw	(P_GAIN*PWM_PERIOD+128)/256-1
						movwf	s1_samples
__cemf_sampling_lp:
						movlb	0
						bcf		s0_PIR1,ADIF

						movlb	1
						movf	t1_target_ratio_6p18+0,W
						addwf	t1_proportional_6p18+0,F
						movf	t1_target_ratio_6p18+1,W
						addwfc	t1_proportional_6p18+1,F
						movf	t1_target_ratio_6p18+2,W
						addwfc	t1_proportional_6p18+2,F

						TEST_PIN_ON	TEST_DUTY_UPDATE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE

						movf	s1_ADRESL,W
						addwf	t1_current_cemf_6p18+1,F
						movf	s1_ADRESH,W
						addwfc	t1_current_cemf_6p18+2,F
						TEST_PIN_OFF	TEST_DUTY_UPDATE

						decfsz	s1_samples,F
						bra		__cemf_sampling_lp

						; wait for last conversion start
						btfss	s1_ADCON0,GO
						bra		$-1

						movlb	0
						; restart PWM generator after ADC sampling end
						bsf		s0_T2CON,TMR2ON
						TEST_PIN_ON	TEST_DUTY_UPDATE
						bcf		s0_PIR1,ADIF

						movlb	6
						; turn-OFF CCP3
						clrf	s6_CCP3CON

						movlb	8
						; turn-OFF Timer 3
						clrf	s8_T3CON

						movlb	1
						; wait for last A/D convertion completion
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s1_PIE1,ADIE				; 46

						movf	s1_ADRESL,W
						addwf	t1_current_cemf_6p18+1,F
						movf	s1_ADRESH,W
						addwfc	t1_current_cemf_6p18+2,F	; 4

						; turn-OFF ADC
						clrf	s1_ADCON0
						clrf	s1_ADACT					; 2

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
						movwf	pulse_width_H				; 10	87

f5_pwm_by_duty_ratio:	movlb	5
						; saturate calculation
						comf	pulse_width_H,W				; expect B'111111xx' 
						addlw	0x04
						subwfb	WREG,W						; W = -1 +C
						iorwf	pulse_width_L,F
						iorwf	pulse_width_H,F				; +6	93

						; pulse_width_H         pulse_width_L
						; 0,0,0,0,0,0,d9,d8     d7,d6,d5,d4,d3,d2,d1,d0
						movf	pulse_width_H,W
						movwf	s5_CCPR2H
						movf	pulse_width_L,W
						movwf	s5_CCPR2L
						return								; +6	99

						; integration
f1_closed_loop_integration:
						movlb	1
	if (TEST_TARGET == TEST_UART_CEMF)
					movf	t1_current_cemf_17p7+1,W
					movwf	INDF1
					movf	t1_current_cemf_17p7+2,W
					movwf	INDF1
	endif
	if (TEST_TARGET == TEST_UART_P)
					movf	t1_proportional_17p7+1,W
					movwf	INDF1
					movf	t1_proportional_17p7+2,W
					movwf	INDF1
	endif
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
	if (TEST_TARGET == TEST_UART_I)
					movf	t1_integral_6p18+1,W
					movwf	INDF1
					movf	t1_integral_6p18+2,W
					movwf	INDF1
	endif

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
						bsf		s2_CM1CON0,C1POL

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

						btfsc	notification_flag,BIT_POLLING
						decfsz	s1_breaking_in_time,F
						goto	f5_pwm_by_duty_ratio	; break-in, call and return
						; fall through

__calib_begin:
						movlb	5
						; set duty to 0%
						clrf	s5_CCPR2L
						clrf	s5_CCPR2H

						movlb	0
						movf	s0_T2PR,W
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

						movlb	5
						; set CCP to A/D conversion interval
						clrf	s5_CCP1CON
						movlw	LOW	 (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s5_CCPR1L
						movlw	HIGH (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s5_CCPR1H
						movlw	CCP1CON_init
						movwf	s5_CCP1CON

						movlb	13
						; halt half-bridge operation
						bcf		s13_CWG1AS0,REN
						bsf		s13_CWG1AS0,SHUTDOWN

						movlb	1
						; halt LED brightness controll
	if TEST_TARGET == TEST_NONE
						bsf		s1_TRISC,TRISC_LED_CATHODE
	endif

						; change WDT period
						movlw	WDTCON_calibration
						movwf	s1_WDTCON

						; set ADC to basic mode using CCP trigger
						movlw	ADACT_CCP1
						movwf	s1_ADACT
						movlw	ADCON0_MUX_MOTOR
						movwf	s1_ADCON0

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
						TEST_PIN_ON TEST_CALIBRATION
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
						subwfb	d1_cemf_mean_14p2+1,F		; 6		56/60

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
						rrf		d1_cemf_range_14p2+0,F		;  8

						; test for minimum
						movf	d1_cemf_min_14p2+0,W
						subwf	d1_cemf_mean_14p2+0,W
						movwf	d1_cemf_work_14p2+0
						movf	d1_cemf_min_14p2+1,W
						subwfb	d1_cemf_mean_14p2+1,W
						movwf	d1_cemf_work_14p2+1
						btfss	STATUS,C
						bra		__calib_lp_decreasing		;  9/ 8	34/33

						; test for maximum
						movf	d1_cemf_range_14p2+0,W
						subwf	d1_cemf_work_14p2+0,F
						movf	d1_cemf_range_14p2+1,W
						subwfb	d1_cemf_work_14p2+1,F
						btfss	STATUS,C
						bra		__calib_lp					;  / 6	 /39

__calib_lp_increasing:	; increasing in CEMF voltage		; cemf_max < cemf_mean
						TEST_PIN_OFF TEST_CALIBRATION
						btfss	s1_sampling_flag,1
						bra		__calib_lp_update
						bcf		s1_sampling_flag,1			; clear bit1 as maximum update

						movlw	0x04
						addwf	FSR0L,F
						btfsc	STATUS,C
						incf	s1_sampled_waves,F
						bra		__calib_lp_update				;  / 9	 /48

__calib_end:
						movlb	5
						; Turn-OFF periodic timer
						clrf	s5_CCP1CON
						movlb	0
						clrf	s0_T1CON

						movlb	1
						; Disable interrupts
						bcf		s1_PIE1,OSFIE
						bcf		s1_PIE4,CCP1IE

						; Turn-OFF ADC
						clrf	s1_ADACT
						clrf	s1_ADCON0

						movf	s1_sampled_waves,F
						btfsc	STATUS,Z
						call	set_factory_default		; call but never return

						; prepare FVR, ADC, and T3 to acquire VDD voltage
						call	f0_prepare_adc_and_fvr

						clrwdt

						movlb	1
						; Equivalent Time Sampling
						decf	FSR0L,F						; wrap-around
						; load the latest snap shot
						moviw	FSR0--
						movwf	motor_constant_HH
						moviw	FSR0--
						movwf	motor_constant_HL
						moviw	FSR0--
						movwf	motor_constant_LH
						movf	INDF0,W
						movwf	motor_constant_LL

						movlw	-4*CALIB_WAVES
						addwf	FSR0L,F						; wrap-around
						; get motor_constant samples by subtraction
						moviw	FSR0++
						subwf	motor_constant_LL,F
						moviw	FSR0++
						subwfb	motor_constant_LH,F
						moviw	FSR0++
						subwfb	motor_constant_HL,F
						moviw	FSR0++
						subwfb	motor_constant_HH,F

						clrf	q1_motor_constant_work+0
						clrf	q1_motor_constant_work+1
						clrf	q1_motor_constant_work+2
						clrf	q1_motor_constant_work+3

						movlw	LOW  CALIB_COEFFICIENT_8p8
						movwf	d1_cemf_work_14p2+0
						movlw	HIGH CALIB_COEFFICIENT_8p8
						movwf	d1_cemf_work_14p2+1

						movlw	0x08
						movwf	s1_sampled_waves
__calc_motor_const_lp1:	lsrf	d1_cemf_work_14p2+0,F
						btfss	STATUS,C
						bra		$+9
						movf	motor_constant_LL,W
						addwf	q1_motor_constant_work+0,F
						movf	motor_constant_LH,W
						addwfc	q1_motor_constant_work+1,F
						movf	motor_constant_HL,W
						addwfc	q1_motor_constant_work+2,F
						movf	motor_constant_HH,W
						addwfc	q1_motor_constant_work+3,F

						lsrf	q1_motor_constant_work+3,F
						rrf		q1_motor_constant_work+2,F
						rrf		q1_motor_constant_work+1,F
						rrf		q1_motor_constant_work+0,F

						call	f1_update_adc_result

						decfsz	s1_sampled_waves,F
						bra		__calc_motor_const_lp1

						movlw	0x08
						movwf	s1_sampled_waves
__calc_motor_const_lp2:	lsrf	d1_cemf_work_14p2+1,F
						btfss	STATUS,C
						bra		$+9
						movf	motor_constant_LL,W
						addwf	q1_motor_constant_work+0,F
						movf	motor_constant_LH,W
						addwfc	q1_motor_constant_work+1,F
						movf	motor_constant_HL,W
						addwfc	q1_motor_constant_work+2,F
						movf	motor_constant_HH,W
						addwfc	q1_motor_constant_work+3,F

						lslf	motor_constant_LL,F
						rlf		motor_constant_LH,F
						rlf		motor_constant_HL,F
						rlf		motor_constant_HH,F

						call	f1_update_adc_result

						decfsz	s1_sampled_waves,F
						bra		__calc_motor_const_lp2

						call	f1_finish_adc_and_fvr

						; 
						clrf	motor_constant_LL
						clrf	motor_constant_LH
						clrf	motor_constant_HL
						clrf	motor_constant_HH
						bsf		motor_constant_LL,1	; sentinel
				;		bra		__calib_div_begin

__calib_div_pos:		movf	d1_adres_FVR_m7p23+0,W
						subwf	q1_motor_constant_work+2,F
						movf	d1_adres_FVR_m7p23+1,W
						subwfb	q1_motor_constant_work+3,F
__calib_div_common:		rlf		motor_constant_LL,F
						rlf		motor_constant_LH,F
						rlf		motor_constant_HL,F
						rlf		motor_constant_HH,F
						btfsc	STATUS,C
						bra		__calib_div_end
__calib_div_begin:		lslf	q1_motor_constant_work+0,F
						rlf		q1_motor_constant_work+1,F
						rlf		q1_motor_constant_work+2,F
						rlf		q1_motor_constant_work+3,F
						btfsc	motor_constant_LL,0
						bra		__calib_div_pos
__calib_div_neg:		movf	d1_adres_FVR_m7p23+0,W
						addwf	q1_motor_constant_work+2,F
						movf	d1_adres_FVR_m7p23+1,W
						addwfc	q1_motor_constant_work+3,F
						bra		__calib_div_common

__calib_div_end:
						movlb	0
						; abort calibration when oscillator is faulty.
						btfsc	s0_PIR1,OSFIF
						reset

						call	update_parameter	; call but never return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Common entry point (cold boot, wake up from sleep, fall into sleep)						;
;		*		*		*		*		*		*		*		*		*		*		*		;
common_entry_point:
						movlb	18
						; speed-up HFINTOSC for backup.
						movlw	OSCFRQ_backup
						movwf	s18_OSCFRQ
						movlw	OSCCON1_backup
						movwf	s18_OSCCON1

						movlb	4
						; enable pull-up
						movlw	WPUA_init
						movwf	s4_WPUA
						movlw	WPUB_init
						movwf	s4_WPUB
						movlw	WPUC_init
						movwf	s4_WPUC

						movlb	17
						; analyze reset
						movf	s17_PCON0,W
						movwf	notification_flag
						movlw	PCON_init
						movwf	s17_PCON0

						TEST_PIN_INIT	s1_TRISC,TRISC_LED_CATHODE
						TEST_PIN_ON	TEST_CPU_USAGE

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

						movlb	8
						clrf	s8_T3CON

						movlb	7
						movlw	INLVLA_init
						movwf	s7_INLVLA
						movlw	INLVLB_init
						movwf	s7_INLVLB
						movlw	INLVLC_init
						movwf	s7_INLVLC

						movlb	3
						; enable input buffer on PORTA
						movlw	ANSELA_init
						movwf	s3_ANSELA

						movlb	2
						; set default state when the port is cofigured for generic use.
						movlw	LATA_init
						movwf	s2_LATA
						movlw	LATB_init
						movwf	s2_LATB
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

						movlb	1
						; hold cemf sense pin to stable state
						bcf		s1_TRISC,TRISC_CEMF_SENSE

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
						clrf	s4_WPUB			; All pullups on PORTB are disabled
						clrf	s4_WPUC			; All pullups on PORTC are disabled

						call	f1_save_lut_index

						; change WDT period
						movlw	WDTCON_sleep
						movwf	s1_WDTCON

						movlb	18
						; speed-down HFINTOSC for sleep
						movlw	OSCCON1_sleep
						movwf	s18_OSCCON1
						movlw	OSCFRQ_sleep
						movwf	s18_OSCFRQ

						movlb	3
						movlw	VREGCON_sleep
						movwf	s3_VREGCON
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

__short_sleep:			TEST_PIN_OFF	TEST_CPU_USAGE
						sleep				; method to wake-up is assertion of MCLR, POR or WDT
						TEST_PIN_ON	TEST_CPU_USAGE
						bra		__sleep_lp

__warm_reset:			movlb	17
						movlw	PCON_cold_boot	; merging into cold boot sequence 
						movwf	s17_PCON0
						reset					; due to unstable condition while sleeping

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Main routine																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
cold_boot:				clrf	notification_flag

						movlb	18
		if TEST_TARGET == TEST_NONE
						; disable unused modules
						movlw	PMD0_init
						movwf	s18_PMD0
						movlw	PMD1_init
						movwf	s18_PMD1
						movlw	PMD2_init
						movwf	s18_PMD2
						movlw	PMD3_init
						movwf	s18_PMD3
						movlw	PMD4_init
						movwf	s18_PMD4
						movlw	PMD5_init
						movwf	s18_PMD5
		endif

						; restore lut_index from DATA EEPROM
						call	f1_restore_lut_index

						movlb	2
						; disconnect power from motor
						movlw	CM1CON0_init
						movwf	s2_CM1CON0

						movlb	5
						; timer assignment
						movlw	CCPTMRS_init
						movwf	s5_CCPTMRS

						; setup PWM for motor control
						clrf	s5_CCPR2L
						clrf	s5_CCPR2H
						movlw	CCP2CON_init
						movwf	s5_CCP2CON

						movlb	13
						; halfbridge
						movlw	CWG1DAT_init
						movwf	s13_CWG1DAT
						movlw	CWG1DBR_init
						movwf	s13_CWG1DBR
						movlw	CWG1DBF_init
						movwf	s13_CWG1DBF
						movlw	CWG1CON0_init
						movwf	s13_CWG1CON0
						movlw	CWG1CON1_init
						movwf	s13_CWG1CON1
						movlw	CWG1AS0_init
						movwf	s13_CWG1AS0
						movlw	CWG1AS1_init
						movwf	s13_CWG1AS1

						movlb	12
						; timer assignment
						movlw	PWMTMRS_init
						movwf	s12_PWMTMRS

						; setup PWM for LED brightness control
						movlw	PWM6DCH_init
						movwf	s12_PWM6DCH
						clrf	s12_PWM6DCL
						movlw	PWM6CON_init
						movwf	s12_PWM6CON

						movlb	5
						movlw	ODCONA_init
						movwf	s5_ODCONA
						movlw	ODCONB_init
						movwf	s5_ODCONB
						movlw	ODCONC_init
						movwf	s5_ODCONC

						movlb	6
						movlw	SLRCONA_init
						movwf	s6_SLRCONA
						movlw	SLRCONB_init
						movwf	s6_SLRCONB
						movlw	SLRCONC_init
						movwf	s6_SLRCONC
		if (TEST_TARGET != TEST_NONE)
						bcf		s6_SLRCONC,SLRC4
		endif

						movlb	29
				;											;  1: VDD
				;		clrf	s29_RA5PPS					;  2: OSC1
				;		clrf	s29_RA4PPS					;  3: OSC2
				;											;  4: RA3/Vpp		power SW
				;		clrf	s29_RC5PPS					;  5: ANC5			LED A Sense
	if (TEST_TARGET == TEST_NONE)
						movlw	PPS_LED_CATHODE_init
						movwf	s29_RC4PPS					;  6: PWM6			LED K Drive
	endif
				;		movwf	s29_RC3PPS					;  7: ANC3			HB1 -> 1k ohm -> ANC3 -> 1nF -> HB2
						movlw	PPS_HB1_HS_init				;                          
						movwf	s29_RC6PPS					;  8: CWG1A 		CWG1A -> 47 ohm -> HB1 HS gate
						movwf	s29_RC7PPS					;  9: CWG1A 		CWG1A -> 47 ohm -> HB1 HS gate
						movwf	s29_RB7PPS					; 10: CWG1A			CWG1A -> 47 ohm -> HB1 HS gate
						movlw	PPS_HB1_LS_init				;                                     
						movwf	s29_RB6PPS					; 11: CWG1B			CWG1B -> 47 ohm -> HB1 LS gate
						movwf	s29_RB5PPS					; 12: CWG1B			CWG1B -> 47 ohm -> HB1 LS gate
						movlw	PPS_HB1_HS_init				;                          
						movwf	s29_RB4PPS					; 13: CWG1A 		CWG1A -> Open drain -> HB1 LS gate
						movwf	s29_RC2PPS					; 14: CWG1A 		CWG1A -> Open drain -> HB1 LS gate
				;		clrf	s29_RC1PPS					; 15: RC1			double speed SW
				;		clrf	s29_RC0PPS					; 16: RC0			stop SW
						movlw	PPS_HB2_LS_init
						movwf	s29_RA2PPS					; 17: CM1OUT		CM1OUT -> 47 ohm -> HB2 LS gate
				;		clrf	s29_RA1PPS					; 18: RA1/ICSPCLK	origin SW
				;		clrf	s29_RA0PPS					; 19: RA0/ICSPDAT	limit SW
				;											; 20: VSS

						movlb	3
						; enable input buffer
						movlw	ANSELB_init
						movwf	s3_ANSELB
						movlw	ANSELC_init
						movwf	s3_ANSELC
	if ((TEST_TARGET == TEST_UART_CEMF)||(TEST_TARGET == TEST_UART_P)||(TEST_TARGET == TEST_UART_I))
						bcf		s3_ANSELC,ANSELC4
	endif

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

						; setup CCP1 for ADC trigger of motor
						call	f5_load_default_period

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
						movlw	TRISB_active
						movwf	s1_TRISB
						movlw	TRISC_active
						movwf	s1_TRISC

						; interrupt enable
						bsf		s1_PIE1,OSFIE
						bsf		s1_PIE4,CCP1IE

						movlb	18
						; enable IDLE mode
						bsf		s18_CPUDOZE,IDLEN

						movlw	INTCON_active
						movwf	INTCON

						; wait for backup osc. ready
						movlw	OSCCON1_backup
						xorwf	s18_OSCCON2,W
						btfss	STATUS,Z
						bra		$-3

						; switch to external osc.
						movlw	OSCCON1_active
						movwf	s18_OSCCON1

__idle_loop:			; wait for interrupt
						bcf		INTCON,GIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bsf		INTCON,GIE
						bra		__idle_loop

f1_get_notification:	movlb	1
						; if reset button has released, MCU sleep
						btfsc	s1_sw_toggle,BIT_SW_RESET
						bra		$+3
						btfss	s1_sw_state,BIT_SW_RESET
						call	__prepare_for_sleep	; toggle == 0 and state == 0, call but never return

						; if origin flag is set, index is reset to lut_start
						btfsc	s1_sw_state,BIT_SW_ORIGIN	;  6
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
						btfsc	STATUS,C					;  5
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
						retlw	CTRL_MOTOR_1x	; set motor speed to 1x	; 9	62/54

__prepare_for_sleep:	; interrupt disable
						bcf		INTCON,GIE

						movlb	2
						; disconnect power from motor
						bcf		s2_CM1CON0,C1POL

						call	wait_1us
						TEST_PIN_OFF	TEST_CPU_USAGE
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
						movlw	0x70
						movwf	FSR0H

						; read motor parameter from EEPROM
						movlw	LOW dfm_motor_constant
						movwf	FSR0L

						moviw	0[FSR0]
						movwf	motor_constant_LL
						moviw	1[FSR0]
						movwf	motor_constant_LH
						moviw	2[FSR0]
						movwf	motor_constant_HL
						moviw	3[FSR0]
						movwf	motor_constant_HH

						; read lut address from EEPROM
						movlw	LOW dfm_lut_address
						movwf	FSR0L

						moviw	0[FSR0]
						movwf	lut_address_L
						moviw	1[FSR0]
						movwf	lut_address_H
						moviw	2[FSR0]
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

						movlb	17
						bsf		s17_NVMCON1,NVMREGS		; Access EEPROM, Configuration, user ID and device ID registers
						movlw	0xF0
						movwf	s17_NVMADRH
						movlw	LOW dfm_lut_address
						movwf	s17_NVMADRL

						; write lower byte of lut index
						movf	lut_address_L,W
						call	__write_data_eeprom

						; write upper byte of lut index
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
						movlb	17
						bsf		s17_NVMCON1,NVMREGS		; Access EEPROM, Configuration, user ID and device ID registers
						movlw	0xF0
						movwf	s17_NVMADRH
						movlw	LOW dfm_motor_constant
						movwf	s17_NVMADRL

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

;		WREG	data to be written
;		EEADR	address of data
;		BSR		17
__write_data_eeprom:	clrwdt
						bsf		s17_NVMCON1,RD
						xorwf	s17_NVMDATL,F
						btfsc	STATUS,Z
						bra		__skip_programming
						movwf	s17_NVMDATL

						TEST_PIN_ON	TEST_NVM

						bsf		s17_NVMCON1,WREN
						movlw	0x55				; Load 55h to get ready for unlock sequence
						movwf	s17_NVMCON2			; First step is to load 55h into NVMCON2
						movlw	0xAA				; Second step is to load AAh into W
						movwf	s17_NVMCON2			; Third step is to load AAh into NVMCON2
						bsf		s17_NVMCON1,WR		; Final step is to set WR bit
						bcf		s17_NVMCON1,WREN

						btfsc	s17_NVMCON1,WR
						bra		$-1

						TEST_PIN_OFF TEST_NVM

__skip_programming:		incf	s17_NVMADRL,F
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Code Flash																				;
;		*		*		*		*		*		*		*		*		*		*		*		;
f5_load_lut_entry:		movlb	1
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

						movlb	5
						; load sampling start time into CCPR1
						clrf	s5_CCP1CON
						movlw	0xFF
						addwf	cemf_sampling_period_L,W
						movwf	s5_CCPR1L
						movlw	0xFF
						addwfc	cemf_sampling_period_H,W
						movwf	s5_CCPR1H
						movlw	CCP1CON_init
						movwf	s5_CCP1CON
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
