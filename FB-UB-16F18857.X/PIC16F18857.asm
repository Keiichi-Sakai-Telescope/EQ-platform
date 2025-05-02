						#include		mpasmx.inc

						errorlevel -302
						radix	DEC

FCY						equ		6000000					; 6MHz
PWM_PERIOD				equ		250						; PWM frequency = FCY/PWM_PERIOD, min: 200, max: 256
OPEN_LOOP_PERIOD		equ		62500					; fixed settings
ADC_INTERVAL			equ		15

 __config	_CONFIG1,	_FEXTOSC_HS & _RSTOSC_HFINT1
 __config	_CONFIG2,	_MCLRE_OFF & _PPS1WAY_OFF & _BOREN_NSLEEP & _BORV_LO & _LPBOREN_OFF
 __config	_CONFIG3,	_WDTE_ON & _WDTCCS_LFINTOSC & _WDTCWS_WDTCWS_7 & _WDTCPS_WDTCPS_6 ; WDT 64ms in LFINTOSC
 __config	_CONFIG4,	_LVP_OFF

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
						;        PWM7  PWM6  CCP5  CCP4  CCP3  CCP2  CCP1
						; 16bit                T5    T3    T3    T1   *T1
						;  8bit    T6   *T6    T6    T4    T4   *T2    T2
CCPTMRS0_init			equ		(B'10'<<C4TSEL0)|(B'10'<<C3TSEL0)|(B'01'<<C2TSEL0)|(B'01'<<C1TSEL0)
CCPTMRS1_init			equ		                 (B'11'<<P7TSEL0)|(B'11'<<P6TSEL0)|(B'11'<<C5TSEL0)

						; Trigger for A/D conversion of CEMF
T1CON_init				equ		(T1_PRESCALER<<T1CKPS0)|(1<<TMR1ON)
T1CLK_init				equ		B'0001'							; FCY
CCP1CON_init			equ		(1<<CCP1EN)|(B'1011'<<CCP1MODE0)	; 0-1-0 pulse, clear on match

						; PWM generator
T2CON_init				equ		(B'000'<<T2CKPS0)|(1<<TMR2ON)	; FCY, pre 1:1, post 1:1
T2CLK_init				equ		B'0001'							; FCY
T2HLT_init				equ		(1<<PSYNC)|(1<<CKSYNC)|(B'00000'<<MODE0)
T2PR_init				equ		PWM_PERIOD - 1
CCP2CON_init			equ		(1<<CCP2EN)|(B'1111'<<CCP2MODE0)	; Right-aligned, PWM
CWG1CON0_init			equ		(1<<EN)|(B'100'<<MODE0)	; PWM HB
CWG1CON1_init			equ		(0<<POLB)|(1<<POLA)		; POLA: active L, POLB: active H
CWG1ISM_init			equ		B'0010'					; CCP2
CWG1AS0_init			equ		(1<<REN)|(B'10'<<LSBD0)|(B'11'<<LSAC0)	; A=H, B=L
CWG1AS1_init			equ		(1<<AS4E)				; by C1
CWG1DBR_init			equ		3						; delay time of rising is 3*Tosc = 125ns
CWG1DBF_init			equ		4						; delay time of falling is 4*Tosc = 167ns
PPS_HB1_HS_init			equ		0x05					; Rxy source is CWG1A
PPS_HB1_LS_init			equ		0x06					; Rxy source is CWG1B
PPS_HB2_LS_init			equ		0x12					; Rxy source is C1
CM1CON0_init			equ		(0<<C1ON)|(0<<C1POL)|(0<<C1HYS)
WPUB_CEMF_SENSE			equ		WPUB0
TRISB_CEMF_SENSE		equ		TRISB0
ANSELB_CEMF_SENSE		equ		ANSB0
PORTB_CEMF_SENSE		equ		RB0

						; Trigger for A/D conversion of FVR
FVR_SETTLING_TIME		set		25
T3CON_init				equ		(1<<TMR3ON)
T3CLK_init				equ		B'0001'					; FCY

						; Constant brightness LED driver
T6CON_init				equ		(B'011'<<T6CKPS0)|(1<<TMR6ON) ; FCY/8, pre 1:8, post 1:1
T6CLK_init				equ		B'0001'							; FCY
T6HLT_init				equ		(1<<PSYNC)|(1<<CKSYNC)|(B'00000'<<MODE0)
T6PR_init				equ		255
PWM6CON_init			equ		(1<<PWM6EN)|(1<<PWM6POL)
PWM6DCH_init			equ		6*(FCY/1000000)/8		;  minimum settling time 5.25us
PPS_LED_CATHODE_init	equ		0x0E					; Rxy source is PWM6
TRISA_LED_CATHODE		equ		TRISA1

; etc.
OSCFRQ_backup			equ		(B'0100'<<HFFRQ0)		; 12MHz
OSCFRQ_sleep			equ		(B'0000'<<HFFRQ0)		; 1MHz
OSCCON1_backup			equ		(B'001'<<NOSC0)			; HFINTOSC with 2x PLL, 1:1
OSCCON1_sleep			equ		(B'110'<<NOSC0)			; HFINTOSC, 1:1
OSCCON1_active			equ		(B'010'<<NOSC0)			; EXTOSC with 4x PLL, 1:1
WDTCON0_active			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON0_sleep			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON0_calibration		equ		(B'01010'<<WDTPS0)		; 1s nominal
WDTCON1_init			equ		(B'111'<<WINDOW0)
PCON_init				equ		(1<<NOT_BOR)|(1<<NOT_POR)|(1<<NOT_RI)|(1<<NOT_RMCLR)
PCON_cold_boot			equ		(1<<NOT_BOR)|(1<<NOT_RMCLR)
INTCON_sleep			equ		(1<<PEIE)
INTCON_active			equ		(1<<PEIE)|(1<<GIE)
VREGCON_active			equ		B'11'		; Main regulator in LP mode
VREGCON_sleep			equ		B'11'		; Main regulator in LP mode
SIDEREAL_TIME_0p16		equ		65536		; [0.5, 1.5)
SOLAR_TIME_0p16			equ		65715		; [0.5, 1.5)
LUNAR_TIME_0p16			equ		68019		; [0.5, 1.5)

PMD0_init				equ		(0<<SYSCMD)|(1<<FVRMD)|(1<<CRCMD)|(1<<SCANMD)|(0<<NVMMD)|(1<<CLKRMD)|(1<<IOCMD)
PMD1_init				equ		(1<<NCOMD)|(0<<TMR6MD)|(1<<TMR5MD)|(1<<TMR4MD)|(1<<TMR3MD)|(0<<TMR2MD)|(0<<TMR1MD)|(1<<TMR0MD)
PMD2_init				equ		(1<<DACMD)|(1<<ADCMD)|(1<<CMP2MD)|(0<<CMP1MD)|(1<<ZCDMD)
PMD3_init				equ		(1<<PWM7MD)|(0<<PWM6MD)|(1<<CCP5MD)|(1<<CCP4MD)|(1<<CCP3MD)|(0<<CCP2MD)|(0<<CCP1MD)
PMD4_init				equ		(1<<UART1MD)|(1<<MSSP2MD)|(1<<MSSP1MD)|(1<<CWG3MD)|(1<<CWG2MD)|(0<<CWG1MD)
PMD5_init				equ		(1<<SMT2MD)|(1<<SMT1MD)|(1<<CLC4MD)|(1<<CLC3MD)|(1<<CLC2MD)|(1<<CLC1MD)|(1<<DSMMD)

; ADC and FVR configurations
FVRCON_init				equ		B'11000001'		; CDAFVR = 00 for OFF, ADFVR = 01 for 1.024V
ADCON0_init				equ		(1<<ADON)|(1<<ADFRM0)	; ADON, flush right
ADCON1_init				equ		0				; no precharge, single conversion
ADCON2_BURST_AVERAGE	equ		(1<<ADCRS0)|(1<<ADACLR)|(B'011'<<ADMD0)		; Burst Average mode
ADCON2_BASIC			equ		(1<<ADCRS0)|(1<<ADACLR)|(B'000'<<ADMD0)		; Basic mode
ADCON3_init				equ		(B'111'<<ADTMD0); Interrupt regardless of threshold test results
ADCLK_init				equ		(FCY/500000)-1
ADREF_VDD_RATIOMETRIC	equ		(0<<ADPREF0)	; Vref+=VDD
ADPCH_FVR				equ		(B'111111'<<ADPCH0)		; FVR(FVR output)
ADPCH_MOTOR				equ		(B'001000'<<ADPCH0)		; ANB0(motor, RB0 pin)
ADPCH_LED				equ		(B'000000'<<ADPCH0)		; ANA0(LED, RA0 pin)
ADPRE_init				equ		0				; no precharge time
ADACQ_init				equ		(ADC_INTERVAL-11)*(FCY/250000)
ADCAP_init				equ		B'11111'		; 31pF
ADRPT_FVR				equ		8				; number of repeat
ADACT_CCP1				equ		0x0B			; CCP1 match
ADACT_T3TMR				equ		0x05			; T3TMR overflow
ADACT_LED				equ		0x08			; T6TMR postscaled

; I/O port configurations
LATA_init				equ		B'00000010'		; initial level:     ,    ,    ,    ,    ,    , RA1,    
WPUA_init				equ		B'00110000'		; pull-up:           ,    , RA5, RA4,    ,    ,    ,    
TRISA_active			equ		B'11111001'		; digital output:    ,    ,    ,    ,    , RA2, RA1,    
ANSELA_init				equ		B'11001111'		; digital input:     ,    , RA5, RA4,    ,    ,    ,    
SLRCONA_init			equ		B'11111111'		; high-speed:        ,    ,    ,    ,    ,    ,    ,    
ODCONA_init				equ		B'00000010'		; open drain:        ,    ,    ,    ,    ,    , RA1,    
INLVLA_init				equ		B'11111111'		; ST input:       RA7, RA6, RA5, RA4, RA3, RA2, RA1, RA0

LATB_init				equ		B'00000000'		; initial level:     ,    ,         ,    ,    ,    ,    
WPUB_init				equ		B'11000000'		; pull-up:        RB7, RB6,         ,    ,    ,    ,    
TRISB_active			equ		B'11000001'		; digital output:    ,    , RB5, RB4, RB3, RB2, RB1,    
ANSELB_init				equ		B'00111111'		; digital input:  RB7, RB6,    ,    ,    ,    ,    ,    
SLRCONB_init			equ		B'11001111'		; high-speed:        ,    , RB5, RB4,    ,    ,    ,    
ODCONB_init				equ		B'00110000'		; open drain:        ,    , RB5, RB4,    ,    ,    ,    
INLVLB_init				equ		B'11111110'		; ST input:       RB7, RB6, RB5, RB4, RB3, RB2, RB1

LATC_init				equ		B'00111111'		; initial level:            RC5, RC4, RC3, RC2, RC1, RC0
WPUC_init				equ		B'00000000'		; pull-up:           ,    ,    ,    ,    ,    ,    ,    
TRISC_active			equ		B'00000000'		; digital output: RC7, RC6, RC5, RC4, RC3, RC2, RC1, RC0
ANSELC_init				equ		B'11111111'		; digital input:     ,    ,    ,    ,    ,    ,    ,    
SLRCONC_init			equ		B'11111111'		; high-speed:        ,    ,    ,    ,    ,    ,    ,    
ODCONC_init				equ		B'00000000'		; open drain:        ,    ,    ,    ,    ,    ,    ,    
INLVLC_init				equ		B'11111111'		; ST input:       RC7, RC6, RC5, RC4, RC3, RC2, RC1, RC0

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
BIT_SW_RESET			equ		3		; RE3	power SW			alternate
BIT_SW_STOP				equ		4		; RA4	stop SW				level
BIT_SW_DOUBLE_SPEED		equ		5		; RA5	double speed SW		level
BIT_SW_ORIGIN			equ		6		; RB6	origin SW			level
BIT_SW_LIMIT			equ		7		; RB7	limit SW			edge (pressing)

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
s0_PORTA            EQU  PORTA
s0_PORTB            EQU  PORTB
s0_PORTC            EQU  PORTC
s0_PORTE            EQU  PORTE
s0_TRISA            EQU  TRISA
s0_TRISB            EQU  TRISB
s0_TRISC            EQU  TRISC
s0_LATA             EQU  LATA
s0_LATB             EQU  LATB
s0_LATC             EQU  LATC
s0_TMR0             EQU  TMR0
s0_TMR0L            EQU  TMR0L
s0_PR0              EQU  PR0
s0_TMR0H            EQU  TMR0H
s0_T0CON0           EQU  T0CON0
s0_T0CON1           EQU  T0CON1

; general purpose RAM (Bank 0) allocation
bank0_ram				udata	0x20

;-----Bank1------------------
s1_ADRESL           EQU  ADRESL
s1_ADRESH           EQU  ADRESH
s1_ADPREVL          EQU  ADPREVL
s1_ADPREVH          EQU  ADPREVH
s1_ADACCL           EQU  ADACCL
s1_ADACCH           EQU  ADACCH
s1_ADCON0           EQU  ADCON0
s1_ADCON1           EQU  ADCON1
s1_ADCON2           EQU  ADCON2
s1_ADCON3           EQU  ADCON3
s1_ADSTAT           EQU  ADSTAT
s1_ADCLK            EQU  ADCLK
s1_ADACT            EQU  ADACT
s1_ADREF            EQU  ADREF
s1_ADCAP            EQU  ADCAP
s1_ADPRE            EQU  ADPRE
s1_ADACQ            EQU  ADACQ
s1_ADPCH            EQU  ADPCH

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
s2_ADCNT            EQU  ADCNT
s2_ADCNT            EQU  ADCNT
s2_ADRPT            EQU  ADRPT
s2_ADLTHL           EQU  ADLTHL
s2_ADLTHH           EQU  ADLTHH
s2_ADUTHL           EQU  ADUTHL
s2_ADUTHH           EQU  ADUTHH
s2_ADSTPTL          EQU  ADSTPTL
s2_ADSTPTH          EQU  ADSTPTH
s2_ADFLTRL          EQU  ADFLTRL
s2_ADFLTRH          EQU  ADFLTRH
s2_ADERRL           EQU  ADERRL
s2_ADERRH           EQU  ADERRH
s2_RC1REG           EQU  RC1REG
s2_TX1REG           EQU  TX1REG
s2_SP1BRGL          EQU  SP1BRGL
s2_SP1BRGH          EQU  SP1BRGH
s2_RC1STA           EQU  RC1STA
s2_TX1STA           EQU  TX1STA
s2_BAUD1CON         EQU  BAUD1CON

;-----Bank3------------------
s3_SSP1BUF          EQU  SSP1BUF
s3_SSP1ADD          EQU  SSP1ADD
s3_SSP1MSK          EQU  SSP1MSK
s3_SSP1STAT         EQU  SSP1STAT
s3_SSP1CON1         EQU  SSP1CON1
s3_SSP1CON2         EQU  SSP1CON2
s3_SSP1CON3         EQU  SSP1CON3
s3_SSP2BUF          EQU  SSP2BUF
s3_SSP2ADD          EQU  SSP2ADD
s3_SSP2MSK          EQU  SSP2MSK
s3_SSP2STAT         EQU  SSP2STAT
s3_SSP2CON1         EQU  SSP2CON1
s3_SSP2CON2         EQU  SSP2CON2
s3_SSP2CON3         EQU  SSP2CON3

;-----Bank4------------------
s4_T1TMRL           EQU  TMR1L
s4_T1TMRH           EQU  TMR1H
s4_T1CON            EQU  T1CON
s4_T1GCON           EQU  T1GCON
s4_T1GATE           EQU  T1GATE
s4_T1CLK            EQU  T1CLK
s4_T3TMRL           EQU  TMR3L
s4_T3TMRH           EQU  TMR3H
s4_T3CON            EQU  T3CON
s4_T3GCON           EQU  T3GCON
s4_T3GATE           EQU  T3GATE
s4_T3CLK            EQU  T3CLK
s4_T5TMRL           EQU  TMR5L
s4_T5TMRH           EQU  TMR5H
s4_T5CON            EQU  T5CON
s4_T5GCON           EQU  T5GCON
s4_T5GATE           EQU  T5GATE
s4_T5CLK            EQU  T5CLK
s4_CCPTMRS0         EQU  CCPTMRS0
s4_CCPTMRS1         EQU  CCPTMRS1

;-----Bank5------------------
s5_T2TMR            EQU  TMR2
s5_T2PR             EQU  PR2
s5_T2CON            EQU  T2CON
s5_T2HLT            EQU  T2HLT
s5_T2CLK            EQU  T2CLK
s5_T2RST            EQU  T2RST
s5_T4TMR            EQU  TMR4
s5_T4PR             EQU  PR4
s5_T4CON            EQU  T4CON
s5_T4HLT            EQU  T4HLT
s5_T4CLK            EQU  T4CLK
s5_T4RST            EQU  T4RST
s5_T6TMR            EQU  TMR6
s5_T6PR             EQU  PR6
s5_T6CON            EQU  T6CON
s5_T6HLT            EQU  T6HLT
s5_T6CLK            EQU  T6CLK
s5_T6RST            EQU  T6RST

;-----Bank6------------------
s6_CCPR1L           EQU  CCPR1L
s6_CCPR1H           EQU  CCPR1H
s6_CCP1CON          EQU  CCP1CON
s6_CCP1CAP          EQU  CCP1CAP
s6_CCPR2L           EQU  CCPR2L
s6_CCPR2H           EQU  CCPR2H
s6_CCP2CON          EQU  CCP2CON
s6_CCP2CAP          EQU  CCP2CAP
s6_CCPR3L           EQU  CCPR3L
s6_CCPR3H           EQU  CCPR3H
s6_CCP3CON          EQU  CCP3CON
s6_CCP3CAP          EQU  CCP3CAP
s6_CCPR4L           EQU  CCPR4L
s6_CCPR4H           EQU  CCPR4H
s6_CCP4CON          EQU  CCP4CON
s6_CCP4CAP          EQU  CCP4CAP
s6_CCPR5L           EQU  CCPR5L
s6_CCPR5H           EQU  CCPR5H
s6_CCP5CON          EQU  CCP5CON
s6_CCP5CAP          EQU  CCP5CAP

;-----Bank7------------------
s7_PWM6DCL          EQU  PWM6DCL
s7_PWM6DCH          EQU  PWM6DCH
s7_PWM6CON          EQU  PWM6CON
s7_PWM7DCL          EQU  PWM7DCL
s7_PWM7DCH          EQU  PWM7DCH
s7_PWM7CON          EQU  PWM7CON

;-----Bank8------------------
s8_SCANLADRL        EQU  SCANLADRL
s8_SCANLADRH        EQU  SCANLADRH
s8_SCANHADRL        EQU  SCANHADRL
s8_SCANHADRH        EQU  SCANHADRH
s8_SCANCON0         EQU  SCANCON0
s8_SCANTRIG         EQU  SCANTRIG
s8_CRCDATL          EQU  CRCDATL
s8_CRCDATH          EQU  CRCDATH
s8_CRCACCL          EQU  CRCACCL
s8_CRCACCH          EQU  CRCACCH
s8_CRCSHIFTL        EQU  CRCSHIFTL
s8_CRCSHIFTH        EQU  CRCSHIFTH
s8_CRCXORL          EQU  CRCXORL
s8_CRCXORH          EQU  CRCXORH
s8_CRCCON0          EQU  CRCCON0
s8_CRCCON1          EQU  CRCCON1

;-----Bank9------------------
s9_SMT1TMRL         EQU  SMT1TMRL
s9_SMT1TMRH         EQU  SMT1TMRH
s9_SMT1TMRU         EQU  SMT1TMRU
s9_SMT1CPRL         EQU  SMT1CPRL
s9_SMT1CPRH         EQU  SMT1CPRH
s9_SMT1CPRU         EQU  SMT1CPRU
s9_SMT1CPWL         EQU  SMT1CPWL
s9_SMT1CPWH         EQU  SMT1CPWH
s9_SMT1CPWU         EQU  SMT1CPWU
s9_SMT1PRL          EQU  SMT1PRL
s9_SMT1PRH          EQU  SMT1PRH
s9_SMT1PRU          EQU  SMT1PRU
s9_SMT1CON0         EQU  SMT1CON0
s9_SMT1CON1         EQU  SMT1CON1
s9_SMT1STAT         EQU  SMT1STAT
s9_SMT1CLK          EQU  SMT1CLK
s9_SMT1SIG          EQU  SMT1SIG
s9_SMT1WIN          EQU  SMT1WIN

;-----Bank10------------------
s10_SMT2TMRL         EQU  SMT2TMRL
s10_SMT2TMRH         EQU  SMT2TMRH
s10_SMT2TMRU         EQU  SMT2TMRU
s10_SMT2CPRL         EQU  SMT2CPRL
s10_SMT2CPRH         EQU  SMT2CPRH
s10_SMT2CPRU         EQU  SMT2CPRU
s10_SMT2CPWL         EQU  SMT2CPWL
s10_SMT2CPWH         EQU  SMT2CPWH
s10_SMT2CPWU         EQU  SMT2CPWU
s10_SMT2PRL          EQU  SMT2PRL
s10_SMT2PRH          EQU  SMT2PRH
s10_SMT2PRU          EQU  SMT2PRU
s10_SMT2CON0         EQU  SMT2CON0
s10_SMT2CON1         EQU  SMT2CON1
s10_SMT2STAT         EQU  SMT2STAT
s10_SMT2CLK          EQU  SMT2CLK
s10_SMT2SIG          EQU  SMT2SIG
s10_SMT2WIN          EQU  SMT2WIN

;-----Bank11------------------
s11_NCO1ACCL         EQU  NCO1ACCL
s11_NCO1ACCH         EQU  NCO1ACCH
s11_NCO1ACCU         EQU  NCO1ACCU
s11_NCO1INCL         EQU  NCO1INCL
s11_NCO1INCH         EQU  NCO1INCH
s11_NCO1INCU         EQU  NCO1INCU
s11_NCO1CON          EQU  NCO1CON
s11_NCO1CLK          EQU  NCO1CLK

;-----Bank12------------------
s12_CWG1CLKCON       EQU  CWG1CLKCON
s12_CWG1ISM          EQU  CWG1ISM
s12_CWG1DBR          EQU  CWG1DBR
s12_CWG1DBF          EQU  CWG1DBF
s12_CWG1CON0         EQU  CWG1CON0
s12_CWG1CON1         EQU  CWG1CON1
s12_CWG1AS0          EQU  CWG1AS0
s12_CWG1AS1          EQU  CWG1AS1
s12_CWG1STR          EQU  CWG1STR

s12_CWG2CLKCON       EQU  CWG2CLKCON
s12_CWG2ISM          EQU  CWG2ISM
s12_CWG2DBR          EQU  CWG2DBR
s12_CWG2DBF          EQU  CWG2DBF
s12_CWG2CON0         EQU  CWG2CON0
s12_CWG2CON1         EQU  CWG2CON1
s12_CWG2AS0          EQU  CWG2AS0
s12_CWG2AS1          EQU  CWG2AS1
s12_CWG2STR          EQU  CWG2STR

;-----Bank13------------------
s13_CWG3CLKCON       EQU  CWG3CLKCON
s13_CWG3ISM          EQU  CWG3ISM
s13_CWG3DBR          EQU  CWG3DBR
s13_CWG3DBF          EQU  CWG3DBF
s13_CWG3CON0         EQU  CWG3CON0
s13_CWG3CON1         EQU  CWG3CON1
s13_CWG3AS0          EQU  CWG3AS0
s13_CWG3AS1          EQU  CWG3AS1
s13_CWG3STR          EQU  CWG3STR

;-----Bank14------------------
s14_PIR0             EQU  PIR0
s14_PIR1             EQU  PIR1
s14_PIR2             EQU  PIR2
s14_PIR3             EQU  PIR3
s14_PIR4             EQU  PIR4
s14_PIR5             EQU  PIR5
s14_PIR6             EQU  PIR6
s14_PIR7             EQU  PIR7
s14_PIR8             EQU  PIR8

s14_PIE0             EQU  PIE0
s14_PIE1             EQU  PIE1
s14_PIE2             EQU  PIE2
s14_PIE3             EQU  PIE3
s14_PIE4             EQU  PIE4
s14_PIE5             EQU  PIE5
s14_PIE6             EQU  PIE6
s14_PIE7             EQU  PIE7
s14_PIE8             EQU  PIE8

;-----Bank15------------------
s15_PMD0             EQU  PMD0
s15_PMD1             EQU  PMD1
s15_PMD2             EQU  PMD2
s15_PMD3             EQU  PMD3
s15_PMD4             EQU  PMD4
s15_PMD5             EQU  PMD5

;-----Bank16------------------
s16_WDTCON0          EQU  WDTCON0
s16_WDTCON1          EQU  WDTCON1
s16_WDTPSL           EQU  WDTPSL
s16_WDTPSH           EQU  WDTPSH
s16_WDTTMR           EQU  WDTTMR

s16_BORCON           EQU  BORCON
s16_VREGCON          EQU  VREGCON
s16_PCON0            EQU  PCON0
s16_CCDCON           EQU  CCDCON

s16_NVMADRL          EQU  NVMADRL
s16_NVMADRH          EQU  NVMADRH
s16_NVMDATL          EQU  NVMDATL
s16_NVMDATH          EQU  NVMDATH
s16_NVMCON1          EQU  NVMCON1
s16_NVMCON2          EQU  NVMCON2

;-----Bank17------------------
s17_CPUDOZE          EQU  CPUDOZE
s17_OSCCON1          EQU  OSCCON1
s17_OSCCON2          EQU  OSCCON2
s17_OSCCON3          EQU  OSCCON3
s17_OSCSTAT          EQU  OSCSTAT
s17_OSCEN            EQU  OSCEN
s17_OSCTUNE          EQU  OSCTUNE
s17_OSCFRQ           EQU  OSCFRQ

s17_CLKRCON          EQU  CLKRCON
s17_CLKRCLK          EQU  CLKRCLK
s17_MDCON0           EQU  MDCON0
s17_MDCON1           EQU  MDCON1
s17_MDSRC            EQU  MDSRC
s17_MDCARL           EQU  MDCARL
s17_MDCARH           EQU  MDCARH

;-----Bank18------------------
s18_FVRCON           EQU  FVRCON
s18_DAC1CON0         EQU  DAC1CON0
s18_DAC1CON1         EQU  DAC1CON1
s18_ZCDCON           EQU  ZCDCON

;-----Bank19------------------
s19_CMOUT            EQU  CMOUT
s19_CM1CON0          EQU  CM1CON0
s19_CM1CON1          EQU  CM1CON1
s19_CM1NSEL          EQU  CM1NSEL
s19_CM1PSEL          EQU  CM1PSEL
s19_CM2CON0          EQU  CM2CON0
s19_CM2CON1          EQU  CM2CON1
s19_CM2NSEL          EQU  CM2NSEL
s19_CM2PSEL          EQU  CM2PSEL

;-----Bank60------------------
s60_CLCDATA          EQU  CLCDATA
s60_CLC1CON          EQU  CLC1CON
s60_CLC1POL          EQU  CLC1POL
s60_CLC1SEL0         EQU  CLC1SEL0
s60_CLC1SEL1         EQU  CLC1SEL1
s60_CLC1SEL2         EQU  CLC1SEL2
s60_CLC1SEL3         EQU  CLC1SEL3
s60_CLC1GLS0         EQU  CLC1GLS0
s60_CLC1GLS1         EQU  CLC1GLS1
s60_CLC1GLS2         EQU  CLC1GLS2
s60_CLC1GLS3         EQU  CLC1GLS3
s60_CLC2CON          EQU  CLC2CON
s60_CLC2POL          EQU  CLC2POL
s60_CLC2SEL0         EQU  CLC2SEL0
s60_CLC2SEL1         EQU  CLC2SEL1
s60_CLC2SEL2         EQU  CLC2SEL2
s60_CLC2SEL3         EQU  CLC2SEL3
s60_CLC2GLS0         EQU  CLC2GLS0
s60_CLC2GLS1         EQU  CLC2GLS1
s60_CLC2GLS2         EQU  CLC2GLS2
s60_CLC2GLS3         EQU  CLC2GLS3
s60_CLC3CON          EQU  CLC3CON
s60_CLC3POL          EQU  CLC3POL
s60_CLC3SEL0         EQU  CLC3SEL0
s60_CLC3SEL1         EQU  CLC3SEL1
s60_CLC3SEL2         EQU  CLC3SEL2
s60_CLC3SEL3         EQU  CLC3SEL3
s60_CLC3GLS0         EQU  CLC3GLS0
s60_CLC3GLS1         EQU  CLC3GLS1
s60_CLC3GLS2         EQU  CLC3GLS2
s60_CLC3GLS3         EQU  CLC3GLS3
s60_CLC4CON          EQU  CLC4CON
s60_CLC4POL          EQU  CLC4POL
s60_CLC4SEL0         EQU  CLC4SEL0
s60_CLC4SEL1         EQU  CLC4SEL1
s60_CLC4SEL2         EQU  CLC4SEL2
s60_CLC4SEL3         EQU  CLC4SEL3
s60_CLC4GLS0         EQU  CLC4GLS0
s60_CLC4GLS1         EQU  CLC4GLS1
s60_CLC4GLS2         EQU  CLC4GLS2
s60_CLC4GLS3         EQU  CLC4GLS3

;-----Bank61------------------
s61_PPSLOCK          EQU  PPSLOCK
s61_INTPPS           EQU  INTPPS
s61_T0CKIPPS         EQU  T0CKIPPS
s61_T1CKIPPS         EQU  T1CKIPPS
s61_T1GPPS           EQU  T1GPPS
s61_T3CKIPPS         EQU  T3CKIPPS
s61_T3GPPS           EQU  T3GPPS
s61_T5CKIPPS         EQU  T5CKIPPS
s61_T5GPPS           EQU  T5GPPS
s61_T2AINPPS         EQU  T2AINPPS
s61_T4AINPPS         EQU  T4AINPPS
s61_T6AINPPS         EQU  T6AINPPS
s61_CCP1PPS          EQU  CCP1PPS
s61_CCP2PPS          EQU  CCP2PPS
s61_CCP3PPS          EQU  CCP3PPS
s61_CCP4PPS          EQU  CCP4PPS
s61_CCP5PPS          EQU  CCP5PPS
s61_SMT1WINPPS       EQU  SMT1WINPPS
s61_SMT1SIGPPS       EQU  SMT1SIGPPS
s61_SMT2WINPPS       EQU  SMT2WINPPS
s61_SMT2SIGPPS       EQU  SMT2SIGPPS
s61_CWG1PPS          EQU  CWG1PPS
s61_CWG2PPS          EQU  CWG2PPS
s61_CWG3PPS          EQU  CWG3PPS
s61_MDCARLPPS        EQU  MDCARLPPS
s61_MDCARHPPS        EQU  MDCARHPPS
s61_MDSRCPPS         EQU  MDSRCPPS
s61_CLCIN0PPS        EQU  CLCIN0PPS
s61_CLCIN1PPS        EQU  CLCIN1PPS
s61_CLCIN2PPS        EQU  CLCIN2PPS
s61_CLCIN3PPS        EQU  CLCIN3PPS
s61_ADCACTPPS        EQU  ADCACTPPS
s61_SSP1CLKPPS       EQU  SSP1CLKPPS
s61_SSP1DATPPS       EQU  SSP1DATPPS
s61_SSP1SSPPS        EQU  SSP1SSPPS
s61_SSP2CLKPPS       EQU  SSP2CLKPPS
s61_SSP2DATPPS       EQU  SSP2DATPPS
s61_SSP2SSPPS        EQU  SSP2SSPPS
s61_RXPPS            EQU  RXPPS
s61_TXPPS            EQU  TXPPS

;-----Bank62------------------
s62_RA0PPS           EQU  RA0PPS
s62_RA1PPS           EQU  RA1PPS
s62_RA2PPS           EQU  RA2PPS
s62_RA3PPS           EQU  RA3PPS
s62_RA4PPS           EQU  RA4PPS
s62_RA5PPS           EQU  RA5PPS
s62_RA6PPS           EQU  RA6PPS
s62_RA7PPS           EQU  RA7PPS
s62_RB0PPS           EQU  RB0PPS
s62_RB1PPS           EQU  RB1PPS
s62_RB2PPS           EQU  RB2PPS
s62_RB3PPS           EQU  RB3PPS
s62_RB4PPS           EQU  RB4PPS
s62_RB5PPS           EQU  RB5PPS
s62_RB6PPS           EQU  RB6PPS
s62_RB7PPS           EQU  RB7PPS
s62_RC0PPS           EQU  RC0PPS
s62_RC1PPS           EQU  RC1PPS
s62_RC2PPS           EQU  RC2PPS
s62_RC3PPS           EQU  RC3PPS
s62_RC4PPS           EQU  RC4PPS
s62_RC5PPS           EQU  RC5PPS
s62_RC6PPS           EQU  RC6PPS
s62_RC7PPS           EQU  RC7PPS

s62_ANSELA           EQU  ANSELA
s62_WPUA             EQU  WPUA
s62_ODCONA           EQU  ODCONA
s62_SLRCONA          EQU  SLRCONA
s62_INLVLA           EQU  INLVLA
s62_IOCAP            EQU  IOCAP
s62_IOCAN            EQU  IOCAN
s62_IOCAF            EQU  IOCAF

s62_ANSELB           EQU  ANSELB
s62_WPUB             EQU  WPUB
s62_ODCONB           EQU  ODCONB
s62_SLRCONB          EQU  SLRCONB
s62_INLVLB           EQU  INLVLB
s62_IOCBP            EQU  IOCBP
s62_IOCBN            EQU  IOCBN
s62_IOCBF            EQU  IOCBF

s62_ANSELC           EQU  ANSELC
s62_WPUC             EQU  WPUC
s62_ODCONC           EQU  ODCONC
s62_SLRCONC          EQU  SLRCONC
s62_INLVLC           EQU  INLVLC
s62_IOCCP            EQU  IOCCP
s62_IOCCN            EQU  IOCCN
s62_IOCCF            EQU  IOCCF

s62_WPUE             EQU  WPUE
s62_INLVLE           EQU  INLVLE
s62_IOCEP            EQU  IOCEP
s62_IOCEN            EQU  IOCEN
s62_IOCEF            EQU  IOCEF

;-----Bank63------------------
s63_STATUS_SHAD      EQU  STATUS_SHAD
s63_WREG_SHAD        EQU  WREG_SHAD
s63_BSR_SHAD         EQU  BSR_SHAD
s63_PCLATH_SHAD      EQU  PCLATH_SHAD
s63_FSR0L_SHAD       EQU  FSR0L_SHAD
s63_FSR0H_SHAD       EQU  FSR0H_SHAD
s63_FSR1L_SHAD       EQU  FSR1L_SHAD
s63_FSR1H_SHAD       EQU  FSR1H_SHAD
s63_STKPTR           EQU  STKPTR
s63_TOSL             EQU  TOSL
s63_TOSH             EQU  TOSH

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Reset vector																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
reset_vector			code	0x000
						movlb	16
						movlw	VREGCON_active
						movwf	s16_VREGCON
						goto	common_entry_point

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Interrupt Routine																		;
;		*		*		*		*		*		*		*		*		*		*		*		;

; interrupt handler is consist of pulse generator and voltage sampler.
;		the pulse generator is driven by T2 cooperatively with CCP2.
;		the voltage sampler is driven by T1 cooperatively with CCP1.
interrupt_handler		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						movlb	14
						btfsc	s14_PIR1,OSFIF
						bra		__intr_osc_fail
						btfsc	s14_PIR6,CCP1IF
						bra		__intr_motor
						reset

__intr_osc_fail:		; clear fail-safe clock operation flag
						bcf		s14_PIR1,OSFIF

						call 	f6_pwm_pause

						movlb	17
						; switch to backup osc.
						movlw	OSCFRQ_backup
						movwf	s17_OSCFRQ
						movlw	OSCCON1_backup
						movwf	s17_OSCCON1
						retfie

__intr_motor:
						bcf		s14_PIR6,CCP1IF

						movlb	15
						; 3.6V,  	6-Speed HE,	no load
						;  900rpm	  15mA,		2.08mA
						; 1800rpm	  24mA,		2.13mA
						;    0rpm	2014uA
						;    OFF	   2uA
						bcf		s15_PMD0,FVRMD
						bcf		s15_PMD1,TMR3MD
						bcf		s15_PMD2,ADCMD

						; prepare FVR, ADC, and T3 to acquire VDD voltage
						movlw	ADRPT_FVR
						call	f1_prepare_adc_and_fvr

						; read keys and remove chattering
						call	f1_read_keys			; 36

						call	f1_get_notification		; 64
						xorwf	notification_flag,W
						movwf	s1_notification_change
						xorwf	notification_flag,F

						; maintain 4Hz flag
						call	f1_maintain_4hz			; 21

						; set default period into sampling_interval
						call	f6_load_default_period	; 16

						; wait for completion of A/D convertion
						call	f1_load_adc_fvr_result
						TEST_PIN_ON	TEST_DUTY_UPDATE

						btfsc	s1_notification_change,BIT_OPEN_LOOP_CTRL
						call	f1_init_open_loop_ctrl

						btfsc	s1_notification_change,BIT_CLOSED_LOOP_CTRL
						call	f1_init_closed_loop_ctrl

						btfsc	notification_flag,BIT_OUT_OF_CTRL
						call	f6_pwm_pause

						btfsc	notification_flag,BIT_OPEN_LOOP_CTRL
						call	f6_open_loop_control

						btfsc	notification_flag,BIT_CLOSED_LOOP_CTRL
						call	f6_closed_loop_control

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

						movlb	17
	if 0
						movlw	OSCCON1_active
						xorwf	s17_OSCCON2,W
						btfss	STATUS,Z
						bra		__led_turn_off
	else
						btfss	s17_OSCSTAT,EXTOR
						bra		__led_turn_off			; External OSC is not working
						btfsc	s17_OSCSTAT,HFOR
						bra		__led_turn_off			; HFINTOSC is working
						btfsc	s17_OSCSTAT,MFOR
						bra		__led_turn_off			; MFINTOSC is working
						btfss	s17_OSCSTAT,LFOR
						bra		__led_turn_off			; LFINTOSC is not working
						btfsc	s17_OSCSTAT,SOR
						bra		__led_turn_off			; Secondary OSC is working
						btfsc	s17_OSCSTAT,ADOR
						bra		__led_turn_off			; ADOSC is working
						btfss	s17_OSCSTAT,PLLR
						bra		__led_turn_off			; PLL is not working
	endif

__led_turn_on:
						movlb	0
						; prepare for LED brightness control, set MUX to LED
						bcf		s0_TRISA,TRISA_LED_CATHODE		; LED is controlled by PWM

						movlb	1
						movlw	ADACT_LED
						movwf	s1_ADACT
						movlw	ADCON2_BASIC
						movwf	s1_ADCON2
						movlw	ADPCH_LED
						movwf	s1_ADPCH
						movlw	ADCON0_init
						movwf	s1_ADCON0

						movlb	1
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

						movlb	14
						bcf		s14_PIR1,ADIF
						bsf		s14_PIE1,ADIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADIE

						movlb	1
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
__led_div_common:		btfsc	STATUS,C				; output is positive logic
						bsf		pulse_width_L,5
						lslf	pulse_width_L,F
						rlf		pulse_width_H,F
						btfsc	STATUS,C
						bra		__led_div_end
						lslf	t1_adres_FVR_work+0,F
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
						movlb	7
						movf	pulse_width_L,W
						movwf	s7_PWM6DCL
						movf	pulse_width_H,W
						movwf	s7_PWM6DCH
						bra		__led_ctrl_end

__led_turn_off:			; halt LED brightness controll
						movlb	0
						bsf		s0_TRISA,TRISA_LED_CATHODE
						; fall through

__led_ctrl_end:			movlb	15
						; shutdown ADC, FVR, and Timer 3
						bsf		s15_PMD0,FVRMD
						bsf		s15_PMD1,TMR3MD
						bsf		s15_PMD2,ADCMD
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Subroutine for Interrupt Routine														;
;		*		*		*		*		*		*		*		*		*		*		*		;

;		WREG	burst length
f1_prepare_adc_and_fvr:
						movlb	2
						movwf	s2_ADRPT

						movlb	18
						; power-up fixed voltage regulator for ADC
						movlw	FVRCON_init
						movwf	s18_FVRCON

						movlb	4
						; enter settling time of FVR
						clrf	s4_T3CON
						movlw	T3CLK_init
						movwf	s4_T3CLK
						movlw	LOW  (0x10000 - FVR_SETTLING_TIME*(FCY/1000000))
						movwf	s4_T3TMRL
						movlw	HIGH (0x10000 - FVR_SETTLING_TIME*(FCY/1000000))
						movwf	s4_T3TMRH
						movlw	T3CON_init
						movwf	s4_T3CON

						; common settings
						movlb	1
						movlw	ADCON1_init
						movwf	s1_ADCON1
						movlw	ADCON3_init
						movwf	s1_ADCON3
						movlw	ADCLK_init
						movwf	s1_ADCLK
						movlw	ADPRE_init
						movwf	s1_ADPRE
						movlw	ADACQ_init
						movwf	s1_ADACQ
						movlw	ADCAP_init
						movwf	s1_ADCAP
						movlw	ADREF_VDD_RATIOMETRIC
						movwf	s1_ADREF

						; configure ADC for input voltage acquisition
						movlw	ADACT_T3TMR
						movwf	s1_ADACT
						movlw	ADCON0_init
						movwf	s1_ADCON0
						movlw	ADCON2_BURST_AVERAGE
						movwf	s1_ADCON2
						movlw	ADPCH_FVR
						movwf	s1_ADPCH
						return

f1_load_adc_fvr_result:
						movlb	14
						bcf		s14_PIR1,ADTIF
						bsf		s14_PIE1,ADTIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADTIE

						movlb	18
						; turn-OFF FVR
						clrf	s18_FVRCON

						movlb	4
						; turn-OFF Timer 3
						clrf	s4_T3CON

						movlb	1
						movf	s1_ADACCL,W
						movwf	d1_adres_FVR_m7p23+0
						movf	s1_ADACCH,W
						movwf	d1_adres_FVR_m7p23+1

						; turn-OFF ADC
						clrf	s1_ADCON0
						clrf	s1_ADACT
						return

f6_load_default_period:	movlb	6
						clrf	s6_CCP1CON
						movlw	LOW  (OPEN_LOOP_PERIOD - 1)
						movwf	s6_CCPR1L
						movlw	HIGH (OPEN_LOOP_PERIOD - 1)
						movwf	s6_CCPR1H
						movlw	CCP1CON_init
						movwf	s6_CCP1CON					;  8

						movlw	LOW  (OPEN_LOOP_PERIOD)
						movwf	cemf_sampling_period_L
						movlw	HIGH (OPEN_LOOP_PERIOD)
						movwf	cemf_sampling_period_H
						return								;  6	14

; PWM module is temporary shutdown
;	 input: none
;	output: duty ratio register in PWM module with duty=0%
f6_pwm_pause:			movlb	19
						; disconnect power from motor
						bcf		s19_CM1CON0,C1POL

						movlb	5
						; stop PWM timer
						bcf		s5_T2CON,TMR2ON
						movf	s5_T2PR,W
						movwf	s5_T2TMR

						movlb	19
						; resume half-bridge operation at next cycle
						bsf		s19_CM1CON0,C1POL	; turn-ON HB2 LS FET

						movlb	6
						; set duty ratio to zero
						clrf	s6_CCPR2L
						clrf	s6_CCPR2H
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

f6_closed_loop_control:
						; load lut entry into sampling_interval
						call	f6_load_lut_entry

						; save duty
						movf	s6_CCPR2L,W
						movwf	pulse_width_L
						movf	s6_CCPR2H,W
						movwf	pulse_width_H

						; set duty to 0%
						clrf	s6_CCPR2L
						clrf	s6_CCPR2H

						movlb	5
						movf	s5_T2PR,W
						movwf	s5_T2TMR					; force update duty register

						bsf		pulse_width_L,0

						; stop PWM timer
						bcf		s5_T2CON,TMR2ON
						movwf	s5_T2TMR

						movlb	6
						; restore duty
						movf	pulse_width_L,W
						movwf	s6_CCPR2L
						movf	pulse_width_H,W
						movwf	s6_CCPR2H

						movlb	19
						; disconnect power from motor
						bcf		s19_CM1CON0,C1POL

						movlb	62
						; turn on bias resister
						bsf		s62_WPUB,WPUB_CEMF_SENSE

						; turn on input buffer
						bcf		s62_ANSELB,ANSELB_CEMF_SENSE

						movlb	0
						; wait for end of fast decay discharge
						btfss	s0_PORTB,PORTB_CEMF_SENSE
						bra		$-1
						btfss	s0_PORTB,PORTB_CEMF_SENSE
						bra		$-1
						btfss	s0_PORTB,PORTB_CEMF_SENSE
						bra		$-1

						movlb	62
						; turn off bias resister
						bcf		s62_WPUB,WPUB_CEMF_SENSE

						; turn off input buffer
						bsf		s62_ANSELB,ANSELB_CEMF_SENSE

						movlb	19
						; resume half-bridge operation at next cycle
						bsf		s19_CM1CON0,C1POL	; turn-ON HB2 LS FET

						movlb	4
						; enter settling time of CEMF
						clrf	s4_T3CON
						movlw	T3CLK_init
						movwf	s4_T3CLK
						movlw	LOW  (0x10000 - CEMF_SETTLING_TIME*(FCY/1000000))
						movwf	s4_T3TMRL
						movlw	HIGH (0x10000 - CEMF_SETTLING_TIME*(FCY/1000000))
						movwf	s4_T3TMRH
						movlw	T3CON_init
						movwf	s4_T3CON

						movlb	1
						; configure ADC for CEMF acquisition
						movlw	ADACT_T3TMR
						movwf	s1_ADACT
						movlw	ADCON2_BURST_AVERAGE
						movwf	s1_ADCON2
						movlw	ADPCH_MOTOR
						movwf	s1_ADPCH
						movlw	ADCON0_init
						movwf	s1_ADCON0
						movlb	2
						movlw	(P_GAIN*PWM_PERIOD+128)/256
						movwf	s2_ADRPT

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
						movf	t1_integral_6p18+1,W
						movwf	pulse_width_L
						movf	t1_integral_6p18+2,W
						movwf	pulse_width_H

						movlw	(P_GAIN*PWM_PERIOD+128)/256-1
						movwf	s1_samples
__cemf_reference_lp:	movf	t1_target_ratio_6p18+0,W
						addwf	t1_proportional_6p18+0,F
						movf	t1_target_ratio_6p18+1,W
						addwfc	t1_proportional_6p18+1,F
						movf	t1_target_ratio_6p18+2,W
						addwfc	t1_proportional_6p18+2,F

						decfsz	s1_samples,F
						bra		__cemf_reference_lp

__cemf_last_acq_wait:
						movlb	14
						bcf		s14_PIR1,ADIF
						bsf		s14_PIE1,ADIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADIE

						movlb	2
						; wait for last acquisition start
						movlw	(P_GAIN*PWM_PERIOD+128)/256-1
						subwf	s2_ADCNT,W
						btfss	STATUS,Z
						bra		__cemf_last_acq_wait

						movlb	1
						; wait for last conversion start
						btfss	s1_ADSTAT,0	
						bra		$-1

						movlb	5
						; restart PWM generator after ADC sampling end
						bsf		s5_T2CON,TMR2ON
						TEST_PIN_ON	TEST_DUTY_UPDATE

						movlb	4
						; turn-OFF Timer 3
						clrf	s4_T3CON

						movlb	14
						; wait for last A/D convertion completion
						bcf		s14_PIR1,ADTIF
						bsf		s14_PIE1,ADTIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADTIE				; 46

						movlb	1
						; proportional = target - current_cemf
						movf	s1_ADACCL,W
						movwf	t1_current_cemf_6p18+1
						subwf	t1_proportional_6p18+1,F
						movf	s1_ADACCH,W
						movwf	t1_current_cemf_6p18+2
						subwfb	t1_proportional_6p18+2,F	; 7

						; turn-OFF ADC
						clrf	s1_ADCON0
						clrf	s1_ADACT					; 2
						
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
						goto	f6_pwm_pause				; 2	call and return

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
						movwf	pulse_width_H				; 10	86

f6_pwm_by_duty_ratio:	movlb	6
						; saturate calculation
						comf	pulse_width_H,W				; expect B'111111xx' 
						addlw	0x04
						subwfb	WREG,W						; W = -1 +C
						iorwf	pulse_width_L,F
						iorwf	pulse_width_H,F				; +6	92

						; pulse_width_H         pulse_width_L
						; 0,0,0,0,0,0,d9,d8     d7,d6,d5,d4,d3,d2,d1,d0
						movf	pulse_width_H,W
						movwf	s6_CCPR2H
						movf	pulse_width_L,W
						movwf	s6_CCPR2L
		if (TEST_TARGET == TEST_VOLTAGE_OUT)
						movf	pulse_width_L,W
						andlw	0xF0
						iorwf	pulse_width_H,W
						swapf	WREG,W
			;			lsrf	WREG,W
						movwf	INDF1
		endif
						return								; +6	98

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
f1_init_open_loop_ctrl: movlb	19
						; connect
						bsf		s19_CM1CON0,C1POL

						movlb	5
						; restart PWM timer
						btfss	s5_T2CON,TMR2ON
						bsf		s5_T2CON,TMR2ON

						movlb	1
						movlw	BREAKING_IN_DURATION*POLLING_FREQUENCY-1
						movwf	s1_breaking_in_time
						movlw	OPEN_LOOP_FREQ
						movwf	s1_sampled_waves
						clrf	t1_integral_6p18+0
						clrf	t1_integral_6p18+1
						return
						
f6_open_loop_control:	movlb	1
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
						goto	f6_pwm_by_duty_ratio	; break-in, call and return
						; fall through

__calib_begin:
						movlb	6
						; set duty to 0%
						clrf	s6_CCPR2L
						clrf	s6_CCPR2H

						movlb	5
						movf	s5_T2PR,W
						movwf	s5_T2TMR					; force update duty register

CALIB_ADC_INTERVAL		equ		18
END_OF_CALIB_VOLTAGE	equ		256
CALIB_WAVES				equ		24
CALIB_COEFFICIENT_8p8	equ		256*FCY_SCALED/1000000*CENTER_WAVE_HEIGHT/CALIB_WAVES*CALIB_ADC_INTERVAL/MEAN_WAVE_HEIGHT
						; since measured value is an arithmetic average, parameter is corrected from it to center.

						movlb	4
						; set Timer to discharge time
						clrf	s4_T1CON
						movlw	HIGH (0x10000-(500-CALIB_ADC_INTERVAL)*(FCY_SCALED/1000000))
						movwf	s4_T1TMRH
						movlw	LOW  (0x10000-(500-CALIB_ADC_INTERVAL)*(FCY_SCALED/1000000))
						movwf	s4_T1TMRL
						movlw	T1CON_init
						movwf	s4_T1CON					; start 500us later

						movlb	6
						; set CCP to A/D conversion interval
						clrf	s6_CCP1CON
						movlw	LOW	 (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s6_CCPR1L
						movlw	HIGH (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s6_CCPR1H
						movlw	CCP1CON_init
						movwf	s6_CCP1CON

						movlb	12
						; halt half-bridge operation
						bcf		s12_CWG1AS0,REN
						bsf		s12_CWG1AS0,SHUTDOWN

						movlb	0
						; halt LED brightness controll
						bsf		s0_TRISA,TRISA_LED_CATHODE

						movlb	16
						; change WDT period
						movlw	WDTCON0_calibration
						movwf	s16_WDTCON0

						movlb	1
						; set ADC to basic mode using CCP trigger
						movlw	ADACT_CCP1
						movwf	s1_ADACT
						movlw	ADCON2_BASIC
						movwf	s1_ADCON2
						movlw	ADPCH_MOTOR
						movwf	s1_ADPCH
						movlw	ADCON0_init
						movwf	s1_ADCON0

						; clear data memory used to hold c-emf values
						clrf	FSR0L
						movlw	0x21
						movwf	FSR0H
						clrf	INDF0
						incfsz	FSR0L,F						; with wrap-round
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

__calib_lp_decreasing:	; decreasing in CEMF voltage		; cemf_mean < cemf_min
						TEST_PIN_ON TEST_CALIBRATION
						bsf		s1_sampling_flag,1			; set bit1 as minimum update

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
						addwfc	d1_cemf_min_14p2+1,F		; 4		48/52

__calib_lp:				; wait for start of A/D convertion
						btfss	s1_ADCON0,ADGO
						bra		$-1

						; IIR LPF, time constant is 8 periods
						movf	d1_cemf_range_14p2+0,W
						subwf	d1_cemf_mean_14p2+0,F
						movf	d1_cemf_range_14p2+1,W
						subwfb	d1_cemf_mean_14p2+1,F		;  6	56/60

						; wait for completion of A/D convertion
						btfsc	s1_ADCON0,ADGO				;		0
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
						bra		__calib_lp_update			;  / 9	 /48

__calib_end:
						movlb	6
						; Turn-OFF periodic timer
						clrf	s6_CCP1CON
						movlb	4
						clrf	s4_T1CON

						movlb	14
						; Disable interrupts
						bcf		s14_PIE1,OSFIE
						bcf		s14_PIE6,CCP1IE

						movlb	1
						; Turn-OFF ADC
						clrf	s1_ADACT
						clrf	s1_ADCON0

						movf	s1_sampled_waves,F
						btfsc	STATUS,Z
						call	set_factory_default			; call but never return

						; prepare FVR, ADC, and T3 to acquire VDD voltage
						movlw	0x20
						call	f1_prepare_adc_and_fvr

						clrwdt

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

						decfsz	s1_sampled_waves,F
						bra		__calc_motor_const_lp2

						; wait for completion of A/D convertion
						call	f1_load_adc_fvr_result

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
						movlb	14
						; abort calibration when oscillator is faulty.
						btfsc	s14_PIR1,OSFIF
						reset

						call	update_parameter			; call but never return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Common entry point (cold boot, wake up from sleep, fall into sleep)						;
;		*		*		*		*		*		*		*		*		*		*		*		;
common_entry_point:
						movlb	17
						; speed-up HFINTOSC for backup.
						movlw	OSCFRQ_backup
						movwf	s17_OSCFRQ
						movlw	OSCCON1_backup
						movwf	s17_OSCCON1

						movlb	62
						; enable pull-up
						movlw	WPUA_init
						movwf	s62_WPUA
						movlw	WPUB_init
						movwf	s62_WPUB
						movlw	WPUC_init
						movwf	s62_WPUC

						movlb	16
						; analyze reset
						movf	s16_PCON0,W
						movwf	notification_flag
						movlw	PCON_init
						movwf	s16_PCON0

						TEST_PIN_INIT
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

						movlb	4
						clrf	s4_T1CON
						clrf	s4_T3CON

						movlb	62
						movlw	INLVLA_init
						movwf	s62_INLVLA
						movlw	INLVLB_init
						movwf	s62_INLVLB
						movlw	INLVLC_init
						movwf	s62_INLVLC

						; enable input buffer on PORTB
						movlw	ANSELB_init
						movwf	s62_ANSELB

						movlb	0
						; set default state when the port is cofigured for generic use.
						movlw	LATA_init
						movwf	s0_LATA
						movlw	LATB_init
						movwf	s0_LATB
						movlw	LATC_init
						movwf	s0_LATC

						btfss	notification_flag,NOT_POR
						bra		cold_boot					; Power on reset

						; wait for RE3/MCLR release when debugger is attached
						movf	s0_PORTB,W
						iorwf	s0_PORTE,W
						andlw	(1<<BIT_SW_RESET)|(1<<BIT_SW_ORIGIN)|(1<<BIT_SW_LIMIT)
						btfsc	STATUS,Z
						bra		$-4
						
						; hold cemf sense pin to stable state
						bcf		s0_TRISB,TRISB_CEMF_SENSE

						; don't wake-up by RB6 or RB7 when debugger is attached.
						andlw	(1<<BIT_SW_ORIGIN)|(1<<BIT_SW_LIMIT)
						movlw	LOW (~(1<<BIT_SW_ORIGIN))
						btfsc	STATUS,Z
						movlw	0xFF

						movlb	62
						movwf	s62_ANSELB
						comf	WREG,W
						clrf	s62_WPUA			; All pullups on PORTA are disabled
						movwf	s62_WPUB
						clrf	s62_WPUC			; All pullups on PORTC are disabled

						call	f1_save_lut_index

						movlb	16
						; change WDT period
						movlw	WDTCON0_sleep
						movwf	s16_WDTCON0

						movlb	17
						; speed-down HFINTOSC for sleep
						movlw	OSCCON1_sleep
						movwf	s17_OSCCON1
						movlw	OSCFRQ_sleep
						movwf	s17_OSCFRQ

						movlb	16
						movlw	VREGCON_sleep
						movwf	s16_VREGCON
__sleep_lp:				movlb	0
						movf	s0_PORTB,W				; negative level
						iorwf	s0_PORTE,W

						movlb	62
						btfss	WREG,BIT_SW_RESET		; is Power button pressed ?
						bra		__warm_reset
						btfss	s62_ANSELB,BIT_SW_ORIGIN	; is buffer for origin SW active ?
						btfsc	WREG,BIT_SW_ORIGIN		; is origin SW pressed ?
						bra		__short_sleep

						bsf		s62_ANSELB,BIT_SW_ORIGIN
						bcf		s62_WPUB,BIT_SW_ORIGIN
						call	f1_set_idx_to_lut_start
						call	f1_save_lut_index

__short_sleep:			TEST_PIN_OFF	TEST_CPU_USAGE
						sleep				; method to wake-up is assertion of MCLR, POR or WDT
						TEST_PIN_ON	TEST_CPU_USAGE
						bra		__sleep_lp

__warm_reset:			movlb	16
						movlw	PCON_cold_boot	; merging into cold boot sequence 
						movwf	s16_PCON0
						reset					; due to unstable condition while sleeping

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Main routine																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
cold_boot:				clrf	notification_flag

						movlb	15
		if TEST_TARGET == TEST_NONE
						; disable unused modules
						movlw	PMD0_init
						movwf	s15_PMD0
						movlw	PMD1_init
						movwf	s15_PMD1
						movlw	PMD2_init
						movwf	s15_PMD2
						movlw	PMD3_init
						movwf	s15_PMD3
						movlw	PMD4_init
						movwf	s15_PMD4
						movlw	PMD5_init
						movwf	s15_PMD5
		endif

						; restore lut_index from DATA EEPROM
						call	f1_restore_lut_index

						movlb	19
						; disconnect power from motor
						movlw	CM1CON0_init
						movwf	s19_CM1CON0

						movlb	4
						; timer assignment
						movlw	CCPTMRS0_init
						movwf	s4_CCPTMRS0
						movlw	CCPTMRS1_init
						movwf	s4_CCPTMRS1

						movlb	6
						; setup PWM for motor control 
						clrf	s6_CCPR2L
						clrf	s6_CCPR2H
						movlw	CCP2CON_init
						movwf	s6_CCP2CON

						movlb	12
						; halfbridge
						movlw	CWG1ISM_init
						movwf	s12_CWG1ISM
						movlw	CWG1DBR_init
						movwf	s12_CWG1DBR
						movlw	CWG1DBF_init
						movwf	s12_CWG1DBF
						movlw	CWG1CON0_init
						movwf	s12_CWG1CON0
						movlw	CWG1CON1_init
						movwf	s12_CWG1CON1
						movlw	CWG1AS0_init
						movwf	s12_CWG1AS0
						movlw	CWG1AS1_init
						movwf	s12_CWG1AS1

						movlb	7
						; setup PWM for LED brightness control
						movlw	PWM6DCH_init
						movwf	s7_PWM6DCH
						clrf	s7_PWM6DCL
						movlw	PWM6CON_init
						movwf	s7_PWM6CON

						movlb	62
						movlw	ODCONA_init
						movwf	s62_ODCONA
						movlw	ODCONB_init
						movwf	s62_ODCONB
						movlw	ODCONC_init
						movwf	s62_ODCONC

						movlw	SLRCONA_init
						movwf	s62_SLRCONA
						movlw	SLRCONB_init
						movwf	s62_SLRCONB
						movlw	SLRCONC_init
						movwf	s62_SLRCONC

			;												;  1 26					Vpp/MCLR/power SW(was RA3)
			;			clrf	s62_RA0PPS					;  2 27					LED current sense
						movlw	PPS_LED_CATHODE_init
						movwf	s62_RA1PPS					;  3 28	PWM6			LED drive
			;			movwf	s62_RA2PPS					;  4  1	DAC1OUT	RA2		debug pin
			;			movwf	s62_RA3PPS					;  5  2			RA3		
			;			clrf	s62_RA4PPS					;  6  3			RA4		stop SW(was RC0)
			;			clrf	s62_RA5PPS					;  7  4			RA5		double speed SW(was RC1)
															;  8  5			VSS		Power
			;			clrf	s62_RA7PPS					;  9  6			CLKIN	X'tal
			;			clrf	s62_RA6PPS					; 10  7	CLKOUT			X'tal
						movlw	PPS_HB1_HS_init
						movwf	s62_RC0PPS					; 11  8	CWG1A			HB1 HS
						movwf	s62_RC1PPS					; 12  9	CWG1A			HB1 HS
						movwf	s62_RC2PPS					; 13 10	CWG1A			HB1 HS
						movwf	s62_RC3PPS					; 14 11	CWG1A			HB1 HS

						movwf	s62_RC4PPS					; 15 12	CWG1A			HB1 HS
						movwf	s62_RC5PPS					; 16 13	CWG1A			HB1 HS
						movlw	PPS_HB2_LS_init
						movwf	s62_RC6PPS					; 17 14	C1OUT			HB2 LS
						movwf	s62_RC7PPS					; 18 15	C1OUT			HB2 LS
															; 19 16			VSS		Power
															; 20 17			VDD		Power
			;			clrf	s62_RB0PPS					; 21 18	ZCDIN	C2IN1+	HB1 voltage sense ADC
						movlw	PPS_HB1_LS_init
						movwf	s62_RB1PPS					; 22 19	CWG1B			HB1 LS
						movwf	s62_RB2PPS					; 23 20	CWG1B			HB1 LS
						movwf	s62_RB3PPS					; 24 21	CWG1B			HB1 LS
						movlw	PPS_HB1_HS_init
						movwf	s62_RB4PPS					; 25 22	CWG1A	OD		HB1 LS
						movwf	s62_RB5PPS					; 26 23	CWG1A	OD		HB1 LS
			;			clrf	s62_RB6PPS					; 27 24			RB6		ICSPCLK/origin SW(was RA1)
			;			clrf	s62_RB7PPS					; 28 25			RB7		ICSPDAT/limit SW(was RA0)

						; enable input buffer
						movlw	ANSELA_init
						movwf	s62_ANSELA
						movlw	ANSELB_init
						movwf	s62_ANSELB
						movlw	ANSELC_init
						movwf	s62_ANSELC

						movlb	4
						; start timer for ADC trigger
						movlw	T1CLK_init
						movwf	s4_T1CLK
						clrf	s4_T1TMRL
						clrf	s4_T1TMRH
						movlw	T1CON_init
						movwf	s4_T1CON

						movlb	5
						; start PWM controller for motor
						movlw	T2CLK_init
						movwf	s5_T2CLK
						movlw	T2HLT_init
						movwf	s5_T2HLT
						movlw	T2PR_init
						movwf	s5_T2PR
						movwf	s5_T2TMR
						movlw	T2CON_init
						movwf	s5_T2CON

						; start PWM controller for LED
						movlw	T6CLK_init
						movwf	s5_T6CLK
						movlw	T6HLT_init
						movwf	s5_T6HLT
						movlw	T6PR_init
						movwf	s5_T6PR
						movwf	s5_T6TMR
						movlw	T6CON_init
						movwf	s5_T6CON

						; setup CCP1 for ADC trigger of motor
						call	f6_load_default_period

						movlb	0
						; initial key state
						comf	s0_PORTA,W
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

						movlb	16
						; change WDT period
						movlw	WDTCON0_active
						movwf	s16_WDTCON0

						movlb	0
						; output enable
						movlw	TRISA_active
		if (TEST_TARGET == TEST_VOLTAGE_OUT)
						bsf		WREG,TRISA2
		endif
						movwf	s0_TRISA
						movlw	TRISB_active
						movwf	s0_TRISB
						movlw	TRISC_active
						movwf	s0_TRISC

						movlb	14
						; interrupt enable
						bsf		s14_PIE1,OSFIE
						bsf		s14_PIE6,CCP1IE

						movlb	17
						; enable IDLE mode
						bsf		s17_CPUDOZE,IDLEN

						movlw	INTCON_active
						movwf	INTCON

						; wait for backup osc. ready
						movlw	OSCCON1_backup
						xorwf	s17_OSCCON2,W
						btfss	STATUS,Z
						bra		$-3

						; switch to external osc.
						movlw	OSCCON1_active
						movwf	s17_OSCCON1

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

						movlb	19
						; disconnect power from motor
						bcf		s19_CM1CON0,C1POL

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
						movf	s0_PORTA,W
						andlw	(1<<BIT_SW_STOP)|(1<<BIT_SW_DOUBLE_SPEED)
						iorwf	s0_PORTB,W
						iorwf	s0_PORTE,W			; {RB7, RB6, RA5, RA4, RE3, 0, 0, RB0}

						movlb	1
						; check debugger
						btfsc	WREG,RB6
						bra		$+3
						btfss	WREG,RB7
						iorlw	(1<<RB6)|(1<<RB7)	; if both RB6 and RB7 are ON, set both to OFF

						; update states of SWs
						xorlw	0xFF				; invert logic (from negative to positive)
						andwf	s1_sw_press,F
						iorwf	s1_sw_release,F
						movwf	d1_sw_transient+0			; 14

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
						return								; 10	34
wait_1us:				bra		$-1

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
f1_restore_lut_index:	movlb	16
						bsf		s16_NVMCON1,NVMREGS		; Access EEPROM, Configuration, user ID and device ID registers
						movlw	0xF0
						movwf	s16_NVMADRH

						; read motor parameter from EEPROM
						movlw	LOW dfm_motor_constant
						movwf	s16_NVMADRL

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
						movwf	s16_NVMADRL

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

						movlb	16
						bsf		s16_NVMCON1,NVMREGS		; Access EEPROM, Configuration, user ID and device ID registers
						movlw	0xF0
						movwf	s16_NVMADRH
						movlw	LOW dfm_lut_address
						movwf	s16_NVMADRL

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
						movlb	16
						bsf		s16_NVMCON1,NVMREGS		; Access EEPROM, Configuration, user ID and device ID registers
						movlw	0xF0
						movwf	s16_NVMADRH
						movlw	LOW dfm_motor_constant
						movwf	s16_NVMADRL

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
;		BSR		16
__read_data_eeprom:		bsf		s16_NVMCON1,RD
						movf	s16_NVMDATL,W
						incf	s16_NVMADRL,F
						return
						
;		WREG	data to be written
;		EEADR	address of data
;		BSR		16
__write_data_eeprom:	clrwdt
						bsf		s16_NVMCON1,RD
						xorwf	s16_NVMDATL,F
						btfsc	STATUS,Z
						bra		__skip_programming
						movwf	s16_NVMDATL

						TEST_PIN_ON	TEST_NVM

						bsf		s16_NVMCON1,WREN
						movlw	0x55				; Load 55h to get ready for unlock sequence
						movwf	s16_NVMCON2			; First step is to load 55h into NVMCON2
						movlw	0xAA				; Second step is to load AAh into W
						movwf	s16_NVMCON2			; Third step is to load AAh into NVMCON2
						bsf		s16_NVMCON1,WR		; Final step is to set WR bit
						bcf		s16_NVMCON1,WREN

						btfsc	s16_NVMCON1,WR
						bra		$-1

						TEST_PIN_OFF TEST_NVM

__skip_programming:		incf	s16_NVMADRL,F
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
						; load sampling start time into CCPR1
						clrf	s6_CCP1CON
						movlw	0xFF
						addwf	cemf_sampling_period_L,W
						movwf	s6_CCPR1L
						movlw	0xFF
						addwfc	cemf_sampling_period_H,W
						movwf	s6_CCPR1H
						movlw	CCP1CON_init
						movwf	s6_CCP1CON
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
