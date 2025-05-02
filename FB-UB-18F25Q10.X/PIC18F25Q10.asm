						#include		mpasmx.inc

						errorlevel -302
						radix	DEC

FCY						equ		6000000					; 6MHz
PWM_PERIOD				equ		250						; PWM frequency = FCY/PWM_PERIOD, min: 200, max: 256
OPEN_LOOP_PERIOD		equ		62500					; fixed settings
ADC_INTERVAL			equ		15

;	boot
;	por/mclr			other reset
;	HP Regulator		LP Regulator
;	HFINTOSC			wait for wakeup
;	EXTOSC				reset

; High supply voltage or Low motor speed
						CONFIG	FEXTOSC = HS         ; HS (crystal oscillator) above 8 MHz; PFM set to high power
						CONFIG	RSTOSC = HFINTOSC_1MHZ	; HFINTOSC with HFFRQ = 4 MHz and CDIV = 4:1
						CONFIG	CLKOUTEN = OFF       ; CLKOUT function is disabled
						CONFIG	CSWEN = ON           ; Writing to NOSC and NDIV is allowed
						CONFIG	FCMEN = ON           ; Fail-Safe Clock Monitor enabled
						CONFIG	MCLRE = INTMCLR      ; MCLR pin (RE3) is input
						CONFIG	PWRTE = OFF          ; Power up timer disabled
						CONFIG	LPBOREN = OFF        ; Low power BOR is disabled
						CONFIG	BOREN = NOSLP        ; Brown-out Reset enabled while running, disabled in Sleep; SBOREN is ignored
						CONFIG	BORV = VBOR_190      ; Brown-out Reset Voltage (VBOR) set to 1.90V
						CONFIG	ZCD = OFF            ; ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
						CONFIG	PPS1WAY = OFF        ; PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence)
						CONFIG	STVREN = ON          ; Stack full/underflow will cause Reset
						CONFIG	XINST = OFF          ; Extended Instruction Set and Indexed Addressing Mode disabled
						CONFIG	WDTCPS = WDTCPS_31   ; Divider ratio 1:65536; software control of WDTPS
						CONFIG	WDTE = ON            ; WDT always enabled
						CONFIG	WDTCWS = WDTCWS_7    ; window always open (100%); software control; keyed access not required
						CONFIG	WDTCCS = LFINTOSC    ; WDT reference clock is the 31.0 kHz LFINTOSC
						CONFIG	WRT0 = OFF           ; Block 0 (000800-001FFFh) not write-protected
						CONFIG	WRT1 = OFF           ; Block 1 (002000-003FFFh) not write-protected
						CONFIG	WRTC = OFF           ; Configuration registers (300000-30000Bh) not write-protected
						CONFIG	WRTB = OFF           ; Boot Block (000000-0007FFh) not write-protected
						CONFIG	WRTD = OFF           ; Data EEPROM not write-protected
						CONFIG	SCANE = OFF          ; Scanner module is NOT available for use, SCANMD bit is ignored
						CONFIG	LVP = OFF            ; HV on MCLR/VPP must be used for programming
						CONFIG	CP = ON              ; UserNVM code protection enabled
						CONFIG	CPD = ON             ; DataNVM code protection enabled
						CONFIG	EBTR0 = OFF          ; Block 0 (000800-001FFFh) not protected from table reads executed in other blocks
						CONFIG	EBTR1 = OFF          ; Block 1 (002000-003FFFh) not protected from table reads executed in other blocks
        ifdef __18F25Q10
						CONFIG	EBTR2 = OFF          ; Block 2 (004000-005FFFh) not protected from table reads executed in other blocks
						CONFIG	EBTR3 = OFF          ; Block 3 (006000-007FFFh) not protected from table reads executed in other blocks
		endif
						CONFIG	EBTRB = OFF          ; Boot Block (000000-0007FFh) not protected from table reads executed in other blocks

LOW_BATT_ERROR			set		2000	; [mV]
LOW_BATT_WARNING		set		2200	; [mV]

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
		dw		period_i
		while fraction != 0
fraction += period_f
				if fraction >= 32
fraction -= 32
						dw		(period_i + 1)
				else
						dw		period_i
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

						dw		period_i
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
CLKRCON_init			equ		(1<<CLKREN)|(B'10'<<CLKRDC0)|(B'010'<<CLKRDIV0)	; Fcy
MDCON0_init				equ		(1<<EN)
MDCON1_init				equ		(1<<CHPOL)
PPS_DSM_init			equ		0x06		; RA6 = CLKIN, OSC1
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
						;        PWM4  PWM3  CCP2  CCP1
						; 16bit                T1   *T1
						;  8bit   *T6    T4   *T2    T2
CCPTMRS_init			equ		(B'11'<<P4TSEL0)|(B'10'<<P3TSEL0)|(B'01'<<C2TSEL0)|(B'01'<<C1TSEL0)

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
PPS_HB1_HS_init			equ		0x01					; Rxy source is CWG1A
PPS_HB1_LS_init			equ		0x02					; Rxy source is CWG1B
PPS_HB2_LS_init			equ		0x0B					; Rxy source is C1
CM1CON0_init			equ		(0<<C1EN)|(0<<C1POL)|(0<<C1HYS)
WPUB_CEMF_SENSE			equ		WPUB0
TRISB_CEMF_SENSE		equ		TRISB0
ANSELB_CEMF_SENSE		equ		ANSELB0
PORTB_CEMF_SENSE		equ		RB0

						; Trigger for A/D conversion of FVR
FVR_SETTLING_TIME		set		50
T3CON_init				equ		(1<<TMR3ON)
T3CLK_init				equ		B'0001'					; FCY

						; Constant brightness LED driver
T6CON_init				equ		(B'011'<<T6CKPS0)|(1<<TMR6ON) ; FCY/8, pre 1:4, post 1:1
T6CLK_init				equ		B'0001'							; FCY
T6HLT_init				equ		(1<<PSYNC)|(1<<CKSYNC)|(B'00000'<<MODE0)
T6PR_init				equ		255
PWM4CON_init			equ		(1<<PWM4EN)|(1<<PWM4POL)
PWM4DCH_init			equ		6*(FCY/1000000)/8		;  minimum settling time 5.25us
PPS_LED_CATHODE_init	equ		0x08					; Rxy source is PWM4
TRISA_LED_CATHODE		equ		TRISA1

; etc.
OSCFRQ_backup			equ		(B'0111'<<FRQ0)			; 48MHz
OSCFRQ_sleep			equ		(B'0000'<<FRQ0)			; 1MHz
OSCCON1_backup			equ		(B'110'<<NOSC0)|(B'001'<<NDIV0)		; HFINTOSC, 1:2
OSCCON1_sleep			equ		(B'110'<<NOSC0)|(B'001'<<NDIV0)		; HFINTOSC, 1:2
OSCCON1_active			equ		(B'010'<<NOSC0)|(B'000'<<NDIV0)		; EXTOSC with 4x PLL, 1:1
WDTCON0_active			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON0_sleep			equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
WDTCON0_calibration		equ		(B'01010'<<WDTPS0)		; 1s nominal
WDTCON1_init			equ		(B'111'<<WINDOW0)
PCON_init				equ		(1<<NOT_BOR)|(1<<NOT_POR)|(1<<NOT_RI)|(1<<NOT_RMCLR)
PCON_cold_boot			equ		(1<<NOT_BOR)|(1<<NOT_RMCLR)
INTCON_sleep			equ		(1<<PEIE)
INTCON_active			equ		(1<<PEIE)|(1<<GIE)
VREGCON_active			equ		(B'01'<<VREGPM0)		; Main regulator in LP mode
VREGCON_sleep			equ		(B'10'<<VREGPM0)		; Main regulator in ULP mode
SIDEREAL_TIME_0p16		equ		65536		; [0.5, 1.5)
SOLAR_TIME_0p16			equ		65715		; [0.5, 1.5)
LUNAR_TIME_0p16			equ		68019		; [0.5, 1.5)

PMD0_init				equ		(0<<SYSCMD)|(1<<FVRMD)|(1<<HLVDMD)|(1<<CRCMD)|(1<<SCANMD)|(0<<NVMMD)|(1<<CLKRMD)|(1<<IOCMD)
PMD1_init				equ		(0<<TMR6MD)|(1<<TMR5MD)|(1<<TMR4MD)|(1<<TMR3MD)|(0<<TMR2MD)|(0<<TMR1MD)|(1<<TMR0MD)
PMD2_init				equ		(1<<DACMD)|(1<<ADCMD)|(1<<CMP2MD)|(0<<CMP1MD)|(1<<ZCDMD)
PMD3_init				equ		(0<<PWM4MD)|(1<<PWM3MD)|(0<<CCP2MD)|(0<<CCP1MD)
PMD4_init				equ		(1<<UART1MD)|(1<<MSSP1MD)|(0<<CWG1MD)
PMD5_init				equ		(1<<DSMMD)

; ADC and FVR configurations
FVRCON_init				equ		B'11000001'		; CDAFVR = 00 for OFF, ADFVR = 01 for 1.024V
ADCON0_init				equ		(1<<ADON)|(1<<ADFM)	; ADON, flush right
ADCON1_init				equ		0					; no precharge, single conversion
ADCON2_BURST_AVERAGE	equ		(1<<ADCRS0)|(1<<ADACLR)|(B'011'<<ADMD0)		; Burst Average mode
ADCON2_BASIC			equ		(1<<ADCRS0)|(1<<ADACLR)|(B'000'<<ADMD0)		; Basic mode
ADCON3_init				equ		(B'111'<<ADTMD0)	; Interrupt regardless of threshold test results
ADCLK_init				equ		(FCY/500000)-1
ADREF_VDD_RATIOMETRIC	equ		(0<<ADPREF0)	; Vref+=VDD
ADPCH_FVR				equ		(B'111111'<<ADPCH0)		; FVR(FVR output)
ADPCH_MOTOR				equ		(B'001000'<<ADPCH0)		; ANB0(motor, RB0 pin)
ADPCH_LED				equ		(B'000000'<<ADPCH0)		; ANA0(LED, RA0 pin)
ADPRE_init				equ		0				; no precharge time
ADACQ_init				equ		(ADC_INTERVAL-11)*(FCY/250000)
ADCAP_init				equ		B'11111'		; 31pF
ADRPT_FVR				equ		8				; number of repeat
ADACT_CCP1				equ		B'01001'		; CCP1 match
ADACT_T3TMR				equ		B'00101'		; T3TMR overflow
ADACT_LED				equ		B'01000'		; T6TMR postscaled

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
bank0_access_ram		udata_acs	0x00
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

;-----Bank 0------------------
; general purpose RAM (Bank 0) allocation
d0a_lut_address_com		res		2

bss_start:
; Switch
d0a_sw_transient		res		2		; transient state of SW
s0a_sw_state			res		1		; current state {0, 0, RC1, RC0, RA3, 0, RA1, RA0}
s0a_sw_press			res		1
s0a_sw_release			res		1
s0a_sw_released			res		1		; changed state of SW
s0a_sw_toggle			res		1

t0a_elapsed_time		res		3		; ticks	[s], to generate 4Hz

s0a_multiplier_bits		res		1
s0a_samples				res		1

s0a_notification_change	res		1		; flags for notification

; battery voltage
s0a_low_batt_flag		res		1
t0a_adres_FVR_work		res		3
; adres_FVR = 8192*1024/Vin[mV], 1/Vin[mV] = adres_FVR/8192/1024
d0a_adres_FVR_m7p23		res		2		; m7p23	  8388608LSB=1/1mV		[1/V]

; PI controller for motor
t0a_target_voltage_15p9	res		3		; 15p9	512LSB=1mV	[V]
t0a_target_ratio_6p18	res		3		; 6p18	[V/V]
t0a_startup_ratio_6p18	res		3		; 6p18	[V/V]

t0a_proportional_6p18	res		3		; 6p18	[V/V]
t0a_integral_6p18		res		3		; 6p18	[V/V]

s0a_sampling_phase		res		1
t0a_current_cemf_6p18	res		3		; 6p18	[V/V]
t0a_previous_cemf_6p18	res		3		; 6p18	[V/V]

t0a_sampling_period		res		3		; ticks	[s]
q0a_motor_constant_work	res		4		; ticks*ADRES_LSB	[s*V]

d0a_tracking_period_0p16	res		2

; open loop controller for motor
s0a_breaking_in_time	res		1

s0a_sampled_waves		res		1
s0a_sampling_flag		res		1

d0a_cemf_mean_14p2		res		2		; 4LSB=1mV [V]
d0a_cemf_range_14p2		res		2		; 4LSB=1mV [V]
d0a_cemf_min_14p2		res		2		; 4LSB=1mV [V]
d0a_cemf_work_14p2		res		2		; 4LSB=1mV [V]

; constant current controller for LED
d0a_led_current			res		2

bss_end:

;-----Bank 14------------------
s14_PPSLOCK				equ		PPSLOCK
s14_INT0PPS				equ		INT0PPS
s14_INT1PPS				equ		INT1PPS
s14_INT2PPS				equ		INT2PPS
s14_T0CKIPPS			equ		T0CKIPPS
s14_T1CKIPPS			equ		T1CKIPPS
s14_T1GPPS				equ		T1GPPS
s14_T3CKIPPS			equ		T3CKIPPS
s14_T3GPPS				equ		T3GPPS
s14_T5CKIPPS			equ		T5CKIPPS
s14_T5GPPS				equ		T5GPPS
s14_T2INPPS				equ		T2INPPS
s14_T4INPPS				equ		T4INPPS
s14_T6INPPS				equ		T6INPPS
s14_ADACTPPS			equ		ADACTPPS
s14_CCP1PPS				equ		CCP1PPS
s14_CCP2PPS				equ		CCP2PPS
s14_CWG1INPPS			equ		CWG1INPPS
s14_CWG1PPS				equ		CWG1PPS
s14_CWGINPPS			equ		CWGINPPS
s14_CWGPPS				equ		CWGPPS
s14_MDCARLPPS			equ		MDCARLPPS
s14_MDCARHPPS			equ		MDCARHPPS
s14_MDSRCPPS			equ		MDSRCPPS
s14_RX1PPS				equ		RX1PPS
s14_RXDT1PPS			equ		RXDT1PPS
s14_RXPPS				equ		RXPPS
s14_CK1PPS				equ		CK1PPS
s14_CKPPS				equ		CKPPS
s14_TX1PPS				equ		TX1PPS
s14_TXCK1PPS			equ		TXCK1PPS
s14_TXPPS				equ		TXPPS
s14_SSP1CLKPPS			equ		SSP1CLKPPS
s14_SSPCLKPPS			equ		SSPCLKPPS
s14_SSP1DATPPS			equ		SSP1DATPPS
s14_SSPDATPPS			equ		SSPDATPPS
s14_SSP1SSPPS			equ		SSP1SSPPS
s14_SSPSSPPS			equ		SSPSSPPS
s14_IPR0				equ		IPR0
s14_IPR1				equ		IPR1
s14_IPR2				equ		IPR2
s14_IPR3				equ		IPR3
s14_IPR4				equ		IPR4
s14_IPR5				equ		IPR5
s14_IPR6				equ		IPR6
s14_IPR7				equ		IPR7
s14_PIE0				equ		PIE0
s14_PIE1				equ		PIE1
s14_PIE2				equ		PIE2
s14_PIE3				equ		PIE3
s14_PIE4				equ		PIE4
s14_PIE5				equ		PIE5
s14_PIE6				equ		PIE6
s14_PIE7				equ		PIE7
s14_PIR0				equ		PIR0
s14_PIR1				equ		PIR1
s14_PIR2				equ		PIR2
s14_PIR3				equ		PIR3
s14_PIR4				equ		PIR4
s14_PIR5				equ		PIR5
s14_PIR6				equ		PIR6
s14_PIR7				equ		PIR7
s14_WDTCON0				equ		WDTCON0
s14_WDTCON1				equ		WDTCON1
s14_WDTPSL				equ		WDTPSL
s14_WDTPSH				equ		WDTPSH
s14_WDTTMR				equ		WDTTMR
s14_CPUDOZE				equ		CPUDOZE
s14_OSCCON1				equ		OSCCON1
s14_OSCCON2				equ		OSCCON2
s14_OSCCON3				equ		OSCCON3
s14_OSCSTAT				equ		OSCSTAT
s14_OSCSTAT1			equ		OSCSTAT1
s14_OSCEN				equ		OSCEN
s14_OSCTUNE				equ		OSCTUNE
s14_OSCFREQ				equ		OSCFREQ
s14_OSCFRQ				equ		OSCFRQ
s14_VREGCON				equ		VREGCON
s14_BORCON				equ		BORCON
s14_PMD0				equ		PMD0
s14_PMD1				equ		PMD1
s14_PMD2				equ		PMD2
s14_PMD3				equ		PMD3
s14_PMD4				equ		PMD4
s14_PMD5				equ		PMD5
s14_RA0PPS				equ		RA0PPS
s14_RA1PPS				equ		RA1PPS
s14_RA2PPS				equ		RA2PPS
s14_RA3PPS				equ		RA3PPS
s14_RA4PPS				equ		RA4PPS
s14_RA5PPS				equ		RA5PPS
s14_RA6PPS				equ		RA6PPS
s14_RA7PPS				equ		RA7PPS
s14_RB0PPS				equ		RB0PPS
s14_RB1PPS				equ		RB1PPS
s14_RB2PPS				equ		RB2PPS
s14_RB3PPS				equ		RB3PPS
s14_RB4PPS				equ		RB4PPS
s14_RB5PPS				equ		RB5PPS
s14_RB6PPS				equ		RB6PPS
s14_RB7PPS				equ		RB7PPS
s14_RC0PPS				equ		RC0PPS
s14_RC1PPS				equ		RC1PPS
s14_RC2PPS				equ		RC2PPS
s14_RC3PPS				equ		RC3PPS
s14_RC4PPS				equ		RC4PPS
s14_RC5PPS				equ		RC5PPS
s14_RC6PPS				equ		RC6PPS
s14_RC7PPS				equ		RC7PPS

;-----Bank 15------------------
s15_IOCAF				equ		IOCAF
s15_IOCAN				equ		IOCAN
s15_IOCAP				equ		IOCAP
s15_INLVLA				equ		INLVLA
s15_SLRCONA				equ		SLRCONA
s15_ODCONA				equ		ODCONA
s15_WPUA				equ		WPUA
s15_ANSELA				equ		ANSELA
s15_IOCBF				equ		IOCBF
s15_IOCBN				equ		IOCBN
s15_IOCBP				equ		IOCBP
s15_INLVLB				equ		INLVLB
s15_SLRCONB				equ		SLRCONB
s15_ODCONB				equ		ODCONB
s15_WPUB				equ		WPUB
s15_ANSELB				equ		ANSELB
s15_IOCCF				equ		IOCCF
s15_IOCCN				equ		IOCCN
s15_IOCCP				equ		IOCCP
s15_INLVLC				equ		INLVLC
s15_SLRCONC				equ		SLRCONC
s15_ODCONC				equ		ODCONC
s15_WPUC				equ		WPUC
s15_ANSELC				equ		ANSELC
s15_IOCEF				equ		IOCEF
s15_IOCEN				equ		IOCEN
s15_IOCEP				equ		IOCEP
s15_INLVLE				equ		INLVLE
s15_WPUE				equ		WPUE
s15_HLVDCON0			equ		HLVDCON0
s15_HLVDCON1			equ		HLVDCON1
s15_FVRCON				equ		FVRCON
s15_ZCDCON				equ		ZCDCON
s15_DAC1CON0			equ		DAC1CON0
s15_DAC1CON1			equ		DAC1CON1
s15_CM2CON0				equ		CM2CON0
s15_CM2CON1				equ		CM2CON1
s15_CM2NCH				equ		CM2NCH
s15_CM2PCH				equ		CM2PCH
s15_CM1CON0				equ		CM1CON0
s15_CM1CON1				equ		CM1CON1
s15_CM1NCH				equ		CM1NCH
s15_CM1PCH				equ		CM1PCH
s15_CMOUT				equ		CMOUT
s15_CLKRCON				equ		CLKRCON
s15_CLKRCLK				equ		CLKRCLK
s15_CWG1CLK				equ		CWG1CLK
s15_CWG1ISM				equ		CWG1ISM
s15_CWG1DBR				equ		CWG1DBR
s15_CWG1DBF				equ		CWG1DBF
s15_CWG1CON0			equ		CWG1CON0
s15_CWG1CON1			equ		CWG1CON1
s15_CWG1AS0				equ		CWG1AS0
s15_CWG1AS1				equ		CWG1AS1
s15_CWG1STR				equ		CWG1STR
s15_SCANLADRL			equ		SCANLADRL
s15_SCANLADRH			equ		SCANLADRH
s15_SCANLADRU			equ		SCANLADRU
s15_SCANHADRL			equ		SCANHADRL
s15_SCANHADRH			equ		SCANHADRH
s15_SCANHADRU			equ		SCANHADRU
s15_SCANCON0			equ		SCANCON0
s15_SCANTRIG			equ		SCANTRIG
s15_MDCON0				equ		MDCON0
s15_MDCON1				equ		MDCON1
s15_MDSRC				equ		MDSRC
s15_MDCARL				equ		MDCARL
s15_MDCARH				equ		MDCARH
s15_ADACT				equ		ADACT
s15_ADCLK				equ		ADCLK
s15_ADREF				equ		ADREF
s15_ADCON1				equ		ADCON1
s15_ADCON2				equ		ADCON2
s15_ADCON3				equ		ADCON3
s15_ADACQ				equ		ADACQ
s15_ADCAP				equ		ADCAP
s15_ADPRE				equ		ADPRE
s15_ADPCH				equ		ADPCH
s15_ADCON0				equ		ADCON0
s15_ADPREVL				equ		ADPREVL
s15_ADPREVH				equ		ADPREVH
s15_ADRESL				equ		ADRESL
s15_ADRESH				equ		ADRESH
s15a_ADSTAT				equ		ADSTAT
s15a_ADRPT				equ		ADRPT
s15a_ADCNT				equ		ADCNT
s15a_ADSTPTL			equ		ADSTPTL
s15a_ADSTPTH			equ		ADSTPTH
s15a_ADLTHL				equ		ADLTHL
s15a_ADLTHH				equ		ADLTHH
s15a_ADUTHL				equ		ADUTHL
s15a_ADUTHH				equ		ADUTHH
s15a_ADERRL				equ		ADERRL
s15a_ADERRH				equ		ADERRH
s15a_ADACCL				equ		ADACCL
s15a_ADACCH				equ		ADACCH
s15a_ADFLTRL			equ		ADFLTRL
s15a_ADFLTRH			equ		ADFLTRH
s15a_CRCDATL			equ		CRCDATL
s15a_CRCDATH			equ		CRCDATH
s15a_CRCACCL			equ		CRCACCL
s15a_CRCACCH			equ		CRCACCH
s15a_CRCSHIFTL			equ		CRCSHIFTL
s15a_CRCSHIFTH			equ		CRCSHIFTH
s15a_CRCXORL			equ		CRCXORL
s15a_CRCXORH			equ		CRCXORH
s15a_CRCCON0			equ		CRCCON0
s15a_CRCCON1			equ		CRCCON1
s15a_NVMADRL			equ		NVMADRL
s15a_NVMADRH			equ		NVMADRH
s15a_NVMADRU			equ		NVMADRU
s15a_NVMDATL			equ		NVMDATL
s15a_NVMDATH			equ		NVMDATH
s15a_NVMCON0			equ		NVMCON0
s15a_NVMCON1			equ		NVMCON1
s15a_NVMCON2			equ		NVMCON2
s15a_LATA				equ		LATA
s15a_LATB				equ		LATB
s15a_LATC				equ		LATC
s15a_DDRA				equ		DDRA
s15a_TRISA				equ		TRISA
s15a_DDRB				equ		DDRB
s15a_TRISB				equ		TRISB
s15a_DDRC				equ		DDRC
s15a_TRISC				equ		TRISC
s15a_PORTA				equ		PORTA
s15a_PORTB				equ		PORTB
s15a_PORTC				equ		PORTC
s15a_PORTE				equ		PORTE
s15a_SSP1BUF			equ		SSP1BUF
s15a_SSP1ADD			equ		SSP1ADD
s15a_SSP1MSK			equ		SSP1MSK
s15a_SSP1STAT			equ		SSP1STAT
s15a_SSP1CON1			equ		SSP1CON1
s15a_SSP1CON2			equ		SSP1CON2
s15a_SSP1CON3			equ		SSP1CON3
s15a_RC1REG				equ		RC1REG
s15a_RCREG				equ		RCREG
s15a_RCREG1				equ		RCREG1
s15a_TX1REG				equ		TX1REG
s15a_TXREG				equ		TXREG
s15a_TXREG1				equ		TXREG1
s15a_SP1BRG				equ		SP1BRG
s15a_SP1BRGL			equ		SP1BRGL
s15a_SPBRG				equ		SPBRG
s15a_SPBRG1				equ		SPBRG1
s15a_SPBRGL				equ		SPBRGL
s15a_SP1BRGH			equ		SP1BRGH
s15a_SPBRGH				equ		SPBRGH
s15a_SPBRGH1			equ		SPBRGH1
s15a_RC1STA				equ		RC1STA
s15a_RCSTA				equ		RCSTA
s15a_RCSTA1				equ		RCSTA1
s15a_TX1STA				equ		TX1STA
s15a_TXSTA				equ		TXSTA
s15a_TXSTA1				equ		TXSTA1
s15a_BAUD1CON			equ		BAUD1CON
s15a_BAUDCON			equ		BAUDCON
s15a_BAUDCON1			equ		BAUDCON1
s15a_BAUDCTL			equ		BAUDCTL
s15a_BAUDCTL1			equ		BAUDCTL1
s15a_PWM4DCL			equ		PWM4DCL
s15a_PWM4DCH			equ		PWM4DCH
s15a_PWM4CON			equ		PWM4CON
s15a_PWM3DCL			equ		PWM3DCL
s15a_PWM3DCH			equ		PWM3DCH
s15a_PWM3CON			equ		PWM3CON
s15a_CCPR2L				equ		CCPR2L
s15a_CCPR2H				equ		CCPR2H
s15a_CCP2CON			equ		CCP2CON
s15a_CCP2CAP			equ		CCP2CAP
s15a_CCPR1L				equ		CCPR1L
s15a_CCPR1H				equ		CCPR1H
s15a_CCP1CON			equ		CCP1CON
s15a_CCP1CAP			equ		CCP1CAP
s15a_CCPTMRS			equ		CCPTMRS
s15a_T6TMR				equ		T6TMR
s15a_T6PR				equ		T6PR
s15a_T6CON				equ		T6CON
s15a_T6HLT				equ		T6HLT
s15a_T6CLK				equ		T6CLK
s15a_T6RST				equ		T6RST
s15a_T4TMR				equ		T4TMR
s15a_T4PR				equ		T4PR
s15a_T4CON				equ		T4CON
s15a_T4HLT				equ		T4HLT
s15a_T4CLK				equ		T4CLK
s15a_T4RST				equ		T4RST
s15a_T2TMR				equ		T2TMR
s15a_T2PR				equ		T2PR
s15a_T2CON				equ		T2CON
s15a_T2HLT				equ		T2HLT
s15a_T2CLK				equ		T2CLK
s15a_T2RST				equ		T2RST
s15a_T5TMRL				equ		TMR5L
s15a_T5TMRH				equ		TMR5H
s15a_T5CON				equ		T5CON
s15a_T5GCON				equ		T5GCON
s15a_T5GATE				equ		T5GATE
s15a_T5CLK				equ		T5CLK
s15a_T3TMRL				equ		TMR3L
s15a_T3TMRH				equ		TMR3H
s15a_T3CON				equ		T3CON
s15a_T3GCON				equ		T3GCON
s15a_T3GATE				equ		T3GATE
s15a_T3CLK				equ		T3CLK
s15a_T1TMRL				equ		TMR1L
s15a_T1TMRH				equ		TMR1H
s15a_T1CON				equ		T1CON
s15a_T1GCON				equ		T1GCON
s15a_T1GATE				equ		T1GATE
s15a_T1CLK				equ		T1CLK
s15a_T0TMRL				equ		TMR0L
s15a_T0PR				equ		PR0
s15a_T0TMRH				equ		TMR0H
s15a_T0CON0				equ		T0CON0
s15a_T0CON1				equ		T0CON1
s15a_PCON1				equ		PCON1
s15a_PCON0				equ		PCON0

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Reset vector																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
reset_vector			code_pack	0x000
						movlb	14
						movlw	VREGCON_active
						movwf	s14_VREGCON,BANKED
						bra		common_entry_point

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Interrupt Routine																		;
;		*		*		*		*		*		*		*		*		*		*		*		;

; interrupt handler is consist of pulse generator and voltage sampler.
;		the pulse generator is driven by T2 cooperatively with CCP2.
;		the voltage sampler is driven by T1 cooperatively with CCP1.
interrupt_handler		code_pack	0x008
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						movlb	14
						btfsc	s14_PIR1,OSCFIF,BANKED
						bra		__intr_osc_fail
						btfsc	s14_PIR6,CCP1IF,BANKED
						bra		__intr_motor
						reset

__intr_osc_fail:		; clear fail-safe clock operation flag
						bcf		s14_PIR1,OSCFIF,BANKED

						rcall 	f15_pwm_pause

						movlb	14
						; switch to backup osc.
						movlw	OSCFRQ_backup
						movwf	s14_OSCFRQ,BANKED
						movlw	OSCCON1_backup
						movwf	s14_OSCCON1,BANKED
						retfie

__intr_motor:
						bcf		s14_PIR6,CCP1IF,BANKED

						; 2.4V,  	6-Speed HE,	no load
						;  900rpm	  19mA,		1.24mA
						; 1800rpm	  33mA, 	1.29mA
						;    0rpm	1209uA					; 1504uA@CR
						;    OFF	   1uA

						; 3.6V,  	6-Speed HE,	no load
						;  900rpm	  14mA,		1.45mA
						; 1800rpm	  24mA, 	1.50mA
						;    0rpm	1404uA					; 2114uA@CR
						;    OFF	   2uA

						; 5.4V,  	6-Speed HE,	no load
						;  900rpm	  10mA,		1.92mA
						; 1800rpm	  17mA, 	1.97mA
						;    0rpm	1840uA					; 1541uA@CR, turn over is 4.15V
						;    OFF	   3uA
						bcf		s14_PMD0,FVRMD,BANKED
						bcf		s14_PMD1,TMR3MD,BANKED
						bcf		s14_PMD2,ADCMD,BANKED

						; prepare FVR, ADC, and T3 to acquire VDD voltage
						movlw	ADRPT_FVR
						rcall	f15_prepare_adc_and_fvr

						; read keys and remove chattering
						rcall	read_keys					; 33

						rcall	get_notification			; 61
						xorwf	notification_flag,W
						movwf	s0a_notification_change
						xorwf	notification_flag,F

						; maintain 4Hz flag
						rcall	maintain_4hz				; 20

						; set default period into sampling_interval
						rcall	load_default_period			; 15

						; wait for completion of A/D convertion
						rcall	f15_load_adc_fvr_result
						TEST_PIN_ON	TEST_DUTY_UPDATE

						btfsc	s0a_notification_change,BIT_OPEN_LOOP_CTRL
						rcall	f15_init_open_loop_ctrl

						btfsc	s0a_notification_change,BIT_CLOSED_LOOP_CTRL
						rcall	init_closed_loop_ctrl

						btfsc	notification_flag,BIT_OUT_OF_CTRL
						rcall	f15_pwm_pause

						btfsc	notification_flag,BIT_OPEN_LOOP_CTRL
						rcall	open_loop_control

						btfsc	notification_flag,BIT_CLOSED_LOOP_CTRL
						rcall	f15_closed_loop_control

						TEST_PIN_OFF	TEST_DUTY_UPDATE

						; Advance lut index during last A/D conversion
						btfsc	notification_flag,BIT_ADVANCE_LUT_INDEX
						rcall	advance_lut_index
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rcall	advance_lut_index

						btfsc	notification_flag,BIT_CLOSED_LOOP_CTRL
						rcall	closed_loop_integration

						; battery voltage check
						btfss	notification_flag,BIT_POLLING
						bra		__skip_LED_control

						movlw	LOW (8192*1024/LOW_BATT_ERROR)
						btfss	s0a_low_batt_flag,0
						movlw	LOW (8192*1024/LOW_BATT_WARNING)
						subwf	d0a_adres_FVR_m7p23+0,W
						movlw	HIGH (8192*1024/LOW_BATT_ERROR)
						btfss	s0a_low_batt_flag,0
						movlw	HIGH (8192*1024/LOW_BATT_WARNING)
						subwfb	d0a_adres_FVR_m7p23+1,W
						rlcf	s0a_low_batt_flag,F

__skip_LED_control:
						btfsc	s0a_low_batt_flag,0
						bra		__led_turn_off

						movlb	14
	if 0
						movlw	OSCCON1_active
						xorwf	s14_OSCCON2,W,BANKED
						btfss	STATUS,Z
						bra		__led_turn_off
	else
						btfss	s14_OSCSTAT,EXTOR,BANKED
						bra		__led_turn_off			; External OSC is not working
						btfsc	s14_OSCSTAT,HFOR,BANKED
						bra		__led_turn_off			; HFINTOSC is working
						btfsc	s14_OSCSTAT,MFOR,BANKED
						bra		__led_turn_off			; MFINTOSC is working
						btfss	s14_OSCSTAT,LFOR,BANKED
						bra		__led_turn_off			; LFINTOSC is not working
						btfsc	s14_OSCSTAT,SOR,BANKED
						bra		__led_turn_off			; Secondary OSC is working
						btfsc	s14_OSCSTAT,ADOR,BANKED
						bra		__led_turn_off			; ADOSC is working
						btfss	s14_OSCSTAT,PLLR,BANKED
						bra		__led_turn_off			; PLL is not working
	endif

__led_turn_on:
						; prepare for LED brightness control, set MUX to LED
						bcf		s15a_TRISA,TRISA_LED_CATHODE		; LED is controlled by PWM

						movlb	15
						movlw	ADACT_LED
						movwf	s15_ADACT,BANKED
						movlw	ADCON2_BASIC
						movwf	s15_ADCON2,BANKED
						movlw	ADPCH_LED
						movwf	s15_ADPCH,BANKED
						movlw	ADCON0_init
						movwf	s15_ADCON0,BANKED

; read LED current and control constant brightness
;	Max LED current = 4V/1k = 4mA, duty ratio = 0.25/4 = 64/1024
						; Average Iled = 0.26mA, Iled*1000 = 256 at 100% duty
						; adres_FVR_work = adres_FVR_m7p23 << 3
						clrf	t0a_adres_FVR_work+2
	if 0
						bcf		STATUS,C
						rlcf	d0a_adres_FVR_m7p23+0,W
						movwf	t0a_adres_FVR_work+0
						rlcf	d0a_adres_FVR_m7p23+1,W
						movwf	t0a_adres_FVR_work+1
						rlcf	t0a_adres_FVR_work+0,F
						rlcf	t0a_adres_FVR_work+1,F
						rlcf	t0a_adres_FVR_work+0,F
						rlcf	t0a_adres_FVR_work+1,F
						rlcf	t0a_adres_FVR_work+0,F
						rlcf	t0a_adres_FVR_work+1,F
						rlcf	t0a_adres_FVR_work+2,F
	else
						movf	d0a_adres_FVR_m7p23+0,W
						mullw	0x10
						movf	PRODL,W
						movwf	t0a_adres_FVR_work+0
						movf	PRODH,W
						movwf	t0a_adres_FVR_work+1
						movf	d0a_adres_FVR_m7p23+1,W
						mullw	0x10
						movf	PRODL,W
						addwf	t0a_adres_FVR_work+1,F
						movf	PRODH,W
						addwfc	t0a_adres_FVR_work+2,F
	endif
						; adres_FVR_work = 131072*1024/Vin[mV]

						movlb	14
						bcf		s14_PIR1,ADIF,BANKED
						bsf		s14_PIE1,ADIF,BANKED
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADIF,BANKED

						movlb	15
						; read LED current
						comf	s15_ADRESL,W,BANKED
						movwf	d0a_led_current+0
						comf	s15_ADRESH,W,BANKED
						andlw	0x03
						movwf	d0a_led_current+1

						; Turn-OFF ADC
						clrf	s15_ADCON0,BANKED
						clrf	s15_ADACT,BANKED
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

__led_div_pos:			movf	d0a_led_current+0,W
						subwf	t0a_adres_FVR_work+1,F
						movf	d0a_led_current+1,W
						subwfb	t0a_adres_FVR_work+2,F
__led_div_common:		btfsc	STATUS,C				; output is positive logic
						bsf		pulse_width_L,5
						bcf		STATUS,C
						rlcf	pulse_width_L,F
						rlcf	pulse_width_H,F
						bc		__led_div_end
						rlcf	t0a_adres_FVR_work+0,F
						rlcf	t0a_adres_FVR_work+1,F
						rlcf	t0a_adres_FVR_work+2,F
						btfsc	pulse_width_L,6			; output is positive logic
						bra		__led_div_pos
__led_div_neg:			movf	d0a_led_current+0,W
						addwf	t0a_adres_FVR_work+1,F
						movf	d0a_led_current+1,W
						addwfc	t0a_adres_FVR_work+2,F
						bra		__led_div_common

__led_div_end:			; update duty register
						movff	pulse_width_L,s15a_PWM4DCL
						movff	pulse_width_H,s15a_PWM4DCH
						bra		__led_ctrl_end

__led_turn_off:			; halt LED brightness controll
						bsf		s15a_TRISA,TRISA_LED_CATHODE
						; fall through

__led_ctrl_end:			movlb	14
						; shutdown ADC, FVR, and Timer 3
						bsf		s14_PMD0,FVRMD,BANKED
						bsf		s14_PMD1,TMR3MD,BANKED
						bsf		s14_PMD2,ADCMD,BANKED
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Subroutine for Interrupt Routine														;
;		*		*		*		*		*		*		*		*		*		*		*		;

;		WREG	burst length
f15_prepare_adc_and_fvr:
						movwf	s15a_ADRPT

						movlb	15
						; power-up fixed voltage regulator for ADC
						movlw	FVRCON_init
						movwf	s15_FVRCON,BANKED

						; enter settling time of FVR
						clrf	s15a_T3CON
						movlw	T3CLK_init
						movwf	s15a_T3CLK
						movlw	LOW  (0x10000 - FVR_SETTLING_TIME*(FCY/1000000))
						movwf	s15a_T3TMRL
						movlw	HIGH (0x10000 - FVR_SETTLING_TIME*(FCY/1000000))
						movwf	s15a_T3TMRH
						movlw	T3CON_init
						movwf	s15a_T3CON

						; common settings
						movlw	ADCON1_init
						movwf	s15_ADCON1,BANKED
						movlw	ADCON3_init
						movwf	s15_ADCON3,BANKED
						movlw	ADCLK_init
						movwf	s15_ADCLK,BANKED
						movlw	ADPRE_init
						movwf	s15_ADPRE,BANKED
						movlw	ADACQ_init
						movwf	s15_ADACQ,BANKED
						movlw	ADCAP_init
						movwf	s15_ADCAP,BANKED
						movlw	ADREF_VDD_RATIOMETRIC
						movwf	s15_ADREF,BANKED

						; configure ADC for input voltage acquisition
						movlw	ADACT_T3TMR
						movwf	s15_ADACT,BANKED
						movlw	ADCON0_init
						movwf	s15_ADCON0,BANKED
						movlw	ADCON2_BURST_AVERAGE
						movwf	s15_ADCON2,BANKED
						movlw	ADPCH_FVR
						movwf	s15_ADPCH,BANKED
						return

f15_load_adc_fvr_result:
						movlb	14
						bcf		s14_PIR1,ADTIF,BANKED
						bsf		s14_PIE1,ADTIF,BANKED
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADTIF,BANKED

						movlb	15
						; turn-OFF FVR
						clrf	s15_FVRCON,BANKED

						; turn-OFF Timer 3
						clrf	s15a_T3CON

						movf	s15a_ADACCL,W
						movwf	d0a_adres_FVR_m7p23+0
						movf	s15a_ADACCH,W
						movwf	d0a_adres_FVR_m7p23+1

						; turn-OFF ADC
						clrf	s15_ADCON0,BANKED
						clrf	s15_ADACT,BANKED
						return

load_default_period:
						clrf	s15a_CCP1CON
						movlw	LOW  (OPEN_LOOP_PERIOD - 1)
						movwf	s15a_CCPR1L
						movlw	HIGH (OPEN_LOOP_PERIOD - 1)
						movwf	s15a_CCPR1H
						movlw	CCP1CON_init
						movwf	s15a_CCP1CON				;  7

						movlw	LOW  (OPEN_LOOP_PERIOD)
						movwf	cemf_sampling_period_L
						movlw	HIGH (OPEN_LOOP_PERIOD)
						movwf	cemf_sampling_period_H
						return								;  6	13

; PWM module is temporary shutdown
;	 input: none
;	output: duty ratio register in PWM module with duty=0%
f15_pwm_pause:			movlb	15
						; disconnect power from motor
						bcf		s15_CM1CON0,C1POL,BANKED

						; stop PWM timer
						bcf		s15a_T2CON,TMR2ON
						movf	s15a_T2PR,W
						movwf	s15a_T2TMR

						; resume half-bridge operation at next cycle
						bsf		s15_CM1CON0,C1POL,BANKED	; turn-ON HB2 LS FET 

						; set duty ratio to zero
						clrf	s15a_CCPR2L
						clrf	s15a_CCPR2H
						return

; closed_loop_control updates pulse_width to drive motor with constant speed
init_closed_loop_ctrl:
						clrf	t0a_current_cemf_6p18+0
						clrf	t0a_current_cemf_6p18+1
						clrf	t0a_current_cemf_6p18+2
						; fall through
clear_integral:
						clrf	t0a_integral_6p18+0
						clrf	t0a_integral_6p18+1
						clrf	t0a_integral_6p18+2
						return

f15_closed_loop_control:
						; load lut entry into sampling_interval
						rcall	nvm_load_lut_entry

						; save duty
						movf	s15a_CCPR2L,W
						movwf	pulse_width_L
						movf	s15a_CCPR2H,W
						movwf	pulse_width_H

						; set duty to 0%
						clrf	s15a_CCPR2L
						clrf	s15a_CCPR2H

						movlb	15
						movf	s15a_T2PR,W
						movwf	s15a_T2TMR					; force update duty register

						bsf		pulse_width_L,0

						; stop PWM timer
						bcf		s15a_T2CON,TMR2ON
						movwf	s15a_T2TMR

						; restore duty
						movf	pulse_width_L,W
						movwf	s15a_CCPR2L
						movf	pulse_width_H,W
						movwf	s15a_CCPR2H

						; disconnect power from motor
						bcf		s15_CM1CON0,C1POL,BANKED

						; turn on bias resister
						bsf		s15_WPUB,WPUB_CEMF_SENSE,BANKED

						; turn on input buffer
						bcf		s15_ANSELB,ANSELB_CEMF_SENSE,BANKED

						; wait for end of fast decay discharge
						btfss	s15a_PORTB,PORTB_CEMF_SENSE
						bra		$-2
						btfss	s15a_PORTB,PORTB_CEMF_SENSE
						bra		$-2
						btfss	s15a_PORTB,PORTB_CEMF_SENSE
						bra		$-2

						; turn off bias resister
						bcf		s15_WPUB,WPUB_CEMF_SENSE,BANKED

						; turn off input buffer
						bsf		s15_ANSELB,ANSELB_CEMF_SENSE,BANKED

						; resume half-bridge operation at next cycle
						bsf		s15_CM1CON0,C1POL,BANKED	; turn-ON HB2 LS FET 

						; enter settling time of CEMF
						clrf	s15a_T3CON
						movlw	T3CLK_init
						movwf	s15a_T3CLK
						movlw	LOW  (0x10000 - CEMF_SETTLING_TIME*(FCY/1000000))
						movwf	s15a_T3TMRL
						movlw	HIGH (0x10000 - CEMF_SETTLING_TIME*(FCY/1000000))
						movwf	s15a_T3TMRH
						movlw	T3CON_init
						movwf	s15a_T3CON

						; configure ADC for CEMF acquisition
						movlw	ADACT_T3TMR
						movwf	s15_ADACT,BANKED
						movlw	ADCON2_BURST_AVERAGE
						movwf	s15_ADCON2,BANKED
						movlw	ADPCH_MOTOR
						movwf	s15_ADPCH,BANKED
						movlw	ADCON0_init
						movwf	s15_ADCON0,BANKED
						movlw	(P_GAIN*PWM_PERIOD+128)/256
						movwf	s15a_ADRPT

						; calculate target_voltage from sampling_interval
						rcall	calc_target_voltage

						; calculate target ratio from target voltage
						rcall	calc_target_ratio

						TEST_PIN_OFF	TEST_DUTY_UPDATE

						; proportional_voltage = P_GAIN*target voltage - P_GAIN*ADRES
						movff	t0a_target_ratio_6p18+0,t0a_proportional_6p18+0
						movff	t0a_target_ratio_6p18+1,t0a_proportional_6p18+1
						movff	t0a_target_ratio_6p18+2,t0a_proportional_6p18+2

	if (CALC_SAMPLES_1X != 0) || (CALC_SAMPLES_2X != 0)
		if (CALC_SAMPLES_1X == 0)
			if (CALC_SAMPLES_2X == 1)	; averaging
						clrf	s0a_sampling_phase
			endif
			if (CALC_SAMPLES_2X == 2)	; phase control
						comf	s0a_sampling_phase,F
			endif
		endif
		if (CALC_SAMPLES_1X == 1)		; averaging
			if (CALC_SAMPLES_2X == 2)	; phase control
						comf	s0a_sampling_phase,F
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
			endif
						clrf	s0a_sampling_phase
		endif
		if (CALC_SAMPLES_1X == 2)		; phase control
						comf	s0a_sampling_phase,F
			if (CALC_SAMPLES_2X == 1)	; averaging
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						clrf	s0a_sampling_phase
			endif
		endif
						clrf	t0a_previous_cemf_6p18+0
						movf	t0a_current_cemf_6p18+1,W
						movwf	t0a_previous_cemf_6p18+1
						movf	t0a_current_cemf_6p18+2,W
						movwf	t0a_previous_cemf_6p18+2
	endif
						movf	t0a_integral_6p18+1,W
						movwf	pulse_width_L
						movf	t0a_integral_6p18+2,W
						movwf	pulse_width_H

						movlw	(P_GAIN*PWM_PERIOD+128)/256-1
						movwf	s0a_samples
__cemf_reference_lp:	movf	t0a_target_ratio_6p18+0,W
						addwf	t0a_proportional_6p18+0,F
						movf	t0a_target_ratio_6p18+1,W
						addwfc	t0a_proportional_6p18+1,F
						movf	t0a_target_ratio_6p18+2,W
						addwfc	t0a_proportional_6p18+2,F

						decfsz	s0a_samples,F
						bra		__cemf_reference_lp

__cemf_last_acq_wait:
						movlb	14
						bcf		s14_PIR1,ADIF,BANKED
						bsf		s14_PIE1,ADIE,BANKED
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADIE,BANKED

						; wait for last acquisition start
						movlw	(P_GAIN*PWM_PERIOD+128)/256-1
						cpfseq	s15a_ADCNT
						bra		__cemf_last_acq_wait

						; wait for last conversion start
						btfss	s15a_ADSTAT,0
						bra		$-2

						nop
						; restart PWM generator after ADC sampling end
						bsf		s15a_T2CON,TMR2ON
						TEST_PIN_ON	TEST_DUTY_UPDATE

						; turn-OFF Timer 3
						clrf	s15a_T3CON

						movlb	14
						; wait for last A/D convertion completion
						bcf		s14_PIR1,ADTIF,BANKED
						bsf		s14_PIE1,ADTIF,BANKED
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bcf		s14_PIE1,ADTIF,BANKED		; 46

						movlb	15
						; proportional = target - current_cemf
						movf	s15a_ADACCL,W
						movwf	t0a_current_cemf_6p18+1
						subwf	t0a_proportional_6p18+1,F
						movf	s15a_ADACCH,W
						movwf	t0a_current_cemf_6p18+2
						subwfb	t0a_proportional_6p18+2,F	; 7

						; turn-OFF ADC
						clrf	s15_ADCON0,BANKED
						clrf	s15_ADACT,BANKED			; 2

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
						movf	t0a_current_cemf_6p18+1,W
						subwf	t0a_previous_cemf_6p18+1,F
						movf	t0a_current_cemf_6p18+2,W
						subwfb	t0a_previous_cemf_6p18+2,F

						rlcf	t0a_previous_cemf_6p18+2,W
						rrcf	t0a_previous_cemf_6p18+2,F
						rrcf	t0a_previous_cemf_6p18+1,F

						; proportional = (target - current) - (previous - current)/2 = target -   current/2 - previous/2
						; proportional = (target - current) + (previous - current)/2 = target - 3*current/2 + previous/2
						rlcf	s0a_sampling_phase,W
						rrcf	s0a_sampling_phase,W			; 0x00 or 0xFF
						xorwf	t0a_previous_cemf_6p18+1,F
						xorwf	t0a_previous_cemf_6p18+2,F

						movf	t0a_previous_cemf_6p18+1,W
						addwfc	t0a_proportional_6p18+1,F
						movf	t0a_previous_cemf_6p18+2,W
						addwfc	t0a_proportional_6p18+2,F	; 17
						; integral += target -   cemf(1)/2 - cemf(0)/2
						; integral += target - 3*cemf(2)/2 + cemf(1)/2
						; integral += target -   cemf(3)/2 - cemf(2)/2
						; integral += target - 3*cemf(4)/2 + cemf(3)/2
__pi_calc_pi:
	endif
						; control = integral + proportional
						movf	t0a_proportional_6p18+1,W
						addwf	pulse_width_L,F
						movf	t0a_proportional_6p18+2,W
						addwfc	pulse_width_H,F				; 4

						btfsc	pulse_width_H,7
						bra		f15_pwm_pause				; 2	call and return

						; limit positive duty
						movf	t0a_startup_ratio_6p18+1,W
						subwf	pulse_width_L,W
						movf	t0a_startup_ratio_6p18+2,W
						subwfb	pulse_width_H,W

						movf	t0a_startup_ratio_6p18+1,W
						btfsc	STATUS,C
						movwf	pulse_width_L
						movf	t0a_startup_ratio_6p18+2,W
						btfsc	STATUS,C
						movwf	pulse_width_H				; 10	88

pwm_by_duty_ratio:
						; saturate calculation
						comf	pulse_width_H,W				; expect B'111111xx' 
						addlw	0x04
						subwfb	WREG,W						; W = -1 +C
						iorwf	pulse_width_L,F
						iorwf	pulse_width_H,F				; +5	93

						; pulse_width_H         pulse_width_L
						; 0,0,0,0,0,0,d9,d8     d7,d6,d5,d4,d3,d2,d1,d0
						movf	pulse_width_H,W
						movwf	s15a_CCPR2H
						movf	pulse_width_L,W
						movwf	s15a_CCPR2L

						return								; +6	99

						; integration
closed_loop_integration:
	if (TEST_TARGET == TEST_UART_CEMF)
					movf	t0a_current_cemf_17p7+1,W
					movwf	INDF1
					movf	t0a_current_cemf_17p7+2,W
					movwf	INDF1
	endif
	if (TEST_TARGET == TEST_UART_P)
					movf	t0a_proportional_17p7+1,W
					movwf	INDF1
					movf	t0a_proportional_17p7+2,W
					movwf	INDF1
	endif
	if (MULT_ITIME_1X_24p8 == 128)
I_GAIN_fxp1p8			set		((512<<8)/I_TIME_8p8)&0x1FF
		if (256 <= MULT_ITIME_2X_24p8)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		__integral_1x
						rlcf	t0a_proportional_6p18+2,W
						rrcf	t0a_proportional_6p18+2,F
						rrcf	t0a_proportional_6p18+1,F
						rrcf	t0a_proportional_6p18+0,F
			if (512 <= MULT_ITIME_2X_24p8)
						rlcf	t0a_proportional_6p18+2,W
						rrcf	t0a_proportional_6p18+2,F
						rrcf	t0a_proportional_6p18+1,F
						rrcf	t0a_proportional_6p18+0,F
			endif
__integral_1x:
		endif
	endif
	if (MULT_ITIME_1X_24p8 == 256)
		if (MULT_ITIME_2X_24p8 == 128)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		$+10
						bcf		STATUS,C
						rlcf	t0a_proportional_6p18+0,F
						rlcf	t0a_proportional_6p18+1,F
						rlcf	t0a_proportional_6p18+2,F
		endif
I_GAIN_fxp1p8			set		((256<<8)/I_TIME_8p8)&0x1FF
		if (MULT_ITIME_2X_24p8 == 512)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		$+10
						rlcf	t0a_proportional_6p18+2,W
						rrcf	t0a_proportional_6p18+2,F
						rrcf	t0a_proportional_6p18+1,F
						rrcf	t0a_proportional_6p18+0,F
		endif
	endif
	if (MULT_ITIME_1X_24p8 == 512)
		if (MULT_ITIME_2X_24p8 <= 256)
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						bra		__integral_1x
						bcf		STATUS,C
						rlcf	t0a_proportional_6p18+0,F
						rlcf	t0a_proportional_6p18+1,F
						rlcf	t0a_proportional_6p18+2,F
			if (MULT_ITIME_2X_24p8 <= 128)
						bcf		STATUS,C
						rlcf	t0a_proportional_6p18+0,F
						rlcf	t0a_proportional_6p18+1,F
						rlcf	t0a_proportional_6p18+2,F
			endif
__integral_1x:
		endif
I_GAIN_fxp1p8			set		((128<<8)/I_TIME_8p8)&0x1FF
	endif
						movlw	I_GAIN_fxp1p8
						mulwf	t0a_proportional_6p18+0
						movf	PRODH,W
						addwf	t0a_integral_6p18+0,F
						clrf	WREG
						addwfc	t0a_integral_6p18+1,F
						clrf	WREG
						addwfc	t0a_integral_6p18+2,F
						movlw	I_GAIN_fxp1p8
						mulwf	t0a_proportional_6p18+1
						movf	PRODL,W
						addwf	t0a_integral_6p18+0,F
						movf	PRODH,W
						addwfc	t0a_integral_6p18+1,F
						clrf	WREG
						addwfc	t0a_integral_6p18+2,F
						movlw	I_GAIN_fxp1p8
						mulwf	t0a_proportional_6p18+2
						movf	PRODL,W
						addwf	t0a_integral_6p18+1,F
						movf	PRODH,W
						addwfc	t0a_integral_6p18+2,F

						movlw	I_GAIN_fxp1p8
						btfsc	t0a_proportional_6p18+2,7
						subwf	t0a_integral_6p18+2,F
	if (TEST_TARGET == TEST_UART_I)
					movf	t0a_integral_6p18+1,W
					movwf	INDF1
					movf	t0a_integral_6p18+2,W
					movwf	INDF1
	endif

						; if integral voltage is negative, set to 0
						btfsc	t0a_integral_6p18+2,7
						bra		clear_integral		; call and return

						; limit positive duty
						movf	t0a_startup_ratio_6p18+1,W
						subwf	t0a_integral_6p18+1,W
						movf	t0a_startup_ratio_6p18+2,W
						subwfb	t0a_integral_6p18+2,W
						btfss	STATUS,C
						return
						clrf	t0a_integral_6p18+0
						movf	t0a_startup_ratio_6p18+1,W
						movwf	t0a_integral_6p18+1
						movf	t0a_startup_ratio_6p18+2,W
						movwf	t0a_integral_6p18+2
						return

; open_loop_control updates pulse_width to drive motor with constant voltage
f15_init_open_loop_ctrl: movlb	15
						; connect
						bsf		s15_CM1CON0,C1POL,BANKED

						; restart PWM timer
						btfss	s15a_T2CON,TMR2ON
						bsf		s15a_T2CON,TMR2ON

						movlw	BREAKING_IN_DURATION*POLLING_FREQUENCY-1
						movwf	s0a_breaking_in_time
						movlw	OPEN_LOOP_FREQ
						movwf	s0a_sampled_waves
						clrf	t0a_integral_6p18+0
						clrf	t0a_integral_6p18+1
						return

open_loop_control:
						; slewing
OPEN_LOOP_VOLTAGE_13p3	equ		PWM_PERIOD*OPEN_LOOP_VOLTAGE_mV/32/OPEN_LOOP_FREQ
						; open_loop_voltage *= pwm_period
						;         16p8 >> 5           0p8
						movf	s0a_sampled_waves,F
						bz		$+12
						decf	s0a_sampled_waves,F
						movlw	LOW  OPEN_LOOP_VOLTAGE_13p3
						addwf	t0a_integral_6p18+0,F
						movlw	HIGH OPEN_LOOP_VOLTAGE_13p3
						addwfc	t0a_integral_6p18+1,F

						; startup_ratio = open_loop_voltage(16bit) * FVR(13bit)
						;    6p26 >> 16                13p3        m7p23
						movf	d0a_adres_FVR_m7p23+0,W
						mulwf	t0a_integral_6p18+0
						movf	PRODH,W
						movwf	t0a_startup_ratio_6p18+0

						movf	d0a_adres_FVR_m7p23+1,W
						mulwf	t0a_integral_6p18+1
						movf	PRODL,W
						movwf	pulse_width_L
						movf	PRODH,W
						movwf	pulse_width_H

						movf	d0a_adres_FVR_m7p23+0,W
						mulwf	t0a_integral_6p18+1
						movf	PRODL,W
						addwf	t0a_startup_ratio_6p18+0,F
						movf	PRODH,W
						addwfc	pulse_width_L,F
						clrf	WREG
						addwfc	pulse_width_H,F

						movf	d0a_adres_FVR_m7p23+1,W
						mulwf	t0a_integral_6p18+0
						movf	PRODL,W
						addwf	t0a_startup_ratio_6p18+0,F
						movf	PRODH,W
						addwfc	pulse_width_L,F
						clrf	WREG
						addwfc	pulse_width_H,F

						btfsc	notification_flag,BIT_POLLING
						decfsz	s0a_breaking_in_time,F
						bra		pwm_by_duty_ratio	; break-in, call and return
						; fall through

__calib_begin:
						; set duty to 0%
						clrf	s15a_CCPR2L
						clrf	s15a_CCPR2H

						movf	s15a_T2PR,W
						movwf	s15a_T2TMR					; force update duty register

CALIB_ADC_INTERVAL		equ		18
END_OF_CALIB_VOLTAGE	equ		256
CALIB_WAVES				equ		24
CALIB_COEFFICIENT_8p8	equ		256*FCY_SCALED/1000000*CENTER_WAVE_HEIGHT/CALIB_WAVES*CALIB_ADC_INTERVAL/MEAN_WAVE_HEIGHT
						; since measured value is an arithmetic average, parameter is corrected from it to center.

						; set Timer to discharge time
						clrf	s15a_T1CON
						movlw	HIGH (0x10000-(500-CALIB_ADC_INTERVAL)*(FCY_SCALED/1000000))
						movwf	s15a_T1TMRH
						movlw	LOW  (0x10000-(500-CALIB_ADC_INTERVAL)*(FCY_SCALED/1000000))
						movwf	s15a_T1TMRL
						movlw	T1CON_init
						movwf	s15a_T1CON					; start 500us later

						; set CCP to A/D conversion interval
						clrf	s15a_CCP1CON
						movlw	LOW	 (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s15a_CCPR1L
						movlw	HIGH (CALIB_ADC_INTERVAL*(FCY_SCALED/1000000))
						movwf	s15a_CCPR1H
						movlw	CCP1CON_init
						movwf	s15a_CCP1CON

						; halt half-bridge operation
						bcf		s15_CWG1AS0,REN,BANKED
						bsf		s15_CWG1AS0,SHUTDOWN,BANKED

						; halt LED brightness controll
						bsf		s15a_TRISA,TRISA_LED_CATHODE

						movlb	14
						; change WDT period
						movlw	WDTCON0_calibration
						movwf	s14_WDTCON0,BANKED

						movlb	15
						; set ADC to basic mode using CCP trigger
						movlw	ADACT_CCP1
						movwf	s15_ADACT,BANKED
						movlw	ADCON2_BASIC
						movwf	s15_ADCON2,BANKED
						movlw	ADPCH_MOTOR
						movwf	s15_ADPCH,BANKED
						movlw	ADCON0_init
						movwf	s15_ADCON0,BANKED

						; clear data memory used to hold c-emf values
						lfsr	0,0x0100
						clrf	INDF0
						incfsz	FSR0L,F						; with wrap-round
						bra		$-4

						; initialize variables
						clrf	d0a_cemf_mean_14p2+0
						movlw	HIGH (END_OF_CALIB_VOLTAGE << 2)
						movwf	d0a_cemf_mean_14p2+1

						clrf	d0a_cemf_min_14p2+0
						clrf	d0a_cemf_min_14p2+1

						clrf	d0a_cemf_range_14p2+0
						clrf	d0a_cemf_range_14p2+1

						clrf	s0a_sampled_waves
						clrf	s0a_sampling_flag

						clrf	q0a_motor_constant_work+0
						clrf	q0a_motor_constant_work+1
						clrf	q0a_motor_constant_work+2
						clrf	q0a_motor_constant_work+3

						bra		__calib_lp

__calib_lp_decreasing:	; decreasing in CEMF voltage		; cemf_mean < cemf_min
						TEST_PIN_ON		TEST_CALIBRATION
						bsf		s0a_sampling_flag,1			; set bit1 as minimum update

						clrwdt
						movff	q0a_motor_constant_work+0,POSTINC0
						movff	q0a_motor_constant_work+1,POSTINC0
						movff	q0a_motor_constant_work+2,POSTINC0
						movff	q0a_motor_constant_work+3,POSTDEC0
						movlw	0x02
						subwf	FSR0L,F						; 12/  	48/

__calib_lp_update:		movf	d0a_cemf_work_14p2+0,W
						addwf	d0a_cemf_min_14p2+0,F
						movf	d0a_cemf_work_14p2+1,W
						addwfc	d0a_cemf_min_14p2+1,F		; 4		52/52

__calib_lp:				; wait for start of A/D convertion
						btfss	s15_ADCON0,ADGO,BANKED
						bra		$-2

						; IIR LPF, time constant is 8 periods
						movf	d0a_cemf_range_14p2+0,W
						subwf	d0a_cemf_mean_14p2+0,F
						movf	d0a_cemf_range_14p2+1,W
						subwfb	d0a_cemf_mean_14p2+1,F		; 6		58/58

						; wait for completion of A/D convertion
						btfsc	s15_ADCON0,ADGO,BANKED		;		0
						bra		$-2

						movf	s15_ADRESL,W,BANKED
						addwf	d0a_cemf_mean_14p2+0,F
						movf	s15_ADRESH,W,BANKED
						addwfc	d0a_cemf_mean_14p2+1,F

						movf	d0a_cemf_mean_14p2+1,W
						addlw	HIGH (0x10000 - (END_OF_CALIB_VOLTAGE << 2))
						bnc		__calib_end

						movf	s15_ADRESL,W,BANKED
						addwf	q0a_motor_constant_work+0,F
						movf	s15_ADRESH,W,BANKED
						addwfc	q0a_motor_constant_work+1,F
						clrf	WREG
						addwfc	q0a_motor_constant_work+2,F
						addwfc	q0a_motor_constant_work+3,F	; 16

						; max-min is kept within 1/8 of mean value
						bcf		STATUS,C
						rrcf	d0a_cemf_mean_14p2+1,W
						movwf	d0a_cemf_range_14p2+1
						rrcf	d0a_cemf_mean_14p2+0,W
						movwf	d0a_cemf_range_14p2+0
						bcf		STATUS,C
						rrcf	d0a_cemf_range_14p2+1,F
						rrcf	d0a_cemf_range_14p2+0,F
						bcf		STATUS,C
						rrcf	d0a_cemf_range_14p2+1,F
						rrcf	d0a_cemf_range_14p2+0,F		; 11

						; test for minimum
						movf	d0a_cemf_min_14p2+0,W
						subwf	d0a_cemf_mean_14p2+0,W
						movwf	d0a_cemf_work_14p2+0
						movf	d0a_cemf_min_14p2+1,W
						subwfb	d0a_cemf_mean_14p2+1,W
						movwf	d0a_cemf_work_14p2+1
						bnc		__calib_lp_decreasing		;  8/ 7	35/34

						; test for maximum
						movf	d0a_cemf_range_14p2+0,W
						subwf	d0a_cemf_work_14p2+0,F
						movf	d0a_cemf_range_14p2+1,W
						subwfb	d0a_cemf_work_14p2+1,F
						bnc		__calib_lp					;   / 5	  /39

__calib_lp_increasing:	; increasing in CEMF voltage		; cemf_max < cemf_mean
						TEST_PIN_OFF	TEST_CALIBRATION
						btfss	s0a_sampling_flag,1
						bra		__calib_lp_update
						bcf		s0a_sampling_flag,1			; clear bit1 as maximum update

						movlw	0x04
						addwf	FSR0L,F
						btfsc	STATUS,C
						incf	s0a_sampled_waves,F
						bra		__calib_lp_update			;  / 9	 /48

__calib_end:
						; Turn-OFF periodic timer
						clrf	s15a_CCP1CON
						clrf	s15a_T1CON

						movlb	14
						; Disable interrupts
						bcf		s14_PIE1,OSCFIE,BANKED
						bcf		s14_PIE6,CCP1IE,BANKED

						movlb	15
						; Turn-OFF ADC
						clrf	s15_ADACT,BANKED
						clrf	s15_ADCON0,BANKED

						movf	s0a_sampled_waves,F
						btfsc	STATUS,Z
						rcall	nvm_set_factory_default		; call but never return

						; prepare FVR, ADC, and T3 to acquire VDD voltage
						movlw	0x20
						rcall	f15_prepare_adc_and_fvr

						clrwdt

						; Equivalent Time Sampling
						decf	FSR0L,F						; wrap-around
						; load the latest snap shot
						movf	POSTDEC0,W
						movwf	motor_constant_HH
						movf	POSTDEC0,W
						movwf	motor_constant_HL
						movf	POSTDEC0,W
						movwf	motor_constant_LH
						movf	INDF0,W
						movwf	motor_constant_LL

						movlw	-4*CALIB_WAVES
						addwf	FSR0L,F						; wrap-around
						; get motor_constant samples by subtraction
						movf	POSTINC0,W
						subwf	motor_constant_LL,F
						movf	POSTINC0,W
						subwfb	motor_constant_LH,F
						movf	POSTINC0,W
						subwfb	motor_constant_HL,F
						movf	POSTINC0,W
						subwfb	motor_constant_HH,F

						clrf	q0a_motor_constant_work+0
						clrf	q0a_motor_constant_work+1
						clrf	q0a_motor_constant_work+2
						clrf	q0a_motor_constant_work+3

						movlw	LOW  CALIB_COEFFICIENT_8p8
						movwf	d0a_cemf_work_14p2+0
						movlw	HIGH CALIB_COEFFICIENT_8p8
						movwf	d0a_cemf_work_14p2+1

						movlw	0x08
						movwf	s0a_sampled_waves
__calc_motor_const_lp1:	rrncf	d0a_cemf_work_14p2+0,F
						btfss	d0a_cemf_work_14p2+0,7
						bra		$+18
						movf	motor_constant_LL,W
						addwf	q0a_motor_constant_work+0,F
						movf	motor_constant_LH,W
						addwfc	q0a_motor_constant_work+1,F
						movf	motor_constant_HL,W
						addwfc	q0a_motor_constant_work+2,F
						movf	motor_constant_HH,W
						addwfc	q0a_motor_constant_work+3,F

						bcf		STATUS,C
						rrcf	q0a_motor_constant_work+3,F
						rrcf	q0a_motor_constant_work+2,F
						rrcf	q0a_motor_constant_work+1,F
						rrcf	q0a_motor_constant_work+0,F

						decfsz	s0a_sampled_waves,F
						bra		__calc_motor_const_lp1

						movlw	0x08
						movwf	s0a_sampled_waves
__calc_motor_const_lp2:	rrncf	d0a_cemf_work_14p2+1,F
						btfss	d0a_cemf_work_14p2+1,7
						bra		$+18
						movf	motor_constant_LL,W
						addwf	q0a_motor_constant_work+0,F
						movf	motor_constant_LH,W
						addwfc	q0a_motor_constant_work+1,F
						movf	motor_constant_HL,W
						addwfc	q0a_motor_constant_work+2,F
						movf	motor_constant_HH,W
						addwfc	q0a_motor_constant_work+3,F

						bcf		STATUS,C
						rlcf	motor_constant_LL,F
						rlcf	motor_constant_LH,F
						rlcf	motor_constant_HL,F
						rlcf	motor_constant_HH,F

						decfsz	s0a_sampled_waves,F
						bra		__calc_motor_const_lp2

						; wait for completion of A/D convertion
						rcall	f15_load_adc_fvr_result

						; 
						clrf	motor_constant_LL
						clrf	motor_constant_LH
						clrf	motor_constant_HL
						clrf	motor_constant_HH
						bsf		motor_constant_LL,1	; sentinel
				;		bra		__calib_div_begin

__calib_div_pos:		movf	d0a_adres_FVR_m7p23+0,W
						subwf	q0a_motor_constant_work+2,F
						movf	d0a_adres_FVR_m7p23+1,W
						subwfb	q0a_motor_constant_work+3,F
__calib_div_common:		rlcf	motor_constant_LL,F
						rlcf	motor_constant_LH,F
						rlcf	motor_constant_HL,F
						rlcf	motor_constant_HH,F
						btfsc	STATUS,C
						bra		__calib_div_end
__calib_div_begin:		bcf		STATUS,C
						rlcf	q0a_motor_constant_work+0,F
						rlcf	q0a_motor_constant_work+1,F
						rlcf	q0a_motor_constant_work+2,F
						rlcf	q0a_motor_constant_work+3,F
						btfsc	motor_constant_LL,0
						bra		__calib_div_pos
__calib_div_neg:		movf	d0a_adres_FVR_m7p23+0,W
						addwf	q0a_motor_constant_work+2,F
						movf	d0a_adres_FVR_m7p23+1,W
						addwfc	q0a_motor_constant_work+3,F
						bra		__calib_div_common

__calib_div_end:
						movlb	14
						; abort calibration when oscillator is faulty.
						btfsc	s14_PIR1,OSCFIF,BANKED
						reset

						rcall	nvm_update_parameter		; call but never return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Common entry point (cold boot, wake up from sleep, fall into sleep)						;
;		*		*		*		*		*		*		*		*		*		*		*		;
common_entry_point:
						movlb	14
						; speed-up HFINTOSC for backup.
						movlw	OSCFRQ_backup
						movwf	s14_OSCFRQ,BANKED
						movlw	OSCCON1_backup
						movwf	s14_OSCCON1,BANKED

						movlb	15
						; enable pull-up
						movlw	WPUA_init
						movwf	s15_WPUA,BANKED
						movlw	WPUB_init
						movwf	s15_WPUB,BANKED
						movlw	WPUC_init
						movwf	s15_WPUC,BANKED

						; analyze reset
						movf	s15a_PCON0,W
						movwf	notification_flag
						movlw	PCON_init
						movwf	s15a_PCON0

						TEST_LED_INIT
						TEST_PIN_ON	TEST_CPU_USAGE

						; clear data memory
						lfsr	0,bss_start

						clrf	POSTINC0
						clrf	POSTINC0
						btfss	FSR0H,1
						bra		$-6

						clrf	s15a_T1CON
						clrf	s15a_T3CON

						movlb	15
						movlw	INLVLA_init
						movwf	s15_INLVLA,BANKED
						movlw	INLVLB_init
						movwf	s15_INLVLB,BANKED
						movlw	INLVLC_init
						movwf	s15_INLVLC,BANKED

						; enable input buffer on PORTB
						movlw	ANSELB_init
						movwf	s15_ANSELB,BANKED

						; set default state when the port is cofigured for generic use.
						movlw	LATA_init
						movwf	s15a_LATA
						movlw	LATB_init
						movwf	s15a_LATB
						movlw	LATC_init
						movwf	s15a_LATC

						btfss	notification_flag,NOT_POR
						bra		cold_boot						; Power on reset

						; wait for RE3/MCLR release when debugger is attached
						movf	s15a_PORTB,W
						iorwf	s15a_PORTE,W
						andlw	(1<<BIT_SW_RESET)|(1<<BIT_SW_ORIGIN)|(1<<BIT_SW_LIMIT)
						btfsc	STATUS,Z
						bra		$-8
						
						; hold cemf sense pin to stable state
						bcf		s15a_TRISB,TRISB_CEMF_SENSE

						; don't wake-up by RB6 or RB7 when debugger is attached.
						andlw	(1<<BIT_SW_ORIGIN)|(1<<BIT_SW_LIMIT)
						movlw	LOW (~(1<<BIT_SW_ORIGIN))
						btfsc	STATUS,Z
						movlw	0xFF

						movwf	s15_ANSELB,BANKED
						comf	WREG,W
						clrf	s15_WPUA,BANKED	; All pullups on PORTA are disabled
						movwf	s15_WPUB,BANKED
						clrf	s15_WPUC,BANKED	; All pullups on PORTC are disabled

						rcall	nvm_save_lut_index

						movlb	14
						; change WDT period
						movlw	WDTCON0_sleep
						movwf	s14_WDTCON0,BANKED

						; speed-down HFINTOSC for sleep
						movlw	OSCCON1_sleep
						movwf	s14_OSCCON1,BANKED
						movlw	OSCFRQ_sleep
						movwf	s14_OSCFRQ,BANKED

						movlw	VREGCON_sleep
						movwf	s14_VREGCON,BANKED
__sleep_lp:				movlb	15
						movf	s15a_PORTB,W				; negative level
						iorwf	s15a_PORTE,W

						btfss	WREG,BIT_SW_RESET		; is Power button pressed ?
						bra		__warm_reset
						btfss	s15_ANSELB,BIT_SW_ORIGIN,BANKED	; is buffer for origin SW active ?
						btfsc	WREG,BIT_SW_ORIGIN		; is origin SW pressed ?
						bra		__short_sleep

						bsf		s15_ANSELB,BIT_SW_ORIGIN,BANKED
						bcf		s15_WPUB,BIT_SW_ORIGIN,BANKED
						rcall	set_idx_to_lut_start
						rcall	nvm_save_lut_index

__short_sleep:			TEST_PIN_OFF	TEST_CPU_USAGE
						sleep				; method to wake-up is assertion of MCLR, POR or WDT
						TEST_PIN_ON	TEST_CPU_USAGE
						bra		__sleep_lp

__warm_reset:
						movlw	PCON_cold_boot	; merging into cold boot sequence 
						movwf	s15a_PCON0
						reset					; due to unstable condition while sleeping

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Main routine																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
cold_boot:				clrf	notification_flag

						movlb	14
		if TEST_TARGET == TEST_NONE
						; disable unused modules
						movlw	PMD0_init
						movwf	s14_PMD0,BANKED
						movlw	PMD1_init
						movwf	s14_PMD1,BANKED
						movlw	PMD2_init
						movwf	s14_PMD2,BANKED
						movlw	PMD3_init
						movwf	s14_PMD3,BANKED
						movlw	PMD4_init
						movwf	s14_PMD4,BANKED
						movlw	PMD5_init
						movwf	s14_PMD5,BANKED
		endif

						; restore lut_index from DATA EEPROM
						rcall	restore_lut_index

						movlb	15
						; disconnect power from motor
						movlw	CM1CON0_init
						movwf	s15_CM1CON0,BANKED

						; timer assignment
						movlw	CCPTMRS_init
						movwf	s15a_CCPTMRS

						; setup PWM for motor control 
						clrf	s15a_CCPR2L
						clrf	s15a_CCPR2H
						movlw	CCP2CON_init
						movwf	s15a_CCP2CON

						; halfbridge
						movlw	CWG1ISM_init
						movwf	s15_CWG1ISM,BANKED
						movlw	CWG1DBR_init
						movwf	s15_CWG1DBR,BANKED
						movlw	CWG1DBF_init
						movwf	s15_CWG1DBF,BANKED
						movlw	CWG1CON0_init
						movwf	s15_CWG1CON0,BANKED
						movlw	CWG1CON1_init
						movwf	s15_CWG1CON1,BANKED
						movlw	CWG1AS0_init
						movwf	s15_CWG1AS0,BANKED
						movlw	CWG1AS1_init
						movwf	s15_CWG1AS1,BANKED

						; setup PWM for LED brightness control
						movlw	PWM4DCH_init
						movwf	s15a_PWM4DCH
						clrf	s15a_PWM4DCL
						movlw	PWM4CON_init
						movwf	s15a_PWM4CON

						movlw	ODCONA_init
						movwf	s15_ODCONA,BANKED
						movlw	ODCONB_init
						movwf	s15_ODCONB,BANKED
						movlw	ODCONC_init
						movwf	s15_ODCONC,BANKED

						movlw	SLRCONA_init
						movwf	s15_SLRCONA,BANKED
						movlw	SLRCONB_init
						movwf	s15_SLRCONB,BANKED
						movlw	SLRCONC_init
						movwf	s15_SLRCONC,BANKED

						movlb	14
			;												;  1 26					Vpp/MCLR/power SW(was RA3)
				;		clrf	s14_RA0PPS,BANKED			;  2 27					LED current sense
						movlw	PPS_LED_CATHODE_init
						movwf	s14_RA1PPS,BANKED			;  3 28	PWM6			LED drive
				;		movwf	s14_RA2PPS,BANKED			;  4  1 DAC1OUT	RA2		debug pin
				;		movwf	s14_RA3PPS,BANKED			;  5  2 		RA3
				;		clrf	s14_RA4PPS,BANKED			;  6  3			RA4		stop SW(was RC0)
				;		clrf	s14_RA5PPS,BANKED			;  7  4			RA5		double speed SW(was RC1)
															;  8  5			VSS		Power
				;		clrf	s14_RA7PPS,BANKED			;  9  6			CLKIN	X'tal
				;		clrf	s14_RA6PPS,BANKED			; 10  7	CLKOUT			X'tal
						movlw	PPS_HB1_HS_init
						movwf	s14_RC0PPS,BANKED			; 11  8	CWG1A			HB1 HS
						movwf	s14_RC1PPS,BANKED			; 12  9	CWG1A			HB1 HS
						movwf	s14_RC2PPS,BANKED			; 13 10	CWG1A			HB1 HS
						movwf	s14_RC3PPS,BANKED			; 14 11	CWG1A			HB1 HS
						
						movwf	s14_RC4PPS,BANKED			; 15 12	CWG1A			HB1 HS
						movwf	s14_RC5PPS,BANKED			; 16 13	CWG1A			HB1 HS
						movlw	PPS_HB2_LS_init
						movwf	s14_RC6PPS,BANKED			; 17 14	C1OUT			HB2 LS
						movwf	s14_RC7PPS,BANKED			; 18 15	C1OUT			HB2 LS
															; 19 16			VSS		Power
															; 20 17			VDD		Power
				;		clrf	s14_RB0PPS,BANKED			; 21 18	ZCDIN	C2IN1+	HB1 voltage sense ADC
						movlw	PPS_HB1_LS_init
						movwf	s14_RB1PPS,BANKED			; 22 19	CWG1B			HB1 LS
						movwf	s14_RB2PPS,BANKED			; 23 20	CWG1B			HB1 LS
						movwf	s14_RB3PPS,BANKED			; 24 21	CWG1B			HB1 LS
						movlw	PPS_HB1_HS_init
						movwf	s14_RB5PPS,BANKED			; 25 22	CWG1A	OD		HB1 LS
						movwf	s14_RB4PPS,BANKED			; 26 23	CWG1A	OD		HB1 LS
				;		clrf	s14_RB6PPS,BANKED			; 27 24			RB6		ICSPCLK/origin SW(was RA1)
				;		clrf	s14_RB7PPS,BANKED			; 28 25			RB7		ICSPDAT/limit SW(was RA0)

						movlb	15
						; enable input buffer
						movlw	ANSELA_init
						movwf	s15_ANSELA,BANKED
						movlw	ANSELB_init
						movwf	s15_ANSELB,BANKED
						movlw	ANSELC_init
						movwf	s15_ANSELC,BANKED

						; start timer for ADC trigger
						movlw	T1CLK_init
						movwf	s15a_T1CLK
						clrf	s15a_T1TMRL
						clrf	s15a_T1TMRH
						movlw	T1CON_init
						movwf	s15a_T1CON

						; start PWM controller for motor
						movlw	T2CLK_init
						movwf	s15a_T2CLK
						movlw	T2HLT_init
						movwf	s15a_T2HLT
						movlw	T2PR_init
						movwf	s15a_T2PR
						movwf	s15a_T2TMR
						movlw	T2CON_init
						movwf	s15a_T2CON

						; start PWM controller for LED
						movlw	T6CLK_init
						movwf	s15a_T6CLK
						movlw	T6HLT_init
						movwf	s15a_T6HLT
						movlw	T6PR_init
						movwf	s15a_T6PR
						clrf	s15a_T6TMR
						movlw	T6CON_init
						movwf	s15a_T6CON

						; setup CCP1 for ADC trigger of motor
						rcall	load_default_period

						; initial key state
						comf	s15a_PORTA,W
						andlw	(1<<BIT_SW_STOP)|(1<<BIT_SW_DOUBLE_SPEED)
						iorlw	(1<<BIT_SW_RESET)

						movwf	s0a_sw_state
						movwf	d0a_sw_transient+0
						movwf	d0a_sw_transient+1

						; set multiplier for tracking
						movlw	( SIDEREAL_TIME_0p16        & 0xFF)
						btfsc	s0a_sw_state,BIT_SW_DOUBLE_SPEED
						movlw	( SOLAR_TIME_0p16           & 0xFF)
						btfsc	s0a_sw_state,BIT_SW_STOP
						movlw	( LUNAR_TIME_0p16           & 0xFF)
						movwf	d0a_tracking_period_0p16+0

						movlw	((SIDEREAL_TIME_0p16 >> 8 ) & 0xFF)
						btfsc	s0a_sw_state,BIT_SW_DOUBLE_SPEED
						movlw	((SOLAR_TIME_0p16 >> 8 )    & 0xFF)
						btfsc	s0a_sw_state,BIT_SW_STOP
						movlw	((LUNAR_TIME_0p16 >> 8 )    & 0xFF)
						movwf	d0a_tracking_period_0p16+1

						movlb	14
						; change WDT period
						movlw	WDTCON0_active
						movwf	s14_WDTCON0,BANKED

						; output enable
						movlw	TRISA_active
						movwf	s15a_TRISA
						movlw	TRISB_active
						movwf	s15a_TRISB
						movlw	TRISC_active
						movwf	s15a_TRISC

						; interrupt enable
						bsf		s14_PIE1,OSCFIE,BANKED
						bsf		s14_PIE6,CCP1IE,BANKED

						; enable IDLE mode
						bsf		s14_CPUDOZE,IDLEN,BANKED

						movlw	INTCON_active
						movwf	INTCON

						; wait for backup osc. ready
						movlw	OSCCON1_backup
						cpfseq	s14_OSCCON2,BANKED
						bra		$-2

						; switch to external osc.
						movlw	OSCCON1_active
						movwf	s14_OSCCON1,BANKED

__idle_loop:			; wait for interrupt
						bcf		INTCON,GIE
						TEST_PIN_OFF	TEST_CPU_USAGE
						sleep
						TEST_PIN_ON		TEST_CPU_USAGE
						bsf		INTCON,GIE
						bra		__idle_loop

get_notification:
						; if reset button has released, MCU sleep
						btfsc	s0a_sw_toggle,BIT_SW_RESET
						bra		$+6
						btfss	s0a_sw_state,BIT_SW_RESET
						rcall	__prepare_for_sleep	; toggle == 0 and state == 0, call but never return

						; if origin flag is set, index is reset to lut_start
						btfsc	s0a_sw_state,BIT_SW_ORIGIN	;  5
						rcall	set_idx_to_lut_start		; 16

	if (REWIND_TIMEOUT != 0) && (LUT_UPDATE_PERIOD != 0)
						; wait for timeout
						movlw	LOW  (pfm_lut_end + _SIZEOF_LUT_ENTRY*REWIND_TIMEOUT/LUT_UPDATE_PERIOD)
						subwf	lut_address_L,W
						movlw	HIGH (pfm_lut_end + _SIZEOF_LUT_ENTRY*REWIND_TIMEOUT/LUT_UPDATE_PERIOD)
						subwfb	lut_address_H,W
						btfsc	STATUS,C
						rcall	__prepare_for_sleep	; timeout, call but never return

						; motor is freed when lut_address >= lut_end
						movlw	LOW pfm_lut_end
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_end
						subwfb	lut_address_H,W
						btfsc	STATUS,C
						retlw	CTRL_MOTOR_FREE

						; if limit flag is set, index is set to lut_end
						btfsc	s0a_sw_state,BIT_SW_LIMIT	; 13
						rcall	set_idx_to_lut_end			; 16
	else
						; index is wrap-round to lut_start when lut_address >= lut_end
						movlw	LOW pfm_lut_end
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_end
						subwfb	lut_address_H,W
						btfsc	STATUS,C					;  5
						rcall	set_idx_to_lut_start		; 16
	endif
						; limit sw or stop sw is pushed
						btfss	s0a_sw_state,BIT_SW_LIMIT
						btfsc	s0a_sw_state,BIT_SW_STOP
						retlw	CTRL_MOTOR_STOP

						; power sw is pushed
						btfss	s0a_sw_toggle,BIT_SW_RESET
						retlw	CTRL_MOTOR_BREAK_IN

						; twice speed sw is pushed
						btfsc	s0a_sw_state,BIT_SW_DOUBLE_SPEED
						retlw	CTRL_MOTOR_2x	; set motor speed to 2x

						; none of sw is pushed.
						retlw	CTRL_MOTOR_1x	; set motor speed to 1x	; 9	59/51

__prepare_for_sleep:		; interrupt disable
						bcf		INTCON,GIE

						movlb	15
						; disconnect power from motor
						bcf		s15_CM1CON0,C1POL,BANKED

						rcall	wait_1us
						TEST_PIN_OFF	TEST_CPU_USAGE
						reset

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Key Handling																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
read_keys:
						movf	s0a_sw_state,W
						movwf	s0a_sw_released

						; prepare to reduce chattering of SWs
						movf	d0a_sw_transient+1,W
						movwf	s0a_sw_press
						movwf	s0a_sw_release

						movf	d0a_sw_transient+0,W
						andwf	s0a_sw_press,F
						iorwf	s0a_sw_release,F
						movwf	d0a_sw_transient+1			; 9

						; read SWs
						movf	s15a_PORTA,W
						andlw	(1<<BIT_SW_STOP)|(1<<BIT_SW_DOUBLE_SPEED)
						iorwf	s15a_PORTB,W
						iorwf	s15a_PORTE,W			; {RB7, RB6, RA5, RA4, RE3, 0, 0, RB0}

						; check debugger
						btfsc	WREG,RB6
						bra		$+6
						btfss	WREG,RB7
						iorlw	(1<<RB6)|(1<<RB7)	; if both RB6 and RB7 are ON, set both to OFF

						; update states of SWs
						xorlw	0xFF				; invert logic (from negative to positive)
						andwf	s0a_sw_press,F
						iorwf	s0a_sw_release,F
						movwf	d0a_sw_transient+0			; 12

						; reduce chattering of SWs
						movf	s0a_sw_press,W
						iorwf	s0a_sw_state,F		; set by triple 1 reads
						movf	s0a_sw_release,W
						andwf	s0a_sw_state,F		; clear by triple 0 reads

						; event generation
						comf	s0a_sw_state,W
						andwf	s0a_sw_released,F		; released SW(1: released)

						movf	s0a_sw_released,W
						xorwf	s0a_sw_toggle,F
						return								; 10	31
wait_1us:				bra		$-2

maintain_4hz:
						bcf		notification_flag,BIT_POLLING
						movf	cemf_sampling_period_L,W
						subwf	t0a_elapsed_time+0,F
						movf	cemf_sampling_period_H,W
						subwfb	t0a_elapsed_time+1,F
						clrf	WREG
						subwfb	t0a_elapsed_time+2,F
						btfsc	STATUS,C
						return								;  9

						bsf		notification_flag,BIT_POLLING
						movlw	( FCY_SCALED/POLLING_FREQUENCY        & 0xFF)
						addwf	t0a_elapsed_time+0,F
						movlw	((FCY_SCALED/POLLING_FREQUENCY >>  8) & 0xFF)
						addwfc	t0a_elapsed_time+1,F
						movlw	((FCY_SCALED/POLLING_FREQUENCY >> 16) & 0xFF)
						addwfc	t0a_elapsed_time+2,F
						return								;  9	18

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Data EEPROM																				;
;		*		*		*		*		*		*		*		*		*		*		*		;
restore_lut_index:		bsf		s15a_NVMCON0,NVMEN
						movlw	UPPER dfm_motor_constant
						movwf	s15a_NVMADRU
						movlw	HIGH dfm_motor_constant
						movwf	s15a_NVMADRH

						; read motor parameter from EEPROM
						movlw	LOW dfm_motor_constant
						movwf	s15a_NVMADRL

						rcall	__nvm_read_data
						movwf	motor_constant_LL
						rcall	__nvm_read_data
						movwf	motor_constant_LH
						rcall	__nvm_read_data
						movwf	motor_constant_HL
						rcall	__nvm_read_data
						movwf	motor_constant_HH

						; read lut address from EEPROM
						movlw	LOW dfm_lut_address
						movwf	s15a_NVMADRL

						rcall	__nvm_read_data
						movwf	lut_address_L
						rcall	__nvm_read_data
						movwf	lut_address_H
						rcall	__nvm_read_data
						movwf	lut_lookup_time

						bcf		s15a_NVMCON0,NVMEN

						; LSB check
						btfsc	lut_address_L,0			; LSB of lower byte must be cleared
						bra		set_idx_to_lut_start

						; upper range check
						movlw	LOW pfm_lut_end
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_end
						subwfb	lut_address_H,W
						bc		set_idx_to_lut_start

						; lower range check
						movlw	LOW pfm_lut_start
						subwf	lut_address_L,W
						movlw	HIGH pfm_lut_start
						subwfb	lut_address_H,W
						bnc		set_idx_to_lut_start

						; upper range check
						decf	lut_lookup_time,W
						addlw	-POLLING_FREQUENCY*LUT_UPDATE_PERIOD
						bnc		__range_ok

set_idx_to_lut_start:	movlw	HIGH pfm_lut_start
						movwf	lut_address_H
						movlw	LOW pfm_lut_start
						movwf	lut_address_L
						bra		__reset_lut_counter			;  6

set_idx_to_lut_end:		movlw	HIGH pfm_lut_end
						movwf	lut_address_H
						movlw	LOW pfm_lut_end
						movwf	lut_address_L
						bra		__reset_lut_counter			;  6

advance_lut_index:
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

__range_ok:
						comf	lut_address_L,W
						movwf	d0a_lut_address_com+0
						comf	lut_address_H,W
						movwf	d0a_lut_address_com+1
						return								;  6	14

; save lut index into latest entry in EEPROM
nvm_save_lut_index:
						comf	lut_address_L,W
						cpfseq	d0a_lut_address_com+0
						return
						comf	lut_address_H,W
						cpfseq	d0a_lut_address_com+1
						return

						bsf		s15a_NVMCON0,NVMEN

						movlw	UPPER dfm_lut_address
						movwf	s15a_NVMADRU
						movlw	HIGH dfm_lut_address
						movwf	s15a_NVMADRH
						movlw	LOW dfm_lut_address
						movwf	s15a_NVMADRL

						; write lower byte of lut index
						movf	lut_address_L,W
						rcall	__nvm_write_data
						; write upper byte of lut index
						movf	lut_address_H,W
						rcall	__nvm_write_data
						; write counter byte
						movf	lut_lookup_time,W
						rcall	__nvm_write_data

						; succesfully wrote
						bcf		s15a_NVMCON0,NVMEN
						movf	lut_address_L,W
						movwf	d0a_lut_address_com+0
						movf	lut_address_H,W
						movwf	d0a_lut_address_com+1
						return

; load factory default
nvm_set_factory_default:
						movlw	( MOTOR_CONSTANT_init        & 0xFF)
						movwf	motor_constant_LL
						movlw	((MOTOR_CONSTANT_init >>  8) & 0xFF)
						movwf	motor_constant_LH
						movlw	((MOTOR_CONSTANT_init >> 16) & 0xFF)
						movwf	motor_constant_HL
						movlw	((MOTOR_CONSTANT_init >> 24) & 0xFF)
						movwf	motor_constant_HH
; update motor parameter
nvm_update_parameter:
						bsf		s15a_NVMCON0,NVMEN

						movlw	UPPER dfm_motor_constant
						movwf	s15a_NVMADRU
						movlw	HIGH dfm_motor_constant
						movwf	s15a_NVMADRH
						movlw	LOW dfm_motor_constant
						movwf	s15a_NVMADRL

						; program parameter area
						movf	motor_constant_LL,W
						rcall	__nvm_write_data
						movf	motor_constant_LH,W
						rcall	__nvm_write_data
						movf	motor_constant_HL,W
						rcall	__nvm_write_data
						movf	motor_constant_HH,W
						rcall	__nvm_write_data

						bcf		s15a_NVMCON0,NVMEN
						reset

;		NVMADR	address of data
__nvm_read_data:		bsf		s15a_NVMCON1,RD
						movf	s15a_NVMDATL,W
						incf	s15a_NVMADRL,F
						return

;		WREG	data to be written
;		NVMADR	address of data
__nvm_write_data:		clrwdt
						bsf		s15a_NVMCON1,RD
						xorwf	s15a_NVMDATL,F
						bz		__skip_programming
						movwf	s15a_NVMDATL

						TEST_PIN_ON	TEST_NVM

						movlw	0x55				; Load 55h to get ready for unlock sequence
						movwf	s15a_NVMCON2			; First step is to load 55h into NVMCON2
						movlw	0xAA				; Second step is to load AAh into W
						movwf	s15a_NVMCON2			; Third step is to load AAh into NVMCON2
						bsf		s15a_NVMCON1,WR		; Final step is to set WR bit

						btfsc	s15a_NVMCON1,WR
						bra		$-2

						TEST_PIN_OFF	TEST_NVM

__skip_programming:		incf	s15a_NVMADRL,F
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Code Flash																				;
;		*		*		*		*		*		*		*		*		*		*		*		;
nvm_load_lut_entry:
						; load lut entry as interval time from code flash
						bsf		s15a_NVMCON0,NVMEN
						movf	lut_address_L,W
						movwf	s15a_NVMADRL
						movf	lut_address_H,W
						movwf	s15a_NVMADRH
						clrf	s15a_NVMADRU			; points to PFM
						bsf		s15a_NVMCON1,RD

						movf	s15a_NVMDATL,W
						mulwf	d0a_tracking_period_0p16+0
						movf	PRODH,W
						movwf	s0a_multiplier_bits

						movf	s15a_NVMDATH,W
						mulwf	d0a_tracking_period_0p16+1
						movf	PRODL,W
						movwf	cemf_sampling_period_L
						movf	PRODH,W
						movwf	cemf_sampling_period_H

						movf	s15a_NVMDATL,W
						mulwf	d0a_tracking_period_0p16+1
						movf	PRODL,W
						addwf	s0a_multiplier_bits,F
						movf	PRODH,W
						addwfc	cemf_sampling_period_L,F
						clrf	WREG
						addwfc	cemf_sampling_period_H,F

						movf	s15a_NVMDATH,W
						mulwf	d0a_tracking_period_0p16+0
						movf	PRODL,W
						addwf	s0a_multiplier_bits,F
						movf	PRODH,W
						addwfc	cemf_sampling_period_L,F
						clrf	WREG
						addwfc	cemf_sampling_period_H,F

						movf	s15a_NVMDATL,W
						btfss	d0a_tracking_period_0p16+1,7
						addwf	cemf_sampling_period_L,F
						movf	s15a_NVMDATH,W
						btfss	d0a_tracking_period_0p16+1,7
						addwfc	cemf_sampling_period_H,F

						bcf		s15a_NVMCON0,NVMEN

	if (MULT_PERIOD_2X_24p8/MULT_PERIOD_1X_24p8 == 2)
						bcf		STATUS,C
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rlcf	cemf_sampling_period_L,F
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rlcf	cemf_sampling_period_H,F
	endif
	if (MULT_PERIOD_1X_24p8/MULT_PERIOD_2X_24p8 == 2)
						bcf		STATUS,C
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rrcf	cemf_sampling_period_H,F
						btfsc	notification_flag,BIT_DOUBLE_VOLTAGE
						rrcf	cemf_sampling_period_L,F
	endif

						; load sampling start time into CCPR1
						clrf	s15a_CCP1CON
						movlw	0xFF
						addwf	cemf_sampling_period_L,W
						movwf	s15a_CCPR1L
						movlw	0xFF
						addwfc	cemf_sampling_period_H,W
						movwf	s15a_CCPR1H
						movlw	CCP1CON_init
						movwf	s15a_CCP1CON
						return

calc_target_voltage:
						; calculate target voltage
						movf	motor_constant_LL,W
						movwf	q0a_motor_constant_work+0		; bit position  7 -  0
						movf	motor_constant_LH,W
						movwf	q0a_motor_constant_work+1		; bit position 15 -  8
						movf	motor_constant_HL,W
						movwf	q0a_motor_constant_work+2		; bit position 23 - 16
						movf	motor_constant_HH,W
						movwf	q0a_motor_constant_work+3		; bit position 31 - 24

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
						bcf		STATUS,C
						rlcf	cemf_sampling_period_L,W
						movwf	t0a_sampling_period+0
						rlcf	cemf_sampling_period_H,W
						movwf	t0a_sampling_period+1
						clrf	t0a_sampling_period+2
						rlcf	t0a_sampling_period+2,F

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
						clrf	t0a_target_voltage_15p9+0
						movwf	t0a_target_voltage_15p9+1
						clrf	t0a_target_voltage_15p9+2		; 20

__pi_ctrl_div_pos:		movf	t0a_sampling_period+0,W
						subwf	q0a_motor_constant_work+1,F
						movf	t0a_sampling_period+1,W
						subwfb	q0a_motor_constant_work+2,F
						movf	t0a_sampling_period+2,W
						subwfb	q0a_motor_constant_work+3,F		; 6
__pi_ctrl_div_common:	rlcf	t0a_target_voltage_15p9+1,F
						rlcf	t0a_target_voltage_15p9+2,F
						btfsc	STATUS,C
						return
						bcf		STATUS,C
						rlcf	q0a_motor_constant_work+0,F
						rlcf	q0a_motor_constant_work+1,F
						rlcf	q0a_motor_constant_work+2,F
						rlcf	q0a_motor_constant_work+3,F
						btfsc	t0a_target_voltage_15p9+1,0
						bra		__pi_ctrl_div_pos					; 11	20+(6+11)*12=224
__pi_ctrl_div_neg:		movf	t0a_sampling_period+0,W			; 10	20+(8+10)*12=236
						addwf	q0a_motor_constant_work+1,F
						movf	t0a_sampling_period+1,W
						addwfc	q0a_motor_constant_work+2,F
						movf	t0a_sampling_period+2,W
						addwfc	q0a_motor_constant_work+3,F
						bra		__pi_ctrl_div_common				; 8

calc_target_ratio:
						; calculate target voltage ratio
						; target_ratio = target_voltage(13bit) * FVR(13bit)
						;   8p32 >> 14             15p9        m7p23
						clrf	t0a_target_ratio_6p18+0
						clrf	t0a_target_ratio_6p18+1
						clrf	t0a_target_ratio_6p18+2

						; startup_ratio = startup_voltage(13bit) * FVR(13bit)
						;    8p32 >> 14              15p9        m7p23
						clrf	t0a_startup_ratio_6p18+1
						clrf	t0a_startup_ratio_6p18+2

						movlw	13
						movwf	s0a_multiplier_bits
						movff	d0a_adres_FVR_m7p23+0,t0a_adres_FVR_work+0
						movff	d0a_adres_FVR_m7p23+1,t0a_adres_FVR_work+1			; 11

__calc_target_ratio_lp:	btfss	t0a_adres_FVR_work+0,0
						bra		$+18

						movf	t0a_target_voltage_15p9+1,W
						addwf	t0a_target_ratio_6p18+1,F
						movf	t0a_target_voltage_15p9+2,W
						addwfc	t0a_target_ratio_6p18+2,F

						movlw	LOW STARTUP_VOLTAGE_15p1
						addwf	t0a_startup_ratio_6p18+1,F
						movlw	HIGH STARTUP_VOLTAGE_15p1
						addwfc	t0a_startup_ratio_6p18+2,F

						bcf		STATUS,C
						rrcf	t0a_target_ratio_6p18+2,F
						rrcf	t0a_target_ratio_6p18+1,F
						rrcf	t0a_target_ratio_6p18+0,F

						bcf		STATUS,C
						rrcf	t0a_startup_ratio_6p18+2,F
						rrcf	t0a_startup_ratio_6p18+1,F

						bcf		STATUS,C
						rrcf	t0a_adres_FVR_work+1,F
						rrcf	t0a_adres_FVR_work+0,F

						decfsz	s0a_multiplier_bits,F
						bra		__calc_target_ratio_lp			; 20

						; startup_voltage = target_voltage + IR-drop
						movf	t0a_target_ratio_6p18+1,W
						addwf	t0a_startup_ratio_6p18+1,F
						movf	t0a_target_ratio_6p18+2,W
						addwfc	t0a_startup_ratio_6p18+2,F

						bcf		STATUS,C
						rrcf	t0a_target_ratio_6p18+2,F
						rrcf	t0a_target_ratio_6p18+1,F
						rrcf	t0a_target_ratio_6p18+0,F

						bcf		STATUS,C
						rrcf	t0a_startup_ratio_6p18+2,F
						rrcf	t0a_startup_ratio_6p18+1,F
						return								; 11		11+20*13+11=282

eeprom					code_pack	0x310000
dfm_lut_address:		de		LOW  pfm_lut_start
						de		HIGH pfm_lut_start
						de		POLLING_FREQUENCY*LUT_UPDATE_PERIOD
dfm_motor_constant:		de		( MOTOR_CONSTANT_init        & 0xFF)	; 32p0
						de		((MOTOR_CONSTANT_init >>  8) & 0xFF)
						de		((MOTOR_CONSTANT_init >> 16) & 0xFF)
						de		((MOTOR_CONSTANT_init >> 24) & 0xFF)

						end
