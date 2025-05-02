						processor		16f18346
						#include		p16f18346.inc

						errorlevel -302
						radix	DEC

FCY						equ		4000000					; 4MHz
KEY_POLLING_FREQ		equ		50						; 50Hz, 20ms
ADC_INTERVAL			equ		15						; 15us, minimum is 15us
PWM_PERIOD				equ		50						; 50us, 20kHz
PULSE_PER_SLOT			set		3						; 3 or 4

; High supply voltage or Low motor speed
 __config	_CONFIG1,	_FEXTOSC_HS & _RSTOSC_HFINT1
 __config	_CONFIG2,	_MCLRE_OFF & _PPS1WAY_OFF & _PWRTE_ON & _WDTE_SLEEP & _BOREN_SLEEP & _BORV_LOW & _LPBOREN_ON
 __config	_CONFIG3,	_LVP_OFF
LOW_BATT_ERROR			set		2300					; [mV]
LOW_BATT_WARNING		set		3000					; [mV]

LUT_UPDATE_PERIOD		set		5						; 5, 4, 3, 2, 1 [second]
REWIND_TIMEOUT			set		600						; 600sec = 10min, 0 is endless

; sampling period has unit of TCY(= 0.25us).
; peak per second is MOTOR_RPM/10, samples per 1-wave is PULSE_PER_SLOT.
; samples per 1-wave is defined as PULSE_PER_SLOT.
; sampling period[clock] = FCY[clock/s]*10/(MOTOR_RPM*PULSE_PER_SLOT)

; target_voltage[mV] = MOTOR_RPM/(MOTOR_SPEED_CONSTANT/1000[mV/V])
;                    = MOTOR_RPM*1000/MOTOR_SPEED_CONSTANT

LUT_ENTRIES				macro	MOTOR_100TH_RPM
						local	dividend, divisor, period_i, period_f, fraction, target_voltage

dividend		= FCY*500/PULSE_PER_SLOT*2
divisor			= MOTOR_100TH_RPM
period_i		= dividend/divisor
period_f		= (dividend - period_i*divisor)*32/divisor
target_voltage	= MOTOR_100TH_RPM*10/MOTOR_SPEED_CONSTANT

period_i = period_i - 1
fraction = period_f
		dw		((target_voltage & 0x3F) << 8) | (LOW period_i)
		dw		((target_voltage >> 6) << 8)  | (HIGH period_i)
		while fraction != 0
fraction += period_f
				if fraction >= 32
fraction -= 32
						dw		((target_voltage & 0x3F) << 8) | (LOW (period_i + 1))
						dw		((target_voltage >> 6) << 8)  | (HIGH (period_i + 1))
				else
						dw		((target_voltage & 0x3F) << 8) | (LOW period_i)
						dw		((target_voltage >> 6) << 8)  | (HIGH period_i)
				endif
		endw
						endm

LUT_ENTRY				macro	MOTOR_10TH_RPM
						local	period_i, target_voltage

period_i = (FCY*100/PULSE_PER_SLOT + MOTOR_10TH_RPM/2)/MOTOR_10TH_RPM
target_voltage = MOTOR_10TH_RPM*100/MOTOR_SPEED_CONSTANT

period_i = period_i - 1
		dw		((target_voltage & 0x3F) << 8) | (LOW period_i)
		dw		((target_voltage >> 6) << 8)  | (HIGH period_i)
						endm

_CYCLE_TIME_L			equ		0						; cycle time is [T1CK] in 16p0
_CYCLE_TIME_H			equ		1
_SIZEOF_LUT_ENTRY		equ		2						; fixed. don't change

lut						code
;P_GAIN					proportional gain(amplitude)
;I_TIME					integration time(amplitude) where the range is from 2 to 255
; target_ratio = target_voltage / supplied_voltage
; back_emf_ratio = back_emf_voltage / supplied_voltage
; difference = target_ratio - back_emf_ratio
; integral_error += difference
; duty_ratio = P_GAIN*(difference + integral_error/I_TIME)
lut_start:
; motor parameter and desired rotation speed
;						#include ../FP030-KN-11160.inc
;						#include ../FP030-KN-13125.inc
;						#include ../FA-130RA-2270.inc
;						#include ../RE-140RA-2270.inc
						#include ../RE-260RA-2670.inc
;						#include ../RE-280RA-2865.inc
;						#include ../RE-280RA-2485.inc
;						#include ../RE-280RA-20120.inc
lut_end:
						LUT_ENTRY		5000

; Timer configurations
						; Timer assignment
						; CCP4: Timer 0, CCP3: Timer 3, CCP2: Timer 0, CCP1: Timer 1
CCPTMRS_init			equ		(B'00'<<C4TSEL0)|(B'10'<<C3TSEL0)|(B'00'<<C2TSEL0)|(B'01'<<C1TSEL0)
						; PWM6: Timer 6, PWM5: Timer 2
PWMTMRS_init			equ		(B'11'<<P6TSEL0)|(B'01'<<P5TSEL0)

						; Trigger for A/D conversion of back-emf
T1CON_init				equ		(B'00'<<T1CKPS0)|(1 << TMR1ON)	; FCY, pre 1:1
CCP1CON_init			equ		(1<<CCP1EN)|(B'1011'<<CCP1MODE0)	; 0-1-0 pulse, clear on match

						; PWM generator
T2CON_init				equ		(B'00'<<T2CKPS0)|(1 << TMR2ON)	; FCY, pre 1:1, post 1:1
PR2_init				equ		PWM_PERIOD*(FCY/1000000) - 1
CWG1CON0_init			equ		(1<<EN)|(B'100'<<MODE0)	; PWM HB
CWG1CON1_init			equ		(0<<POLB)|(1<<POLA)		; POLA: active L, POLB: active H
CWG1DAT_init			equ		B'0111'					; PWM5
CWG1AS0_init			equ		(1<<SHUTDOWN)|(B'10'<<LSBD0)|(B'11'<<LSAC0)	; A=H, B=L
CWG1AS1_init			equ		(1<<AS1E)				; by C1
CWG1DBR_init			equ		3						; for 1nF, delay time of rising is 4*Tosc = 250ns
CWG1DBF_init			equ		4						; for 1nF, delay time of falling is 4*Tosc = 250ns
PPS_HB1_HS_init			equ		B'01000'				; Rxy source is CWG1A
PPS_HB1_LS_init			equ		B'01001'				; Rxy source is CWG1B
PPS_HB2_LS_init			equ		B'10110'				; Rxy source is C1
CM1CON0_init			equ		(0<<C2ON)|(0<<C1POL)|(1<<C1SP)|(0<<C2HYS)
CM2CON0_init			equ		(1<<C2ON)|(1<<C2POL)|(1<<C2SP)|(1<<C2HYS)
CM2CON1_init			equ		B'00111011'				; C2VP: VSS, C2VN: ANC3 -> C2IN3-
WPUC_B_EMF_SENSE		equ		WPUC3
LATC_B_EMF_SENSE		equ		LATC3
ODCONC_B_EMF_SENSE		equ		ODCC3
PWM5CON_init			equ		(1<<PWM5EN)

						; Trigger for A/D conversion of FVR
T3CON_init				equ		(B'00'<<T3CKPS0)|(1 << TMR3ON)	; FCY, pre 1:1
CCP3CON_init			equ		(1<<CCP3EN)|(B'1011'<<CCP3MODE0)	; 0-1-0 pulse, clear on match

						; Key polling timer
T4CON_init				equ		(9 << T4OUTPS0)|(B'11' << T4CKPS0)|(1 << TMR4ON)
PR4_init				equ		(FCY/640/KEY_POLLING_FREQ) - 1 ; FCY/640, pre 1:64, post 1:10

						; Constant brightness LED driver
T6CON_init				equ		(B'01' << T6CKPS0)|(1 << TMR6ON) ; FCY/4, pre 1:4, post 1:1
PR6_init				equ		255
PWM6DCH_init			equ		(256-4)					; 	minimum settling time 4.25us
PPS_LED_K_init			equ		B'00011'				; Rxy source is PWM6
TRISC_LED_K				equ		TRISC4
ODCONC_LED_K			equ		ODCC4
ODCONC_init				equ		(1<<ODCONC_LED_K)|(1<<ODCONC_B_EMF_SENSE)
PWM6CON_init			equ		(1<<PWM6EN)

; RA2 output
;PPS_RA2_init			equ		B'011110'				; Rxy source is CLKR
;PPS_RA2_init			equ		B'000100'				; Rxy source is CLC1OUT
;PPS_RA2_init			equ		B'001110'				; Rxy source is CCP3
;PPS_RA2_init			equ		B'100000'				; Rxy source is RA2, duty update
;PPS_RA2_init			equ		B'100001'				; Rxy source is RA2, EEPROM update
PPS_RA2_init			equ		B'111111'				; Rxy source is RA2, 0
CLKRCON_init			equ		(1<<CLKREN)|(B'10'<<CLKRDC0)|(B'000'<<CLKRDIV0)	; Fcy
CLC1CON_init			equ		(1<<LC1EN)|(B'010'<<LC1MODE0)	; 4-input AND
CLC1POL_init			equ		B'1110'

; etc.
OSCCON1_EXTOSC			equ		(B'001'<<NOSC0)			; EXTOSC
OSCCON1_HFINTOSC		equ		(B'110'<<NOSC0)			; HFINTOSC
WDTCON_init				equ		(B'00110'<<WDTPS0)		; 64ms nominal, 48ms minimum, 80ms maximum
PCON_init				equ		(1<<NOT_BOR)|(1<<NOT_POR)|(1<<NOT_RI)|(1<<NOT_RMCLR)

; ADC and FVR configurations
FVRCON_init				equ		B'11000010'		; CDAFVR = 00 for OFF, ADFVR = 10 for 2.048V
ADCON0_MUX_FVR			equ		(B'111111'<<CHS0)|(1<<ADON)		; FVR(FVR output)
ADCON0_MUX_MOTOR		equ		(B'010011'<<CHS0)|(1<<ADON)		; ANC3(motor, RC3 pin)
ADCON0_MUX_LED			equ		(B'010101'<<CHS0)|(1<<ADON)		; ANC5(LED, RC5 pin)
ADCON1_VINPUT			equ		(5<<ADCS0)|(0<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=VDD, flush right
ADCON1_MOTOR			equ		(5<<ADCS0)|(3<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=FVR, flush right
ADCON1_LED				equ		(5<<ADCS0)|(0<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=VDD, flush right
ADACT_VINPUT			equ		B'01110'		; Timer3-CCP3 match
ADACT_MOTOR				equ		B'01100'		; Timer1-CCP1 match
ADACT_LED				equ		B'00010'		; Timer6-PR6 match

; I/O port configurations
WPUA_init				equ		B'001011'		; pull-up:                  RA3,      RA1, RA0
TRISA_active			equ		B'001011'		; input:                    RA3,      RA1, RA0
ANSELA_init				equ		B'110100'		; digital input:            RA3,      RA1, RA0 

WPUC_init				equ		B'00000011'		; pull-up:                            RC1, RC0
TRISC_active			equ		B'00100011'		; input:          RC5,      RC3,      RC1, RC0
ANSELC_init				equ		B'11111100'		; digital input:                      RC1, RC0

; bit position of notification_flag
BIT_CLOSED_LOOP_CTRL	equ		0	;-----------+
BIT_OPEN_LOOP_CTRL		equ		1	;----------+|
BIT_PI_CTRL				equ		2	;---------+||
BIT_DOUBLE_VOLTAGE		equ		2	;---------+||
BIT_PP_CTRL				equ		3	;--------+|||
BIT_OUT_OF_CTRL			equ		4	;-------+||||
BIT_UPDATE_LED			equ		5	;------+|||||
BIT_UPDATE_LUT_INDEX	equ		6	;-----+||||||
BIT_LOAD_LUT_ENTRY		equ		7	;----+|||||||
;									     ||||||||
CTRL_MOTOR_BREAK_IN		equ			   B'11100010'
CTRL_MOTOR_2x			equ			   B'11100101'
CTRL_MOTOR_1x			equ			   B'11101001'
CTRL_MOTOR_STOP			equ			   B'11110000'

; bit position of sw_state, sw_released, sw_pressed, sw_toggle
BIT_SW_LIMIT			equ		0		; RA0	limit SW			edge (pressing)
BIT_SW_ORIGIN			equ		1		; RA1	origin SW			level
BIT_SW_RESET			equ		3		; RA3	power SW			alternate
BIT_SW_STOP				equ		4		; RC0	stop SW				level
BIT_SW_DOUBLE_SPEED		equ		5		; RC1	double speed SW		level

; common RAM allocation
common_ram				udata_shr		0x70
pulse_width_F			res		1		; 0p16	[V/V]
pulse_width_L			res		1
startup_pulse_width_L	res		1
notification_flag		res		1		; flags for notification
sampling_interval_L		res		1
sampling_interval_H		res		1
led_pulse_width_L		res		1
led_pulse_width_H		res		1
address_lut_L			res		1
address_lut_H			res		1
sequence_bit_L			res		1
sequence_bit_H			res		1
eeprom_adrs				res		1
counter_lut				res		1

; prefix
; s0_ means single byte in bank 0.
;		single byte memory		s
;		double byte memory		d
;		function				f
;		array					a

;-----Bank0------------------
s0_PORTA            EQU  PORTA
s0_PORTB            EQU  PORTB
s0_PORTC            EQU  PORTC
s0_PIR0             EQU  PIR0
s0_PIR1             EQU  PIR1
s0_PIR2             EQU  PIR2
s0_PIR3             EQU  PIR3
s0_PIR4             EQU  PIR4
s0_TMR0             EQU  TMR0
s0_TMR0L            EQU  TMR0L
s0_PR0              EQU  PR0
s0_TMR0H            EQU  TMR0H
s0_T0CON0           EQU  T0CON0
s0_T0CON1           EQU  T0CON1
s0_TMR1L            EQU  TMR1L
s0_TMR1H            EQU  TMR1H
s0_T1CON            EQU  T1CON
s0_T1GCON           EQU  T1GCON
s0_TMR2             EQU  TMR2
s0_PR2              EQU  PR2
s0_T2CON            EQU  T2CON

; general purpose RAM (Bank 0) allocation
bank0_ram				udata	0x20
; Switch
s0_sw_transient			res		1		; transient state of SW
s0_sw_state				res		1		; current state {0, 0, RC1, RC0, RA3, 0, RA1, RA0}
s0_sw_released			res		1		; changed state of SW
s0_sw_pressed			res		1		; changed state of SW
s0_sw_toggle			res		1
s0_osc_recovery_delay	res		1		; recovery delay for oscillator failure

;-----Bank1------------------
s1_TRISA            EQU  TRISA
s1_TRISB            EQU  TRISB
s1_TRISC            EQU  TRISC
s1_PIE0             EQU  PIE0
s1_PIE1             EQU  PIE1
s1_PIE2             EQU  PIE2
s1_PIE3             EQU  PIE3
s1_PIE4             EQU  PIE4
s1_WDTCON           EQU  WDTCON
s1_ADRESL           EQU  ADRESL
s1_ADRESH           EQU  ADRESH
s1_ADCON0           EQU  ADCON0
s1_ADCON1           EQU  ADCON1
s1_ADACT            EQU  ADACT

; general purpose RAM (Bank 1) allocation
bank1_ram				udata	0xA0
; battery voltage
s1_batt_blink			res		1
s1_batt_flag			res		1
s1_adres_FVR_work_L		res		1
s1_adres_FVR_work_H		res		1
s1_adres_FVR_3p13_L		res		1		; 3p13	[V/V]
s1_adres_FVR_3p13_H		res		1
s1_adres_FVR_0p16_L		res		1		; 0p16	[V/V]
s1_adres_FVR_0p16_H		res		1
s1_voltage_L			res		1		; [V]
s1_voltage_H			res		1
s1_ratio_L				res		1		; [V/V]
s1_ratio_H				res		1

; Back-EMF voltage
s1_adjustment_range_L	res		1		; 7p9	[V]
s1_adjustment_range_H	res		1
s1_adres_back_emf_Tm1_L	res		1		; 7p9	[V]
s1_adres_back_emf_Tm1_H	res		1
s1_adres_back_emf_T0_L	res		1		; 7p9	[V]
s1_adres_back_emf_T0_H	res		1
s1_back_emf_ratio_L		res		1		; 6p10	[V/V]	2x gain
s1_back_emf_ratio_H		res		1
s1_back_emf_timeout		res		1

; PI controller for motor
s1_target_voltage_L		res		1		; 6p10	[V]
s1_target_voltage_H		res		1
s1_target_ratio_F		res		1		; 6p18	[V/V]
s1_target_ratio_L		res		1		; 6p10	[V/V]
s1_target_ratio_H		res		1
s1_target_ratio_half_F	res		1		; 6p18	[V/V]
s1_target_ratio_half_L	res		1		; 6p10	[V/V]
s1_target_ratio_half_H	res		1
s1_proportional_L		res		1		; 6p10	[V/V]	2x gain
s1_proportional_H		res		1
s1_slope				res		1		; valley, rise, peak, fall
s1_integral_F			res		1		; 6p18	[V/V]
s1_integral_L			res		1		; 6p10	[V/V]
s1_integral_H			res		1
s1_integration_work_F	res		1		; 6p18	[V/V]
s1_integration_work_L	res		1		; 6p10	[V/V]
s1_integration_work_H	res		1

s1_startup_voltage_L	res		1		; [mV]
s1_startup_voltage_H	res		1
s1_duty_limit_L			res		1		; [V/V]
s1_duty_limit_H			res		1

; open loop controller for motor
s1_adres_break_in_F		res		1
s1_adres_break_in_L		res		1

; constant current controller for LED
s1_divider_L			res		1
s1_divider_H			res		1
s1_dividend_F			res		1
s1_dividend_L			res		1
s1_dividend_H			res		1

;-----Bank2------------------
s2_LATA             EQU  LATA
s2_LATB             EQU  LATB
s2_LATC             EQU  LATC
s2_CM1CON0          EQU  CM1CON0
s2_CM1CON1          EQU  CM1CON1
s2_CM2CON0          EQU  CM2CON0
s2_CM2CON1          EQU  CM2CON1
s2_CMOUT            EQU  CMOUT
s2_BORCON           EQU  BORCON
s2_FVRCON           EQU  FVRCON
s2_DACCON0          EQU  DACCON0
s2_DACCON1          EQU  DACCON1

;-----Bank3------------------
s3_ANSELA           EQU  ANSELA
s3_ANSELB           EQU  ANSELB
s3_ANSELC           EQU  ANSELC
s3_VREGCON          EQU  VREGCON
s3_RC1REG           EQU  RC1REG
s3_RCREG            EQU  RCREG
s3_RCREG1           EQU  RCREG1
s3_TX1REG           EQU  TX1REG
s3_TXREG            EQU  TXREG
s3_TXREG1           EQU  TXREG1
s3_SP1BRG           EQU  SP1BRG
s3_SP1BRGL          EQU  SP1BRGL
s3_SPBRG            EQU  SPBRG
s3_SPBRG1           EQU  SPBRG1
s3_SPBRGL           EQU  SPBRGL
s3_SP1BRGH          EQU  SP1BRGH
s3_SPBRGH           EQU  SPBRGH
s3_SPBRGH1          EQU  SPBRGH1
s3_RC1STA           EQU  RC1STA
s3_RCSTA            EQU  RCSTA
s3_RCSTA1           EQU  RCSTA1
s3_TX1STA           EQU  TX1STA
s3_TXSTA            EQU  TXSTA
s3_TXSTA1           EQU  TXSTA1
s3_BAUD1CON         EQU  BAUD1CON
s3_BAUDCON          EQU  BAUDCON
s3_BAUDCON1         EQU  BAUDCON1
s3_BAUDCTL          EQU  BAUDCTL
s3_BAUDCTL1         EQU  BAUDCTL1

; general purpose RAM (Bank 3) allocation
;bank3_ram				udata	0x1A0

;-----Bank4------------------
s4_WPUA             EQU  WPUA
s4_WPUB             EQU  WPUB
s4_WPUC             EQU  WPUC
s4_SSP1BUF          EQU  SSP1BUF
s4_SSPBUF           EQU  SSPBUF
s4_SSP1ADD          EQU  SSP1ADD
s4_SSPADD           EQU  SSPADD
s4_SSP1MSK          EQU  SSP1MSK
s4_SSPMSK           EQU  SSPMSK
s4_SSP1STAT         EQU  SSP1STAT
s4_SSPSTAT          EQU  SSPSTAT
s4_SSP1CON          EQU  SSP1CON
s4_SSP1CON1         EQU  SSP1CON1
s4_SSPCON           EQU  SSPCON
s4_SSPCON1          EQU  SSPCON1
s4_SSP1CON2         EQU  SSP1CON2
s4_SSPCON2          EQU  SSPCON2
s4_SSP1CON3         EQU  SSP1CON3
s4_SSPCON3          EQU  SSPCON3
s4_SSP2BUF          EQU  SSP2BUF
s4_SSP2ADD          EQU  SSP2ADD
s4_SSP2MSK          EQU  SSP2MSK
s4_SSP2STAT         EQU  SSP2STAT
s4_SSP2CON          EQU  SSP2CON
s4_SSP2CON1         EQU  SSP2CON1
s4_SSP2CON2         EQU  SSP2CON2
s4_SSP2CON3         EQU  SSP2CON3

;-----Bank5------------------
s5_ODCONA           EQU  ODCONA
s5_ODCONB           EQU  ODCONB
s5_ODCONC           EQU  ODCONC
s5_CCPR1L           EQU  CCPR1L
s5_CCPR1H           EQU  CCPR1H
s5_CCP1CON          EQU  CCP1CON
s5_CCP1CAP          EQU  CCP1CAP
s5_CCPR2L           EQU  CCPR2L
s5_CCPR2H           EQU  CCPR2H
s5_CCP2CON          EQU  CCP2CON
s5_CCP2CAP          EQU  CCP2CAP
s5_CCPTMRS          EQU  CCPTMRS

;-----Bank6------------------
s6_SLRCONA          EQU  SLRCONA
s6_SLRCONB          EQU  SLRCONB
s6_SLRCONC          EQU  SLRCONC
s6_CCPR3L           EQU  CCPR3L
s6_CCPR3H           EQU  CCPR3H
s6_CCP3CON          EQU  CCP3CON
s6_CCP3CAP          EQU  CCP3CAP
s6_CCPR4L           EQU  CCPR4L
s6_CCPR4H           EQU  CCPR4H
s6_CCP4CON          EQU  CCP4CON
s6_CCP4CAP          EQU  CCP4CAP

;-----Bank7------------------
s7_INLVLA           EQU  INLVLA
s7_INLVLB           EQU  INLVLB
s7_INLVLC           EQU  INLVLC
s7_IOCAP            EQU  IOCAP
s7_IOCAN            EQU  IOCAN
s7_IOCAF            EQU  IOCAF
s7_IOCBP            EQU  IOCBP
s7_IOCBN            EQU  IOCBN
s7_IOCBF            EQU  IOCBF
s7_IOCCP            EQU  IOCCP
s7_IOCCN            EQU  IOCCN
s7_IOCCF            EQU  IOCCF
s7_CLKRCON          EQU  CLKRCON
s7_MDCON            EQU  MDCON
s7_MDSRC            EQU  MDSRC
s7_MDCARH           EQU  MDCARH
s7_MDCARL           EQU  MDCARL

;-----Bank8------------------
s8_TMR3L            EQU  TMR3L
s8_TMR3H            EQU  TMR3H
s8_T3CON            EQU  T3CON
s8_T3GCON           EQU  T3GCON
s8_TMR4             EQU  TMR4
s8_PR4              EQU  PR4
s8_T4CON            EQU  T4CON
s8_TMR5L            EQU  TMR5L
s8_TMR5H            EQU  TMR5H
s8_T5CON            EQU  T5CON
s8_T5GCON           EQU  T5GCON
s8_TMR6             EQU  TMR6
s8_PR6              EQU  PR6
s8_T6CON            EQU  T6CON
s8_CCDCON           EQU  CCDCON

;-----Bank9------------------
s9_NCO1ACC          EQU  NCO1ACC
s9_NCO1ACCL         EQU  NCO1ACCL
s9_NCO1ACCH         EQU  NCO1ACCH
s9_NCO1ACCU         EQU  NCO1ACCU
s9_NCO1INC          EQU  NCO1INC
s9_NCO1INCL         EQU  NCO1INCL
s9_NCO1INCH         EQU  NCO1INCH
s9_NCO1INCU         EQU  NCO1INCU
s9_NCO1CON          EQU  NCO1CON
s9_NCO1CLK          EQU  NCO1CLK

;-----Bank12------------------
s12_PWM5DCL          EQU  PWM5DCL
s12_PWM5DCH          EQU  PWM5DCH
s12_PWM5CON          EQU  PWM5CON
s12_PWM6DCL          EQU  PWM6DCL
s12_PWM6DCH          EQU  PWM6DCH
s12_PWM6CON          EQU  PWM6CON
s12_PWMTMRS          EQU  PWMTMRS

;-----Bank13------------------
s13_CWG1CLKCON       EQU  CWG1CLKCON
s13_CWG1DAT          EQU  CWG1DAT
s13_CWG1DBR          EQU  CWG1DBR
s13_CWG1DBF          EQU  CWG1DBF
s13_CWG1CON0         EQU  CWG1CON0
s13_CWG1CON1         EQU  CWG1CON1
s13_CWG1AS0          EQU  CWG1AS0
s13_CWG1AS1          EQU  CWG1AS1
s13_CWG1STR          EQU  CWG1STR

;-----Bank14------------------
s14_CWG2CLKCON       EQU  CWG2CLKCON
s14_CWG2DAT          EQU  CWG2DAT
s14_CWG2DBR          EQU  CWG2DBR
s14_CWG2DBF          EQU  CWG2DBF
s14_CWG2CON0         EQU  CWG2CON0
s14_CWG2CON1         EQU  CWG2CON1
s14_CWG2AS0          EQU  CWG2AS0
s14_CWG2AS1          EQU  CWG2AS1
s14_CWG2STR          EQU  CWG2STR

;-----Bank17------------------
s17_EEADR            EQU  EEADR
s17_EEADRL           EQU  EEADRL
s17_NVMADR           EQU  NVMADR
s17_NVMADRL          EQU  NVMADRL
s17_PMADR            EQU  PMADR
s17_PMADRL           EQU  PMADRL
s17_EEADRH           EQU  EEADRH
s17_NVMADRH          EQU  NVMADRH
s17_PMADRH           EQU  PMADRH
s17_EEDAT            EQU  EEDAT
s17_EEDATL           EQU  EEDATL
s17_NVMDAT           EQU  NVMDAT
s17_NVMDATL          EQU  NVMDATL
s17_PMDAT            EQU  PMDAT
s17_PMDATL           EQU  PMDATL
s17_EEDATH           EQU  EEDATH
s17_NVMDATH          EQU  NVMDATH
s17_PMDATH           EQU  PMDATH
s17_EECON1           EQU  EECON1
s17_NVMCON1          EQU  NVMCON1
s17_PMCON1           EQU  PMCON1
s17_EECON2           EQU  EECON2
s17_NVMCON2          EQU  NVMCON2
s17_PMCON2           EQU  PMCON2
s17_PCON0            EQU  PCON0

;-----Bank18------------------
s18_PMD0             EQU  PMD0
s18_PMD1             EQU  PMD1
s18_PMD2             EQU  PMD2
s18_PMD3             EQU  PMD3
s18_PMD4             EQU  PMD4
s18_PMD5             EQU  PMD5
s18_CPUDOZE          EQU  CPUDOZE
s18_OSCCON1          EQU  OSCCON1
s18_OSCCON2          EQU  OSCCON2
s18_OSCCON3          EQU  OSCCON3
s18_OSCSTAT1         EQU  OSCSTAT1
s18_OSCEN            EQU  OSCEN
s18_OSCTUNE          EQU  OSCTUNE
s18_OSCFRQ           EQU  OSCFRQ

;-----Bank28------------------
s28_PPSLOCK          EQU  PPSLOCK
s28_INTPPS           EQU  INTPPS
s28_T0CKIPPS         EQU  T0CKIPPS
s28_T1CKIPPS         EQU  T1CKIPPS
s28_T1GPPS           EQU  T1GPPS
s28_CCP1PPS          EQU  CCP1PPS
s28_CCP2PPS          EQU  CCP2PPS
s28_CCP3PPS          EQU  CCP3PPS
s28_CCP4PPS          EQU  CCP4PPS
s28_CWG1PPS          EQU  CWG1PPS
s28_CWG2PPS          EQU  CWG2PPS
s28_MDCIN1PPS        EQU  MDCIN1PPS
s28_MDCIN2PPS        EQU  MDCIN2PPS
s28_MDMINPPS         EQU  MDMINPPS
s28_SSP2CLKPPS       EQU  SSP2CLKPPS
s28_SSP2DATPPS       EQU  SSP2DATPPS
s28_SSP2SSPPS        EQU  SSP2SSPPS
s28_SSP1CLKPPS       EQU  SSP1CLKPPS
s28_SSP1DATPPS       EQU  SSP1DATPPS
s28_SSP1SSPPS        EQU  SSP1SSPPS
s28_RXPPS            EQU  RXPPS
s28_TXPPS            EQU  TXPPS
s28_CLCIN0PPS        EQU  CLCIN0PPS
s28_CLCIN1PPS        EQU  CLCIN1PPS
s28_CLCIN2PPS        EQU  CLCIN2PPS
s28_CLCIN3PPS        EQU  CLCIN3PPS
s28_T3CKIPPS         EQU  T3CKIPPS
s28_T3GPPS           EQU  T3GPPS
s28_T5CKIPPS         EQU  T5CKIPPS
s28_T5GPPS           EQU  T5GPPS

;-----Bank29------------------
s29_RA0PPS           EQU  RA0PPS
s29_RA1PPS           EQU  RA1PPS
s29_RA2PPS           EQU  RA2PPS
s29_RA4PPS           EQU  RA4PPS
s29_RA5PPS           EQU  RA5PPS
s29_RB4PPS           EQU  RB4PPS
s29_RB5PPS           EQU  RB5PPS
s29_RB6PPS           EQU  RB6PPS
s29_RB7PPS           EQU  RB7PPS
s29_RC0PPS           EQU  RC0PPS
s29_RC1PPS           EQU  RC1PPS
s29_RC2PPS           EQU  RC2PPS
s29_RC3PPS           EQU  RC3PPS
s29_RC4PPS           EQU  RC4PPS
s29_RC5PPS           EQU  RC5PPS
s29_RC6PPS           EQU  RC6PPS
s29_RC7PPS           EQU  RC7PPS

;-----Bank30------------------
s30_CLCDATA          EQU  CLCDATA
s30_CLC1CON          EQU  CLC1CON
s30_CLC1POL          EQU  CLC1POL
s30_CLC1SEL0         EQU  CLC1SEL0
s30_CLC1SEL1         EQU  CLC1SEL1
s30_CLC1SEL2         EQU  CLC1SEL2
s30_CLC1SEL3         EQU  CLC1SEL3
s30_CLC1GLS0         EQU  CLC1GLS0
s30_CLC1GLS1         EQU  CLC1GLS1
s30_CLC1GLS2         EQU  CLC1GLS2
s30_CLC1GLS3         EQU  CLC1GLS3
s30_CLC2CON          EQU  CLC2CON
s30_CLC2POL          EQU  CLC2POL
s30_CLC2SEL0         EQU  CLC2SEL0
s30_CLC2SEL1         EQU  CLC2SEL1
s30_CLC2SEL2         EQU  CLC2SEL2
s30_CLC2SEL3         EQU  CLC2SEL3
s30_CLC2GLS0         EQU  CLC2GLS0
s30_CLC2GLS1         EQU  CLC2GLS1
s30_CLC2GLS2         EQU  CLC2GLS2
s30_CLC2GLS3         EQU  CLC2GLS3
s30_CLC3CON          EQU  CLC3CON
s30_CLC3POL          EQU  CLC3POL
s30_CLC3SEL0         EQU  CLC3SEL0
s30_CLC3SEL1         EQU  CLC3SEL1
s30_CLC3SEL2         EQU  CLC3SEL2
s30_CLC3SEL3         EQU  CLC3SEL3
s30_CLC3GLS0         EQU  CLC3GLS0
s30_CLC3GLS1         EQU  CLC3GLS1
s30_CLC3GLS2         EQU  CLC3GLS2
s30_CLC3GLS3         EQU  CLC3GLS3
s30_CLC4CON          EQU  CLC4CON
s30_CLC4POL          EQU  CLC4POL
s30_CLC4SEL0         EQU  CLC4SEL0
s30_CLC4SEL1         EQU  CLC4SEL1
s30_CLC4SEL2         EQU  CLC4SEL2
s30_CLC4SEL3         EQU  CLC4SEL3
s30_CLC4GLS0         EQU  CLC4GLS0
s30_CLC4GLS1         EQU  CLC4GLS1
s30_CLC4GLS2         EQU  CLC4GLS2
s30_CLC4GLS3         EQU  CLC4GLS3

;-----Bank31------------------
s31_STATUS_SHAD      EQU  STATUS_SHAD
s31_WREG_SHAD        EQU  WREG_SHAD
s31_BSR_SHAD         EQU  BSR_SHAD
s31_PCLATH_SHAD      EQU  PCLATH_SHAD
s31_FSR0L_SHAD       EQU  FSR0L_SHAD
s31_FSR0H_SHAD       EQU  FSR0H_SHAD
s31_FSR1L_SHAD       EQU  FSR1L_SHAD
s31_FSR1H_SHAD       EQU  FSR1H_SHAD
s31_STKPTR           EQU  STKPTR
s31_TOSL             EQU  TOSL
s31_TOSH             EQU  TOSH

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Reset vector																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
reset_vector			code	0x000
						movlb	18
						movlw	OSCCON1_HFINTOSC
						movwf	s18_OSCCON1
						goto	start

;		*		*		*		*		*		*		*		*		*		*		*		;
;		calculate pulse width for startup														;
;		*		*		*		*		*		*		*		*		*		*		*		;
; Vin*duty = Rcoil*Is + target_voltage
; duty = (Rcoil*Is + target_voltage)/Vin
STARTUP_VOLTAGE			equ		MOTOR_RESISTANCE*STARTUP_CURRENT/1000	; IR-drop, [mV]

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Interrupt Routine																		;
;		*		*		*		*		*		*		*		*		*		*		*		;

; interrupt handler is consist of pulse generator and voltage sampler.
;		the pulse generator is driven by T2 cooperatively with PWM5.
;		the voltage sampler is driven by T3 cooperatively with CCP3.
interrupt_handler		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						clrf	PCLATH
						movlb	0
						bcf		s0_PIR1,ADIF
						movlb	1
						movlw	ADCON0_MUX_FVR			; AN31(FVR output)
						xorwf	s1_ADCON0,W
						btfsc	STATUS,Z
						goto	f1_intr_fvr			; FVR
						movlw	ADCON0_MUX_LED				; AN7(LED, RC3 pin)
						xorwf	s1_ADCON0,W
						btfsc	STATUS,Z
						goto	f1_intr_led			; ANC5
						retfie

f1_intr_fvr:
						movlb	6
						movf	sampling_interval_L,W
						movwf	s6_CCPR3L
						movf	sampling_interval_H,W
						movwf	s6_CCPR3H

						movlb	1
						; use IIR LPF filter (x := 3*x/4 + ADRES) for ADC result (5p11) of FVR
						lsrf	s1_adres_FVR_3p13_H,W
						movwf	s1_adres_FVR_work_H
						rrf		s1_adres_FVR_3p13_L,W
						movwf	s1_adres_FVR_work_L

						lsrf	s1_adres_FVR_work_H,F
						rrf		s1_adres_FVR_work_L,F

						movf	s1_adres_FVR_work_L,W
						subwf	s1_adres_FVR_3p13_L,F
						movf	s1_adres_FVR_work_H,W
						subwfb	s1_adres_FVR_3p13_H,F

						movf	s1_ADRESL,W
						addwf	s1_adres_FVR_3p13_L,F
						movf	s1_ADRESH,W
						addwfc	s1_adres_FVR_3p13_H,F

						; adres_FVR_0p16 = adres_FVR_3p13 << 3
						lslf	s1_adres_FVR_3p13_L,W
						movwf	s1_adres_FVR_0p16_L
						rlf		s1_adres_FVR_3p13_H,W
						movwf	s1_adres_FVR_0p16_H
						lslf	s1_adres_FVR_0p16_L,F
						rlf		s1_adres_FVR_0p16_H,F
						lslf	s1_adres_FVR_0p16_L,F
						rlf		s1_adres_FVR_0p16_H,F

						btfsc	notification_flag,BIT_CLOSED_LOOP_CTRL
						call	closed_loop_control

						btfsc	notification_flag,BIT_OPEN_LOOP_CTRL
						call	open_loop_control

						btfsc	notification_flag,BIT_OUT_OF_CTRL
						call	out_of_control

						movlb	1
						; prepare for LED brightness control, set MUX to LED
						movlw	ADCON0_MUX_LED
						movwf	s1_ADCON0

						bsf		s1_PIE1,TMR2IE

						movlb	0
						; interrupt setup
						bcf		s0_PIR1,ADIF

						; wait for PWM update
						sleep
	if (PPS_RA2_init == B'100000')				; Rxy source is RA2, duty update
					movlb	2
					bcf		s2_LATA,LATA2		; turn off LED
	endif

						movlb	12
						; set new duty to duty register
						movf	pulse_width_L,W
						movwf	s12_PWM5DCH		;		d9,d8,d7,d6,d5,d4,d3,d2
						movf	pulse_width_F,W
						movwf	s12_PWM5DCL		;		d1,d0,...

						movlb	1
						bcf		s1_PIE1,TMR2IE

						; prepare for LED brightness control, set trigger source
						movlw	ADACT_LED
						movwf	s1_ADACT
						movlw	ADCON1_LED
						movwf	s1_ADCON1
						bcf		s1_ADCON0,GO

						; load lut entry into target_voltage and sampling_interval
						btfsc	notification_flag,BIT_LOAD_LUT_ENTRY
						call	load_lut_entry

						; save lut index into eeprom
						btfsc	notification_flag,BIT_UPDATE_LUT_INDEX
						call	save_lut_index
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Subroutine for Interrupt Routine														;
;		*		*		*		*		*		*		*		*		*		*		*		;
; closed_loop_control updates pulse_width to drive motor with constant speed
closed_loop_control:	movlb	1
						; target_ratio = target_voltage(12bit) * FVR(12bit)
						;   9p23 >> 13             6p10         3p13
						; proportional = 2*target_ratio
						movf	s1_target_voltage_L,W
						movwf	s1_voltage_L
						movf	s1_target_voltage_H,W
						movwf	s1_voltage_H
						call	f1_conv_volt_to_ratio
						movf	s1_ratio_H,W
						movwf	s1_target_ratio_H
						movwf	s1_proportional_H
						movf	s1_ratio_L,W
						movwf	s1_target_ratio_L			; 110
						movwf	s1_proportional_L
						clrf	s1_target_ratio_F
						lsrf	s1_target_ratio_H,F
						rrf		s1_target_ratio_L,F			; 6p10
						rrf		s1_target_ratio_F,F			; 6p18

						; target_ratio_half = target_ratio/2
						lsrf	s1_target_ratio_H,W
						movwf	s1_target_ratio_half_H
						rrf		s1_target_ratio_L,W
						movwf	s1_target_ratio_half_L
						rrf		s1_target_ratio_F,W
						movwf	s1_target_ratio_half_F

						; clear integration work
						clrf	s1_integration_work_F
						clrf	s1_integration_work_L
						clrf	s1_integration_work_H	; integration_work = 0

						movlw	10
						movwf	s1_back_emf_timeout

						; configure ADC for back-emf acquisition
						movlw	ADCON0_MUX_MOTOR
						movwf	s1_ADCON0
						movlw	ADCON1_MOTOR
						movwf	s1_ADCON1
						movlw	ADACT_MOTOR
						movwf	s1_ADACT

						; for synchronization on matching CCP1 and TMR1
						bsf		s1_PIE4,CCP1IE

						movlb	2
						; configure Comparator 2 for back-emf acquisition
						movlw	CM2CON0_init
						movwf	s2_CM2CON0
						movlw	CM2CON1_init
						movwf	s2_CM2CON1

						movlb	1
						; duty_limit = startup_voltage(13bit) * FVR(12bit)
						; 3p23 >> 13              3p10         0p13
						movf	s1_startup_voltage_L,W
						movwf	s1_voltage_L
						movf	s1_startup_voltage_H,W
						movwf	s1_voltage_H
						call	f1_conv_volt_to_ratio
						lsrf	s1_ratio_H,W
						movwf	s1_duty_limit_H
						rrf		s1_ratio_L,W
						movwf	s1_duty_limit_L			; 110

						movlb	2
						; reset Capacitor voltage to 0 in LPF
						bcf		s2_LATC,LATC_B_EMF_SENSE
						; disconnect power from motor
						bcf		s2_CM1CON0,C1POL

						movlb	12
						; duty buffer = 0
						clrf	s12_PWM5DCH
						clrf	s12_PWM5DCL

						movlb	2
						bsf		s2_LATC,LATC_B_EMF_SENSE

						movlb	4
						; turn on bias resister
						bsf		s4_WPUC,WPUC_B_EMF_SENSE

						movlb	0
						; restart new cycle with duty = 0
						decf	s0_PR2,W
						movwf	s0_TMR2

						movlb	2
; wait for discharging coil current in fast decay mode
						btfss	s2_CM2CON0,C2OUT
						bra		$-1

						; turn-ON HB2 LS FET 
						bsf		s2_CM1CON0,C1POL

; wait for discharging coil current
						movlw	DISCHARGING_TIME
						btfsc	s2_CM2CON0,C2OUT
						decfsz	WREG,F
						bra		$-2

						; turn off Comparator 2
						bcf		s2_CM2CON0,C2ON

						movlb	4
						; turn off bias resister
						bcf		s4_WPUC,WPUC_B_EMF_SENSE

						movlb	0
						; stop PWM timer
						bcf		s0_T2CON,TMR2ON
						movf	s0_PR2,W
						movwf	s0_TMR2

						; enter setup time of A/D conversion
						movlw	LOW	 ((ADC_INTERVAL - CONVERSION_TIME)*(FCY/1000000))
						movwf	s0_TMR1L
						movlw	HIGH ((ADC_INTERVAL - CONVERSION_TIME)*(FCY/1000000))
						movwf	s0_TMR1H
						movlw	T1CON_init
						movwf	s0_T1CON

						bcf		s0_PIR1,ADIF
						bcf		s0_PIR4,CCP1IF

						movlb	13
						; resume half-bridge operation at next cycle
						bcf		s13_CWG1AS0,SHUTDOWN

						movlb	12
						; set PWM5DC to start-up pulse width
						movf	startup_pulse_width_L,W
						movwf	s12_PWM5DCH
						movlw	0x40
						movwf	s12_PWM5DCL

range_check_back_emf:	; wait for start of A/D convertion
						sleep

						movlb	0
						bcf		s0_PIR4,CCP1IF

						; wait for completion of A/D convertion
						sleep
						bcf		s0_PIR1,ADIF
						movlb	1
						; start Back-EMF sampling
						; check voltage
						movf	s1_adjustment_range_L,W
						subwf	s1_ADRESL,W
						movf	s1_adjustment_range_H,W
						subwfb	s1_ADRESH,W
						btfss	STATUS,C
						bra		convert_back_emf

						; check time-out
						decfsz	s1_back_emf_timeout,F
						bra		range_check_back_emf

						; timed-out
						clrf	s1_integral_F
						clrf	s1_integral_L
						clrf	s1_integral_H

convert_back_emf:
						; wait for start of A/D convertion
						sleep

						movlb	0
						; turn-OFF Timer 1
						bcf		s0_T1CON,TMR1ON

						; restart PWM generator after ADC sampling ends
						bsf		s0_T2CON,TMR2ON

						movlb	1
						; mask interrupt from compare match between CCP1 and Timer 1
						bcf		s1_PIE4,CCP1IE

						; IIR LPF, back_emf_ratio = 1*back_emf_ratio/2 + ADRES/Vin
						lsrf	s1_back_emf_ratio_H,F
						rrf		s1_back_emf_ratio_L,F

						; wait for completion of A/D convertion
						sleep

						; adres_back_emf_ratio(16bit) = ADRES(10bit) * FVR(12bit)
						;          10p22 >> 12            7p9         3p13
						movf	s1_ADRESL,W
						movwf	s1_voltage_L
						movwf	s1_adres_back_emf_T0_L
						movf	s1_ADRESH,W
						movwf	s1_voltage_H
						movwf	s1_adres_back_emf_T0_H
						call	f1_conv_volt_to_ratio

						; back_emf_ratio += adres_back_emf_ratio
						movf	s1_ratio_L,W
						addwf	s1_back_emf_ratio_L,F
						movf	s1_ratio_H,W
						addwfc	s1_back_emf_ratio_H,F

						; proportional -= back_emf_ratio
						movf	s1_back_emf_ratio_L,W
						subwf	s1_proportional_L,F
						movf	s1_back_emf_ratio_H,W
						subwfb	s1_proportional_H,F

		if P_GAIN == 3
						; proportional *= 2
						lslf	s1_proportional_L,F
						rlf		s1_proportional_H,F
		endif
		if P_GAIN == 1
						; proportional /= 2
						asrf	s1_proportional_H,F
						rrf		s1_proportional_L,F
		endif

						; pulse_width = proportional + integral
						movf	s1_integral_L,W
						addwf	s1_proportional_L,W
						movwf	pulse_width_L			; _L holds lower byte of 6p10
						movf	s1_integral_H,W
						addwfc	s1_proportional_H,W
						movwf	pulse_width_F			; _F holds higher byte of 6p10

						; calculate startup pulse width
						movf	s1_target_ratio_L,W
						subwf	pulse_width_L,W			; _L holds lower byte of 6p10
						movwf	startup_pulse_width_L
						movf	s1_target_ratio_H,W
						subwfb	pulse_width_F,W			; _F holds higher byte of 6p10
						btfsc	STATUS,Z		; if higher byte is not 0, overflow
						bra		$+4
						clrf	startup_pulse_width_L
						btfss	WREG,7
						comf	startup_pulse_width_L,F

						movlb	12
						; set PWM5 to start-up pulse width
						movf	startup_pulse_width_L,W
						movwf	s12_PWM5DCH

						movlb	0
						bcf		s0_PIR1,TMR2IF
	if (PPS_RA2_init == B'100000')				; Rxy source is RA2, duty update
					movlb	2
					bsf		s2_LATA,LATA2		; turn on LED
	endif

						; if control voltage is negative, pulse_width is set to 0.
						btfss	pulse_width_F,7			; _F holds higher byte of 6p10
						bra		$+3
						clrf	pulse_width_F			; _F holds higher byte of 6p10
						clrf	pulse_width_L			; _L holds lower byte of 6p10

						movlb	1
						; limit positive duty
						movf	s1_duty_limit_L,W
						subwf	pulse_width_L,W			; _L holds lower byte of 6p10
						movf	s1_duty_limit_H,W
						subwfb	pulse_width_F,W			; _F holds higher byte of 6p10
						btfss	STATUS,C
						bra		$+5
						movf	s1_duty_limit_L,W
						movwf	pulse_width_L			; _L holds lower byte of 6p10
						movf	s1_duty_limit_H,W
						movwf	pulse_width_F			; _F holds higher byte of 6p10

						; change from 6p10 to 0p16
						; pulse_width_F         pulse_width_L
						; 0,0,0,0,0,0,d9,d8     d7,d6,d5,d4,d3,d2,d1,d0
						lsrf	pulse_width_F,F
						rrf		pulse_width_L,F
						rrf		pulse_width_F,F
						rrf		pulse_width_L,F
						rrf		pulse_width_F,F
						; pulse_width_F         pulse_width_L
						; d1,d0,0,0,0,0,0,0     d9,d8,d7,d6,d5,d4,d3,d2

						; obtain proprtional value to integrate
						btfsc	notification_flag,BIT_PP_CTRL
						call	f1_PP_control

						btfsc	notification_flag,BIT_PI_CTRL
						call	f1_PI_control

I_GAIN_fxp0p9			set		(512/I_TIME)&0x1FF
		while I_GAIN_fxp0p9
				if (I_GAIN_fxp0p9 & 0x100)
						movf	s1_integration_work_F,W
						addwf	s1_integral_F,F
						movf	s1_integration_work_L,W
						addwfc	s1_integral_L,F
						movf	s1_integration_work_H,W
						addwfc	s1_integral_H,F
				endif
I_GAIN_fxp0p9			set		(I_GAIN_fxp0p9<<1)&0x1FF
				if (I_GAIN_fxp0p9 != 0)
						asrf	s1_integration_work_H,F
						rrf		s1_integration_work_L,F
						rrf		s1_integration_work_F,F
				endif
		endw

						; limit integral_voltage
						btfsc	s1_integral_H,7
						bra		__integral_neg_lim
__integral_pos_lim:		movf	s1_duty_limit_L,W
						subwf	s1_integral_L,W
						movf	s1_duty_limit_H,W
						subwfb	s1_integral_H,W
						btfss	STATUS,C
						return
						movf	s1_duty_limit_L,W
						movwf	s1_integral_L
						movf	s1_duty_limit_H,W
						movwf	s1_integral_H
						return
__integral_neg_lim:		movf	s1_duty_limit_L,W
						addwf	s1_integral_L,W
						movf	s1_duty_limit_H,W
						addwfc	s1_integral_H,W
						btfsc	STATUS,C
						return
						comf	s1_duty_limit_L,W
						movwf	s1_integral_L
						comf	s1_duty_limit_H,W
						movwf	s1_integral_H
						return

; f1_PP_control is used when motor speed is set to 1x.
f1_PP_control:			; detect a valley by sequence of back-EMF voltages
								; diff = back_emf_Tm1 - back_emf_T0
	if PULSE_PER_SLOT == 3
						movf	s1_adres_back_emf_T0_L,W
						subwf	s1_adres_back_emf_Tm1_L,F
						movwf	s1_adres_back_emf_Tm1_L
						movf	s1_adres_back_emf_T0_H,W
						subwfb	s1_adres_back_emf_Tm1_H,F
						movwf	s1_adres_back_emf_Tm1_H
						rlf		s1_slope,F				; 1: falling or equal	0: rising
						btfss	s1_slope,0				; expecting a sequence of B'10'
						btfss	s1_slope,1
						bra		__not_valley

__valley:				; valley is detected
						movf	s1_target_ratio_F,W
						subwf	s1_integration_work_F,F
						movf	s1_target_ratio_L,W
						subwfb	s1_integration_work_L,F
						movf	s1_target_ratio_H,W
						subwfb	s1_integration_work_H,F
						return
	else
						movf	s1_adres_back_emf_T0_L,W
						subwf	s1_adres_back_emf_Tm1_L,F
						movf	s1_adres_back_emf_T0_H,W
						subwfb	s1_adres_back_emf_Tm1_H,F

						asrf	s1_slope,F				; insert last result 'unchanged' to sequence

						movlw	(LOW 3)
						addwf	s1_adres_back_emf_Tm1_L,W
						movlw	(HIGH 3)
						addwfc	s1_adres_back_emf_Tm1_H,W
						btfsc	WREG,7
						bsf		s1_slope,7				; overwrite to 1 as falling (diff < -threshold)

						movlw	(LOW -4)
						addwf	s1_adres_back_emf_Tm1_L,W
						movlw	(HIGH -4)
						addwfc	s1_adres_back_emf_Tm1_H,W
						btfss	WREG,7
						bcf		s1_slope,7				; overwrite to 0 as rising (+threshold <= diff )

						movf	s1_adres_back_emf_T0_L,W
						movwf	s1_adres_back_emf_Tm1_L
						movf	s1_adres_back_emf_T0_H,W
						movwf	s1_adres_back_emf_Tm1_H

						lslf	s1_slope,W
						xorwf	s1_slope,W				; expecting a sequence of B'10' or B'01'
						btfss	WREG,7
						bra		__not_valley

__valley:				; valley/peak is detected
						movf	s1_target_ratio_half_F,W
						subwf	s1_integration_work_F,F
						movf	s1_target_ratio_half_L,W
						subwfb	s1_integration_work_L,F
						movf	s1_target_ratio_half_H,W
						subwfb	s1_integration_work_H,F
						return
	endif

__not_valley:			; valley/peak is not detected
						movf	s1_target_ratio_half_F,W
						movwf	s1_integration_work_F
						movf	s1_target_ratio_half_L,W
						movwf	s1_integration_work_L
						movf	s1_target_ratio_half_H,W
						movwf	s1_integration_work_H
						return

; f1_PI_control is used when motor speed is set to 2x.
f1_PI_control:			asrf	s1_proportional_H,W
						movwf	s1_integration_work_H
						rrf		s1_proportional_L,W
						movwf	s1_integration_work_L
						rrf		s1_integration_work_F,F
						return

; open_loop_control updates pulse_width to drive motor with constant voltage
open_loop_control:		movlb	2
						; connect
						bsf		s2_CM1CON0,C1POL

						movlb	13
						; resume half-bridge operation at next cycle
						bcf		s13_CWG1AS0,SHUTDOWN

						movlb	0
						bcf		s0_PIR1,TMR2IF

						clrf	startup_pulse_width_L

						movlb	1
						; set duty ratio to adres_FVR*1.5 * 200/256 to obtain target voltage 1.5V
						; estimate the result by adres_FVR*9/8 to use simple operation
						; adres_break_in = adres_FVR * 9/8 = adres_FVR + (adres_FVR>>3)
						movf	s1_adres_FVR_3p13_L,W
						addwf	s1_adres_FVR_0p16_L,W
						movwf	s1_adres_break_in_F
						movf	s1_adres_FVR_3p13_H,W
						addwfc	s1_adres_FVR_0p16_H,W
						movwf	s1_adres_break_in_L
						; s1_adres_break_in_L               s1_adres_break_in_F    
						; d9,d8,d7,d6,d5,d4,d3,d2     d1,d0,0,0,0,0,0,0

						; pulse_width += s1_adres_break_in/256
						movf	s1_adres_break_in_L,W
						addwf	pulse_width_F,F
						clrw
						addwfc	pulse_width_L,F

						; select smallest
						movf	s1_adres_break_in_F,W
						subwf	pulse_width_F,W
						movf	s1_adres_break_in_L,W
						subwfb	pulse_width_L,W
						btfss	STATUS,C
						return
						movf	s1_adres_break_in_F,W
						movwf	pulse_width_F
						movf	s1_adres_break_in_L,W
						movwf	pulse_width_L
						return

; out_of_control clears pulse_width when motor is stopping.
out_of_control:			movlb	2
						; disconnect
						bcf		s2_CM1CON0,C1POL

						movlb	0
						bcf		s0_PIR1,TMR2IF

						movlw	0xFF
						movwf	startup_pulse_width_L
						clrf	pulse_width_F
						clrf	pulse_width_L
						return

; ratio       = voltage * FVR(12bit)
;  9p23 >> 12      6p10  3p13
f1_conv_volt_to_ratio:	clrf	s1_ratio_L
						clrf	s1_ratio_H
BIT_POS					set		0
		while BIT_POS < 11
						movf	s1_voltage_L,W
						btfsc	s1_adres_FVR_3p13_L+(BIT_POS/8),(BIT_POS%8)
						addwf	s1_ratio_L,F
						movf	s1_voltage_H,W
						btfsc	s1_adres_FVR_3p13_L+(BIT_POS/8),(BIT_POS%8)
						addwfc	s1_ratio_H,F
						lsrf	s1_ratio_H,F
						rrf		s1_ratio_L,F
BIT_POS					set		BIT_POS + 1
		endw
						movf	s1_voltage_L,W		; last cycle
						btfsc	s1_adres_FVR_3p13_L+(BIT_POS/8),(BIT_POS%8)
						addwf	s1_ratio_L,F
						movf	s1_voltage_H,W
						btfsc	s1_adres_FVR_3p13_L+(BIT_POS/8),(BIT_POS%8)
						addwfc	s1_ratio_H,F
						lsrf	s1_ratio_H,F
						rrf		s1_ratio_L,F
						return				; 8*12 + 4 = 100

; read LED current and control constant brightness
; Duty ratio = dividend/divider
;		dividend = adres_FVR/Vin*8	= 8V/Vin in 6p10 fxp
;		divider	 = 1023 - adres_LED	= Iled*R/Vin in 6p10 fxp
;	Iled*ratio = 8V/R >> 5
f1_intr_led:			
						; configure ADC for input voltage acquisition
						movlw	ADCON0_MUX_FVR
						movwf	s1_ADCON0
						movlw	ADCON1_VINPUT
						movwf	s1_ADCON1
						movlw	ADACT_VINPUT
						movwf	s1_ADACT

						; update LED current 
						btfss	notification_flag,BIT_UPDATE_LED
						bra		skip_LED_control
						bcf		notification_flag,BIT_UPDATE_LED

						decfsz	s1_batt_blink,F
						bra		skip_LED_control
						bsf		s1_batt_blink,4

						movlw	LOW (65536*1000/LOW_BATT_ERROR)
						btfss	s1_batt_flag,0
						movlw	LOW (65536*1000/LOW_BATT_WARNING)
						subwf	s1_adres_FVR_0p16_L,W
						movlw	HIGH (65536*1000/LOW_BATT_ERROR)
						btfss	s1_batt_flag,0
						movlw	HIGH (65536*1000/LOW_BATT_WARNING)
						subwfb	s1_adres_FVR_0p16_H,W
						rlf		s1_batt_flag,F

skip_LED_control:		movlw	PWM6DCH_init
						movwf	led_pulse_width_H
						clrf	led_pulse_width_L

						movlb	18
						movf	s18_OSCSTAT1,W
						movlb	1

						btfss	s1_batt_flag,0
						btfsc	WREG,HFOR
						bra		led_turn_off

led_turn_on:			; is LED brightness controll halted?
						btfss	s1_TRISC,TRISC_LED_K
						bra		led_calculate_duty

						; LED off -> on
						; resume LED brightness controll at next cycle
						bcf		s1_TRISC,TRISC_LED_K
						bra		div_end

led_turn_off:			; halt LED brightness controll
						bsf		s1_TRISC,TRISC_LED_K
						bra		div_end

						; LED on -> on
led_calculate_duty:
						; read LED current
						comf	s1_ADRESL,W
						movwf	s1_divider_L
						comf	s1_ADRESH,W
						andlw	0x03
						movwf	s1_divider_H

						movf	s1_adres_FVR_0p16_L,W
						movwf	s1_dividend_F
						movf	s1_adres_FVR_0p16_H,W
						movwf	s1_dividend_L
						clrf	s1_dividend_H

						; dividend_H                 dividend_L                dividend_F
						;  0, 0, 0, 0, 0, 0, 0, 0     0, 0,d9,d8,d7,d6,d5,d4    d3,d2,d1,d0, 0, 0, 0, 0
						; divider_H                  divider_L
						;  0, 0, 0, 0, 0, 0,d9,d8    d7,d6,d5,d4,d3,d2,d1,d0
						clrf	led_pulse_width_H
						bsf		led_pulse_width_L,6
						bra		div_begin
div_pos:
						lslf	led_pulse_width_L,F
						rlf		led_pulse_width_H,F
						btfsc	STATUS,C
						bra		div_end
div_begin:				lslf	s1_dividend_F,F
						rlf		s1_dividend_L,F
						rlf		s1_dividend_H,F
						movf	s1_divider_L,W
						subwf	s1_dividend_L,F
						movf	s1_divider_H,W
						subwfb	s1_dividend_H,F
						btfsc	STATUS,C
						bra		div_pos
						;bra		div_neg
div_neg:
						lslf	led_pulse_width_L,F
						rlf		led_pulse_width_H,F
						bsf		led_pulse_width_L,6			; output is negative logic
						btfsc	STATUS,C
						bra		div_end
						lslf	s1_dividend_F,F
						rlf		s1_dividend_L,F
						rlf		s1_dividend_H,F
						movf	s1_divider_L,W
						addwf	s1_dividend_L,F
						movf	s1_divider_H,W
						addwfc	s1_dividend_H,F
						btfsc	STATUS,C
						bra		div_pos
						bra		div_neg
div_end:												; 15*10+3 = 153
						movlb	12
						; update duty register(PWM6)
						movf	led_pulse_width_L,W
						movwf	s12_PWM6DCL
						movf	led_pulse_width_H,W
						movwf	s12_PWM6DCH
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Main routine																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
start:	
	if (PPS_RA2_init > B'11111')
						movlw	(1<<FVRMD)|(1<<CLKRMD)
						movwf	s18_PMD0
						movlw	0xFF
						movwf	s18_PMD1
						movwf	s18_PMD2
						movwf	s18_PMD3
						movwf	s18_PMD4
						movwf	s18_PMD5
	endif

						bcf		INTCON,GIE
						bsf		INTCON,PEIE

						movlb	17
						; analyze reset
						movf	s17_PCON0,W
						movwf	notification_flag
						movlw	PCON_init
						movwf	s17_PCON0

						movlw	LOW (~((1<<RA3)|(1<<RA1)))	; short sleep: RA3 & RA1 are active
						btfss	notification_flag,NOT_POR
						bra		wake_up						; Power on reset
						btfss	notification_flag,NOT_BOR
						bra		fall_into_a_short_sleep		; Brownout reset
						btfss	notification_flag,NOT_RI
						bra		fall_into_a_short_sleep		; Reset instruction
fall_into_a_dead_sleep: movlw	LOW (~(1<<RA3))				; dead sleep: RA3 is active
fall_into_a_short_sleep:movlb	3
						movwf	s3_ANSELA
						comf	WREG,W
						bsf		s3_VREGCON,VREGPM			; low power sleep

						movlb	4
						movwf	s4_WPUA

						movlb	7
						movwf	s7_IOCAN				; interrupt on change (negative edge)
						clrf	s7_IOCAP
						clrf	s7_IOCAF

						movlb	0
						bcf		s0_PIR0,IOCIF
						xorwf	s0_PORTA,W				; negative level

						movlb	7
						movwf	s7_IOCAF

						movlb	1
						bsf		s1_PIE0,IOCIE
						sleep				; method to wake-up is assertion of MCLR, POR or IOC
						bcf		s1_PIE0,IOCIE
						movlb	7
						clrf	s7_IOCAN				; interrupt on change (negative edge)

						btfsc	s7_IOCAF,IOCAF3 ; RA3
						bra		wake_up					; power button is pushed

						btfss	s7_IOCAF,IOCAF1 ; RA1
						bra		fall_into_a_dead_sleep	; spurious

						call	restore_lut_index		; rewind sw is pushed while sleep
						call	set_idx_to_lut_start
						bsf		notification_flag,BIT_UPDATE_LUT_INDEX
						call	save_lut_index
						btfsc	notification_flag,BIT_UPDATE_LUT_INDEX
						bra		$-2
						bra		fall_into_a_dead_sleep

wake_up:				; clear data memory
						movlw	0x20
						movwf	FSR0H
						clrf	FSR0L			; linear data memory is starting from 0x2000
						movlw	(1024-16)/4
						movwf	0x7F
						clrw
						movwi	FSR0++
						movwi	FSR0++
						movwi	FSR0++
						movwi	FSR0++
						decfsz	0x7F,F
						bra		$-5

						movlw	0xF0
						movwf	FSR0L
						clrf	FSR0H			; common memory is starting from 0x00F0
						clrf	INDF0
						incfsz	FSR0L,F
						bra		$-2
						; FSR0H is set to zero except for initialization procedure.

						call	osc_init

						; restore lut_index from DATA EEPROM
						call	restore_lut_index

						; load lut entry into target_voltage and sampling_interval
						call	load_lut_entry

	.if 0
						movlb	28
						movlw	0x55
						movwf	s28_PPSLOCK
						movlw	0xAA
						movwf	s28_PPSLOCK
						bcf		s28_PPSLOCK,PPSLOCKED
	.endif

						movlb	29
				;											;  1: VDD
				;		clrf	s29_RA5PPS					;  2: OSC1
				;		clrf	s29_RA4PPS					;  3: OSC2
				;											;  4: RA3/Vpp
				;		clrf	s29_RC5PPS					;  5: ANC5
						movlw	PPS_LED_K_init
						movwf	s29_RC4PPS					;  6: PWM6
				;		clrf	s29_RC3PPS					;  7: ANC3
						movlw	PPS_HB1_LS_init
						movwf	s29_RC6PPS					;  8: CWG1B
						movwf	s29_RC7PPS					;  9: CWG1B
						movwf	s29_RB7PPS					; 10: CWG1B
						movlw	PPS_HB1_HS_init
						movwf	s29_RB6PPS					; 11: CWG1A
						movwf	s29_RB5PPS					; 12: CWG1A
						movlw	PPS_HB2_LS_init
						movwf	s29_RB4PPS					; 13: CM1OUT
						movwf	s29_RC2PPS					; 14: CM1OUT
				;		clrf	s29_RC1PPS					; 15: RC1
				;		clrf	s29_RC0PPS					; 16: RC0
	if (PPS_RA2_init <= B'11111')
						movlw	PPS_RA2_init
						movwf	s29_RA2PPS					; 17: RA2
	endif
				;		clrf	s29_RA1PPS					; 18: RA1/ICSPCLK
				;		clrf	s29_RA0PPS					; 19: RA0/ICSPDAT
				;											; 20: VSS

	if (PPS_RA2_init > B'11111')
						movlb	18
						movlw	(1<<CLKRMD)|(1<<IOCMD)
						movwf	s18_PMD0
						movlw	(1<<NCOMD)|(1<<TMR5MD)|(1<<TMR0MD)
						movwf	s18_PMD1
						movlw	(1<<DACMD)
						movwf	s18_PMD2
						movlw	(1<<CWG2MD)|(1<<CCP4MD)|(1<<CCP2MD)
						movwf	s18_PMD3
						movlw	(1<<UART1MD)|(1<<MSSP2MD)|(1<<MSSP1MD)
						movwf	s18_PMD4
						movlw	(1<<CLC4MD)|(1<<CLC3MD)|(1<<CLC2MD)|(1<<CLC1MD)|(1<<DSMMD)
						movwf	s18_PMD5
	endif

						movlb	4
						; enable pull-up for input pin
						movlw	WPUA_init
						movwf	s4_WPUA
						movlw	WPUC_init
						movwf	s4_WPUC

						movlb	5
						; timer assignment
						movlw	CCPTMRS_init
						movwf	s5_CCPTMRS

						; setup CCP1 for ADC trigger of motor
						movlw	CCP1CON_init
						movwf	s5_CCP1CON
						movlw	LOW	 (ADC_INTERVAL*(FCY/1000000) - 1)
						movwf	s5_CCPR1L
						movlw	HIGH (ADC_INTERVAL*(FCY/1000000) - 1)
						movwf	s5_CCPR1H
						movlw	ODCONC_init
						movwf	s5_ODCONC

						movlb	6
						; setup CCP3 for ADC trigger of FVR
						movlw	CCP3CON_init
						movwf	s6_CCP3CON
						clrf	s6_CCPR3L
						movlw	0x10
						movwf	s6_CCPR3H

						movlb	2
						; Fixed Voltage for ADC
						movlw	FVRCON_init
						movwf	s2_FVRCON

						; disconnect power from motor
						movlw	CM1CON0_init
						movwf	s2_CM1CON0

						movlb	3
						; enable input buffer
						movlw	ANSELA_init
						movwf	s3_ANSELA
						movlw	ANSELC_init
						movwf	s3_ANSELC

						movlb	13
			;			movlw	CWG1CLKCON_init
			;			movwf	s13_CWG1CLKCON
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
			;			movlw	CWG1STR_init
			;			movwf	s13_CWG1STR

						movlb	12
						; timer assignment
						movlw	PWMTMRS_init
						movwf	s12_PWMTMRS

						; setup PWM5 for motor PWM generator 
						clrf	s12_PWM5DCH
						clrf	s12_PWM5DCL
						movlw	PWM5CON_init
						movwf	s12_PWM5CON

						; setup PWM6 for LED brightness control
						movlw	PWM6DCH_init
						movwf	s12_PWM6DCH
						clrf	s12_PWM6DCL
						movlw	PWM6CON_init
						movwf	s12_PWM6CON

						movlb	8
						; start timer for ADC trigger
						clrf	s8_TMR3L
						clrf	s8_TMR3H
						movlw	T3CON_init
						movwf	s8_T3CON

						; start key polling timer
						movlw	PR4_init
						movwf	s8_PR4
						clrf	s8_TMR4
						movlw	T4CON_init
						movwf	s8_T4CON

						; start PWM controller for LED
						movlw	PR6_init
						movwf	s8_PR6
						movwf	s8_TMR6
						movlw	T6CON_init
						movwf	s8_T6CON

						movlb	0
						; start PWM controller for motor
						movlw	PR2_init
						movwf	s0_PR2
						movwf	s0_TMR2
						movlw	T2CON_init
						movwf	s0_T2CON

						; interrupt setup
						clrf	s0_PIR1

						; initial key state
						bsf		s0_sw_transient,RA3
						bsf		s0_sw_state,RA3

						movlb	1
						; time of first voltage check is 320ms from start
						bsf		s1_batt_blink,4

						; setup ADC
						movlw	ADCON0_MUX_FVR
						movwf	s1_ADCON0	; set MUX to FVR
						movlw	ADCON1_VINPUT
						movwf	s1_ADCON1
						movlw	ADACT_VINPUT
						movwf	s1_ADACT

						; change WDT period from 2s to 64ms
						movlw	WDTCON_init
						movwf	s1_WDTCON

						; interrupt enable
						bsf		s1_PIE1,ADIE

						; output enable
						movlw	TRISA_active
						movwf	s1_TRISA
						clrf	s1_TRISB
						movlw	TRISC_active
						movwf	s1_TRISC

	if (PPS_RA2_init == B'11110')				; Rxy source is CLKR
						movlb	6
						bcf		s6_SLRCONA,SLRA2
						movlb	7
						movlw	CLKRCON_init
						movwf	s7_CLKRCON
	endif
	if (PPS_RA2_init == B'00100')				; Rxy source is CLC1OUT
						movlb	30
						movlw	CLC1CON_init
						movwf	s30_CLC1CON
						movlw	CLC1POL_init
						movwf	s30_CLC1POL
						movlw	12				; CCP1
				;		movlw	35				; PR6 match
				;		movlw	26				; PR2 match
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

main_loop:				;movlb	0
						; oscillator controll
						call	f0_osc_ctrl

						call	f0_read_keys

						; update lut entry
						call	advance_lut_index

						; if reset button has released, MCU sleep
						btfsc	s0_sw_toggle,BIT_SW_RESET
						bra		$+3
						btfss	s0_sw_state,BIT_SW_RESET
						bra		prepare_for_sleep	; toggle == 0 and state == 0

	if (REWIND_TIMEOUT != 0)
						; wait for timeout
						movlw	LOW  (lut_end + _SIZEOF_LUT_ENTRY*REWIND_TIMEOUT/LUT_UPDATE_PERIOD)
						subwf	address_lut_L,W
						movlw	HIGH (lut_end + _SIZEOF_LUT_ENTRY*REWIND_TIMEOUT/LUT_UPDATE_PERIOD)
						subwfb	address_lut_H,W
						btfsc	STATUS,C
						bra		prepare_for_sleep	; timeout
	endif

						call	f0_get_notification
						movwf	notification_flag
						bra		main_loop

f0_get_notification:
						; if origin flag is set, index is reset to lut_start
						btfsc	s0_sw_state,BIT_SW_ORIGIN
						call	set_idx_to_lut_start

						; check address_lut is less than lut_end
						movlw	LOW lut_end
						subwf	address_lut_L,W
						movlw	HIGH lut_end
						subwfb	address_lut_H,W
	if (REWIND_TIMEOUT != 0)
						btfsc	STATUS,C
						retlw	CTRL_MOTOR_STOP
	else
						btfsc	STATUS,C
						call	set_idx_to_lut_start ; wrap-round to lut_start
	endif
						; if limit flag is set, index is set to lut_end
						btfss	s0_sw_pressed,BIT_SW_LIMIT
						bra		$+3
						call	set_idx_to_lut_end
						retlw	CTRL_MOTOR_STOP

						; stop sw is pushed
						btfsc	s0_sw_state,BIT_SW_STOP
						retlw	CTRL_MOTOR_STOP

						; power sw is pushed
						btfss	s0_sw_toggle,BIT_SW_RESET
						retlw	CTRL_MOTOR_BREAK_IN

						; twice speed sw is pushed
						btfsc	s0_sw_state,BIT_SW_DOUBLE_SPEED
						retlw	CTRL_MOTOR_2x	; set motor speed to 2x

						; none of sw is pushed.
						retlw	CTRL_MOTOR_1x	; set motor speed to 1x

prepare_for_sleep:		; interrupt disable
						bcf		INTCON,GIE

						movlb	2
						; disconnect power from motor
						bcf		s2_CM1CON0,C1POL

						movlb	17
						; wait for completion of EEPROM write
						btfsc	s17_EECON1,WR
						bra		$-1

						reset

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Oscillator Controll																		;
;		*		*		*		*		*		*		*		*		*		*		*		;
f0_osc_ctrl:			movlb	1
						; wait for event
						bcf		INTCON,GIE
						bsf		s1_PIE2,TMR4IE
						sleep
						bcf		s1_PIE2,TMR4IE
						bsf		INTCON,GIE
						movlb	0
						; wait for polling timer match
						btfss	s0_PIR2,TMR4IF
						bra		$-8

						bcf		s0_PIR2,TMR4IF			; 50Hz

						btfss	s0_PIR2,OSFIF	; check fail-safe clock operation flag
						bra		__osc_ok

						decfsz	s0_osc_recovery_delay,F
						return

						; clear fail-safe clock operation flag
						bcf		s0_PIR2,OSFIF
						; fall through

osc_init:				movlb	18
						; enable IDLE mode
						bsf		s18_CPUDOZE,IDLEN
						; switch to external osc.
						movlw	OSCCON1_EXTOSC
						movwf	s18_OSCCON1
						movlb	0
						; initialize oscillator recovery delay
__osc_ok:				movlw	KEY_POLLING_FREQ
						movwf	s0_osc_recovery_delay
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Key Handling																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
f0_read_keys:			movf	s0_sw_state,W
						movwf	s0_sw_released
						movwf	s0_sw_pressed

						; reduce chattering of key
						swapf	s0_PORTC,W
						andlw	B'00110000'
						iorwf	s0_PORTA,W				; {0, 0, RC1, RC0, RA3, 0, RA1, RA0}
						xorlw	B'00111011'		; invert logic (from negative to positive)

						xorwf	s0_sw_transient,W		; W = changed SW mask
						andwf	s0_sw_state,F	; erase unchanged bits
						xorwf	s0_sw_transient,F		; update changed bits
						xorlw	0xFF			; W = unchanged SW mask
						andwf	s0_sw_transient,W		; W = unchanged bits
						iorwf	s0_sw_state,F	; update unchanged bits

						; if RA0 == RA1, ignore RA1 and RA0
						; if RC0 == RC1, ignore RC1 and RC0
						lslf	s0_sw_state,W
						xorwf	s0_sw_state,W
						iorlw	B'11011101'		; mask = {1, 1, RC1^RC0, 1, 1, 1, RA1^RA0, 1}
						andwf	s0_sw_state,F
						asrf	WREG,F			; mask = {1, 1, 1, RC1^RC0, 1, 1, 1, RA1^RA0}
						andwf	s0_sw_state,F

						comf	s0_sw_state,W
						andwf	s0_sw_released,F		; released SW(1: released)
						iorwf	s0_sw_pressed,F
						comf	s0_sw_pressed,F			; pressed SW(1: pressed)

						movf	s0_sw_released,W
						xorwf	s0_sw_toggle,F

			;			clrwdt					; cleared by sleep instruction in interrupt
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Data EEPROM																				;
;		*		*		*		*		*		*		*		*		*		*		*		;
restore_lut_index:		movlw	0x70
						movwf	FSR0H					; Select Data EEPROM
						clrf	FSR0L					; n = 0
__find_last_entry_lp0:	; read lower byte and store it to address_lut_L
						moviw	0[FSR0]
						movwf	address_lut_L			; address_lut_L = eeprom[n]

						; read upper byte
						incf	FSR0L,F					; n++ with wrap-round

						; exit if FSR0L wrapped round to 0
						btfsc	STATUS,Z
						bra		__found_last_entry

						; find difference in sequence bits between lower and upper byte
						xorwf	INDF0,W					; W = eeprom[n]^eeprom[n+1]
						btfss	WREG,7
						bra		__find_last_entry_lp0

__found_last_entry:		; read upper byte and store it to address_lut_H
						moviw	0[FSR0]
						movwf	address_lut_H			; address_lut_H = eeprom[n+1]
						movwf	sequence_bit_L			; sequence_bit_L = eeprom[n+1]

						; point to lower byte of next write
						movf	FSR0L,W					; is equal to n+1
						movwf	eeprom_adrs

						; load sequence bit of next MSB
						incf	FSR0L,F					; n++ with wrap-round
						moviw	0[FSR0]
						movwf	sequence_bit_H			; sequence_bit_H = eeprom[n+2]

						; clear sequence bit
						lslf	address_lut_L,F			; LSB = 0
						bsf		address_lut_H,7			; MSB = 1

						movlw	0x80
						; invert sequence bit
						xorwf	sequence_bit_L,F
						; clear eeprom data except for sequence bit
						andwf	sequence_bit_L,F
						andwf	sequence_bit_H,F

						; upper range check
						movlw	LOW lut_end
						subwf	address_lut_L,W
						movlw	HIGH lut_end
						subwfb	address_lut_H,W
						btfsc	STATUS,C
						bra		set_idx_to_lut_start

						; lower range check
						movlw	LOW lut_start
						subwf	address_lut_L,W
						movlw	HIGH lut_start
						subwfb	address_lut_H,W
						btfsc	STATUS,C
						bra		__reset_counter_lut

set_idx_to_lut_start:	movlw	HIGH lut_start
						movwf	address_lut_H
						movlw	LOW lut_start
						movwf	address_lut_L
						bra		__reset_counter_lut

set_idx_to_lut_end:		movlw	HIGH lut_end
						movwf	address_lut_H
						movlw	LOW lut_end
						movwf	address_lut_L
						bra		__reset_counter_lut

advance_lut_index:		decfsz	counter_lut,F
						return
						movlw	LOW _SIZEOF_LUT_ENTRY
						addwf	address_lut_L,F
						movlw	HIGH _SIZEOF_LUT_ENTRY
						addwfc	address_lut_H,F			; address_lut ++

__reset_counter_lut:	movlw	LUT_UPDATE_PERIOD*KEY_POLLING_FREQ
						movwf	counter_lut
						return

; save lut index into latest entry in EEPROM
save_lut_index:			movlb	17
						btfsc	s17_EECON1,WR
						return
						bsf		s17_EECON1,CFGS			; Select Data EEPROM
						movf	eeprom_adrs,W
						movwf	s17_EEADRL
						movlw	0x70					; Select Data EEPROM
						movwf	s17_EEADRH

						; write lower byte
						lsrf	address_lut_L,W
						iorwf	sequence_bit_L,W
						call	f17_write_data_eeprom

						btfsc	s17_EECON1,WR
						return
						incf	s17_EEADRL,F

						; write upper byte
						movf	address_lut_H,W
						bcf		WREG,7
						iorwf	sequence_bit_H,W
						call	f17_write_data_eeprom

						btfss	s17_EECON1,WR
						bcf		notification_flag,BIT_UPDATE_LUT_INDEX
						return

;		WREG	data to be written
;		EEADR	address of data
f17_write_data_eeprom:	bsf		s17_EECON1,RD
						xorwf	s17_EEDATL,F
						btfsc	STATUS,Z
						return
						movwf	s17_EEDATL
	if (PPS_RA2_init == B'100001')				; Rxy source is RA2, EEPROM update
					movlb	2
					bcf		s2_LATA,LATA2
					movlb	17
	endif

						bsf		s17_EECON1,WREN
						movlw	0x55
						movwf	s17_EECON2
						movlw	0xAA
						movwf	s17_EECON2
						bsf		s17_EECON1,WR
						bcf		s17_EECON1,WREN
	if (PPS_RA2_init == B'100001')				; Rxy source is RA2, EEPROM update
					movlb	2
					bsf		s2_LATA,LATA2
					movlb	17
	endif
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Code Flash																				;
;		*		*		*		*		*		*		*		*		*		*		*		;

; load lut entry from code flash
;		Memory Image in Program Flash ROM
;				target_voltage in EEDATH          sampling_interval in EEDATL
; 1st word		0    0    v5  v4  v3  v2  v1  v0  i7  i6  i5  i4  i3  i2  i1  i0
; 2nd word		0    0    v11 v10 v9  v8  v7  v6  i15 i14 i13 i12 i11 i10 i9  i8
load_lut_entry:			movlb	17
						btfsc	s17_EECON1,WR
						return

						bcf		notification_flag,BIT_LOAD_LUT_ENTRY
						bcf		s17_EECON1,CFGS			; Select Program Flash ROM
						movf	address_lut_L,W
						btfss	notification_flag,BIT_CLOSED_LOOP_CTRL
						movlw	LOW lut_end
						movwf	s17_EEADRL
						movf	address_lut_H,W
						btfss	notification_flag,BIT_CLOSED_LOOP_CTRL
						movlw	HIGH lut_end
						movwf	s17_EEADRH					; 9

						; read LS word
						bcf		s17_EEADRL,0
						bsf		s17_EECON1,RD
						movf	s17_EEDATL,W
						movwf	sampling_interval_L

						lslf	s17_EEDATH,W
						movlb	1
						movwf	s1_target_voltage_L
						lslf	s1_target_voltage_L,F			; 9

						movlb	17
						; read MS word
						bsf		s17_EEADRL,0
						bsf		s17_EECON1,RD
						movf	s17_EEDATL,W
						movwf	sampling_interval_H

						lsrf	s17_EEDATH,W
						movlb	1
						movwf	s1_target_voltage_H
						rrf		s1_target_voltage_L,F			; 9

						;	0    v11 v10 v9  v8  v7  v6  v5  v4  v3  v2  v1  v0   0		2x speed
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						lsrf	s1_target_voltage_H,F
						btfss	notification_flag,BIT_DOUBLE_VOLTAGE
						rrf		s1_target_voltage_L,F
						;	0   0    v11 v10 v9  v8  v7  v6  v5  v4  v3  v2  v1  v0		1x speed

						; adjustment_range = 1.5*target_voltage >> 1
						;          2mV/LSB              1mV/LSB
						lsrf	s1_target_voltage_H,W
						movwf	s1_adjustment_range_H
						rrf		s1_target_voltage_L,W
						movwf	s1_adjustment_range_L
						movf	s1_target_voltage_L,W
						addwf	s1_adjustment_range_L,F
						movf	s1_target_voltage_H,W
						addwfc	s1_adjustment_range_H,F		; 1mv/LSB
						lsrf	s1_adjustment_range_H,F
						rrf		s1_adjustment_range_L,F		; 2mV/LSB

						; startup_voltage = target_voltage + IR-drop
						movlw	LOW STARTUP_VOLTAGE		
						addwf	s1_target_voltage_L,W
						movwf	s1_startup_voltage_L
						movlw	HIGH STARTUP_VOLTAGE	
						addwfc	s1_target_voltage_H,W
						movwf	s1_startup_voltage_H
						return

						end

