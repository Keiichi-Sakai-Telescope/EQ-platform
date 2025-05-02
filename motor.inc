
BREAKING_IN_DURATION	set		10		; max. 64sec, [second]
OPEN_LOOP_VOLTAGE_mV	set		2000

;	LUT_UPDATE_PERIOD		5, 4, 3, 2, 1 [second]
;	REWIND_TIMEOUT			600 [sec], 0 is endless

;	P_GAIN		proportional gain
;	I_TIME_8p8	integration time where range is from 2.0 to 256.0

;	*	*	*	*	*	*	*	*	*	*	*	*
; FA-130RA-2270									*
;	Generic FA-130RA motor						*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_FA_130RA_2270		macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		6660		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		682			; mOhm
STARTUP_CURRENT			equ		660			; mA, 660mA at maximum efficiency
AMPLITUDE_ANGLE			equ		45			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		200			; 200us, minimum is 80us
MODE_DOUBLE_SPEED		equ		SAMPLING_MOVING_AVERAGE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/15000		; speed[rpm] / 150.00
; usable	 450.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=1, with P_GAIN=4 and I_TIME= 3
; usable	 562.5rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME= 4
; usable	 720.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME= 5
; usable	 900.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME= 6
; usable	 900.0rpm MODE_NORMAL_SPEED=1, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME= 6
; usable	1125.0rpm MODE_NORMAL_SPEED=1, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME= 8
; usable	1440.0rpm MODE_NORMAL_SPEED=1, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME=10
; usable	1800.0rpm MODE_NORMAL_SPEED=1, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME=12
; usable	2250.0rpm MODE_NORMAL_SPEED=1, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME=16
; usable	2880.0rpm MODE_NORMAL_SPEED=1, MODE_DOUBLE_SPEED=1, with P_GAIN=5 and I_TIME=20
		if                                  ((MOTOR_100TH_RPM) <  50312)
			; center speed is  450.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_DOUBLE
P_GAIN					equ		4
		endif
		if ( 50312 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_DOUBLE
P_GAIN					equ		5
		endif
		if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_DOUBLE
P_GAIN					equ		5
		endif
		if ( 80498 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 100623)
			; center speed is  900.0 rpm
; MODE_NORMAL_SPEED		equ		SAMPLING_DOUBLE
MODE_NORMAL_SPEED		equ		SAMPLING_MOVING_AVERAGE
P_GAIN					equ		5
		endif
		if (100623 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 127279)
			; center speed is 1125.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_MOVING_AVERAGE
P_GAIN					equ		5
		endif
		if (127279 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 160997)
			; center speed is 1440.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_MOVING_AVERAGE
P_GAIN					equ		5
		endif
		if (160997 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 201246)
			; center speed is 1800.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_MOVING_AVERAGE
P_GAIN					equ		5
		endif
		if ( 201246<= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 254558)
			; center speed is 2250.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_MOVING_AVERAGE
P_GAIN					equ		5
		endif
		if (254558 <= (MOTOR_100TH_RPM))
			; center speed is 2880.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_MOVING_AVERAGE
P_GAIN					equ		5
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; FC-130RA-18100								*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_FC_130RA_18100	macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		4415		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		1430		; mOhm
STARTUP_CURRENT			equ		560			; mA, 560mA at maximum efficiency
AMPLITUDE_ANGLE			equ		30			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		250			; 250us, minimum is 80us
MODE_NORMAL_SPEED		set		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/9000		; speed[rpm] / 90.00
; usable	 450.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 5
; usable	 562.5rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 6
; stable	 562.5rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 6
; usable	 720.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 7
; usable	 720.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 7
; usable	 900.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME=10
; usable	 900.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME=10
; usable	1125.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=12
; usable	1440.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=15
; usable	1800.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=20
		if                                  ((MOTOR_100TH_RPM) <  50312)
			; center speed is  450.0 rpm
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 50312 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 80498 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 100623)
			; center speed is  900.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if (100623 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 127279)
			; center speed is 1125.0 rpm
P_GAIN					equ		10
		endif
		if (127279 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 160997)
			; center speed is 1440.0 rpm
P_GAIN					equ		10
		endif
		if (160997 <= (MOTOR_100TH_RPM))
			; center speed is 1800.0 rpm
P_GAIN					equ		10
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; Tamiya 75028 Low RPM 130 Motor				*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_TAMIYA_75028_130	macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		3860		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		2600		; mOhm
STARTUP_CURRENT			equ		450			; mA, 450mA at maximum efficiency
AMPLITUDE_ANGLE			equ		30			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		160			; 160us, minimum is 80us
MODE_NORMAL_SPEED		set		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/9000		; speed[rpm] / 90.00
; usable	 450.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 5
; usable	 562.5rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 6
; stable	 562.5rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 6
; usable	 720.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 7
; usable	 720.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME= 7
; usable	 900.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME=10
; usable	 900.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 9 and I_TIME=10
; usable	1125.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=12
; usable	1440.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=15
; usable	1800.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=20
		if                                  ((MOTOR_100TH_RPM) <  50312)
			; center speed is  450.0 rpm
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 50312 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 80498 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 100623)
			; center speed is  900.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if (100623 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 127279)
			; center speed is 1125.0 rpm
P_GAIN					equ		10
		endif
		if (127279 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 160997)
			; center speed is 1440.0 rpm
P_GAIN					equ		10
		endif
		if (160997 <= (MOTOR_100TH_RPM))
			; center speed is 1800.0 rpm
P_GAIN					equ		10
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; FC-130RA-14150								*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_FC_130RA_14150	macro	MOTOR_100TH_RPM
;MOTOR_SPEED_CONSTANT	equ		3300		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_SPEED_CONSTANT	equ		3860		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		3600		; mOhm
STARTUP_CURRENT			equ		420			; mA, 420mA at maximum efficiency
AMPLITUDE_ANGLE			equ		30			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		250			; 250us, minimum is 80us
MODE_NORMAL_SPEED		set		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_DOUBLE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/9000		; speed[rpm] / 90.00
; usable	 450.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME= 5
; usable	 562.5rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME= 6
; stable	 562.5rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME= 6
; usable	 720.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME= 7
; usable	 720.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME= 7
; usable	 900.0rpm MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME=10
; usable	 900.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=2, with P_GAIN= 9 and I_TIME=10
; usable	1125.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=2, with P_GAIN=10 and I_TIME=12
; usable	1440.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=2, with P_GAIN=10 and I_TIME=15
; usable	1800.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=2, with P_GAIN=10 and I_TIME=20
		if                                (MOTOR_100TH_RPM <  50312)
			; center speed is  450.0 rpm
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 50312 <= MOTOR_100TH_RPM) && (MOTOR_100TH_RPM <  63640)
			; center speed is  562.5 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 63640 <= MOTOR_100TH_RPM) && (MOTOR_100TH_RPM <  80498)
			; center speed is  720.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if ( 80498 <= MOTOR_100TH_RPM) && (MOTOR_100TH_RPM < 100623)
			; center speed is  900.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9
		endif
		if (100623 <= MOTOR_100TH_RPM) && (MOTOR_100TH_RPM < 127279)
			; center speed is 1125.0 rpm
P_GAIN					equ		10
		endif
		if (127279 <= MOTOR_100TH_RPM) && (MOTOR_100TH_RPM < 160997)
			; center speed is 1440.0 rpm
P_GAIN					equ		10
		endif
		if (160997 <= MOTOR_100TH_RPM)
			; center speed is 1800.0 rpm
P_GAIN					equ		10
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; RE-140RA-2270									*
;	Generic RE-140RA motor						*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_RE_140RA_2270		macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		5980		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		714			; mOhm
STARTUP_CURRENT			equ		660			; mA, 660mA at maximum efficiency
AMPLITUDE_ANGLE			equ		40			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		210			; 210us, minimum is 80us
MODE_NORMAL_SPEED		set		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/10000		; speed[rpm] / 100.00
; stable	 562.5rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN= 8 and I_TIME= 6
; usable	 720.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME= 7
; usable	 900.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME= 9
; usable	1125.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=11
; usable	1440.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=14
; usable	1800.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=10 and I_TIME=18
		if                                  ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
P_GAIN					equ		8
		endif
		if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
P_GAIN					equ		10
		endif
		if ( 80498 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 100623)
			; center speed is  900.0 rpm
P_GAIN					equ		10
		endif
		if (100623 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 127279)
			; center speed is 1125.0 rpm
P_GAIN					equ		10
		endif
		if (127279 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 160997)
			; center speed is 1440.0 rpm
P_GAIN					equ		10
		endif
		if (160997 <= (MOTOR_100TH_RPM))
			; center speed is 1800.0 rpm
P_GAIN					equ		10
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; RE-260RA-2670									*
;	Generic RE-260RA motor						*
; RC-260RA-2670									*
;	Tamiya planetary gear box					*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_RE_260RA_2670		macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		4300		; reciprocal of counter-emf constant, [rpm/V]
;MOTOR_SPEED_CONSTANT	equ		4480		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		586			; mOhm
STARTUP_CURRENT			equ		640			; mA, 640mA at maximum efficiency
;AMPLITUDE_ANGLE			equ		45			; use 30 for ideal motor 
AMPLITUDE_ANGLE			equ		40			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		160			; 160us, minimum is 80us
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/11000		; speed[rpm] / 110.00
; usable	 360.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN= 6 and I_TIME= 3
; usable	 450.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN= 8 and I_TIME= 4
; usable	 562.5rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN= 9 and I_TIME= 5
; usable	 720.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN=10 and I_TIME= 6
; usable	 720.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=10 and I_TIME= 6
; usable	 900.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=10 and I_TIME= 8
; usable	1125.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=10 and I_TIME=10
; usable	1440.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=10 and I_TIME=12
; usable	1800.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=10 and I_TIME=16
	if                                  ((MOTOR_100TH_RPM) <  40249)
			; center speed is  360.0 rpm
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		6				; NG: new Mabuchi, Asahi
	endif
	if ( 40249 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  50312)
			; center speed is  450.0 rpm
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		8				; NG: new Mabuchi, Asahi	OK: old Mabuchi, Tamiya
	endif
	if ( 50312 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		9				; NG: new Mabuchi, Asahi	OK: old Mabuchi, Tamiya
	endif
	if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
;MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
MODE_NORMAL_SPEED		equ		SAMPLING_DOUBLE
P_GAIN					equ		10				; NG: new Mabuchi, Asahi	OK: old Mabuchi, Tamiya
	endif
	if ( 80498 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 100623)
			; center speed is  900.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		10
	endif
	if (100623 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 127279)
			; center speed is 1125.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		10
	endif
	if (127279 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 160997)
			; center speed is 1440.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		10
	endif
	if (160997 <= (MOTOR_100TH_RPM))
			; center speed is 1800.0 rpm
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		10
	endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; RE-280RA-2865									*
;	Generic RE-280RA motor						*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_RE_280RA_2865		macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		3175		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		638			; mOhm
STARTUP_CURRENT			equ		870			; mA, 870mA at maximum efficiency
AMPLITUDE_ANGLE			equ		40			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		250			; 250us, minimum is 80us
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/15000		; speed[rpm] / 150.00
; usable	 360.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=3 and I_TIME=2
; usable	 450.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=4 and I_TIME=3
; usable	 562.5rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=5 and I_TIME=4
; usable	 720.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=5 and I_TIME=5
; usable	 900.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=5 and I_TIME=6
; usable	1125.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=6 and I_TIME=7
; usable	1440.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=6 and I_TIME=9
; usable	1800.0rpm MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, with P_GAIN=8 and I_TIME=12
		if                                  ((MOTOR_100TH_RPM) <  40249)
			; center speed is  360.0 rpm
P_GAIN					equ		3
		endif
		if ( 40249 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  50312)
			; center speed is  450.0 rpm
P_GAIN					equ		4
		endif
		if ( 50312 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
P_GAIN					equ		5
		endif
		if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
P_GAIN					equ		5
		endif
		if ( 80498 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 100623)
			; center speed is  900.0 rpm
P_GAIN					equ		5
		endif
		if (100623 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 127279)
			; center speed is 1125.0 rpm
P_GAIN					equ		6
		endif
		if (127279 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) < 160997)
			; center speed is 1440.0 rpm
P_GAIN					equ		6
		endif
		if (160997 <= (MOTOR_100TH_RPM))
			; center speed is 1800.0 rpm
P_GAIN					equ		8
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; RE-280RA-2485									*
;	Daisen RA250100-58Y91						*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_RE_280RA_2485		macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		2438		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		1070		; mOhm
STARTUP_CURRENT			equ		580			; mA, 580mA at maximum efficiency
AMPLITUDE_ANGLE			equ		30			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		360			; 360us, minimum is 80us
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		10
I_TIME_8p8				equ		(16<<8)
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; RE-280RA-20120									*
;	JAPAN ROBOTECH RDO-501						*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_RE_280RA_20120	macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		1728		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		2036		; mOhm
STARTUP_CURRENT			equ		420			; mA, 420mA at maximum efficiency
AMPLITUDE_ANGLE			equ		30			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		360			; 360us, minimum is 80us
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		6
I_TIME_8p8				equ		(16<<8)
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; FP030-KN/11160								*
;	Tamiya mini motor gear box					*
;	Tamiya mini motor low-speed gear box		*
;	Tamiya mini motor multi-ratio gear box		*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_FP030_KN_11160	macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		2990		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		5500		; mOhm
STARTUP_CURRENT			equ		250			; mA, 250mA at maximum efficiency
AMPLITUDE_ANGLE			equ		40			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		160			; 360us, minimum is 80us
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		((MOTOR_100TH_RPM)+16000)/32000	; speed[rpm] / 320.00
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/20000		; speed[rpm] / 200.00
; unstable 800rpm. with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=3 and I_TIME= 4
; usable  1000rpm. with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=3 and I_TIME= 5
; usable  1280rpm. with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=4 and I_TIME= 6
; usable  1600rpm. with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=5 and I_TIME= 8
; usable  2000rpm. with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=6 and I_TIME=10
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; FP030-KN/13125								*
;	Tamiya mini motor (Item number 75026)		*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_FP030_KN_13125	macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		3750		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		2830		; mOhm
STARTUP_CURRENT			equ		280			; mA, 280mA at maximum efficiency 
AMPLITUDE_ANGLE			equ		30			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		360			; 360us, minimum is 80us
MODE_NORMAL_SPEED		equ		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		equ		SAMPLING_SINGLE
P_GAIN					equ		4
I_TIME_8p8				equ		(8<<8)
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

;	*	*	*	*	*	*	*	*	*	*	*	*
; RC300-FT/14270								*
;	Tamiya Eco motor		*
;	*	*	*	*	*	*	*	*	*	*	*	*
MOTOR_RC300_FT_14270	macro	MOTOR_100TH_RPM
MOTOR_SPEED_CONSTANT	equ		1970		; reciprocal of counter-emf constant, [rpm/V]
MOTOR_RESISTANCE		equ		4900		; mOhm
;STARTUP_CURRENT			equ		321			; mA, 321mA at maximum output (a half of the no load rotation speed)
STARTUP_CURRENT			equ		138			; mA, 138mA at maximum efficiency 
AMPLITUDE_ANGLE			equ		36			; use 30 for ideal motor 
LEAD_ANGLE				equ		0
CEMF_SETTLING_TIME		set		300			; 300us, minimum is 80us
MODE_NORMAL_SPEED		set		SAMPLING_SINGLE
MODE_DOUBLE_SPEED		set		SAMPLING_SINGLE
; unusable	 360.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN=4 and I_TIME=3
; stable	 450.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN=5 and I_TIME=4 (light load)
; stable	 450.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=4 and I_TIME=3 (light load)
; usable	 562.5rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN=5 and I_TIME=4
; usable	 562.5rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=5 and I_TIME=4
; usable	 720.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN=6 and I_TIME=5
; usable	 720.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=6 and I_TIME=5
; usable	 900.0rpm with MODE_NORMAL_SPEED=2, MODE_DOUBLE_SPEED=0, P_GAIN=8 and I_TIME=6
; usable	 900.0rpm with MODE_NORMAL_SPEED=0, MODE_DOUBLE_SPEED=0, P_GAIN=8 and I_TIME=6
		if                                  ((MOTOR_100TH_RPM) <  40249)
			; center speed is  360.0 rpm (light load)
MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		4
I_TIME_8p8				equ		(3<<8)
		endif
		if ( 40249 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  50312)
			; center speed is  450.0 rpm (light load)
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		5
I_TIME_8p8				equ		(4<<8)
		endif
		if ( 50312 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  63640)
			; center speed is  562.5 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		5
I_TIME_8p8				equ		(4<<8)
		endif
		if ( 63640 <= (MOTOR_100TH_RPM)) && ((MOTOR_100TH_RPM) <  80498)
			; center speed is  720.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		6
I_TIME_8p8				equ		(5<<8)
		endif
		if ( 80498 <= (MOTOR_100TH_RPM))
			; center speed is  900.0 rpm
; MODE_NORMAL_SPEED		set		SAMPLING_DOUBLE
P_GAIN					equ		8
I_TIME_8p8				equ		((MOTOR_100TH_RPM)<<8)/15000		; speed[rpm] / 150.00
		endif
		DECIDE_PRESCALER	MOTOR_100TH_RPM
	endm

#if 0
LUT_UPDATE_PERIOD		equ		0			; 5, 4, 3, 2, 1 [second]
REWIND_TIMEOUT			equ		0			; endless

; Axle/Rotor/Gear
;motor_speed_100th	equ	   6429	;   64.29rpm,                                                                  900rpm/14T
;motor_speed_100th	equ	   7500	;   75.00rpm,                                                     900rpm/12T,  
;motor_speed_100th	equ	   8036	;   80.36rpm,                                                                 1125rpm/14T
;motor_speed_100th	equ	   9000	;   90.00rpm,                                        900rpm/10T,  
;motor_speed_100th	equ	   9375	;   93.75rpm,                                                    1125rpm/12T,  
;motor_speed_100th	equ	  10000	;  100.00rpm,                            900rpm/9T,  
;motor_speed_100th	equ	  10286	;  102.86rpm,                                                                 1440rpm/14T
;motor_speed_100th	equ	  11250	;  112.50rpm,                900rpm/8T,             1125rpm/10T,  
;motor_speed_100th	equ	  12000	;  120.00rpm,                                                    1440rpm/12T,  
;motor_speed_100th	equ	  12500	;  125.00rpm,                           1125rpm/9T,  
;motor_speed_100th	equ	  12857	;  128.57rpm,                                                                 1800rpm/14T
;motor_speed_100th	equ	  14063	;  140.63rpm,               1125rpm/8T,  
;motor_speed_100th	equ	  14400	;  144.00rpm,                                       1440rpm/10T,  
;motor_speed_100th	equ	  15000	;  150.00rpm,                                                    1800rpm/12T,  
;motor_speed_100th	equ	  16000	;  160.00rpm,                           1440rpm/9T,  
;motor_speed_100th	equ	  16071	;  160.71rpm,                                                                 2250rpm/14T
;motor_speed_100th	equ	  18000	;  180.00rpm,               1440rpm/8T,             1800rpm/10T,  
;motor_speed_100th	equ	  18750	;  187.50rpm,                                                    2250rpm/12T,  
;motor_speed_100th	equ	  20000	;  200.00rpm,                           1800rpm/9T,  
;motor_speed_100th	equ	  20571	;  205.71rpm,                                                                 2880rpm/14T
;motor_speed_100th	equ	  22500	;  225.00rpm,               1800rpm/8T,             2250rpm/10T,  
;motor_speed_100th	equ	  24000	;  240.00rpm,                                                    2880rpm/12T,  
;motor_speed_100th	equ	  25000	;  250.00rpm,                           2250rpm/9T,  
;motor_speed_100th	equ	  25714	;  257.14rpm,                                                                 3600rpm/14T
;motor_speed_100th	equ	  28125	;  281.25rpm,               2250rpm/8T,  
;motor_speed_100th	equ	  28800	;  288.00rpm,                                       2880rpm/10T,  
;motor_speed_100th	equ	  30000	;  300.00rpm,    900rpm/3T,                                      3600rpm/12T,  
;motor_speed_100th	equ	  32000	;  320.00rpm,                           2880rpm/9T,  
;motor_speed_100th	equ	  32143	;  321.43rpm,                                                                 4500rpm/14T
;motor_speed_100th	equ	  36000	;  360.00rpm,               2880rpm/8T,             3600rpm/10T,  
;motor_speed_100th	equ	  37500	;  375.00rpm,   1125rpm/3T,                                      4500rpm/12T,  
;motor_speed_100th	equ	  40000	;  400.00rpm,                           3600rpm/9T,  
;motor_speed_100th	equ	  41143	;  411.43rpm,                                                                 5760rpm/14T
;motor_speed_100th	equ	  45000	;  450.00rpm,               3600rpm/8T,             4500rpm/10T,  
;motor_speed_100th	equ	  48000	;  480.00rpm,   1440rpm/3T,                                      5760rpm/12T,  
;motor_speed_100th	equ	  50000	;  500.00rpm,                           4500rpm/9T,  
;motor_speed_100th	equ	  51429	;  514.29rpm,                                                                 7200rpm/14T
;motor_speed_100th	equ	  56250	;  562.50rpm,               4500rpm/8T,  
;motor_speed_100th	equ	  57600	;  576.00rpm,                                       5760rpm/10T,  
;motor_speed_100th	equ	  60000	;  600.00rpm,   1800rpm/3T,                                      7200rpm/12T,  
;motor_speed_100th	equ	  64000	;  640.00rpm,                           5760rpm/9T,  
;motor_speed_100th	equ	  64286	;  642.86rpm,                                                                 9000rpm/14T
;motor_speed_100th	equ	  72000	;  720.00rpm,               5760rpm/8T,             7200rpm/10T,  
;motor_speed_100th	equ	  75000	;  750.00rpm,   2250rpm/3T,                                      9000rpm/12T,  
;motor_speed_100th	equ	  80000	;  800.00rpm,                           7200rpm/9T,  
;motor_speed_100th	equ	  82286	;  822.86rpm,                                                                11520rpm/14T
;motor_speed_100th	equ	  90000	;  900.00rpm/1,             7200rpm/8T,             9000rpm/10T,  
;motor_speed_100th	equ	  96000	;  960.00rpm,   2880rpm/3T,                                     11520rpm/12T,  
;motor_speed_100th	equ	 100000	; 1000.00rpm,                           9000rpm/9T,  
;motor_speed_100th	equ	 102857	; 1028.57rpm,                                                                14400rpm/14T
;motor_speed_100th	equ	 112500	; 1125.00rpm/1,             9000rpm/8T,  
;motor_speed_100th	equ	 115200	; 1152.00rpm,                                      11520rpm/10T,  
;motor_speed_100th	equ	 120000	; 1200.00rpm,   3600rpm/3T,                                     14400rpm/12T,  
;motor_speed_100th	equ	 128000	; 1280.00rpm,                          11520rpm/9T,  
;motor_speed_100th	equ	 128571	; 1285.71rpm,                                                                18000rpm/14T
;motor_speed_100th	equ	 144000	; 1440.00rpm/1,            11520rpm/8T,            14400rpm/10T,  
;motor_speed_100th	equ	 150000	; 1500.00rpm,   4500rpm/3T,                                     18000rpm/12T,  
;motor_speed_100th	equ	 160000	; 1600.00rpm,                          14400rpm/9T,  
;motor_speed_100th	equ	 164571	; 1645.71rpm,                                                                23040rpm/14T
;motor_speed_100th	equ	 180000	; 1800.00rpm/1,            14400rpm/8T,            18000rpm/10T,  
;motor_speed_100th	equ	 192000	; 1920.00rpm,   5760rpm/3T,                                     23040rpm/12T,  
;motor_speed_100th	equ	 200000	; 2000.00rpm,                          18000rpm/9T,  
;motor_speed_100th	equ	 205714	; 2057.14rpm,                                                                28800rpm/14T
;motor_speed_100th	equ	 225000	; 2250.00rpm/1,            18000rpm/8T,  
;motor_speed_100th	equ	 230400	; 2304.00rpm,                                      23040rpm/10T,  
;motor_speed_100th	equ	 240000	; 2400.00rpm,   7200rpm/3T,                                     28800rpm/12T,  
;motor_speed_100th	equ	 256000	; 2560.00rpm,                          23040rpm/9T,  
;motor_speed_100th	equ	 288000	; 2880.00rpm/1,            23040rpm/8T,            28800rpm/10T,  
;motor_speed_100th	equ	 300000	; 3000.00rpm,   9000rpm/3T,  
;motor_speed_100th	equ	 320000	; 3200.00rpm,                          28800rpm/9T,  
;motor_speed_100th	equ	 360000	; 3600.00rpm/1,            28800rpm/8T,  
;motor_speed_100th	equ	 384000	; 3840.00rpm,  11520rpm/3T,  
;motor_speed_100th	equ	 450000	; 4500.00rpm/1,
;motor_speed_100th	equ	 480000	; 4800.00rpm,  14400rpm/3T,  
;motor_speed_100th	equ	 576000	; 5760.00rpm/1,
;motor_speed_100th	equ	 600000	; 6000.00rpm,  18000rpm/3T,  
;motor_speed_100th	equ	 720000	; 7200.00rpm/1,
;motor_speed_100th	equ	 768000	; 7680.00rpm,  23040rpm/3T,  
;motor_speed_100th	equ	 900000	; 9000.00rpm/1,
;motor_speed_100th	equ	 960000	; 9600.00rpm,  28800rpm/3T,  
;motor_speed_100th	equ	1152000	;11520.00rpm/1,
;motor_speed_100th	equ	1440000	;14400.00rpm/1,
;motor_speed_100th	equ	1800000	;18000.00rpm/1,
;motor_speed_100th	equ	2304000	;23040.00rpm/1,
;motor_speed_100th	equ	2880000	;28800.00rpm/1,

; 6-Speed Gearbox HE
;motor_speed_100th	equ	 115599	; 1155.99rpm, 59.94Hz*60s/14*36/8
;motor_speed_100th	equ	 115714	; 1157.14rpm, 60.00Hz*60s/14*36/8
;motor_speed_100th	equ	 148627	; 1486.27rpm, 29.97Hz*60s/14*36/14*36/8
;motor_speed_100th	equ	 148776	; 1487.76rom, 30.00Hz*60s/14*36/14*36/8
;motor_speed_100th	equ	 231429	; 2314.29rpm, 120.0Hz*60s/14*36/8
;motor_speed_100th	equ	 297253	; 2972.53rpm, 59.94Hz*60s/14*36/14*36/8
;motor_speed_100th	equ	 297551	; 2975.51rpm, 60.00Hz*60s/14*36/14*36/8

; Modified 6-Speed Gearbox HE
;motor_speed_100th	equ	 70775  ;  707.75rpm, 29.97Hz*60/14*36/14*30/14
;motor_speed_100th	equ	 70845  ;  708.45rpm, 30.00Hz*60/14*36/14*30/14
;motor_speed_100th	equ	 141549 ; 1415.49rpm, 59.94Hz*60/14*36/14*30/14
;motor_speed_100th	equ	 141691 ; 1416.91rpm, 60.00Hz*60/14*36/14*30/14

; 4-Speed Single Gearbox
;motor_speed_100th	equ	 255000	; 2550.00rpm, 120.0Hz*60s/12*34/8

; 3-Speed Crank Axle Gearbox
;motor_speed_100th	equ	 285000	; 2850.00rpm, 120.0Hz*60s/12*38/8

; 12-speed gearbox
;motor_speed_100th	equ	 168000	; 1680.00rpm, 120.0Hz*60s/10*21/9
;motor_speed_100th	equ	 352800	; 3528.00rpm, 120.0Hz*60s/10*21/10*21/9

; SP/GP(144T), 64:15, 4-speed Crank, Rotation of Earth
;motor_speed_100th	equ	 231127	; 2311.27rpm, 5402.25*64/15*144/1436.068

; SP/GP(144T), 45:15, 4-speed Crank, Rotation of Earth
;motor_speed_100th	equ	 162511	; 1625.11rpm, 5402.25*45/15*144/1436.068

; PortaII(120T), 64:15, 4-speed Crank, Rotation of Earth
;motor_speed_100th	equ	 192606	; 1926.06rpm, 5402.25*64/15*120/1436.068

; SP/GP(144T), DS1 45:12, 6-speed Gearbox HE, Rotation of Earth
;motor_speed_100th	equ	  48919	;  489.19rpm, 1300.95*45/12*144/1436.068

; Sensor(144T), DS1 70:12, 6-speed Gearbox HE, Rotation of Earth
;motor_speed_100th	equ	  76096	;  760.96rpm, 1300.95*70/12*144/1436.068

; SP/GP(144T), DS0.8 56:16, 6-speed Gearbox HE, Rotation of Earth
;motor_speed_100th	equ	  45658	;  456.58rpm, 1300.95*56/16*144/1436.068

; PortaII(120T), 70:12, 6-speed Gearbox HE, Rotation of Earth
;motor_speed_100th	equ	  63414	;  634.14rpm, 1300.95*70/12*120/1436.068

; SP/GP(144T), 64:15, Twin Motor Gear Box, Rotation of Earth
;motor_speed_100th	equ	 106736	; 1067.36rpm, 2494.79*64/15*144/1436.068

; PortaII(120T), 64:15, Twin Motor Gear Box, Rotation of Earth
;motor_speed_100th	equ	  88946	;  889.46rpm, 2494.79*64/15*120/1436.068

; SP/GP(144T), 50:15, Double Gear Box, Rotation of Earth
;motor_speed_100th	equ	  82846	;  828.46rpm, 2478.60*50/15*144/1436.068

; PortaII(120T), 64:15, Double Gear Box, Rotation of Earth
;motor_speed_100th	equ	  98188	;  981.88rpm, 2754.00*64/15*120/1436.068

;						MOTOR_FA_130RA_2270		motor_speed_100th
;						MOTOR_FC_130RA_18100	motor_speed_100th
;						MOTOR_TAMIYA_75028_130	motor_speed_100th
;						MOTOR_FC_130RA_14150	motor_speed_100th
;						MOTOR_RE_140RA_2270		motor_speed_100th
						MOTOR_RE_260RA_2670		motor_speed_100th
;						MOTOR_RE_280RA_2865		motor_speed_100th
;						MOTOR_RE_280RA_2485		motor_speed_100th
;						MOTOR_RE_280RA_20120	motor_speed_100th
;						MOTOR_FP030_KN_11160	motor_speed_100th
;						MOTOR_FP030_KN_13125	motor_speed_100th
;						MOTOR_RC300_FT_14270	motor_speed_100th

						LUT_ENTRIES				motor_speed_100th
#else
;#include ../3-speed-crank-20+42-150.s
;#include ../6-speed-4th-60min.s			; Tamaki, Yokoyama, Sakai, 2nd prototype
;#include ../6-speed-5th-60min.s
;#include ../6-speed-5th.s					; Numata, 1st prototype
;#include ../6-speed-4th-40min-16in.s		; Sakai's 16in
;#include ../6-speed-4th-unbalanced-12.s		; unblanced poncet
;#include ../up-down-test.s
;#include ../6-speed-mod-5th-40min-16in.s	; Sakai's 16in
;#include ../6-speed-mod-5th-60min-8in.s	; Sakai's 8in
#include ../6-speed-mod-5th-40min-13in.s	; Sakai's 13.1in
#endif

;	*	*	*	*	*	*	*	*	*	*	*	*
; lead angle									*
;	*	*	*	*	*	*	*	*	*	*	*	*
PEAK_WAVE_HEIGHT		equ		32768
angle_leading_edge		equ		AMPLITUDE_ANGLE+LEAD_ANGLE	; absolute value
angle_trailing_edge		equ		AMPLITUDE_ANGLE-LEAD_ANGLE	; absolute value

sin_leading				set		0
cos_leading				set		PEAK_WAVE_HEIGHT
sin_trailing			set		0
cos_trailing			set		PEAK_WAVE_HEIGHT

ROTATE_ANGLE			macro	bit_mask, sin_in, cos_in
						local	sin_tmp, cos_tmp
	if (angle_leading_edge & bit_mask)
sin_tmp = sin_leading * cos_in + cos_leading * sin_in
cos_tmp = cos_leading * cos_in - sin_leading * sin_in
sin_leading = sin_tmp/PEAK_WAVE_HEIGHT
cos_leading = cos_tmp/PEAK_WAVE_HEIGHT
	endif
	if (angle_trailing_edge & bit_mask)
sin_tmp =  sin_trailing * cos_in - cos_trailing * sin_in
cos_tmp =  cos_trailing * cos_in + sin_trailing * sin_in
sin_trailing = sin_tmp/PEAK_WAVE_HEIGHT
cos_trailing = cos_tmp/PEAK_WAVE_HEIGHT
	endif
						endm

						ROTATE_ANGLE	 1,   572, 32763
						ROTATE_ANGLE	 2,  1144, 32748
						ROTATE_ANGLE	 4,  2286, 32688
						ROTATE_ANGLE	 8,  4560, 32449
						ROTATE_ANGLE	16,  9032, 31499
						ROTATE_ANGLE	32, 17364, 27789

MEAN_WAVE_HEIGHT		equ		(938734/AMPLITUDE_ANGLE)*(-sin_trailing+sin_leading)/PEAK_WAVE_HEIGHT		; average voltage relative to peak voltage
CENTER_WAVE_HEIGHT		equ		(PEAK_WAVE_HEIGHT+cos_leading)/2			; center voltage relative to peak voltage

