						processor		16f1769
						#include		p16f1769.inc

						errorlevel -302
						radix	DEC

FCY						equ		4000000					; 4MHz
KEY_POLLING_FREQ		equ		50						; 50Hz, 20ms
ADC_INTERVAL			equ		18						; 15us, minimum is 15us
PWM_PERIOD				equ		54						; 50us, 20kHz

; High supply voltage or Low motor speed
 __config	_CONFIG1,	_FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _BOREN_ON & _CLKOUTEN_OFF
 __config	_CONFIG2,	_PPS1WAY_OFF & _BORV_LO & _LPBOR_OFF & _LVP_OFF

	if 0
; Timer configurations
						; Timer assignment
						; PWM4: T2, PWM3: T2, CCP2: T5, CCP1: T2
CCPTMRS_init			equ		(B'00'<<C4TSEL0)|(B'00'<<C3TSEL0)|(B'10'<<C2TSEL0)|(B'00'<<C1TSEL0)

						; Trigger for A/D conversion of FVR
T1CON_init				equ		(B'00'<<T1CKPS0)|(1 << TMR1ON)	; FCY, pre 1:1
CCP1CON_init			equ		(1<<CCP1EN)|(B'1011'<<CCP1MODE0)	; 0-1-0 pulse, clear on match
	endif

						; Motor Current Sense
OPA2NCHS_init           equ		0	; OPA2IN0-
OPA2PCHS_init           equ		0	; OPA2IN0+
OPA2CON_init            equ		(1<<EN)
OPA2ORS_init            equ		0	; Don't use

						; PWM generator
PWM6PH_init             equ		0
PWM6DC_init             equ		200
PWM6PR_init             equ		799
PWM6OF_init             equ		399
PWM6CON_init            equ		(1<<EN)|(B'00'<<PWM6MODE0)
PWM6INTE_init           equ		0
PWM6INTF_init           equ		0
PWM6CLKCON_init         equ		0		; No Prescaler, Fosc
PWM6LDCON_init          equ		0
PWM6OFCON_init          equ		0		; Independent Run Mode
						; FET Driver	COG1A -> pMOS, COG1B -> nMOS
COG1PHR_init            equ		12		; 750ns@16MHz, Release time of Csamp
COG1PHF_init            equ		12		; 750ns@16MHz, Release time of Csamp
COG1BLKR_init           equ		0
COG1BLKF_init           equ		0
COG1DBR_init            equ		63		; Dead-Band = 315ns
COG1DBF_init            equ		63		; Dead-Band = 315ns
COG1CON1_init           equ		(1<<RDBS)|(1<<FDBS)|(1<<POLA)|(0<<POLB)	; Dead-Band Delay 5ns step
COG1RIS0_init           equ		0
COG1RIS1_init           equ		(1<<RIS10)	; PWM6
COG1RSIM0_init          equ		0
COG1RSIM1_init          equ		(1<<RSIM10)	; Edge, add phase delay
COG1FIS0_init           equ		0
COG1FIS1_init           equ		(1<<FIS10)	; PWM6
COG1FSIM0_init          equ		0
COG1FSIM1_init          equ		(1<<FSIM10)	; Edge, add phase delay
COG1ASD0_init           equ		(1<<ASE)|(B'11'<<ASDAC0)|(B'10'<<ASDBD0)	; Shutdown, A <- 1, B <- 0
COG1ASD1_init           equ		0		; software trigger
COG1STR_init            equ		0		; don't use
COG1CON0_init           equ		(1<<EN)|(B'01'<<G1CS0)|(B'100'<<G1MD0)	; EN, Fosc, Half-Bridge
PPS_HB_HS_init			equ		B'00100'	; COG1A -> pMOS
PPS_HB_LS_init			equ		B'00101'	; COG1B -> nMOS

						; Key polling timer
T4CLKCON_init           equ		B'0001'		; Fcy
T4CON_init				equ		(9 << T4OUTPS0)|(B'110' << T4CKPS0)|(1 << T4ON)
T4PR_init				equ		(FCY/640/KEY_POLLING_FREQ) - 1 ; FCY/640, pre 1:64, post 1:10
T4HLT_init              equ		0
T4RST_init              equ		0	; Don't use

						; Constant brightness LED driver
DAC4CON0_init           equ		(1<<EN)|(B'10'<<PSS0)	; Vref+ <- FVR Buffer2
DAC4REF_init            equ		20
OPA1NCHS_init           equ		0					; OPA1IN0-
OPA1PCHS_init           equ		(B'0101'<<PCH0)		; DAC4
OPA1CON_init            equ		(1<<EN)
OPA1ORS_init            equ		0	; Don't use

; RA2 output
;PPS_RA2_init			equ		B'011110'				; Rxy source is CLKR
;PPS_RA2_init			equ		B'000100'				; Rxy source is CLC1OUT
;PPS_RA2_init			equ		B'001110'				; Rxy source is CCP3
;PPS_RA2_init			equ		B'100000'				; Rxy source is RA2, duty update
;PPS_RA2_init			equ		B'100001'				; Rxy source is RA2, EEPROM update
PPS_RA2_init			equ		B'00000'				; Rxy source is RA2, 0
;CLKRCON_init			equ		(1<<CLKREN)|(B'10'<<CLKRDC0)|(B'000'<<CLKRDIV0)	; Fcy
;CLC1CON_init			equ		(1<<LC1EN)|(B'010'<<LC1MODE0)	; 4-input AND
;CLC1POL_init			equ		B'1110'

; etc.
OSCCON_init				equ		(B'1111'<<IRCF0)		; 16MHz HFINTOSC
FVRCON_init				equ		(1<<FVREN)|(B'01'<<2)|(B'10')	; 1V for DAC, 2V for ADC
DAC2CON0_init           equ		(1<<EN)|(1<<OE1)|(B'00'<<PSS0)	; Vref+ <- VDD

; ADC and FVR configurations
ADCON0_MUX_MOTOR		equ		(B'00111'<<CHS0)|(1<<ADON)		; AN7(RC3, OPA2OUT)
ADCON0_MUX_FVR			equ		(B'11111'<<CHS0)|(1<<ADON)		; FVR(FVR output)
ADCON0_MUX_LED			equ		(B'01010'<<CHS0)|(1<<ADON)		; AN10(RB4, OPA1OUT)
ADCON1_MOTOR			equ		(5<<4)|(3<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=FVR, flush right
ADCON1_VINPUT			equ		(5<<4)|(0<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=VDD, flush right
ADCON1_LED				equ		(5<<4)|(3<<ADPREF0)|(1<<ADFM); Fosc/16, Vref+=FVR, flush right
ADCON2_MOTOR			equ		(B'11000'<<TRIGSEL0)		; PWM6-PR6 match
ADCON2_VINPUT			equ		(B'11010'<<TRIGSEL0)		; PWM6-OF6 match
ADCON2_LED				equ		(B'11010'<<TRIGSEL0)		; PWM6-OF6 match

; I/O port configurations
WPUA_init				equ		B'001011'		; pull-up:                  RA3,      RA1, RA0
TRISA_active			equ		B'001011'		; digital output:                RA2          
ANSELA_init				equ		B'110100'		; digital input:            RA3,      RA1, RA0 

WPUB_init				equ		B'11000000'		; pull-up:        RB7, RB6
TRISB_active			equ		B'11110000'		; digital output:                            
ANSELB_init				equ		B'00110000'		; digital input:  RB7, RB6

WPUC_init				equ		B'00000011'		; pull-up:                             RC1, RC0
TRISC_active			equ		B'11001111'		; digital output: RC5, RC4,
ANSELC_init				equ		B'11111100'		; digital input:                       RC1, RC0
HIDRVC_init				equ		B'00110000'

; common RAM allocation
common_ram				udata_shr		0x70
pulse_width_L			res		1		; 0p16	[V/V]
pulse_width_H			res		1

; prefix
; s0_ means single byte in bank 0.
;		single byte memory		s
;		double byte memory		d
;		function				f
;		array					a

;-----Bank0------------------
s0_PORTA                equ		PORTA
s0_PORTB                equ		PORTB
s0_PORTC                equ		PORTC
s0_PIR1                 equ		PIR1
s0_PIR2                 equ		PIR2
s0_PIR3                 equ		PIR3
s0_PIR4                 equ		PIR4
s0_TMR0                 equ		TMR0
d0_TMR1                 equ		TMR1
s0_TMR1L                equ		TMR1L
s0_TMR1H                equ		TMR1H
s0_T1CON                equ		T1CON
s0_T1GCON               equ		T1GCON
s0_T2TMR                equ		T2TMR
s0_T2PR                 equ		T2PR
s0_T2CON                equ		T2CON
s0_T2HLT                equ		T2HLT
s0_T2CLKCON             equ		T2CLKCON
s0_T2RST                equ		T2RST

; general purpose RAM (Bank 0) allocation
bank0_ram				udata	0x20
; Switch
s0_sw_transient			res		1		; transient state of SW
s0_sw_state				res		1		; current state {0, 0, RC1, RC0, RA3, 0, RA1, RA0}
s0_sw_released			res		1		; changed state of SW
s0_sw_pressed			res		1		; changed state of SW
s0_sw_toggle			res		1

;-----Bank1------------------
s1_TRISA                equ		TRISA
s1_TRISB                equ		TRISB
s1_TRISC                equ		TRISC
s1_PIE1                 equ		PIE1
s1_PIE2                 equ		PIE2
s1_PIE3                 equ		PIE3
s1_PIE4                 equ		PIE4
s1_OPTION_REG           equ		OPTION_REG
s1_PCON                 equ		PCON
s1_WDTCON               equ		WDTCON
s1_OSCTUNE              equ		OSCTUNE
s1_OSCCON               equ		OSCCON
s1_OSCSTAT              equ		OSCSTAT
d1_ADRES                equ		ADRES
s1_ADRESL               equ		ADRESL
s1_ADRESH               equ		ADRESH
s1_ADCON0               equ		ADCON0
s1_ADCON1               equ		ADCON1
s1_ADCON2               equ		ADCON2

; general purpose RAM (Bank 1) allocation
bank1_ram				udata	0xA0
s1_adres_Tm1_L			res		1
s1_adres_Tm1_H			res		1

;-----Bank2------------------
s2_LATA                 equ		LATA
s2_LATB                 equ		LATB
s2_LATC                 equ		LATC
s2_CMOUT                equ		CMOUT
s2_CM1CON0              equ		CM1CON0
s2_CM1CON1              equ		CM1CON1
s2_CM1NSEL              equ		CM1NSEL
s2_CM1PSEL              equ		CM1PSEL
s2_CM2CON0              equ		CM2CON0
s2_CM2CON1              equ		CM2CON1
s2_CM2NSEL              equ		CM2NSEL
s2_CM2PSEL              equ		CM2PSEL
s2_CM3CON0              equ		CM3CON0
s2_CM3CON1              equ		CM3CON1
s2_CM3NSEL              equ		CM3NSEL
s2_CM3PSEL              equ		CM3PSEL
s2_CM4CON0              equ		CM4CON0
s2_CM4CON1              equ		CM4CON1
s2_CM4NSEL              equ		CM4NSEL
s2_CM4PSEL              equ		CM4PSEL

;-----Bank3------------------
s3_ANSELA               equ		ANSELA
s3_ANSELB               equ		ANSELB
s3_ANSELC               equ		ANSELC
d3_PMADR                equ		PMADR
s3_PMADRL               equ		PMADRL
s3_PMADRH               equ		PMADRH
d3_PMDAT                equ		PMDAT
s3_PMDATL               equ		PMDATL
s3_PMDATH               equ		PMDATH
s3_PMCON1               equ		PMCON1
s3_PMCON2               equ		PMCON2
s3_VREGCON              equ		VREGCON
s3_RC1REG               equ		RC1REG
s3_TX1REG               equ		TX1REG
d3_SP1BRG               equ		SP1BRG
s3_SP1BRGL              equ		SP1BRGL
s3_SP1BRGH              equ		SP1BRGH
s3_RC1STA               equ		RC1STA
s3_TX1STA               equ		TX1STA
s3_BAUD1CON             equ		BAUD1CON

; general purpose RAM (Bank 3) allocation
;bank3_ram				udata	0x1A0

;-----Bank4------------------
s4_WPUA                 equ		WPUA
s4_WPUB                 equ		WPUB
s4_WPUC                 equ		WPUC
s4_SSP1BUF              equ		SSP1BUF
s4_SSP1ADD              equ		SSP1ADD
s4_SSP1MSK              equ		SSP1MSK
s4_SSP1STAT             equ		SSP1STAT
s4_SSP1CON1             equ		SSP1CON1
s4_SSP1CON2             equ		SSP1CON2
s4_SSP1CON3             equ		SSP1CON3
s4_BORCON               equ		BORCON
s4_FVRCON               equ		FVRCON
s4_ZCD1CON              equ		ZCD1CON

;-----Bank5------------------
s5_ODCONA               equ		ODCONA
s5_ODCONB               equ		ODCONB
s5_ODCONC               equ		ODCONC
d5_CCPR1                equ		CCPR1
s5_CCPR1L               equ		CCPR1L
s5_CCPR1H               equ		CCPR1H
s5_CCP1CON              equ		CCP1CON
s5_CCP1CAP              equ		CCP1CAP
d5_CCPR2                equ		CCPR2
s5_CCPR2L               equ		CCPR2L
s5_CCPR2H               equ		CCPR2H
s5_CCP2CON              equ		CCP2CON
s5_CCP2CAP              equ		CCP2CAP
s5_CCPTMRS              equ		CCPTMRS

;-----Bank6------------------
s6_SLRCONA              equ		SLRCONA
s6_SLRCONB              equ		SLRCONB
s6_SLRCONC              equ		SLRCONC
s6_MD2CON0              equ		MD2CON0
s6_MD2CON1              equ		MD2CON1
s6_MD2SRC               equ		MD2SRC
s6_MD2CARL              equ		MD2CARL
s6_MD2CARH              equ		MD2CARH

;-----Bank7------------------
s7_INLVLA               equ		INLVLA
s7_INLVLB               equ		INLVLB
s7_INLVLC               equ		INLVLC
s7_IOCAP                equ		IOCAP
s7_IOCAN                equ		IOCAN
s7_IOCAF                equ		IOCAF
s7_IOCBP                equ		IOCBP
s7_IOCBN                equ		IOCBN
s7_IOCBF                equ		IOCBF
s7_IOCCP                equ		IOCCP
s7_IOCCN                equ		IOCCN
s7_IOCCF                equ		IOCCF
s7_MD1CON0              equ		MD1CON0
s7_MD1CON1              equ		MD1CON1
s7_MD1SRC               equ		MD1SRC
s7_MD1CARL              equ		MD1CARL
s7_MD1CARH              equ		MD1CARH

;-----Bank8------------------
s8_HIDRVC               equ		HIDRVC
s8_T4TMR                equ		T4TMR
s8_T4PR                 equ		T4PR
s8_T4CON                equ		T4CON
s8_T4HLT                equ		T4HLT
s8_T4CLKCON             equ		T4CLKCON
s8_T4RST                equ		T4RST
s8_T6TMR                equ		T6TMR
s8_T6PR                 equ		T6PR
s8_T6CON                equ		T6CON
s8_T6HLT                equ		T6HLT
s8_T6CLKCON             equ		T6CLKCON
s8_T6RST                equ		T6RST

;-----Bank9------------------
d9_TMR3                 equ		TMR3
s9_TMR3L                equ		TMR3L
s9_TMR3H                equ		TMR3H
s9_T3CON                equ		T3CON
s9_T3GCON               equ		T3GCON
d9_TMR5                 equ		TMR5
s9_TMR5L                equ		TMR5L
s9_TMR5H                equ		TMR5H
s9_T5CON                equ		T5CON
s9_T5GCON               equ		T5GCON

;-----Bank10------------------
s10_OPA1NCHS            equ		OPA1NCHS
s10_OPA1PCHS            equ		OPA1PCHS
s10_OPA1CON             equ		OPA1CON
s10_OPA1ORS             equ		OPA1ORS
s10_OPA2NCHS            equ		OPA2NCHS
s10_OPA2PCHS            equ		OPA2PCHS
s10_OPA2CON             equ		OPA2CON
s10_OPA2ORS             equ		OPA2ORS

;-----Bank11------------------
s11_DACLD               equ		DACLD
s11_DAC1CON0            equ		DAC1CON0
d11_DAC1REF             equ		DAC1REF
s11_DAC1REFL            equ		DAC1REFL
s11_DAC1REFH            equ		DAC1REFH
s11_DAC2CON0            equ		DAC2CON0
d11_DAC2REF             equ		DAC2REF
s11_DAC2REFL            equ		DAC2REFL
s11_DAC2REFH            equ		DAC2REFH
s11_DAC3CON0            equ		DAC3CON0
s11_DAC3REF             equ		DAC3REF
s11_DAC4CON0            equ		DAC4CON0
s11_DAC4REF             equ		DAC4REF

;-----Bank12------------------
s12_PWM3DCL             equ		PWM3DCL
s12_PWM3DCH             equ		PWM3DCH
s12_PWM3CON             equ		PWM3CON
s12_PWM4DCL             equ		PWM4DCL
s12_PWM4DCH             equ		PWM4DCH
s12_PWM4CON             equ		PWM4CON

;-----Bank13------------------
s13_COG1PHR             equ		COG1PHR
s13_COG1PHF             equ		COG1PHF
s13_COG1BLKR            equ		COG1BLKR
s13_COG1BLKF            equ		COG1BLKF
s13_COG1DBR             equ		COG1DBR
s13_COG1DBF             equ		COG1DBF
s13_COG1CON0            equ		COG1CON0
s13_COG1CON1            equ		COG1CON1
s13_COG1RIS0            equ		COG1RIS0
s13_COG1RIS1            equ		COG1RIS1
s13_COG1RSIM0           equ		COG1RSIM0
s13_COG1RSIM1           equ		COG1RSIM1
s13_COG1FIS0            equ		COG1FIS0
s13_COG1FIS1            equ		COG1FIS1
s13_COG1FSIM0           equ		COG1FSIM0
s13_COG1FSIM1           equ		COG1FSIM1
s13_COG1ASD0            equ		COG1ASD0
s13_COG1ASD1            equ		COG1ASD1
s13_COG1STR             equ		COG1STR

;-----Bank14------------------
s14_COG2PHR             equ		COG2PHR
s14_COG2PHF             equ		COG2PHF
s14_COG2BLKR            equ		COG2BLKR
s14_COG2BLKF            equ		COG2BLKF
s14_COG2DBR             equ		COG2DBR
s14_COG2DBF             equ		COG2DBF
s14_COG2CON0            equ		COG2CON0
s14_COG2CON1            equ		COG2CON1
s14_COG2RIS0            equ		COG2RIS0
s14_COG2RIS1            equ		COG2RIS1
s14_COG2RSIM0           equ		COG2RSIM0
s14_COG2RSIM1           equ		COG2RSIM1
s14_COG2FIS0            equ		COG2FIS0
s14_COG2FIS1            equ		COG2FIS1
s14_COG2FSIM0           equ		COG2FSIM0
s14_COG2FSIM1           equ		COG2FSIM1
s14_COG2ASD0            equ		COG2ASD0
s14_COG2ASD1            equ		COG2ASD1
s14_COG2STR             equ		COG2STR

;-----Bank15------------------
s15_PRG1RTSS            equ		PRG1RTSS
s15_PRG1FTSS            equ		PRG1FTSS
s15_PRG1INS             equ		PRG1INS
s15_PRG1CON0            equ		PRG1CON0
s15_PRG1CON1            equ		PRG1CON1
s15_PRG1CON2            equ		PRG1CON2
s15_PRG2RTSS            equ		PRG2RTSS
s15_PRG2FTSS            equ		PRG2FTSS
s15_PRG2INS             equ		PRG2INS
s15_PRG2CON0            equ		PRG2CON0
s15_PRG2CON1            equ		PRG2CON1
s15_PRG2CON2            equ		PRG2CON2

;-----Bank27------------------
s27_PWMEN               equ		PWMEN
s27_PWMLD               equ		PWMLD
s27_PWMOUT              equ		PWMOUT
d27_PWM5PH              equ		PWM5PH
s27_PWM5PHL             equ		PWM5PHL
s27_PWM5PHH             equ		PWM5PHH
d27_PWM5DC              equ		PWM5DC
s27_PWM5DCL             equ		PWM5DCL
s27_PWM5DCH             equ		PWM5DCH
d27_PWM5PR              equ		PWM5PR
s27_PWM5PRL             equ		PWM5PRL
s27_PWM5PRH             equ		PWM5PRH
d27_PWM5OF              equ		PWM5OF
s27_PWM5OFL             equ		PWM5OFL
s27_PWM5OFH             equ		PWM5OFH
d27_PWM5TMR             equ		PWM5TMR
s27_PWM5TMRL            equ		PWM5TMRL
s27_PWM5TMRH            equ		PWM5TMRH
s27_PWM5CON             equ		PWM5CON
s27_PWM5INTE            equ		PWM5INTE
s27_PWM5INTF            equ		PWM5INTF
s27_PWM5CLKCON          equ		PWM5CLKCON
s27_PWM5LDCON           equ		PWM5LDCON
s27_PWM5OFCON           equ		PWM5OFCON
d27_PWM6PH              equ		PWM6PH
s27_PWM6PHL             equ		PWM6PHL
s27_PWM6PHH             equ		PWM6PHH
d27_PWM6DC              equ		PWM6DC
s27_PWM6DCL             equ		PWM6DCL
s27_PWM6DCH             equ		PWM6DCH
d27_PWM6PR              equ		PWM6PR
s27_PWM6PRL             equ		PWM6PRL
s27_PWM6PRH             equ		PWM6PRH
d27_PWM6OF              equ		PWM6OF
s27_PWM6OFL             equ		PWM6OFL
s27_PWM6OFH             equ		PWM6OFH
d27_PWM6TMR             equ		PWM6TMR
s27_PWM6TMRL            equ		PWM6TMRL
s27_PWM6TMRH            equ		PWM6TMRH
s27_PWM6CON             equ		PWM6CON
s27_PWM6INTE            equ		PWM6INTE
s27_PWM6INTF            equ		PWM6INTF
s27_PWM6CLKCON          equ		PWM6CLKCON
s27_PWM6LDCON           equ		PWM6LDCON
s27_PWM6OFCON           equ		PWM6OFCON

;-----Bank28------------------
s28_PPSLOCK             equ		PPSLOCK
s28_INTPPS              equ		INTPPS
s28_T0CKIPPS            equ		T0CKIPPS
s28_T1CKIPPS            equ		T1CKIPPS
s28_T1GPPS              equ		T1GPPS
s28_CCP1PPS             equ		CCP1PPS
s28_CCP2PPS             equ		CCP2PPS
s28_COG1INPPS           equ		COG1INPPS
s28_COG2INPPS           equ		COG2INPPS
s28_T2INPPS             equ		T2INPPS
s28_T3CKIPPS            equ		T3CKIPPS
s28_T3GPPS              equ		T3GPPS
s28_T4INPPS             equ		T4INPPS
s28_T5CKIPPS            equ		T5CKIPPS
s28_T5GPPS              equ		T5GPPS
s28_T6INPPS             equ		T6INPPS
s28_SSPCLKPPS           equ		SSPCLKPPS
s28_SSPDATPPS           equ		SSPDATPPS
s28_SSPSSPPS            equ		SSPSSPPS
s28_RXPPS               equ		RXPPS
s28_CKPPS               equ		CKPPS
s28_CLCIN0PPS           equ		CLCIN0PPS
s28_CLCIN1PPS           equ		CLCIN1PPS
s28_CLCIN2PPS           equ		CLCIN2PPS
s28_CLCIN3PPS           equ		CLCIN3PPS
s28_PRG1RPPS            equ		PRG1RPPS
s28_PRG1FPPS            equ		PRG1FPPS
s28_PRG2RPPS            equ		PRG2RPPS
s28_PRG2FPPS            equ		PRG2FPPS
s28_MD1CHPPS            equ		MD1CHPPS
s28_MD1CLPPS            equ		MD1CLPPS
s28_MD1MODPPS           equ		MD1MODPPS
s28_MD2CHPPS            equ		MD2CHPPS
s28_MD2CLPPS            equ		MD2CLPPS
s28_MD2MODPPS           equ		MD2MODPPS

;-----Bank29------------------
s29_RA0PPS              equ		RA0PPS
s29_RA1PPS              equ		RA1PPS
s29_RA2PPS              equ		RA2PPS
s29_RA4PPS              equ		RA4PPS
s29_RA5PPS              equ		RA5PPS
s29_RB4PPS              equ		RB4PPS
s29_RB5PPS              equ		RB5PPS
s29_RB6PPS              equ		RB6PPS
s29_RB7PPS              equ		RB7PPS
s29_RC0PPS              equ		RC0PPS
s29_RC1PPS              equ		RC1PPS
s29_RC2PPS              equ		RC2PPS
s29_RC3PPS              equ		RC3PPS
s29_RC4PPS              equ		RC4PPS
s29_RC5PPS              equ		RC5PPS
s29_RC6PPS              equ		RC6PPS
s29_RC7PPS              equ		RC7PPS

;-----Bank30------------------
s30_CLCDATA             equ		CLCDATA
s30_CLC1CON             equ		CLC1CON
s30_CLC1POL             equ		CLC1POL
s30_CLC1SEL0            equ		CLC1SEL0
s30_CLC1SEL1            equ		CLC1SEL1
s30_CLC1SEL2            equ		CLC1SEL2
s30_CLC1SEL3            equ		CLC1SEL3
s30_CLC1GLS0            equ		CLC1GLS0
s30_CLC1GLS1            equ		CLC1GLS1
s30_CLC1GLS2            equ		CLC1GLS2
s30_CLC1GLS3            equ		CLC1GLS3
s30_CLC2CON             equ		CLC2CON
s30_CLC2POL             equ		CLC2POL
s30_CLC2SEL0            equ		CLC2SEL0
s30_CLC2SEL1            equ		CLC2SEL1
s30_CLC2SEL2            equ		CLC2SEL2
s30_CLC2SEL3            equ		CLC2SEL3
s30_CLC2GLS0            equ		CLC2GLS0
s30_CLC2GLS1            equ		CLC2GLS1
s30_CLC2GLS2            equ		CLC2GLS2
s30_CLC2GLS3            equ		CLC2GLS3
s30_CLC3CON             equ		CLC3CON
s30_CLC3POL             equ		CLC3POL
s30_CLC3SEL0            equ		CLC3SEL0
s30_CLC3SEL1            equ		CLC3SEL1
s30_CLC3SEL2            equ		CLC3SEL2
s30_CLC3SEL3            equ		CLC3SEL3
s30_CLC3GLS0            equ		CLC3GLS0
s30_CLC3GLS1            equ		CLC3GLS1
s30_CLC3GLS2            equ		CLC3GLS2
s30_CLC3GLS3            equ		CLC3GLS3

;-----Bank31------------------
s31_STATUS_SHAD         equ		STATUS_SHAD
s31_WREG_SHAD           equ		WREG_SHAD
s31_BSR_SHAD            equ		BSR_SHAD
s31_PCLATH_SHAD         equ		PCLATH_SHAD
s31_FSR0L_SHAD          equ		FSR0L_SHAD
s31_FSR0H_SHAD          equ		FSR0H_SHAD
s31_FSR1L_SHAD          equ		FSR1L_SHAD
s31_FSR1H_SHAD          equ		FSR1H_SHAD
s31_STKPTR              equ		STKPTR
s31_TOSL                equ		TOSL
s31_TOSH                equ		TOSH


;		*		*		*		*		*		*		*		*		*		*		*		;
;		Reset vector																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
reset_vector			code	0x000
						movlb	1
						movlw	OSCCON_init
						movwf	s1_OSCCON
						goto	start

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Interrupt Routine																		;
;		*		*		*		*		*		*		*		*		*		*		*		;

; interrupt handler is consist of pulse generator and voltage sampler.
interrupt_handler		code	0x004
						; W, STATUS, BSR, FSRs, PCLATH are shadowed, latency 3-5
						clrf	PCLATH
						movlb	0
						bcf		s0_PIR1,ADIF

						movlb	1
						movf	ADRESL,W
						movlb	11
						movwf	s11_DAC2REFL
						movlb	1
						subwf	s1_adres_Tm1_L,W
						movf	ADRESH,W
						movlb	11
						movwf	s11_DAC2REFH
						bsf		s11_DACLD,DAC2LD
						movlb	1
						subwfb	s1_adres_Tm1_H,W
						movlb	2
						btfss	STATUS,C
						bsf		s2_LATA,RA2
						btfsc	STATUS,C
						bcf		s2_LATA,RA2

						movlb	1
						movlw	(LOW -20)
						addwf	s1_ADRESL,W
						movwf	s1_adres_Tm1_L
						movlw	(HIGH -20)
						addwfc	s1_ADRESH,W
						movwf	s1_adres_Tm1_H
						retfie

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Main routine																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
start:	
						bcf		INTCON,GIE
						bsf		INTCON,PEIE

						; clear data memory
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


						movlb	4
						movlw	WPUA_init
						movwf	s4_WPUA
						movlw	WPUB_init
						movwf	s4_WPUB
						movlw	WPUC_init
						movwf	s4_WPUC
						movlw	FVRCON_init
						movwf	s4_FVRCON

						movlb	1
						bcf		s1_OPTION_REG,NOT_WPUEN
	if 1
						movlw	ADCON0_MUX_MOTOR
						movwf	s1_ADCON0
						movlw	ADCON1_MOTOR
						movwf	s1_ADCON1
						movlw	ADCON2_MOTOR
						movwf	s1_ADCON2
	endif
	if 0
						movlw	ADCON0_MUX_FVR
						movwf	s1_ADCON0
						movlw	ADCON1_VINPUT
						movwf	s1_ADCON1
						movlw	ADCON2_VINPUT
						movwf	s1_ADCON2
	endif
	if 0
						movlw	ADCON0_MUX_LED
						movwf	s1_ADCON0
						movlw	ADCON1_LED
						movwf	s1_ADCON1
						movlw	ADCON2_LED
						movwf	s1_ADCON2
	endif

						movlb	3
						movlw	ANSELA_init
						movwf	s3_ANSELA
						movlw	ANSELB_init
						movwf	s3_ANSELB
						movlw	ANSELC_init
						movwf	s3_ANSELC

						movlb	8
						movlw	HIDRVC_init
						movwf	s8_HIDRVC

						; Key polling timer
						clrf	s8_T4TMR
						movlw	T4PR_init
						movwf	s8_T4PR
						movlw	T4HLT_init
						movwf	s8_T4HLT
						movlw	T4CLKCON_init
						movwf	s8_T4CLKCON
						movlw	T4RST_init
						movwf	s8_T4RST
						movlw	T4CON_init
						movwf	s8_T4CON

						movlb	10
						movlw	OPA1NCHS_init
						movwf	s10_OPA1NCHS
						movlw	OPA1PCHS_init
						movwf	s10_OPA1PCHS
						movlw	OPA1CON_init
						movwf	s10_OPA1CON
						movlw	OPA1ORS_init
						movwf	s10_OPA1ORS
						movlw	OPA2NCHS_init
						movwf	s10_OPA2NCHS
						movlw	OPA2PCHS_init
						movwf	s10_OPA2PCHS
						movlw	OPA2CON_init
						movwf	s10_OPA2CON
						movlw	OPA2ORS_init
						movwf	s10_OPA2ORS

						movlb	11
						movlw	DAC2CON0_init
						movwf	s11_DAC2CON0

						; Constant brightness LED driver
						movlw	DAC4CON0_init
						movwf	s11_DAC4CON0
						movlw	DAC4REF_init
						movwf	s11_DAC4REF

						; FET Driver	COG1A -> pMOS, COG1B -> nMOS
						movlb	13
						bsf	s13_COG1CON0,EN
						movlw	COG1PHR_init
						movwf	s13_COG1PHR
						movlw	COG1PHF_init
						movwf	s13_COG1PHF
						movlw	COG1BLKR_init
						movwf	s13_COG1BLKR
						movlw	COG1BLKF_init
						movwf	s13_COG1BLKF
						movlw	COG1DBR_init
						movwf	s13_COG1DBR
						movlw	COG1DBF_init
						movwf	s13_COG1DBF
						movlw	COG1CON1_init
						movwf	s13_COG1CON1
						movlw	COG1RIS0_init
						movwf	s13_COG1RIS0
						movlw	COG1RIS1_init
						movwf	s13_COG1RIS1
						movlw	COG1RSIM0_init
						movwf	s13_COG1RSIM0
						movlw	COG1RSIM1_init
						movwf	s13_COG1RSIM1
						movlw	COG1FIS0_init
						movwf	s13_COG1FIS0
						movlw	COG1FIS1_init
						movwf	s13_COG1FIS1
						movlw	COG1FSIM0_init
						movwf	s13_COG1FSIM0
						movlw	COG1FSIM1_init
						movwf	s13_COG1FSIM1
						movlw	COG1ASD0_init
						movwf	s13_COG1ASD0
						movlw	COG1ASD1_init
						movwf	s13_COG1ASD1
						movlw	COG1STR_init
						movwf	s13_COG1STR
						movlw	COG1CON0_init
						movwf	s13_COG1CON0
						bsf		s13_COG1CON0,LD

						; PWM generator
						movlb	27
						movlw	LOW PWM6PH_init
						movwf	s27_PWM6PHL
						movlw	HIGH PWM6PH_init
						movwf	s27_PWM6PHH
						movlw	LOW PWM6DC_init
						movwf	s27_PWM6DCL
						movlw	HIGH PWM6DC_init
						movwf	s27_PWM6DCH
						movlw	LOW PWM6PR_init
						movwf	s27_PWM6PRL
						movlw	HIGH PWM6PR_init
						movwf	s27_PWM6PRH
						movlw	LOW PWM6OF_init
						movwf	s27_PWM6OFL
						movlw	HIGH PWM6OF_init
						movwf	s27_PWM6OFH
						clrf	s27_PWM6TMRL
						clrf	s27_PWM6TMRH
						movlw	PWM6INTE_init
						movwf	s27_PWM6INTE
						movlw	PWM6INTF_init
						movwf	s27_PWM6INTF
						movlw	PWM6CLKCON_init
						movwf	s27_PWM6CLKCON
						movlw	PWM6LDCON_init
						movwf	s27_PWM6LDCON
						movlw	PWM6OFCON_init
						movwf	s27_PWM6OFCON
						movlw	PWM6CON_init
						movwf	s27_PWM6CON

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
				;											;  4: RA3/Vpp	Power SW
						movlw	PPS_HB_HS_init
						movwf	s29_RC5PPS					;  5: COG1B
						movlw	PPS_HB_LS_init
						movwf	s29_RC4PPS					;  6: COG1A
				;		clrf	s29_RC3PPS					;  7: AN7/OPA2OUT
				;		movwf	s29_RC6PPS					;  8: AN8/OPA2IN0-
				;		movwf	s29_RC7PPS					;  9: AN9/OPA2IN0+
				;		movwf	s29_RB7PPS					; 10: RB7		Origin SW
				;		movwf	s29_RB6PPS					; 11: RB6		Limit SW
				;		movwf	s29_RB5PPS					; 12: AN11/OPA1IN0+
				;		movwf	s29_RB4PPS					; 13: AN10/OPAIN0-
				;		movwf	s29_RC2PPS					; 14: PWM5/OPA1OUT
				;		clrf	s29_RC1PPS					; 15: RC1		Double SW
				;		clrf	s29_RC0PPS					; 16: RC0		Stop SW
						movlw	PPS_RA2_init
						movwf	s29_RA2PPS					; 17: RA2
				;		clrf	s29_RA1PPS					; 18: ICSPCLK
				;		clrf	s29_RA0PPS					; 19: ICSPDAT
				;											; 20: VSS

						movlb	1
						; output enable
						movlw	TRISA_active
						movwf	s1_TRISA
						movlw	TRISB_active
						movwf	s1_TRISB
						movlw	TRISC_active
						movwf	s1_TRISC

						; interrupt enable
						clrf	s1_PIE1
						clrf	s1_PIE2
						clrf	s1_PIE3
						clrf	s1_PIE4
						bsf		s1_PIE1,ADIE

						bsf		s1_PIE1,ADIE

						bsf		INTCON,GIE

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

						; connect
						movlb	13
						bcf		s13_COG1ASD0,ASE

main_loop:				movlb	0
						; oscillator controll
						call	f0_osc_ctrl

						call	f0_read_keys
						bra		main_loop

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Oscillator Controll																		;
;		*		*		*		*		*		*		*		*		*		*		*		;
f0_osc_ctrl:			movlb	0
						nop		; -200uA
						nop
						nop
						nop
						nop
						nop
						nop		; -100uA
						nop
						nop
						; wait for polling timer match
						btfss	s0_PIR4,TMR4IF
						bra		$-10

						bcf		s0_PIR4,TMR4IF			; 50Hz
						return

;		*		*		*		*		*		*		*		*		*		*		*		;
;		Key Handling																			;
;		*		*		*		*		*		*		*		*		*		*		*		;
f0_read_keys:			movf	s0_sw_state,W
						movwf	s0_sw_released
						movwf	s0_sw_pressed

						; reduce chattering of key
						movf	s0_PORTB,W
						andlw	B'11000000'
						iorwf	s0_PORTA,W			; {RB7, RB6, 0, 0, RA3, 0, RA1, RA0}
						xorlw	B'11001011'		; invert logic (from negative to positive)

						xorwf	s0_sw_transient,W		; W = changed SW mask
						andwf	s0_sw_state,F	; erase unchanged bits
						xorwf	s0_sw_transient,F		; update changed bits
						xorlw	0xFF			; W = unchanged SW mask
						andwf	s0_sw_transient,W		; W = unchanged bits
						iorwf	s0_sw_state,F	; update unchanged bits

						; if RA0 == RA1, ignore RA1 and RA0
						; if RB7 == RB6, ignore RB7 and RB6
						lslf	s0_sw_state,W
						xorwf	s0_sw_state,W
						iorlw	B'01111101'		; mask = {RB7^RB6, 1, 1, 1, 1, 1, RA1^RA0, 1}
						andwf	s0_sw_state,F
						lsrf	WREG,F
						bsf		WREG,7			; mask = {1, RB7^RB6, 1, 1, 1, 1, 1, 1, RA1^RA0}
						andwf	s0_sw_state,F

						comf	s0_sw_state,W
						andwf	s0_sw_released,F		; released SW(1: released)
						iorwf	s0_sw_pressed,F
						comf	s0_sw_pressed,F			; pressed SW(1: pressed)

						movf	s0_sw_released,W
						xorwf	s0_sw_toggle,F

						clrwdt
						return

						end

