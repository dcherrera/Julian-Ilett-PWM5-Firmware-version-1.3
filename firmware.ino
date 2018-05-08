;12F683 PWM Charge Controller version 1.3 (1F3)
;Requires hardware version 1.1 (1H1)
;(82k/20k resistor divider - 220pF - no zener)
;Ensure MCLRE_ON for production units

;*Features

;*Generates FET gate voltage using charge pump
;*Adjusts PWM duty cycle to obtain target voltage
;*Variable speed duty cycle adjustment
;*Indicates voltage as sequence of flashes on LED
;*Indicates voltage in units and tenths
;*Steady flashing indicates battery undervolt (< 11v)
;*Simulates PWM on LED when target voltage is reached
;*Switches from saturation to float after variable time period
;*Immune from temperature variations
;*Battery disconnect detect

;Since last version

;To do

;set ADC to Frc oscillator (for conversion during sleep) - done
;set result to right justify (ADFM=1) for 10-bit mode - done
;subtract 0180h from ADRESH/L, put result (with limits) in 'voltage' - done
;set 'zener' voltage to 96 decivolts (voltage at 0180h) - done
;switch to active low PWM - done
;synchronise adc measurement with PWM - done
;perform adc during sleep - done
;construct interrupt structure - done
;read ADC and adjust PWM on interrupt - done
;fix bugs in low volt display - done
;introduce PWM window for accurate measurement - done
;adjust PWM bulk window for optimum performance - done
;devise detection of battery disconnect - done
;adjust setpoint switchover times - done
;universalise pwmlimit section for all limit types - done
;introduce itemp for use in isr - done
;implement peak voltage variable - done


 list p=12F683
#include "p12f683.inc"
	
 __CONFIG _INTRC_OSC_NOCLKOUT & _MCLRE_ON & _WDT_ON & _PWRTE_ON & _CP_ON

;Constants

ZenerVoltage	equ	d'96'	;zener voltage in decivolts	
SatVoltage	equ	d'144'	;saturation voltage in decivolts
FloatVoltage	equ	d'135'	;float voltage in decivolts

;mode bits
;bit 0 - LED display mode:	0=LED disabled, 1=LED enabled
;bit 2 - voltage range	0=below 13.0v, 1=above 13.5v
;bit 3 - PWM status:	0=bulk charge mode, 1=pwm charge mode
;bit 4 - regulation mode:	0=float, 1=saturation
;bit 5 - 
;bit 6 - 
;bit 7 - button status:	0=pressed, 1=released

mode	equ	0x20	;mode flags - bitwise usage
temp	equ	0x21	;general purpose temporary register
pattern	equ	0x22	;incrementing pattern generates LED sequence
voltunits	equ	0x23	;units of measured voltage (from 10.0 base)
volttenths	equ	0x24	;tenths of measured voltage
duty	equ	0x25	;duty cycle value
decivolts	equ	0x26	;decivolts from a 10v baseline (table index)
target	equ	0x27	;target voltage (2s complement)
voltage	equ	0x28	;8 bit voltage value from ADC
floatcount	equ	0x29	;counter to invoke float mode
floatseconds	equ	0x2A	;number of seconds between float counts
adccount	equ	0x2B	;adc cycle counter
avoltage	equ	0x2C	;accurate 8 bit voltage value from ADC
limitcount	equ	0x2D	;introduces n pwm off (or 50%) cycles
dutylimit	equ	0x2E	;used to limit duty cycle
itemp	equ	0x2F	;temporary register for use in isr
pvoltage	equ	0x30	;peak voltage indicator

save_w	equ	0x70	;save location for W during isr
save_status	equ	0x71	;save location for STATUS during isr


 org 0x000

jump
	goto	setclocks


 org 0x004

isr
	movwf	save_w
	swapf	STATUS,w
	movwf	save_status
	banksel	0x00	;bank bit saved, now ensure low bank
	bcf	PIR1,TMR2IF	;Timer2 just overflowed - clear the flag
	call	adcread
	call	chargepump
	call	cyclecheck
	call	chargepump
	call	pwmadjust
	call	chargepump
;	call	button
	swapf	save_status,w
	movwf	STATUS
	swapf	save_w,f
	swapf	save_w,w
	retfie

setclocks
	banksel	0x00	;select lower register bank
	movlw	0x10
	movwf	WDTCON	;set WDT ripple counter to 1:8192

	banksel	0x80	;select upper register bank
	movlw	0x31
	movwf	OSCCON	;set CPU clock to 500kHz

delay
	banksel	0x00	;select lower register bank
	movlw	0xE0
	movwf	TMR1H	;set Timer1 for 200ms delay
	clrf	TMR1L
	movlw	0x21
	movwf	T1CON	;enable timer1 with 1:4 prescale
	bcf	PIR1,0
delaywait	clrwdt		;watch the doggy during initial delay
	btfss	PIR1,0
	goto	delaywait	;wait for timer1 flag to be set

setports
	banksel	0x00	;select lower register bank
	movlw	0x07
	movwf	CMCON0	;turn off comparator (for proper I/O use)
	bcf	GPIO,4	;clear charge pump first output
	bsf	GPIO,5	;set charge pump second output
	clrf	mode	;have to do this before switch routine

	banksel	0x80	;select upper register bank
	movlw	0x71	;Frc = 2-6us aquisition
	movwf	ANSEL	;GP0 is analogue input, set A/D clock
	bcf	TRISIO,1	;RA1 is output to drive LED
	bcf	TRISIO,2	;RA2 is output for PWM
	bsf	TRISIO,3	;RA3 is input for test button
	bcf	TRISIO,4	;RA4 is output for charge pump
	bcf	TRISIO,5	;RA5 is output for charge pump

switch
	banksel	0x80
	clrf	EEADR	;only EEprom location 00 is used
	bsf	EECON1,0
	movf	EEDAT,w	;read EE data into w
	banksel	0x00
	bsf	mode,0	;change to bcf for LED disable feature
	btfss	STATUS,Z
	bsf	mode,0	;enable LED display mode
	banksel	0x80
	movlw	0x00
	movwf	EEDAT	;write 00 to EEprom
	bsf	EECON1,2
	movlw	0x55	;
	movwf	EECON2	;required EE write sequence
	movlw	0xAA	;
	movwf	EECON2	;
	bsf	EECON1,1
	banksel	0x00
	clrf	TMR1H	;clear timer1 registers for full delay
	clrf	TMR1L
	bsf	GPIO,1	;turn on LED
	bcf	PIR1,0
switchwait	clrwdt		;watch the doggy during LED on
	btfss	PIR1,0
	goto	switchwait	;wait for timer1 flag to be set
	bcf	GPIO,1	;turn off LED
	banksel	0x80
	movlw	0xFF
	movwf	EEDAT	;write FF to EEprom
	bsf	EECON1,2
	movlw	0x55	;
	movwf	EECON2	;required EE write sequence
	movlw	0xAA	;
	movwf	EECON2	;
	bsf	EECON1,1

initv
	banksel	0x00	;select lower register bank
	movlw	-1 * (FloatVoltage - ZenerVoltage) * 4
	movwf	target	;float target voltage
	clrf	pattern
	clrf	duty	;start duty cycle at 0%
	clrf	floatcount	;set at the float end
	incf	floatcount,f	;floatcount runs from 01 to FF
	clrf	floatseconds
	clrf	decivolts
	clrf	voltunits
	clrf	volttenths
	clrf	adccount
	clrf	limitcount
	movlw	0xFF
	movwf	dutylimit	;dutylimit at FB for bulk window
	
initadc
	banksel	0x00	;select lower register bank
	movlw	0x81	;right justify, Vdd = vref
	movwf	ADCON0	;select channel 0, enable A/D

	banksel	0x80
	bsf	PIE1,ADIE	;enable adc interrupt

inithw
	banksel	0x80	;select upper register bank
	movlw	0x83	;set up timer0, prescale=16
	movwf	OPTION_REG	;LED sequence cycle time approx 5 secs
	movlw	0xFF
	movwf	PR2	;set PWM period to 8.192ms (122Hz)
	bsf	PIE1,TMR2IE	;enable timer2 interrupt

	banksel	0x00	;select lower register bank
	movlw	0x0E	;0C for active high PWM, 0E active low
	movwf	CCP1CON	;select PWM mode (active low)
	movlw	0x11	;Timer1 on with 1:2 prescale
	movwf	T1CON	;Timer1 period: 1.05 seconds 
	movlw	0x05
	movwf	T2CON	;Timer2 on, 1:4 prescale, 1:1 postscale
	bsf	INTCON,PEIE	;enable peripheral interrupts
	bsf	INTCON,GIE	;enable global interrupts

main
	btfsc	INTCON,2	;check if Timer0 overflow flag is set
	call	leddisplay	;call sub to inc pattern and light LED
	call	chargepump	;call sub to generate FET gate voltage
	btfsc	PIR1,0	;check if Timer1 overflow flag is set
	call	floater	;call sub to determine regulation mode
	call	chargepump	;call sub to generate FET gate voltage
	goto	main

button
	btfss	GPIO,3
	goto	buttonlo
buttonhi	bsf	mode,7
	return
buttonlo	btfsc	mode,7
	goto	buttondown
	return
buttondown	bcf	mode,7	;button has just been pressed
	nop		;do this on button press
	return

adcread
	bcf	PIR1,ADIF	;clear the adc interrupt flag
	nop		;isr has 12us
	nop		;adcread has 6us
	nop		;plus 12us of nops
	nop		;makes 30us total delay
	nop		;before A/D sample
	nop		;bulk window is 128us
	bsf	ADCON0,1	;start an A/D conversion (after sleep)
	sleep		;let's have some quiet around here
	bcf	PIR1,ADIF
	call	chargepump	;pump charge after adc conversion
adcsubtract	movlw	0x80	;now subtract hex 0180 from ADRESH/L
	bsf	STATUS,RP0
	subwf	ADRESL,w	;ADRESL is in the high bank (bizarely)
	bcf	STATUS,RP0
	movwf	itemp
	swapf	itemp,f	;cos we swap when we bring it back
	movlw	0x01
	btfss	STATUS,C
	movlw	0x02
	subwf	ADRESH,w
	swapf	itemp,w	;swap back to w (don't upset Z flag)
	btfss	STATUS,Z
	call	overvolt	;voltage measurement is above range
	btfss	STATUS,C
	call	undervolt	;voltage measurement is below range
adccycle	movf	adccount,f	;doing an accurate measurement?
	btfsc	STATUS,Z
	movwf	avoltage	;voltage measurement is accurate
	movwf	voltage	;put it into voltage
	movwf	itemp	;put into itemp for decimal conversion
adcpeak	subwf	pvoltage,w	;check if voltage > peak voltage
	btfsc	STATUS,C
	goto	adcbatt	;voltage does not exceed peak
	movf	voltage,w	;voltage exceeds peak
	movwf	pvoltage	;put voltage into peak voltage
adcbatt	movf	ADRESH,f
	btfsc	STATUS,Z
	call	badbatt	;battery voltage very low (<6.4 volts)
adcdeci	movlw	(d'100' - ZenerVoltage) * 4
	subwf	itemp,f
	btfss	STATUS,C
	clrf	itemp	;don't allow negative values (<10v)
	bcf	STATUS,C	;
	rrf	itemp,f	;divide by 4
	bcf	STATUS,C	;
	rrf	itemp,f	;
	movf	itemp,w
	movwf	decivolts	;number of volt tenths above 10v
	return

overvolt
	movlw	0x00	;battery voltage >= 16.0 volts
	movwf	dutylimit
	movlw	0x02
	movwf	limitcount
	retlw	0xFF

undervolt
	movlw	0x80	;battery voltage < 9.6 volts
	movwf	dutylimit
	movlw	0x02
	movwf	limitcount
	retlw	0x00

badbatt
	movlw	0x00	;battery voltage < 6.4 volts
	movwf	dutylimit
	movlw	0x7C
	movwf	limitcount
	return

cyclecheck
	incf	adccount,f
	btfss	STATUS,Z
	goto	cycledecay
cycle0	movlw	0x01	;cycle zero - create bulk window
	movwf	limitcount
	movlw	0xFB
	movwf	dutylimit
cycledecay	movlw	0x1F
	andwf	adccount,w
	btfsc	STATUS,Z
	decf	pvoltage,f	;pvoltage decrements 8x per cycle
	return

pwmadjust
	movf	target,w
	addwf	voltage,w	;compare battery voltage with target
	btfsc	STATUS,Z
	goto	pwmlimit	;battery voltage is equal to target!
	btfss	STATUS,C
	goto	pwminc	;battery voltage is below target
	goto	pwmdec	;battery voltage is above target
pwminc	xorlw	0xFF	;invert
	addlw	0x01	;add 1
	movwf	itemp
	rlf	itemp,f	;puts MSB into C
	movwf	itemp
	rrf	itemp,w	;divide by 2
	addwf	duty,w	;try increasing duty cycle value
	btfss	STATUS,C
	goto	pwmchange
	movlw	0xFF	;duty cycle hits upper limit
	goto	pwmchange
pwmdec	xorlw	0xFF	;invert
	addlw	0x01	;add 1
	movwf	itemp
	rlf	itemp,f	;puts MSB into C
	movwf	itemp
	rrf	itemp,w	;divide by 2
	addwf	duty,w	;try decreasing duty cycle value
	btfsc	STATUS,C
	goto	pwmchange
	movlw	0x00	;duty cycle hits lower limit
	goto	pwmchange
pwmchange	movwf	duty	;store altered duty cycle value
pwmlimit	movf	duty,w
	movf	limitcount,f
	btfsc	STATUS,Z
	goto	pwmhardware	;limitcount=0, no limits apply
	decf	limitcount,f
	subwf	dutylimit,w	;check if duty > dutylimit
	movf	duty,w	;restore duty into w (C unaffected)
	btfsc	STATUS,C	;for accurate adc measurement
	goto	pwmhardware	;duty <= dutylimit: use duty
	movf	dutylimit,w	;duty > dutylimit: use dutylimit
pwmhardware	xorlw	0xFF	;invert duty value for active low PWM
	movwf	CCPR1L	;and update the PWM hardware
	movwf	itemp
	rrf	itemp,f	;now manage duty cycle LSBs
	rrf	itemp,w	;rotate CCPR1L <7:6> into W <5:4>
	andlw	0x30	;clear W except bits <5:4>
	iorwf	CCP1CON,f	;transfer set bits to CCP1CON
	iorlw	0xCF	;set all W except bits <5:4>
	andwf	CCP1CON,f	;transfer clear bits to CCP1CON
pwmhysteresis	incf	duty,w	;try incrementing duty
	btfsc	STATUS,Z	;is duty at FF hex?
	bcf	mode,3	;bulk charge mode - pwm at 100%
	movf	duty,w
	sublw	0xF8
	btfsc	STATUS,C
	bsf	mode,3	;pwm charge mode (saturation/float)
	return

leddisplay
	bcf	INTCON,2	;timer0 just overflowed - clear the flag
	btfss	mode,0
	goto	ledoff	;LED inactive if mode bit 0 is zero
	btfsc	mode,3
	goto	ledpwm	;if duty cycle is not 255, display pwm
ledflash	call	chargepump
	incf	pattern,f	;increment the pattern register
	btfsc	STATUS,Z
	call	decimalise	;pattern is zero, get volt data
	movlw	0x0A
	subwf	decivolts,w
	btfss	STATUS,C
	goto	ledlow	;if voltage is below 11v, display error
	btfsc	pattern,7
	goto	ledoff	;bit 7 is set - LED off
	btfsc	pattern,6
	goto	ledtenths	;2nd quadrant - display tenths
ledunits	movlw	0x07 
	andwf	pattern,w	;zero bits 7,6,5,4,3 of pattern
	btfss	STATUS,Z
	goto	ledoff	;turn LED off because bits 2,1,0 aren't zero
	movf	pattern,w
	movwf	temp
	rrf	temp,f	;
	rrf	temp,f	;rotate bits 5,4,3 of pattern to bits 2,1,0
	rrf	temp,f	;
	movlw	0x07
	andwf	temp,f	;zero bits 7,6,5,4,3 of pattern
	btfsc	STATUS,Z
	goto	ledoff	;turn LED off when pattern is zero
	movf	temp,w
	subwf	voltunits,w	;compare bits 2,1,0 of pattern with voltunits
	btfss	STATUS,C
	goto	ledoff	;turn LED off because pattern exceeds voltunits
	goto	ledon
ledtenths	movlw	0x03 
	andwf	pattern,w	;zero bits 7,6,5,4,3,2 of pattern
	btfss	STATUS,Z
	goto	ledoff	;turn LED off because bits 1,0 aren't zero
	movf	pattern,w
	movwf	temp
	rrf	temp,f	;
	rrf	temp,f	;rotate bits 5,4,3,2 of pattern to bits 3,2,1,0
	movlw	0x0F
	andwf	temp,f	;zero bits 7,6,5,4 of pattern
	btfsc	STATUS,Z
	goto	ledoff	;turn LED off when pattern is zero
	movf	temp,w
	subwf	volttenths,w	;compare bits 3,2,1,0 of pattern with volttenths
	btfss	STATUS,C
	goto	ledoff	;turn LED off because pattern exceeds volttenths
	goto	ledon
ledlow	movlw	0x0F
	andwf	pattern,w	;mask upper 4 bits of pattern
	btfss	STATUS,Z	;light LED on zero (1 in 16)
	goto	ledoff
	goto	ledon
ledpwm	movlw	0xE8	;relaxed rate for float
	btfsc	mode,4	;0=float, 1=saturation
	movlw	0xF0	;frantic rate for saturation
	movwf	TMR0	;pre-loading timer0 speeds it up
	incf	pattern,f
	movf	pattern,w
	andlw	0x0f	;mask off high nibble of pattern
	btfsc	STATUS,Z
	goto	ledoff	;LED on when low nibble of pattern is zero
	movwf	temp	;temp contains low nibble of pattern
	swapf	duty,w	;swap nibbles of duty - divide by 16
	andlw	0x0f	;mask off high nibble - divide by 16
	subwf	temp,w
	btfss	STATUS,C	;leave LED off when pattern >= duty
	return
	goto	ledon
ledon	bsf	GPIO,1	;turn LED on
	return
ledoff	bcf	GPIO,1	;turn LED off
	return

decimalise
	clrf	voltunits
	movf	avoltage,w
	movwf	temp	;and into temp for decimal conversion
	movlw	(d'100' - ZenerVoltage) * 4
	subwf	temp,f
	btfss	STATUS,C
	clrf	temp	;don't allow negative values (<10v)
	bcf	STATUS,C	;
	rrf	temp,f	;divide by 4
	bcf	STATUS,C	;
	rrf	temp,f	;
	movf	temp,w
deciconv	addlw	0xF6	;subtract decimal ten
	btfss	STATUS,C
	goto	decidone	;all whole volts counted, tenths remain
	incf	voltunits,f	;add a whole volt
	goto	deciconv
decidone	addlw	0x0A	;add decimal ten back
	movwf	volttenths	;remainder becomes tenths
	return

floater
	bcf	PIR1,0	;Timer1 just overflowed - clear the flag
	movf	floatseconds,w
	btfsc	STATUS,Z	;is the downcounter at zero?
	goto	floatzero
	decf	floatseconds,f;decrement the downcounter
	return
floatzero	movlw	HIGH timetable;means 'high byte of timetable start addr'
	movwf	PCLATH
	movf	decivolts,w	;voltage in decivolts from a 10 volt base
	andlw	0x3F	;ensure table index in range 0-63
	call	timetable
	iorlw	0	;check table data for zero
	btfsc	STATUS,Z
	return		;return if table data is zero (deadband)
	movwf	temp
	bcf	mode,2
	btfsc	temp,7	;test top bit of table data
	bsf	mode,2
	andlw	0x7F	;zero the top bit of table data
	movwf	floatseconds
	btfss	mode,2	;test voltage range
	goto	floatbulk
floatpwm	decfsz	floatcount,w	;try decrementing floatcount
	goto	floatdec
	bcf	mode,4	;switch to float charge mode
	movlw	-1 * (FloatVoltage - ZenerVoltage) * 4
	movwf	target	;target voltage - 13.5v
	return
floatdec	decf	floatcount,f
	return
floatbulk	incfsz	floatcount,w	;try incrementing floatcount
	goto	floatinc
	bsf	mode,4	;switch to saturation charge mode
	movlw	-1 * (SatVoltage - ZenerVoltage) * 4
	movwf	target	;target voltage - 14.4v
	return
floatinc	incf	floatcount,f
	return

chargepump	
	clrwdt		;watch the doggy
	movlw	0x30	;pump some charge, baby!
	xorwf	GPIO,f	;toggle GP5 and GP4 (charge pump)
	return

fibonacci
	banksel	0x80	;
	rlf	PR2,w	;fibonacci linear feedback shift register
	xorwf	PR2,w	;
	andlw	0x40	;
	addlw	0xFF	;
	rlf	PR2,f	;
	bsf	PR2,7	;
	banksel	0x00	;
	return

 org 0x700

timetable
	addwf	PCL,f	;make sure PCLATH is set correctly!
	retlw	0x01	;10.0v
	retlw	0x01	;10.1v
	retlw	0x01	;10.2v
	retlw	0x01	;10.3v
	retlw	0x01	;10.4v
	retlw	0x01	;10.5v
	retlw	0x01	;10.6v
	retlw	0x01	;10.7v
	retlw	0x01	;10.8v
	retlw	0x01	;10.9v
	retlw	0x01	;11.0v
	retlw	0x01	;11.1v
	retlw	0x01	;11.2v
	retlw	0x01	;11.3v
	retlw	0x01	;11.4v - 4.2 mins
	retlw	0x01	;11.5v - 4.2 mins
	retlw	0x01	;11.6v - 4.2 mins
	retlw	0x01	;11.7v - 4.2 mins
	retlw	0x02	;11.8v - 8.5 mins
	retlw	0x02	;11.9v - 8.5 mins
	retlw	0x03	;12.0v - 12.8 mins
	retlw	0x04	;12.1v - 17.0 mins
	retlw	0x06	;12.2v - 25.6 mins
	retlw	0x09	;12.3v - 38.4 mins
	retlw	0x0C	;12.4v - 51.2 mins
	retlw	0x11	;12.5v - 77 mins
	retlw	0x16	;12.6v - 94 mins
	retlw	0x1C	;12.7v - 2 hrs
	retlw	0x2A	;12.8v - 3 hrs
	retlw	0x46	;12.9v - 5 hrs
	retlw	0	;13.0v
	retlw	0	;13.1v
	retlw	0	;13.2v
	retlw	0	;13.3v
	retlw	0	;13.4v
	retlw	0	;13.5v
	retlw	0xC6	;13.6v - 5 hrs
	retlw	0xAA	;13.7v - 3 hrs
	retlw	0x9C	;13.8v - 2 hrs
	retlw	0x96	;13.9v - 94 mins
	retlw	0x91	;14.0v - 77 mins
	retlw	0x8C	;14.1v - 51.2 mins
	retlw	0x89	;14.2v - 38.4 mins
	retlw	0x86	;14.3v - 25.6 mins
	retlw	0x84	;14.4v - 17.0 mins
	retlw	0x83	;14.5v - 12.8 mins
	retlw	0x82	;14.6v - 8.5 mins
	retlw	0x82	;14.7v - 8.5 mins
	retlw	0x81	;14.8v - 4.2 mins
	retlw	0x81	;14.9v - 4.2 mins
	retlw	0x81	;15.0v - 4.2 mins
	retlw	0x81	;15.1v - 4.2 mins
	retlw	0x81	;15.2v
	retlw	0x81	;15.3v
	retlw	0x81	;15.4v
	retlw	0x81	;15.5v
	retlw	0x81	;15.6v
	retlw	0x81	;15.7v
	retlw	0x81	;15.8v
	retlw	0x81	;15.9v
	retlw	0x81	;16.0v
	retlw	0x81	;16.1v
	retlw	0x81	;16.2v
	retlw	0x81	;16.3v
	retlw	0x81	;16.4v

 end
