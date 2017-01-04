; Программная реализация протокола I2C
; Стырил с Atmel AppNote 300

;**** A P P L I C A T I O N   N O T E   A V R 3 0 0 ************************
;*
;* Title		: I2C (Single) Master Implementation
;* Version		: 1.0 (BETA)
;* Last updated		: 97.08.27
;* Target		: AT90Sxxxx (any AVR device)
;*
;* Support email	: avr@atmel.com
;*
;* DESCRIPTION
;* 	Basic routines for communicating with I2C slave devices. This
;*	"single" master implementation is limited to one bus master on the
;*	I2C bus. Most applications do not need the multimaster ability
;*	the I2C bus provides. A single master implementation uses, by far,
;*	less resources and is less XTAL frequency dependent.
;*
;*	Some features :
;*	* All interrupts are free, and can be used for other activities.
;*	* Supports normal and fast mode.
;*	* Supports both 7-bit and 10-bit addressing.
;*	* Supports the entire AVR microcontroller family.
;*
;*	Main I2C functions :
;*	'i2c_start' -		Issues a start condition and sends address
;*				and transfer direction.
;*	'i2c_rep_start' -	Issues a repeated start condition and sends
;*				address and transfer direction.
;*	'i2c_do_transfer' -	Sends or receives data depending on
;*				direction given in address/dir byte.
;*	'i2c_stop' -		Terminates the data transfer by issue a
;*				stop condition.
;*
;* USAGE
;*	Transfer formats is described in the AVR300 documentation.
;*	(An example is shown in the 'main' code).	
;*
;* NOTES
;*	The I2C routines can be called either from non-interrupt or
;*	interrupt routines, not both.
;*
;* STATISTICS
;*	Code Size	: 81 words (maximum)
;*	Register Usage	: 4 High, 0 Low
;*	Interrupt Usage	: None
;*	Other Usage	: Uses two I/O pins on port D
;*	XTAL Range	: N/A
;*
;***************************************************************************

;**** Global I2C Constants ****

.equ	SCLP	= 3			; SCL Pin number (port B)
.equ	SDAP	= 4			; SDA Pin number (port B)

.equ	b_dir	= 0			; transfer direction bit in i2cadr

.equ	i2crd	= 1
.equ	i2cwr	= 0

;**** Global Register Variables ****

.def	i2cdelay= r21			; Delay loop variable
.def	i2cdata	= r22			; I2C data transfer register
.def	i2cadr	= r23			; I2C address and direction register
.def	i2cstat	= r24			; I2C bus status register

;***************************************************************************
;*
;* FUNCTION
;*	i2c_hp_delay
;*	i2c_qp_delay
;*
;* DESCRIPTION
;*	hp - half i2c clock period delay (normal: 5.0us / fast: 1.3us)
;*	qp - quarter i2c clock period delay (normal: 2.5us / fast: 0.6us)
;*
;*	SEE DOCUMENTATION !!!
;*
;* USAGE
;*	no parameters
;*
;* RETURN
;*	none
;*
;***************************************************************************

i2c_hp_delay:
	ldi		i2cdelay,16
i2c_hp_delay_loop:
	dec		i2cdelay
	brne	i2c_hp_delay_loop
	ret

i2c_qp_delay:
	ldi		i2cdelay,11	
i2c_qp_delay_loop:
	dec		i2cdelay
	brne	i2c_qp_delay_loop
	ret


;***************************************************************************
;*
;* FUNCTION
;*	i2c_rep_start
;*
;* DESCRIPTION
;*	Assert repeated start condition and sends slave address.
;*
;* USAGE
;*	i2cadr - Contains the slave address and transfer direction.
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to the address.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_start.
;*
;***************************************************************************

i2c_rep_start:
	sbi		DDRB,SCLP		; force SCL low
	cbi		DDRB,SDAP		; release SDA
	rcall	i2c_hp_delay	; half period delay
	cbi		DDRB,SCLP		; release SCL
	rcall	i2c_qp_delay	; quarter period delay


;***************************************************************************
;*
;* FUNCTION
;*	i2c_start
;*
;* DESCRIPTION
;*	Generates start condition and sends slave address.
;*
;* USAGE
;*	i2cadr - Contains the slave address and transfer direction.
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to the address.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_write.
;*
;***************************************************************************

i2c_start:				
	mov		i2cdata, i2cadr		; copy address to transmitt register
	sbi		DDRB, SDAP			; force SDA low
	rcall	i2c_qp_delay		; quarter period delay


;***************************************************************************
;*
;* FUNCTION
;*	i2c_write
;*
;* DESCRIPTION
;*	Writes data (one byte) to the I2C bus. Also used for sending
;*	the address.
;*
;* USAGE
;*	i2cdata - Contains data to be transmitted.
;*
;* RETURN
;*	Carry flag - Set if the slave respond transfer.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_get_ack.
;*
;***************************************************************************

i2c_write:
	sec				; set carry flag
	rol	i2cdata			; shift in carry and out bit one
	rjmp	i2c_write_first
i2c_write_bit:
	lsl	i2cdata			; if transmit register empty
i2c_write_first:
	breq	i2c_get_ack		;	goto get acknowledge
	sbi	DDRB,SCLP		; force SCL low

	brcc	i2c_write_low		; if bit high
	nop				;	(equalize number of cycles)
	cbi	DDRB,SDAP		;	release SDA
	rjmp	i2c_write_high
i2c_write_low:				; else
	sbi	DDRB,SDAP		;	force SDA low
	rjmp	i2c_write_high		;	(equalize number of cycles)
i2c_write_high:
	rcall	i2c_hp_delay		; half period delay
	cbi	DDRB,SCLP		; release SCL
	rcall	i2c_hp_delay		; half period delay

	rjmp	i2c_write_bit


;***************************************************************************
;*
;* FUNCTION
;*	i2c_get_ack
;*
;* DESCRIPTION
;*	Get slave acknowledge response.
;*
;* USAGE
;*	(used only by i2c_write in this version)
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to a request.
;*
;***************************************************************************

i2c_get_ack:
	sbi	DDRB,SCLP		; force SCL low
	cbi	DDRB,SDAP		; release SDA
	rcall	i2c_hp_delay		; half period delay
	cbi	DDRB,SCLP		; release SCL

i2c_get_ack_wait:
	sbis	PINB,SCLP		; wait SCL high 
					;(In case wait states are inserted)
	rjmp	i2c_get_ack_wait

	clc				; clear carry flag
	sbic	PINB,SDAP		; if SDA is high
	sec				;	set carry flag
	rcall	i2c_hp_delay		; half period delay
	ret


;***************************************************************************
;*
;* FUNCTION
;*	i2c_do_transfer
;*
;* DESCRIPTION
;*	Executes a transfer on bus. This is only a combination of i2c_read
;*	and i2c_write for convenience.
;*
;* USAGE
;*	i2cadr - Must have the same direction as when i2c_start was called.
;*	see i2c_read and i2c_write for more information.
;*
;* RETURN
;*	(depends on type of transfer, read or write)
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_read.
;*
;***************************************************************************

i2c_do_transfer:
	sbrs	i2cadr,b_dir		; if dir = write
	rjmp	i2c_write		;	goto write data


;***************************************************************************
;*
;* FUNCTION
;*	i2c_read
;*
;* DESCRIPTION
;*	Reads data (one byte) from the I2C bus.
;*
;* USAGE
;*	Carry flag - 	If set no acknowledge is given to the slave
;*			indicating last read operation before a STOP.
;*			If cleared acknowledge is given to the slave
;*			indicating more data.
;*
;* RETURN
;*	i2cdata - Contains received data.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_put_ack.
;*
;***************************************************************************

i2c_read:
	rol	i2cstat			; store acknowledge
					; (used by i2c_put_ack)
	ldi	i2cdata,0x01		; data = 0x01
i2c_read_bit:				; do
	sbi	DDRB,SCLP		; 	force SCL low

;******** Добавлено******************************
;    nop; даем время опуститься SCL (300ns)
;   nop
;	nop
;    cbi    DDRB,SDAP; отпускаем SDA
;******** ***************************************

	rcall	i2c_hp_delay		;	half period delay

	cbi	DDRB,SCLP		;	release SCL
	rcall	i2c_hp_delay		;	half period delay

	clc				;	clear carry flag
	sbic	PINB,SDAP		;	if SDA is high
	sec				;		set carry flag

	rol	i2cdata			; 	store data bit
	brcc	i2c_read_bit		; while receive register not full


;***************************************************************************
;*
;* FUNCTION
;*	i2c_put_ack
;*
;* DESCRIPTION
;*	Put acknowledge.
;*
;* USAGE
;*	(used only by i2c_read in this version)
;*
;* RETURN
;*	none
;*
;***************************************************************************

;i2c_put_ack:
;	sbi	DDRB,SCLP		; force SCL low
;
;	ror	i2cstat			; get status bit
;	brcc	i2c_put_ack_low		; if bit low goto assert low
;	cbi	DDRB,SDAP		;	release SDA
;	rjmp	i2c_put_ack_high
;i2c_put_ack_low:			; else
;	sbi	DDRB,SDAP		;	force SDA low
;i2c_put_ack_high:
;
;	rcall	i2c_hp_delay		; half period delay
;	cbi	DDRB,SCLP		; release SCL
;i2c_put_ack_wait:
;	sbis	PINB,SCLP		; wait SCL high
;	rjmp	i2c_put_ack_wait
;	rcall	i2c_hp_delay		; half period delay
;	ret

;There's a slight design error in the i2c code implementation for "put ack..." which prevents
;"multiple byte reads" (SDA not released).
;
;Just use the following (corrected) snippet:
;
i2c_put_ack:
sbi DDRB,SCLP ; force SCL low
ror i2cstat ; get status bit
brcc i2c_put_ack_low ; if bit low goto assert low
cbi DDRB,SDAP ; release SDA
rjmp i2c_put_ack_high
i2c_put_ack_low: ; else
sbi DDRB,SDAP ; force SDA low (master acknowledge)
i2c_put_ack_high:
rcall i2c_hp_delay ; half period delay
cbi DDRB,SCLP ; release SCL
i2c_put_ack_wait:
sbis PINB,SCLP ; wait SCL high
rjmp i2c_put_ack_wait
rcall i2c_hp_delay ; half period delay
brcs i2c_put_ack_leave ; if "master acknowledge" (SDA low),
sbi DDRB,SCLP ; force SCL low,
nop ; give pin time to react and
cbi DDRB,SDAP ; release SDA
i2c_put_ack_leave:
ret


;***************************************************************************
;*
;* FUNCTION
;*	i2c_stop
;*
;* DESCRIPTION
;*	Assert stop condition.
;*
;* USAGE
;*	No parameters.
;*
;* RETURN
;*	None.
;*
;***************************************************************************

i2c_stop:
	sbi	DDRB,SCLP		; force SCL low
	sbi	DDRB,SDAP		; force SDA low
	rcall	i2c_hp_delay		; half period delay
	cbi	DDRB,SCLP		; release SCL
	rcall	i2c_qp_delay		; quarter period delay
	cbi	DDRB,SDAP		; release SDA
	rcall	i2c_hp_delay		; half period delay
	ret


;***************************************************************************
;*
;* FUNCTION
;*	i2c_init
;*
;* DESCRIPTION
;*	Initialization of the I2C bus interface.
;*
;* USAGE
;*	Call this function once to initialize the I2C bus. No parameters
;*	are required.
;*
;* RETURN
;*	None
;*
;* NOTE
;*	PORTD and DDRD pins not used by the I2C bus interface will be
;*	set to Hi-Z (!).
;*
;* COMMENT
;*	This function can be combined with other PORTD initializations.
;*
;***************************************************************************

i2c_init:
	cbi		PORTB, PORTB3
	cbi		PORTB, PORTB4
	cbi		DDRB, DDB3
	cbi		DDRB, DDB4
;	clr	i2cstat			; clear I2C status register (used as a temporary register)
;	out	PORTB,i2cstat		; set I2C pins to open colector
;	out	DDRB,i2cstat
	ret
