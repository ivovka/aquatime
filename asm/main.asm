;
; AquaTime.asm
;
; Created: 16.11.2015 23:12:52
; Author : vivakin-local
; �������� ����
; ����������� ��������, ��������� � �.�.
.include "symbols.inc"
; ����������� ��������
#define _AT_MACRO_SECTION_
#include "timers.inc"
#undef	_AT_MACRO_SECTION_
; ����� ����������� ��������

; RAM Start ===================================================================
.DSEG
; ������� ��������
; ������� - ������ ������������ ����������. ������� ���� - ������!
timers:	.BYTE	COUNT_TIMERS*2
; ������ ��� ��������/��������� ��������� RTC
; ������� ��������� �����: ��������/����������� ������� � 0x00 ������ ��� RTC
rtc_buf_current: .BYTE RTC_BUF_LENGTH
; ����� ���������� ��������: ��������/����������� � ������ 0x08 ��� RTC
rtc_buf_off: .BYTE RTC_BUF_LENGTH
; ����������� ������� RTC - ����� ��������� ��� ������� ������. ������������� �� ������ 0x07 ��� RTC
rtc_buf_control: .BYTE 1

; RAM End =====================================================================

; Flash Start =================================================================
.CSEG
.org	$0000									; (Reset)
	rjmp Reset
.org	$0001									; (INT0)
	reti
.org	$0002									; (PCINT0)
	reti
.org	$0003									; (TIM0_OVF)
	reti
.org	$0004									; (EE_RDY)
	reti
.org	$0005									; (ANA_COMP)
	reti
.org	$0006									; (TIM0_COMPA)
	rjmp	int_comp_a
.org	$0007									; (TIM0_COMPB)
	reti
.org	$0008									; (WDT)
	reti
.org	$0009									; (ADC)
	reti
.org INT_VECTORS_SIZE							; ����� ������� ����������

; ����������� ���������� ������ ===============================================
Reset:
	ldi		REG_TMP, Low(RAMEND)				; ������������� �����
	out		SPL, REG_TMP
; ��� tiny - ��� ��� SPH
;	ldi		R16, High(RAMEND)
;	out		SPH, R16
.include "coreinit.inc"
; ������������� ���������� ��������� ==========================================
; ��������� ���������� �� �������:
												; 1. ������� Clock Select Bit
	ldi		REG_TMP, CLKDIV64
	out		TCCR0B, REG_TMP
												; 2. ��������� ����� ������ �����������
	ldi		REG_TMP, (1<<WGM01)+(0<<WGM00)		; ����� CTC (Clear Timer on Compare Match)
	out		TCCR0A, REG_TMP
												; 2. �������� � OCR0A ��������, � ������� ����� ������������ �������� �������
	ldi		REG_TMP, TMTICKS
	out		OCR0A, REG_TMP
												; 3. ��������� � TIMSK0 ���������� �� Output Compare Match A
	ldi		REG_TMP, (1<<OCIE0A)
	out		TIMSK0, REG_TMP

	ldi		REG_TMP, (1<<PORTB1)				; PORTB:
												; pin 1 - �������� � ������� (������)
												; pin 3, pin 4 - � ���� - ������������ � I2C
	out		PORTB, REG_TMP

	ldi		REG_TMP, (1<<DDB2)|(1<<DDB0)		; DDRB:
												; PIN0 �� ����� - ���������
												; PIN2 �� ����� - ��������
	out		DDRB, REG_TMP						; pin 3, pin 4 - � ���� - ������������ � I2C

; ����� ������������� ���������� ��������� ====================================

; ������������� ������� ��������� =============================================

; ����� ������������� ������� ��������� =======================================
	sei		; ���������� ����������
	ldi		REG_FLAGS, (1<<IND_ST_CH_BIT)|(1<<RTC_OFF_UNDEF)
	rcall	proc_rtc_tm_st_tm
;	sbr		REG_FLAGS, (1<<IND_ST_CH_BIT)		; ���������� ��� ����, ����� ��������� ��������� ����������
;	rcall	rtc_switch_mode						; ���� ������� ������� ��������� ����� � ���������
	rjmp Main

; ���������� �������
; � ���������� ����� ������� ������, � ������ - ���������� ����� ����� ����������� �������
; ��������� ����������� �������� ���������� ������ � �������� ����� ���������
int_comp_a:
	push	REG_TMP								; �������� ��������� �������, � ������� ���� ��������
	in		REG_TMP, SREG						; �������� ������� ������
	inc		REG_HW_TM_TICKS
	out		SREG, REG_TMP						; �������������� ��������� �������� ������
	pop		REG_TMP								; � ���������� ��������
	reti
; ����������� ���������� �� ���� ==============================================

; Run =========================================================================
; End Run =====================================================================

; ������� ���� ================================================================
Main:
	rcall	sw_tm_ticks							; ���������� ����������� ��������
	rcall	proc_indc							; ���������� ���������
	rcall	proc_inp							; ��������� ��������� ��������� ����� (������)
	rcall	proc_rtc							; ��������� ��������� ��������� �����
	rcall	proc_load							; ��������� ��������� ��������� ��������
	rcall	proc_flags							; ��������� ��������� ������
	rcall	indc_upd							; ��������� ��������� ���������
    rjmp	Main
; ����� �������� ����� ========================================================

; ������ ���������:
; ������� ��� ����������, ������ (1) ��� �������� (0) ���������
; ��������� ���� - ���������� ����� �������, � ������� ������� ��������� ������ ���������� � ������ ���������
; ������� ���� - ������� ����� ������
ST0_LED_STATES: .dw 0x80fa,0x00fa,0x80fa,0x00fa,0x80fa,0x05dc,0x0000		; ��������� ���������� 3 ����, ����� ���� �����
ST1_LED_STATES:	.dw	0x03e8,0x0000				; ��������� �� �����
ST2_LED_STATES:	.dw	0x8032,0x00c8,0x0000		; ��������� ���������� 4 ���� � �������
ST3_LED_STATES:	.dw	0x83e8,0x0000				; ��������� ����� ���������
ST4_LED_STATES:	.dw	0x81f4,0x01f4,0x0000		; ��������� ���������� 1 ��� � �������
ST5_LED_STATES:	.dw	0x81f4,0x0dac,0x0000		; ��������� ���������� 1 ��� � 4 �������
; ������ ���������� �� ������ ���������
LED_MODE_PTRS: .db low(ST0_LED_STATES*2), low(ST1_LED_STATES*2), low(ST2_LED_STATES*2), low(ST3_LED_STATES*2), low(ST4_LED_STATES*2), low(ST5_LED_STATES*2)

RTC_235959:	.db 0x59, 0x59, 0x33, 0x00
; ������������ ================================================================

; ��������� ���������� �������� ����������� ��������
; �� ������� ��������� ������������ ������� ����������
; ������ ���������� ����� ����������� �������
; ���� �������� ������������ ������� ������� ����� 0,
; �� ���������� �������� ���������������� ������������ ������� � 0
; �� ���������� - �������� ���������� ����� ����������� �������
; ��������� ������� ��������� ����������� �������� �� ����
; ���� 0 - �������� �������� ������ �� �������� - �����
; �������� ��������� ������ ������� ��������� �������� �������� - ������� ��� ������ � ���� ��������
; ���� ���� �������� ����������, ��������� ��������������� ������
; ��� ����������� �� ����� �������� ����������� ��������� �� ��������� ������
sw_tm_ticks:
	cli											; �� ����� ������ � REG_HW_TM_TICKS �������� ����������
	mov		REG_TMP2, REG_HW_TM_TICKS			; ��������� �����
	clr		REG_HW_TM_TICKS						; ���������� ����� ����������� ������� ������ - ����� ��������
	sei											; � �������� ����������. ����� ���� ��������� ����, ���
												; ���� �� ���������� cli ���������� ���� ��������� ���-�� ���
												; ��� ���������� ���� �����������.
												; ���� �������� �� ��� ������, �� ����� ��������� ������� ������,
												; ��������� ����������, ������ ������ � ��������������� �������
												; ������.
	mov		REG_TMP3, REG_SW_TM_STATE			; ��������� �����
	ldi		ZL, low(timers)						; �������� � ������� Z
	ldi		ZH, high(timers)					; ������ ������� �������� �� ���
sw_tm_ticks_loop:
	tst		REG_TMP3							; ���� ������� ��������� �������� = 0
	breq	sw_tm_ticks_end						; = 0, �������� �������� ������ ��� - � ������

	lsr		REG_TMP3							; ������� ��� �������� ��������� �������� ������ � ���� ��������
	brcc	sw_tm_ticks_skip					; ������� ������ ���������� - ������ � ��� �� �����
	ld		REG_TMP, Z							; �������� � REG_TMP ������� ���� ���������� �������
	ldd		REG_TMP1, Z+1						; �������� ������� ���� ���������� �������
	sub		REG_TMP, REG_TMP2					; �������� ������� ���� �� ���������� ����� ����������� �������
	sbci	REG_TMP1, 0							; �������� ���� �������� � ������� ����� ���������� �������
	brpl	sw_tm_ticks_cont					; ���� ��������� ������ ���� - ������
	clr		REG_TMP								; ����� - �������� - ������������� ������ � 0
	clr		REG_TMP1
sw_tm_ticks_cont:
	st		Z, REG_TMP							; �������� � ��� ������� ���� ���������� �������
	std		Z+1, REG_TMP1						; �������� � ��� ������� ���� ���������� �������
sw_tm_ticks_skip:
	subi	ZL, LOW(-2)							; �������� � ���������� �������
	sbci	ZH, HIGH(-2)
	rjmp	sw_tm_ticks_loop
sw_tm_ticks_end:
	ret											; sw_tm_ticks

; ������������ ���������� ����������
; ��� ��������: � REG_IND_STATE - ���������� ��������������� ����� ������
; � �������� ������ IND_ST_CH_BIT - ���� ������������� ����� ������
; ��������:
; ���� ������ ��������� �� ��������� ���������
; - ���������� ���������� ��������� ��������� ��������� �� ��������������� ST<X>_LED_STATES (1)
; - � ����������� �� ��������� ������� ����, ������/�������� ��������� (2)
; - ���������� �������� ������� ��������� � ��������� ������ ��������� (3)
; ��� ��������� �� ��������� ���������
; - ��������� ��������� ������� ���������:
; - ���� ������ ��������� �����
; -- �������� ���������� ��������� ��������� ��������� �� ��������� �������
; -- ���� � ��������� ������� ������� ����� ������ ��������� ���������,
; --- ��������� ���� 1-2-3
; -- �����
; --- ��������� ���� 2-3
proc_indc:
	sbrs	REG_FLAGS, IND_ST_CH_BIT			; ���� ��������� ��������� ����������?
	rjmp	proc_indc_st_proc					; ��� - ��������� ������������� ���������
												; ��������� ���������� - ����� ������������� �� ��������������� ���������

	cbr		REG_FLAGS, (1<<IND_ST_CH_BIT)		; ������ ��� ��������� ���������, �.�. ��������� ���
	ldi		ZL, LOW(LED_MODE_PTRS*2)			; �������� ��������� �� ������ ������� ���������� �� ������
	add		ZL, REG_IND_STATE					; ��������� � ����� ��������� ����� ������
	lpm		REG_IND_ST_PTR, Z					; � REG_IND_ST_PTR - ��������� �� ��������������� �����

proc_indc_set:
	; ������ ������ �������
	; ��������� �� ������ ������� - � REG_IND_ST_PTR
												; ������������ ��������� ����� ��� ������ � ��������������� ������
	mov		REG_IND_STM_PTR, REG_IND_ST_PTR
	clear_timer	IND_TM_NUM						; ��������� ������ ��������� (��������: clear_timer ������ REG_TMP)
	clr		ZH
	mov		ZL, REG_IND_STM_PTR					; � �������� Z - ��������� �� ����� ����������
	lpm		REG_TMP, Z+							; � REG_TMP - ����� ���������� - ������� ����
	lpm		REG_TMP1, Z							; � REG_TMP - ����� ���������� - ������� ����
	rjmp	proc_indc_st_proc1					; ��������� ������� � ���������� ���������� ������ ���������

proc_indc_st_proc:
	ldi		REG_TMP_TM_NUM, IND_TM_NUM
	rcall	tm_check
	brtc	proc_indc_end						; ���� ����� �� ����� - �����
												; ������ ���������� - ����� ��������� �����
	clr		ZH
	mov		ZL, REG_IND_STM_PTR
	adiw	ZH:ZL, 2							; ������� � ���������� ������� - ������ ������������
	mov		REG_IND_STM_PTR, ZL					; �������� �����
	lpm		REG_TMP, Z+							; � REG_TMP - ����� ���������� - ������� ����
	lpm		REG_TMP1, Z							; � REG_TMP1 - ����� ���������� - ������� ����
	tst		REG_TMP								; ��������� ��������� �� ��� ����� (������� ���������� ������ - 0x0000)
	brne	proc_indc_st_proc1					; �� ��������� ����� - ������
	tst		REG_TMP1
	brne	proc_indc_st_proc1					; �� ��������� ����� - ������

	rjmp	proc_indc_set						; � ��� �������

proc_indc_st_proc1:
	bst		REG_TMP1, 7							; � REG_TMP1 - ������� ���� ������ ����������; �������� ��������� (7-�� ���: ���/����) � ���� T
	cbr		REG_TMP1, (1<<7)					; ���, ����� 7-�� ���� - ��� ���-�� ����� �������. ������ 7-�� ���
	rcall	proc_indc_hw						; ������� ��������� ���������� ��������� ���������� (������/�������� ����)

	ldi		REG_TMP_TM_NUM, IND_TM_NUM			; ������ �������
	rcall	tm_start
proc_indc_end:
	ret
; ����� ������������ ���������� ����������

; ������������ ��������� ������
proc_inp:
	rcall	proc_inp_hw							; ����� ������ ���� ������������ ��������� ������ ����� ������������� � ����� �
	brtc	proc_inp_released					; �������, ���� ������ �� ������
proc_inp_pressed:								; ������ ������
	sbrc	REG_LOCAL_FLAGS, L_LONG_PRESS_BIT	; ���� ���� �������� ������� ���������� - ��� ������ �� ������
	rjmp	proc_inp_end						; ���� ���������� - �����
	clr		REG_TMP1							; ������������
	ldi		REG_TMP, SHORT_PRESS_TIME			; �� 20��
	ldi		REG_TMP_TM_NUM, INP_SHORT_TM_NUM	; ������ ��������� �������
	sbrs	REG_SW_TM_STATE, INP_SHORT_TM_NUM	; ���� ������ ����������
	rcall	tm_start							; �������� ������
	ldi		REG_TMP_TM_NUM, INP_SHORT_TM_NUM	; ������ ��������� �������
	rcall	tm_check							; �������� - �� ����� ��
	brtc	proc_inp_chk_long					; ���� �� ����� - ������
	sbr		REG_LOCAL_FLAGS, (1<<L_SHORT_PRESS_BIT)	; ���� ����� - �������� ���� ��������� �������
proc_inp_chk_long:
	ldi		REG_TMP1, LONG_PRESS_TIME_HIGH		; ������������
	ldi		REG_TMP, LONG_PRESS_TIME_LOW		; �� 500��
	ldi		REG_TMP_TM_NUM, INP_LONG_TM_NUM		; ������ �������� �������
	sbrs	REG_SW_TM_STATE, INP_LONG_TM_NUM	; ���� ������ ����������
	rcall	tm_start							; �������� ������
	ldi		REG_TMP_TM_NUM, INP_LONG_TM_NUM		; ������ �������� �������
	rcall	tm_check							; �������� - �� ����� ��
	brtc	proc_inp_end						; ���� �� ����� - �����

	sbr		REG_LOCAL_FLAGS, (1<<L_LONG_PRESS_BIT)	; ������������ ���� �������� �������
	cbr		REG_LOCAL_FLAGS, (1<<L_SHORT_PRESS_BIT)	; ��������� ���� ��������� �������
	sbr		REG_FLAGS, (1<<LONG_PRESS_BIT)		; ������������ ���� �������� ������� � �������� ������
	cbr		REG_FLAGS, (1<<SHORT_PRESS_BIT)		; ��������� ���� ��������� ������� (�� ������ ������) - ����� ���������� - �� �� ������ ���� ����������
	rjmp	proc_inp_end
proc_inp_released:
	sbrs	REG_SW_TM_STATE, INP_SHORT_TM_NUM	; ���� ������ ��������� ������� ����������
	rjmp	proc_inp_end						; �����
	ldi		REG_TMP_TM_NUM, INP_SHORT_TM_NUM	; ������ ��������� �������
	rcall	tm_check							; �������� - �� ����� ��
	brtc	proc_inp_chk_long					; ���� �� ����� - �����
	sbrs	REG_LOCAL_FLAGS, L_SHORT_PRESS_BIT	; ��������, ���������� �� ��������� ���� ��������� �������
	rjmp	proc_inp_clear						; ���� �� ���������� - ������� �� ������� ������ � ��������
												; ���� ����������:
	sbr		REG_FLAGS, (1<<SHORT_PRESS_BIT)		; ������������ ���� ��������� ������� � �������� ������
proc_inp_clear:									; ������� ������ � ��������
	cbr		REG_LOCAL_FLAGS, (1<<L_LONG_PRESS_BIT)+(1<<L_SHORT_PRESS_BIT)
	clear_timer	INP_SHORT_TM_NUM
	clear_timer INP_LONG_TM_NUM
proc_inp_end:
	ret


; ��������� ��������� ��������� �����
proc_rtc:
	sbrs	REG_FLAGS, RTC_SET_BIT
	rjmp	proc_rtc_tm							; �� ������ - ������ �� ��������� �������
	rcall	proc_rtc_set_state					; ������� ��������� ������ �����/��������� ���������
	sbrs	REG_FLAGS, RTC_ON_BIT
	rjmp	proc_rtc_st_night
												; ��������� ������� �� ��������� � ������ ����:
												; � ��������� ��������� ������� �����
	ldi		YL, low(rtc_buf_current)			; �� rtc_buf_current
	ldi		REG_TMP, RTC_ALARM_ADDR				; � ��� �����
	ldi		REG_TMP1, RTC_BUF_LENGTH
	rcall	rtc_write							; ���������
	cbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; ��������� ���� ��������������� ������� ����������
	rjmp	proc_rtc_tm_wr
proc_rtc_st_night:
												; ��������� ������� �� ��������� � ������ ����:
												; � ������� ����� ��������� ����
	clr		YH
	ldi		YL, low(rtc_buf_current)			; � rtc_buf_current
	st		Y, YH								; ������ 0 ������
	std		Y+1, YH								; ������ 0 �����
	std		Y+2, YH								; ������ 0 �����
	ldi		REG_TMP, RTC_CURR_ADDR				; � ������� �����
	ldi		REG_TMP1, RTC_BUF_LENGTH
	rcall	rtc_write							; ���������
	rcall	rtc_reset_off						; � ����� ���������� ��������� 23:59:59

proc_rtc_tm_wr:
	cbr		REG_FLAGS, (1<<RTC_SET_BIT)
	rcall	proc_rtc_set_state					; ������� ��������� ������ �����/��������� ���������
proc_rtc_tm:
	ldi		REG_TMP_TM_NUM, RTC_TM_NUM			; ������ ������ RTC
	sbrs	REG_SW_TM_STATE, RTC_TM_NUM			; ���� ������ ������ RTC ����������
	rjmp	proc_rtc_tm_start					; ������� �� ��������� ������ ����� � ������ �������
												; ������ ������ RTC ��������												
	rcall	tm_check							; �������� - �� ����� ��
	brtc	proc_rtc_end						; ���� �� ����� - �����
proc_rtc_tm_start:								; ������ ���� �����, ���� ����������
	rcall	proc_rtc_set_state					; ������� ��������� ������ �����/��������� ���������
												
												; �� ��� ����� ���� ��� ����� rcall
												; ������ ������ ��� ����, ����� ��� ������ �� �����
												; �������� ���������� ����, � ���� ����� ��� �������������
												; ���������� �����. �� �������� ��� ����� "����� �������"
												; ��� ��� ��, ��� ��� ������
proc_rtc_tm_st_tm:
	ldi		REG_TMP_TM_NUM, RTC_TM_NUM			; ������ ������ RTC
	ldi		REG_TMP1, RTC_TIMER_HIGH			; ������������� ������ �� 10 ��� - ������� ���
	ldi		REG_TMP, RTC_TIMER_LOW				; � �������
	rcall	tm_start							; �������� ������
proc_rtc_end:
	ret

; ��������� ��������� ��������� �����:
; ������������� ���� �� ���������:
; RTC_ST_DAY
; RTC_ST_NIGHT
proc_rtc_set_state:
	ldi		YL, low(rtc_buf_current)			; ������� ����� ���� ������ � rtc_buf_current
	ldi		REG_TMP, RTC_CURR_ADDR				; �������� �����
	ldi		REG_TMP1, RTC_BUF_LENGTH			; RTC_BUF_LENGTH ����
	rcall	rtc_read							; ���������
	ldi		YL, low(rtc_buf_off)				; ����� ���������� ���� ������ � rtc_buf
	ldi		REG_TMP, RTC_ALARM_ADDR				; �������� ���
	ldi		REG_TMP1, RTC_BUF_LENGTH			; RTC_BUF_LENGTH ����
	rcall	rtc_read							; ���������
												; ��������, ����������� �� ����� ����������

	cbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; ��������� ���� ����, ��� ����� ���������� �� �����������
	ldi		YL, low(rtc_buf_off)				; ��� ����� ��������� ������ ����
	ld		REG_TMP, Y							; ���������� ������� ����������
	cpi		REG_TMP, RTC_UNSET_MARK				; � ���������� 0xFF
	brne	proc_rtc_ss_cmp						; ����������� - ������ � ��������� �������
	rcall	rtc_reset_off						; �� ����������� - ��������� 23:59:59
proc_rtc_ss_cmp:								; ��������� �������� ������� � �������� ����������

												; ������� ������ � 23:59:59 �,
												; ���� �����, ��������� ���� RTC_OFF_UNDEF - �� ����� ��� ���������� ���������
	ldi		REG_TMP2, RTC_BUF_LENGTH
	ldi		YL, LOW(rtc_buf_off)
	ldi		ZL, low(rtc_235959*2)
proc_rtc_ss_cmp235959_loop:
	ld		REG_TMP, Y+
	lpm		REG_TMP1, Z+
	cp		REG_TMP, REG_TMP1
	brne	proc_rtc_ss_cmp_start
	dec		REG_TMP2
	brne	proc_rtc_ss_cmp235959_loop
	sbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; ����� - ������������ ���� ����, ��� ����� ���������� �� �����������

proc_rtc_ss_cmp_start:
												; � ��� ����� ��������� ��������� �������...
												; TODO: ������ ���� � ���
	ldi		REG_TMP2, RTC_BUF_LENGTH			; �������
	ldi		YL, LOW(rtc_buf_current+RTC_BUF_LENGTH)
	ldi		ZL, LOW(rtc_buf_off+RTC_BUF_LENGTH)

proc_rtc_ss_cmp_loop:
	ld		REG_TMP, -Y
	ld		REG_TMP1, -Z
	cp		REG_TMP, REG_TMP1
	breq	proc_rtc_ss_cmp_cont
	brpl	proc_rtc_ss_night					; ����� �������� ������� ������, ��� ����� ������� ���������� - ���������� ������ RTC_ST_NIGHT
	brlo	proc_rtc_ss_day						; ����� �������� ������� ������, ��� ����� ������� ���������� - ���������� ������ RTC_ST_DAY
proc_rtc_ss_cmp_cont:
	dec		REG_TMP2
	brne	proc_rtc_ss_cmp_loop
proc_rtc_ss_day:								; ������������ ��������� DAY
	sbr		REG_FLAGS, (1<<RTC_ON_BIT)
	rjmp	proc_rtc_ss_end
proc_rtc_ss_night:								; ������������ ��������� NIGHT
	cbr		REG_FLAGS, (1<<RTC_ON_BIT)
proc_rtc_ss_end:
;	ret											; ��������������� - ����� ������������� ����� ���������� ����� ������ ���� ������ � ������

; ��������� ��������� ���������� ����� (������/�� ������)
rtc_switch_mode:
	set
	sbrc	REG_FLAGS, MANUAL_MODE_BIT
	clt
												; � �������������� ������ ����� �������� �������� �� RTC ��� � �������
												; � ������ ������ - ���������
												; ��������� ������ - �� ����� T: 1 - ��������������
	clr		REG_TMP1
	bld		REG_TMP1, RTC_CONTROL_BIT			; ���� � ������ � 4-�� ��� REG_TMP1
	sts		rtc_buf_control, REG_TMP1			; �������� � ������
	ldi		YL, LOW(rtc_buf_control)			; ��������� �� �����
	ldi		REG_TMP, RTC_CONTROL_ADDR			; ������� ���������� RTC
	ldi		REG_TMP1, RTC_CONTROL_LEHGTH		; ����� ������
	rcall	rtc_write							; ��������
	ret

; ��������� ��������� ���������� � RTC � 23:59:59
rtc_reset_off:
	ldi		REG_TMP1, RTC_BUF_LENGTH
	ldi		ZL, low(rtc_235959*2)				; �� ��������� rtc_235959
	ldi		YL, low(rtc_buf_off)				; � rtc_buf_off
rtc_reset_off_loop:
	lpm		REG_TMP, Z+
	st		Y+, REG_TMP							; �������
	dec		REG_TMP1
	brne	rtc_reset_off_loop					; RTC_BUF_LENGTH ���� 
	ldi		REG_TMP1, RTC_BUF_LENGTH
	ldi		YL, low(rtc_buf_off)				; �� rtc_buf_off
	ldi		REG_TMP, RTC_ALARM_ADDR				; � ��� �����
	rcall	rtc_write							; ���������
	sbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; ������������ ���� ����, ��� ����� ���������� �� �����������
	ret

; ��������� ��������� ��������� ��������
proc_load:
	bst		REG_FLAGS, LOAD_ON_BIT
	rcall	proc_load_hw
	ret

; ������������ �������� ���������� ���������
; � ����������� �� ����� T �������� (�=1) ��� ����� (T=0) ��������
proc_load_hw:
	brts	proc_load_hw_on
	sbi		PORTB, PORTB2						; ���� ���������
	ret
proc_load_hw_on:
	cbi		PORTB, PORTB2						; ������� ���������
	ret

; ������������ �������� ���������� ���������� - ���� ���������
; � ����������� �� ����� T �������� (�=1) ��� ����� (T=0) ���������
proc_indc_hw:
	brts	proc_indc_hw_ledon
	sbi		PORTB, PORTB0						; ���� ���������
	ret
proc_indc_hw_ledon:
	cbi		PORTB, PORTB0						; ������� ���������
	ret

; ������������ �������� ���������� ������ - ������
; ���� ������ ������ - ������������� ���� �; � ��������� ������ - ����������
proc_inp_hw:
	clt
	sbis	PINB, PINB1							; ���� ������ �������� (�� pin = 1), ���������� ����. ������
	set
	ret

; ������������ ��������� ������
proc_flags:
	sbrc	REG_FLAGS, SHORT_PRESS_BIT
	rcall	fl_short_press
	sbrc	REG_FLAGS, LONG_PRESS_BIT
	rcall	fl_long_press
	sbrc	REG_FLAGS, MANUAL_MODE_BIT
	rcall	fl_manual_mode
	sbrc	REG_FLAGS, LOAD_ON_BIT
	rcall	fl_load_on

	sbrc	REG_FLAGS, RTC_ON_BIT
	rcall	fl_rtc_on
	sbrs	REG_FLAGS, RTC_ON_BIT
	rcall	fl_rtc_off

	sbrc	REG_FLAGS, IND_ST_CH_BIT
	rcall	fl_ind_st_change
	sbrc	REG_FLAGS, RTC_SET_BIT
	rcall	fl_rtc_set
	rjmp	proc_flags_end
fl_short_press:
											; �� ��������� �������:
											; ���� ���������� manual mode, �� ������������� LOAD_ON_BIT
											; ���� manual mode �� ���������� - 
											; ���������� ���� RTC_SET_BIT
	cbr		REG_FLAGS, (1<<SHORT_PRESS_BIT)
	sbrs	REG_FLAGS, MANUAL_MODE_BIT
	rjmp	fl_short_press_auto
	ldi		REG_TMP, (1<<LOAD_ON_BIT)
	eor		REG_FLAGS, REG_TMP
	ret
fl_short_press_auto:
	sbr		REG_FLAGS, (1<<RTC_SET_BIT)
	ret
fl_long_press:
											; �� �������� �������:
											; ������������� manual mode
	ldi		REG_TMP, (1<<MANUAL_MODE_BIT)
	eor		REG_FLAGS, REG_TMP
	rcall	rtc_switch_mode
	cbr		REG_FLAGS, (1<<LONG_PRESS_BIT)
	ret
fl_manual_mode:
	ret
fl_load_on:
	ret
fl_rtc_on:
	sbrs	REG_FLAGS, MANUAL_MODE_BIT
	sbr		REG_FLAGS, (1<<LOAD_ON_BIT)
	ret
fl_rtc_off:
	sbrs	REG_FLAGS, MANUAL_MODE_BIT
	cbr		REG_FLAGS, (1<<LOAD_ON_BIT)
	ret
fl_ind_st_change:
	ret
fl_rtc_set:
	ret
proc_flags_end:
	ret

indc_upd:
	mov		REG_TMP, REG_IND_STATE
	sbrc	REG_FLAGS, MANUAL_MODE_BIT
	rjmp	indc_upd_manual
indc_upd_auto:
	sbrs	REG_FLAGS, LOAD_ON_BIT
	rjmp	indc_upd_auto_off					; ������� �� ��������� � ���� ������, ����� �������� ���������
												; ��������� � ���� ������, ����� �������� ��������
	ldi		REG_TMP, ST_IND_A_ON_DEF			; ��������� �����, ��������������� ���� � ������������� �������� ����������
	sbrc	REG_FLAGS, RTC_OFF_UNDEF
	ldi		REG_TMP, ST_IND_A_ON_UNDEF			; ��������� �����, ��������������� ���� � ��������������� �������� ����������
	rjmp	indc_upd_end
indc_upd_auto_off:								; ��������� � ���� ������, ����� �������� ���������
	ldi		REG_TMP, ST_IND_A_OFF_DEF			; ��������� �����, ��������������� ���� � ������������� �������� ����������
	sbrc	REG_FLAGS, RTC_OFF_UNDEF
	ldi		REG_TMP, ST_IND_A_OFF_UNDEF			; ��������� �����, ��������������� ���� � ��������������� �������� ����������
	rjmp	indc_upd_end
indc_upd_manual:								; ��������� � ������ ������
	ldi		REG_TMP, ST_IND_M_OFF				; ��������� �����, ��������������� ��������� � ������ ������
	sbrc	REG_FLAGS, LOAD_ON_BIT
	ldi		REG_TMP, ST_IND_M_ON				; ��������� �����, ��������������� �������� � ������ ������
indc_upd_end:
	cp		REG_TMP, REG_IND_STATE
	breq	indc_upd_end_end
	sbr		REG_FLAGS, (1<<IND_ST_CH_BIT)
indc_upd_end_end:
	mov		REG_IND_STATE, REG_TMP
	ret
	

#define _AT_SUBROUTINE_SECTION_
; ��������� ���������� ��������� I2C
.include "I2C.inc"
.include "timers.inc"
.include "I2C_RTC.inc"
#undef _AT_SUBROUTINE_SECTION_
; ����� ����������� ===========================================================

; EEPROM ======================================================================
.ESEG
; EEPROM ����� ================================================================
