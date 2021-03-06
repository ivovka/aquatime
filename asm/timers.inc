/*
 * timers.inc
 *
 *  Created: 18.12.2015 13:29:18
 *   Author: vladimir.ivakin
 */ 
#ifdef _AT_MACRO_SECTION_
; ����� ������������ �������
; @0 - ����� �������
.macro clear_timer
	clt
	bld		REG_SW_TM_STATE, @0
.endm
#endif /*_AT_MACRO_SECTION_*/
#ifdef _AT_SUBROUTINE_SECTION_
; �������� ��������� �������
; ����� ������� � REG_TMP_TM_NUM
; ���� ������ � ������� REG_TMP_TM_NUM �������� � �������� �� �����, ��������������� ���� T
; � ��������� ������ ���� T ����� ������� 
; ������:
; REG_TMP
; REG_TMP1
; REG_TMP_TM_NUM
; ZL
tm_check:
	mov		REG_TMP, REG_TMP_TM_NUM
	clt
	inc		REG_TMP						; ����� ��������� ����� ������� �� REG_TMP_TM_NUM � "���������" ��� � REG_TMP1
	clr		REG_TMP1					; ��� ����� ������� REG_TMP1,
	sec									; ������������ ���� ��������
tm_check_loop:
	rol		REG_TMP1					; � ���������� ������� ������� ����� ���� �������� �����
	dec		REG_TMP
	brne	tm_check_loop
	and		REG_TMP1, REG_SW_TM_STATE	; ����� ���� �������� � REG_TMP1 - ������� � ����, ��������������� ������ �������
	breq	tm_check_end				; ������ ����������? ������� � ��������� ������ T

	lsl		REG_TMP_TM_NUM				; ������� �������� �� ������� � ������� timers, ������� REG_TMP_TM_NUM �� 2 (������� - �����������)
	ldi		ZL, LOW(timers)				; �������� ��������� �� ������
	add		ZL, REG_TMP_TM_NUM			; � �������� ��������
	ldi		REG_TMP1, 2					; ������� - ��������� 2 �����
tm_check_loop1:
	ld		REG_TMP, Z+					; ����� ��������� ���� �������
	tst		REG_TMP						; ���� �� �� �������
	brne	tm_check_end				; ������ �� ���������� ������ T
	dec		REG_TMP1					; ����� - �������� �������
	brne	tm_check_loop1				; � � ������ �����
	set									; ��� ����� ������� - ������������� ���� T
tm_check_end:
	ret

; ������ �������
; ����� ������� � REG_TMP_TM_NUM
; �������� ������� � REG_TMP1(�������):REG_TMP(�������)
; ������:
; REG_TMP2
; REG_TMP3
; REG_TMP_TM_NUM
tm_start:
	mov		REG_TMP2, REG_TMP_TM_NUM
	inc		REG_TMP2					; ����� ��������� ����� ������� �� REG_TMP_TM_NUM � "���������" ��� � REG_TMP3
	clr		REG_TMP3					; ��� ����� ������� REG_TMP3
	sec									; ������������ ���� ��������
tm_start_loop:
	rol		REG_TMP3					; � ���������� ������� ������� ����� ���� �������� �����
	dec		REG_TMP2
	brne	tm_start_loop
	or		REG_SW_TM_STATE, REG_TMP3	; ����� ���� �������� � REG_TMP3 - ������� � ����, ��������������� ������ �������

	lsl		REG_TMP_TM_NUM				; ������� �������� �� ������� � ������� timers, ������� REG_TMP_TM_NUM �� 2 (������� - �����������)
	ldi		ZL, low(timers)
	add		ZL, REG_TMP_TM_NUM
	st		Z+, REG_TMP
	st		Z, REG_TMP1
	ret
#endif /*_AT_SUBROUTINE_SECTION_*/
