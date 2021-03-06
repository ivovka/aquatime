/*
 * IncFile1.inc
 *
 *  Created: 24.12.2015 23:23:45
 *   Author: vivakin-local
 */ 
; ��������� ������ � RTC
; ��������� ������ ������� ���� �� RTC
; ���������:
; Y - ��������� �� ������� ������, ���� ��������� ������ ����
; REG_TMP - ����� ���������� �����, ������� � �������� ����� ������
; REG_TMP1 - ���������� ����, ������� ���������� ���������
rtc_read:
	ldi		i2cadr, I2C_RTC_ADDR+i2cwr	; ����� ���������� + ���� ����, ��� ���� ���������� ����� ���������� ��������
	rcall	i2c_start
	mov		i2cdata, REG_TMP
	rcall	i2c_do_transfer				; ������� ���� ��������� �����
	ldi		i2cadr, I2C_RTC_ADDR+i2crd	; ��� ������ - ����� ����� ���������� � ���� ������
	rcall	i2c_rep_start				; ����������� ������ - repeated start
	dec		REG_TMP1					; ���� ����� �� �������������� �����
	breq	rtc_read_1byte
	clc									; ���� ������ ��������� ����, ������� ������ ���� �������� - �������� ���� ACK
rtc_read_loop:
	rcall	i2c_do_transfer				; ������ ����
	st		Y+, i2cdata
	dec		REG_TMP1
	brne	rtc_read_loop
rtc_read_1byte:
	sec									; ����� ������� ���������� ����� ������������ ���� �������� - �������� ���� NACK
	rcall	i2c_do_transfer				; ��������� ����
	st		Y, i2cdata
	rcall	i2c_stop					; � ����
	ret

; ��������� ������ ������� ���� �� RTC
; ���������:
; Y - ��������� �� ������, ������� ���������� ��������
; REG_TMP - ����� ���������� ����� � RTC, ������� � �������� ����� ������
; REG_TMP1 - ������ ������� Y
rtc_write:
	ldi		i2cadr, I2C_RTC_ADDR+i2cwr	; ����� ���������� + ���� ����, ��� ����� ������
	rcall	i2c_start
	mov		i2cdata, REG_TMP
	rcall	i2c_do_transfer				; ������� ��������� �����
rtc_write_loop:
	ld		i2cdata, Y+
	rcall	i2c_do_transfer
	dec		REG_TMP1
	brne	rtc_write_loop
	rcall	i2c_stop
	ret

; ��������� ��������, �������� �� � ������ RTC ����� ����������
; ���� �������� - ������������� ���� T
;rtc_chk_ramtm:
;	ldi		YL, low(RTC_BUF)			; ������������ - ���� ������
;	ldi		REG_TMP, 0x8				; ������ ���
;	ldi		REG_TMP1, 1					; ��������� 1 ����
;	rcall	rtc_read					; ������
;	lds		REG_TMP1, RTC_BUF			; ���� ����, ������� ��������
;	clt									; ������ ���� T
;	cpi		REG_TMP1, 0xFF				; ���� �������� ����, �� ��� �� ����� ���� ������ ��������
;	breq	rtc_chk_ramtm_end
;	set									; ������, ��� ���, ��� ����� 0xFF - ��� ����
;rtc_chk_ramtm_end:
;	ret
