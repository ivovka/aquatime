/*
 * IncFile1.inc
 *
 *  Created: 24.12.2015 23:23:45
 *   Author: vivakin-local
 */ 
; процедуры работы с RTC
; процедура чтени€ массива байт из RTC
; параметры:
; Y - указатель на область пам€ти, куда прочитать массив байт
; REG_TMP - адрес стартового байта, начина€ с которого нужно читать
; REG_TMP1 - количество байт, которые необходимо прочитать
rtc_read:
	ldi		i2cadr, I2C_RTC_ADDR+i2cwr	; адрес устройства + флаг того, что буду записывать адрес стартового регистра
	rcall	i2c_start
	mov		i2cdata, REG_TMP
	rcall	i2c_do_transfer				; передаю этот начальный адрес
	ldi		i2cadr, I2C_RTC_ADDR+i2crd	; ƒл€ чтени€ - снова адрес устройства и флаг чтени€
	rcall	i2c_rep_start				; —пециальный сигнал - repeated start
	dec		REG_TMP1					; цикл будет до предпоследнего байта
	breq	rtc_read_1byte
	clc									; Ѕуду читать несколько байт, поэтому очищаю флаг переноса - отвечать буду ACK
rtc_read_loop:
	rcall	i2c_do_transfer				; ѕервый байт
	st		Y+, i2cdata
	dec		REG_TMP1
	brne	rtc_read_loop
rtc_read_1byte:
	sec									; ѕеред чтением последнего байта устанавливаю флаг переноса - отвечать буду NACK
	rcall	i2c_do_transfer				; ѕоследний байт
	st		Y, i2cdata
	rcall	i2c_stop					; » стоп
	ret

; процедура записи массива байт из RTC
; параметры:
; Y - указатель на массив, который необходимо записать
; REG_TMP - адрес стартового байта в RTC, начина€ с которого нужно писать
; REG_TMP1 - размер массива Y
rtc_write:
	ldi		i2cadr, I2C_RTC_ADDR+i2cwr	; адрес устройства + флаг того, что будет запись
	rcall	i2c_start
	mov		i2cdata, REG_TMP
	rcall	i2c_do_transfer				; передал стартовый адрес
rtc_write_loop:
	ld		i2cdata, Y+
	rcall	i2c_do_transfer
	dec		REG_TMP1
	brne	rtc_write_loop
	rcall	i2c_stop
	ret

; процедура проверки, записано ли в пам€ть RTC врем€ выключени€
; если записано - устанавливает флаг T
;rtc_chk_ramtm:
;	ldi		YL, low(RTC_BUF)			; устанавливаю - куда читать
;	ldi		REG_TMP, 0x8				; начало ќ«”
;	ldi		REG_TMP1, 1					; прочитать 1 байт
;	rcall	rtc_read					; чтение
;	lds		REG_TMP1, RTC_BUF			; беру байт, который прочитал
;	clt									; очищаю флаг T
;	cpi		REG_TMP1, 0xFF				; если записаны часы, то там не может быть такого значени€
;	breq	rtc_chk_ramtm_end
;	set									; считаю, что все, что кроме 0xFF - это часы
;rtc_chk_ramtm_end:
;	ret
