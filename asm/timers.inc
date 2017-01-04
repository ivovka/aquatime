/*
 * timers.inc
 *
 *  Created: 18.12.2015 13:29:18
 *   Author: vladimir.ivakin
 */ 
#ifdef _AT_MACRO_SECTION_
; сброс программного таймера
; @0 - номер таймера
.macro clear_timer
	clt
	bld		REG_SW_TM_STATE, @0
.endm
#endif /*_AT_MACRO_SECTION_*/
#ifdef _AT_SUBROUTINE_SECTION_
; проверка истечения таймера
; номер таймера в REG_TMP_TM_NUM
; Если таймер с номером REG_TMP_TM_NUM активный и отсчитал до конца, устанавливается флаг T
; В противном случае флаг T будет сброшен 
; Портит:
; REG_TMP
; REG_TMP1
; REG_TMP_TM_NUM
; ZL
tm_check:
	mov		REG_TMP, REG_TMP_TM_NUM
	clt
	inc		REG_TMP						; нужно перевести номер таймера из REG_TMP_TM_NUM в "зажженный" бит в REG_TMP1
	clr		REG_TMP1					; для этого обнуляю REG_TMP1,
	sec									; устанавливаю флаг переноса
tm_check_loop:
	rol		REG_TMP1					; и циклически сдвигаю единицу через флаг переноса влево
	dec		REG_TMP
	brne	tm_check_loop
	and		REG_TMP1, REG_SW_TM_STATE	; перед этой командой в REG_TMP1 - единица в бите, соответствующем номеру таймера
	breq	tm_check_end				; таймер неактивный? выходим с очищенным флагом T

	lsl		REG_TMP_TM_NUM				; получаю смещение до таймера в массиве timers, умножая REG_TMP_TM_NUM на 2 (таймеры - двухбайтные)
	ldi		ZL, LOW(timers)				; загружаю указатель на массив
	add		ZL, REG_TMP_TM_NUM			; и добавляю смещение
	ldi		REG_TMP1, 2					; счетчик - прочитать 2 байта
tm_check_loop1:
	ld		REG_TMP, Z+					; читаю очередной байт таймера
	tst		REG_TMP						; если он не нулевой
	brne	tm_check_end				; выхожу со сброшенным флагом T
	dec		REG_TMP1					; иначе - уменьшаю счетчик
	brne	tm_check_loop1				; и в начало цикла
	set									; оба байта нулевые - устанавлигваю флаг T
tm_check_end:
	ret

; запуск таймера
; номер таймера в REG_TMP_TM_NUM
; значение таймера в REG_TMP1(старший):REG_TMP(младший)
; портит:
; REG_TMP2
; REG_TMP3
; REG_TMP_TM_NUM
tm_start:
	mov		REG_TMP2, REG_TMP_TM_NUM
	inc		REG_TMP2					; нужно перевести номер таймера из REG_TMP_TM_NUM в "зажженный" бит в REG_TMP3
	clr		REG_TMP3					; для этого обнуляю REG_TMP3
	sec									; устанавливаю флаг переноса
tm_start_loop:
	rol		REG_TMP3					; и циклически сдвигаю единицу через флаг переноса влево
	dec		REG_TMP2
	brne	tm_start_loop
	or		REG_SW_TM_STATE, REG_TMP3	; перед этой командой в REG_TMP3 - единица в бите, соответствующем номеру таймера

	lsl		REG_TMP_TM_NUM				; получаю смещение до таймера в массиве timers, умножая REG_TMP_TM_NUM на 2 (таймеры - двухбайтные)
	ldi		ZL, low(timers)
	add		ZL, REG_TMP_TM_NUM
	st		Z+, REG_TMP
	st		Z, REG_TMP1
	ret
#endif /*_AT_SUBROUTINE_SECTION_*/
