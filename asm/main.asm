;
; AquaTime.asm
;
; Created: 16.11.2015 23:12:52
; Author : vivakin-local
; Основной файл
; определения констант, регистров и т.д.
.include "symbols.inc"
; определение макросов
#define _AT_MACRO_SECTION_
#include "timers.inc"
#undef	_AT_MACRO_SECTION_
; конец определения макросов

; RAM Start ===================================================================
.DSEG
; таблица таймеров
; таймеры - массив двухбайтовых переменных. Младший байт - первый!
timers:	.BYTE	COUNT_TIMERS*2
; буферы для хранения/изменения состояния RTC
; текущее состояние часов: читается/сохраняется начиная с 0x00 адреса ОЗУ RTC
rtc_buf_current: .BYTE RTC_BUF_LENGTH
; время выключения нагрузки: читается/сохраняется в адресе 0x08 ОЗУ RTC
rtc_buf_off: .BYTE RTC_BUF_LENGTH
; управляющий регистр RTC - здесь необходим для мигания диодом. Располагается по адресу 0x07 ОЗУ RTC
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
.org INT_VECTORS_SIZE							; Конец таблицы прерываний

; Обработчики прерываний отсюда ===============================================
Reset:
	ldi		REG_TMP, Low(RAMEND)				; Инициализация стека
	out		SPL, REG_TMP
; Это tiny - тут нет SPH
;	ldi		R16, High(RAMEND)
;	out		SPH, R16
.include "coreinit.inc"
; Инициализация внутренней периферии ==========================================
; Настройка прерывания по таймеру:
												; 1. Выбрать Clock Select Bit
	ldi		REG_TMP, CLKDIV64
	out		TCCR0B, REG_TMP
												; 2. Настроить режим работы компаратора
	ldi		REG_TMP, (1<<WGM01)+(0<<WGM00)		; Режим CTC (Clear Timer on Compare Match)
	out		TCCR0A, REG_TMP
												; 2. Записать в OCR0A значение, с которым будет сравниваться значение таймера
	ldi		REG_TMP, TMTICKS
	out		OCR0A, REG_TMP
												; 3. Разрешить в TIMSK0 прерывание от Output Compare Match A
	ldi		REG_TMP, (1<<OCIE0A)
	out		TIMSK0, REG_TMP

	ldi		REG_TMP, (1<<PORTB1)				; PORTB:
												; pin 1 - подтяжка к питанию (кнопка)
												; pin 3, pin 4 - в нули - используются в I2C
	out		PORTB, REG_TMP

	ldi		REG_TMP, (1<<DDB2)|(1<<DDB0)		; DDRB:
												; PIN0 на выход - индикация
												; PIN2 на выход - нагрузка
	out		DDRB, REG_TMP						; pin 3, pin 4 - в нули - используется в I2C

; Конец инициализации внутренней периферии ====================================

; Инициализация внешней периферии =============================================

; Конец инициализации внешней периферии =======================================
	sei		; разрешение прерываний
	ldi		REG_FLAGS, (1<<IND_ST_CH_BIT)|(1<<RTC_OFF_UNDEF)
	rcall	proc_rtc_tm_st_tm
;	sbr		REG_FLAGS, (1<<IND_ST_CH_BIT)		; необходимо для того, чтобы процедура индикации обработала
;	rcall	rtc_switch_mode						; этим вызовом привожу индикацию часов к ожидаемой
	rjmp Main

; прерывание таймера
; в прерывании делаю минимум работы, а именно - увеличиваю число тиков аппаратного таймера
; изменение программных таймеров необходимо делать в основном цикле программы
int_comp_a:
	push	REG_TMP								; сохраняю временный регистр, с которым буду работать
	in		REG_TMP, SREG						; сохраняю регистр флагов
	inc		REG_HW_TM_TICKS
	out		SREG, REG_TMP						; восстанавливаю состояние регистра флагов
	pop		REG_TMP								; и временного регистра
	reti
; Обработчики прерываний до сюда ==============================================

; Run =========================================================================
; End Run =====================================================================

; Главный цикл ================================================================
Main:
	rcall	sw_tm_ticks							; обновление программных таймеров
	rcall	proc_indc							; обновление индикации
	rcall	proc_inp							; обработка изменения состояния ввода (кнопки)
	rcall	proc_rtc							; обработка изменения состояния часов
	rcall	proc_load							; обработка изменения состояния нагрузки
	rcall	proc_flags							; обработка изменения флагов
	rcall	indc_upd							; изменение состояния индикации
    rjmp	Main
; Конец главного цикла ========================================================

; Списки индикации:
; старший бит показывает, зажечь (1) или погасить (0) индикатор
; остальные биты - количество тиков таймера, в течении которых индикатор должен оставаться в данном состоянии
; нулевой байт - признак конца списка
ST0_LED_STATES: .dw 0x80fa,0x00fa,0x80fa,0x00fa,0x80fa,0x05dc,0x0000		; Индикатор вспыхивает 3 раза, после чего пауза
ST1_LED_STATES:	.dw	0x03e8,0x0000				; Индикатор не горит
ST2_LED_STATES:	.dw	0x8032,0x00c8,0x0000		; Индикатор вспыхивает 4 раза в секунду
ST3_LED_STATES:	.dw	0x83e8,0x0000				; Индикатор горит постоянно
ST4_LED_STATES:	.dw	0x81f4,0x01f4,0x0000		; Индикатор вспыхивает 1 раз в секунду
ST5_LED_STATES:	.dw	0x81f4,0x0dac,0x0000		; Индикатор вспыхивает 1 раз в 4 секунды
; массив указателей на режимы индикации
LED_MODE_PTRS: .db low(ST0_LED_STATES*2), low(ST1_LED_STATES*2), low(ST2_LED_STATES*2), low(ST3_LED_STATES*2), low(ST4_LED_STATES*2), low(ST5_LED_STATES*2)

RTC_235959:	.db 0x59, 0x59, 0x33, 0x00
; Подпрограммы ================================================================

; Процедура обновления значений программных таймеров
; от каждого активного программного таймера необходимо
; отнять количество тиков аппаратного таймера
; если значение программного таймера перешло через 0,
; то установить значение соответствующего программного таймера в 0
; по завершении - сбросить количество тиков аппаратного таймера
; проверить регистр состояния программных таймеров на ноль
; если 0 - активных таймеров больше не осталось - выход
; сдвинуть логически вправо регистр состояния активных таймеров - младший бит уходит в флаг переноса
; если флаг переноса установлен, уменьшить соответствующий таймер
; вне зависимости от флага переноса переместить указатель на следующий таймер
sw_tm_ticks:
	cli											; на время работы с REG_HW_TM_TICKS запрещаю прерывания
	mov		REG_TMP2, REG_HW_TM_TICKS			; локальная копия
	clr		REG_HW_TM_TICKS						; количество тиков аппаратного таймера забрал - можно обнулять
	sei											; и разрешаю прерывания. Здесь есть опасность того, что
												; если до выполнения cli прерывания были запрещены где-то еще
												; они разрешатся этой инструкцией.
												; Если наступлю на эти грабли, то нужно сохранять регистр флагов,
												; запрещать прерывания, делать работу и восстанавливать регистр
												; флагов.
	mov		REG_TMP3, REG_SW_TM_STATE			; локальная копия
	ldi		ZL, low(timers)						; загружаю в регистр Z
	ldi		ZH, high(timers)					; начало таблицы таймеров из ОЗУ
sw_tm_ticks_loop:
	tst		REG_TMP3							; если регистр состояния таймеров = 0
	breq	sw_tm_ticks_end						; = 0, активных таймеров больше нет - к выходу

	lsr		REG_TMP3							; младший бит регистра состояния таймеров уходит в флаг переноса
	brcc	sw_tm_ticks_skip					; текущий таймер неактивный - ничего с ним не делаю
	ld		REG_TMP, Z							; загружаю в REG_TMP младший байт очередного таймера
	ldd		REG_TMP1, Z+1						; загружаю старший байт очередного таймера
	sub		REG_TMP, REG_TMP2					; уменьшаю младший байт на количество тиков аппаратного таймера
	sbci	REG_TMP1, 0							; учитываю флаг переноса в старшем байте очередного таймера
	brpl	sw_tm_ticks_cont					; если результат больше нуля - дальше
	clr		REG_TMP								; иначе - досчитал - устанавливаем таймер в 0
	clr		REG_TMP1
sw_tm_ticks_cont:
	st		Z, REG_TMP							; сохраняю в ОЗУ младший байт очередного таймера
	std		Z+1, REG_TMP1						; сохраняю в ОЗУ старший байт очередного таймера
sw_tm_ticks_skip:
	subi	ZL, LOW(-2)							; перехожу к следующему таймеру
	sbci	ZH, HIGH(-2)
	rjmp	sw_tm_ticks_loop
sw_tm_ticks_end:
	ret											; sw_tm_ticks

; Подпрограмма управления индикацией
; Все упростил: в REG_IND_STATE - ПЕРЕДАЕТСЯ НЕПОСРЕДСТВЕННО НОМЕР РЕЖИМА
; В РЕГИСТРЕ ФЛАГОВ IND_ST_CH_BIT - ФЛАГ НЕОБХОДИМОСТИ СМЕНЫ РЕЖИМА
; Алгоритм:
; Если пришло сообщение об изменении состояния
; - установить глобальный указатель состояния индикации на соответствующий ST<X>_LED_STATES (1)
; - в зависимости от состояния первого бита, зажечь/погасить индикатор (2)
; - установить значение таймера индикации и запустить таймер индикации (3)
; Нет сообщения об изменении состояния
; - проверить состояние таймера индикации:
; - если таймер индикации истек
; -- сдвинуть глобальный указатель состояния индикации на следующую позицию
; -- если в следующей позиции признак конца списка состояний индикации,
; --- выполнить шаги 1-2-3
; -- иначе
; --- выполнить шаги 2-3
proc_indc:
	sbrs	REG_FLAGS, IND_ST_CH_BIT			; флаг изменения состояния установлен?
	rjmp	proc_indc_st_proc					; нет - обработка существующего состояния
												; состояние изменилось - нужно переключиться на соответствующую индикацию

	cbr		REG_FLAGS, (1<<IND_ST_CH_BIT)		; очищаю бит изменения состояния, т.к. обработал его
	ldi		ZL, LOW(LED_MODE_PTRS*2)			; загружаю указатель на начало массива указателей на режимы
	add		ZL, REG_IND_STATE					; прибавляю к этому указателю номер режима
	lpm		REG_IND_ST_PTR, Z					; в REG_IND_ST_PTR - указатель на соответствующий режим

proc_indc_set:
	; запуск списка режимов
	; указатель на список режимов - в REG_IND_ST_PTR
												; устанавливаю начальный режим как первый в соответствующем списке
	mov		REG_IND_STM_PTR, REG_IND_ST_PTR
	clear_timer	IND_TM_NUM						; сбрасываю таймер индикации (ВНИМАНИЕ: clear_timer портит REG_TMP)
	clr		ZH
	mov		ZL, REG_IND_STM_PTR					; в регистре Z - указатель на режим индикатора
	lpm		REG_TMP, Z+							; в REG_TMP - режим индикатора - младший байт
	lpm		REG_TMP1, Z							; в REG_TMP - режим индикатора - старший байт
	rjmp	proc_indc_st_proc1					; установка таймера и управление аппаратным портом индикации

proc_indc_st_proc:
	ldi		REG_TMP_TM_NUM, IND_TM_NUM
	rcall	tm_check
	brtc	proc_indc_end						; если время не вышло - выход
												; таймер завершился - берем следующий режим
	clr		ZH
	mov		ZL, REG_IND_STM_PTR
	adiw	ZH:ZL, 2							; переход к следующему таймеру - таймер двухбайтовый
	mov		REG_IND_STM_PTR, ZL					; сохраняю адрес
	lpm		REG_TMP, Z+							; в REG_TMP - режим индикатора - младший байт
	lpm		REG_TMP1, Z							; в REG_TMP1 - режим индикатора - старший байт
	tst		REG_TMP								; проверяем последний ли это режим (признак последнего режима - 0x0000)
	brne	proc_indc_st_proc1					; не последний режим - дальше
	tst		REG_TMP1
	brne	proc_indc_st_proc1					; не последний режим - дальше

	rjmp	proc_indc_set						; и все сначала

proc_indc_st_proc1:
	bst		REG_TMP1, 7							; в REG_TMP1 - старший байт режима индикатора; сохраняю состояние (7-ой бит: вкл/выкл) в флаг T
	cbr		REG_TMP1, (1<<7)					; все, кроме 7-го бита - это кол-во тиков таймера. Очищаю 7-ой бит
	rcall	proc_indc_hw						; вызываю процедуру аппаратной обработки индикатора (зажечь/погасить диод)

	ldi		REG_TMP_TM_NUM, IND_TM_NUM			; запуск таймера
	rcall	tm_start
proc_indc_end:
	ret
; Конец подпрограммы управления индикацией

; Подпрограмма обработки кнопки
proc_inp:
	rcall	proc_inp_hw							; После вызова этой подпрограммы состояние кнопки будет зафиксировано в флаге Т
	brtc	proc_inp_released					; Переход, если кнопка не нажата
proc_inp_pressed:								; кнопка нажата
	sbrc	REG_LOCAL_FLAGS, L_LONG_PRESS_BIT	; если флаг длинного нажатия установлен - уже ничего не делать
	rjmp	proc_inp_end						; если установлен - выход
	clr		REG_TMP1							; устанавливаю
	ldi		REG_TMP, SHORT_PRESS_TIME			; на 20мс
	ldi		REG_TMP_TM_NUM, INP_SHORT_TM_NUM	; таймер короткого нажатия
	sbrs	REG_SW_TM_STATE, INP_SHORT_TM_NUM	; если таймер неактивный
	rcall	tm_start							; запускаю таймер
	ldi		REG_TMP_TM_NUM, INP_SHORT_TM_NUM	; таймер короткого нажатия
	rcall	tm_check							; проверяю - не истек ли
	brtc	proc_inp_chk_long					; если не истек - дальше
	sbr		REG_LOCAL_FLAGS, (1<<L_SHORT_PRESS_BIT)	; если истек - поднимаю флаг короткого нажатия
proc_inp_chk_long:
	ldi		REG_TMP1, LONG_PRESS_TIME_HIGH		; устанавливаю
	ldi		REG_TMP, LONG_PRESS_TIME_LOW		; на 500мс
	ldi		REG_TMP_TM_NUM, INP_LONG_TM_NUM		; таймер длинного нажатия
	sbrs	REG_SW_TM_STATE, INP_LONG_TM_NUM	; если таймер неактивный
	rcall	tm_start							; запускаю таймер
	ldi		REG_TMP_TM_NUM, INP_LONG_TM_NUM		; таймер длинного нажатия
	rcall	tm_check							; проверяю - не истек ли
	brtc	proc_inp_end						; если не истек - выход

	sbr		REG_LOCAL_FLAGS, (1<<L_LONG_PRESS_BIT)	; устанавливаю флаг длинного нажатия
	cbr		REG_LOCAL_FLAGS, (1<<L_SHORT_PRESS_BIT)	; сбрасываю флаг короткого нажатия
	sbr		REG_FLAGS, (1<<LONG_PRESS_BIT)		; устанавливаю флаг длинного нажатия в регистре флагов
	cbr		REG_FLAGS, (1<<SHORT_PRESS_BIT)		; сбрасываю флаг короткого нажатия (на всякий случай) - можно отказаться - он не должен быть установлен
	rjmp	proc_inp_end
proc_inp_released:
	sbrs	REG_SW_TM_STATE, INP_SHORT_TM_NUM	; если таймер короткого нажатия неактивный
	rjmp	proc_inp_end						; выход
	ldi		REG_TMP_TM_NUM, INP_SHORT_TM_NUM	; таймер короткого нажатия
	rcall	tm_check							; проверяю - не истек ли
	brtc	proc_inp_chk_long					; если не истек - выход
	sbrs	REG_LOCAL_FLAGS, L_SHORT_PRESS_BIT	; проверяю, установлен ли локальный флаг короткого нажатия
	rjmp	proc_inp_clear						; если не установлен - переход на очистку флагов и таймеров
												; если установлен:
	sbr		REG_FLAGS, (1<<SHORT_PRESS_BIT)		; устанавливаю флаг короткого нажатия в регистре флагов
proc_inp_clear:									; очистка флагов и таймеров
	cbr		REG_LOCAL_FLAGS, (1<<L_LONG_PRESS_BIT)+(1<<L_SHORT_PRESS_BIT)
	clear_timer	INP_SHORT_TM_NUM
	clear_timer INP_LONG_TM_NUM
proc_inp_end:
	ret


; Обработка изменения состояния часов
proc_rtc:
	sbrs	REG_FLAGS, RTC_SET_BIT
	rjmp	proc_rtc_tm							; не пришло - дальше на обработку таймера
	rcall	proc_rtc_set_state					; вызываю процедуру чтения часов/установки состояния
	sbrs	REG_FLAGS, RTC_ON_BIT
	rjmp	proc_rtc_st_night
												; обработка сигнала на установку в режиме день:
												; в будильник записываю текущее время
	ldi		YL, low(rtc_buf_current)			; из rtc_buf_current
	ldi		REG_TMP, RTC_ALARM_ADDR				; в ОЗУ часов
	ldi		REG_TMP1, RTC_BUF_LENGTH
	rcall	rtc_write							; записываю
	cbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; сбрасываю флаг неопределенного времени выключения
	rjmp	proc_rtc_tm_wr
proc_rtc_st_night:
												; обработка сигнала на установку в режиме ночь:
												; в текущее время записываю нули
	clr		YH
	ldi		YL, low(rtc_buf_current)			; в rtc_buf_current
	st		Y, YH								; заношу 0 секунд
	std		Y+1, YH								; заношу 0 минут
	std		Y+2, YH								; заношу 0 часов
	ldi		REG_TMP, RTC_CURR_ADDR				; в текущее время
	ldi		REG_TMP1, RTC_BUF_LENGTH
	rcall	rtc_write							; записываю
	rcall	rtc_reset_off						; в время выключения записываю 23:59:59

proc_rtc_tm_wr:
	cbr		REG_FLAGS, (1<<RTC_SET_BIT)
	rcall	proc_rtc_set_state					; вызываю процедуру чтения часов/установки состояния
proc_rtc_tm:
	ldi		REG_TMP_TM_NUM, RTC_TM_NUM			; таймер опроса RTC
	sbrs	REG_SW_TM_STATE, RTC_TM_NUM			; если таймер опроса RTC неактивный
	rjmp	proc_rtc_tm_start					; переход на процедуру чтения часов и запуск таймера
												; таймер опроса RTC активный												
	rcall	tm_check							; проверяю - не истек ли
	brtc	proc_rtc_end						; если не истек - выход
proc_rtc_tm_start:								; таймер либо истек, либо неактивный
	rcall	proc_rtc_set_state					; вызываю процедуру чтения часов/установки состояния
												
												; на эту метку один раз будет rcall
												; заведу таймер для того, чтобы при старте не сразу
												; начинать опрашивать часы, а дать время для инициализации
												; микросхемы часов. По даташиту это время "менее секунды"
												; так что ХЗ, что это значит
proc_rtc_tm_st_tm:
	ldi		REG_TMP_TM_NUM, RTC_TM_NUM			; таймер опроса RTC
	ldi		REG_TMP1, RTC_TIMER_HIGH			; устанавливаем таймер на 10 сек - старший бит
	ldi		REG_TMP, RTC_TIMER_LOW				; и младший
	rcall	tm_start							; запускаю таймер
proc_rtc_end:
	ret

; Процедура установки состояния часов:
; Устанавливает одно из состояний:
; RTC_ST_DAY
; RTC_ST_NIGHT
proc_rtc_set_state:
	ldi		YL, low(rtc_buf_current)			; текущее время буду читать в rtc_buf_current
	ldi		REG_TMP, RTC_CURR_ADDR				; регистры часов
	ldi		REG_TMP1, RTC_BUF_LENGTH			; RTC_BUF_LENGTH байт
	rcall	rtc_read							; прочитать
	ldi		YL, low(rtc_buf_off)				; время выключения буду читать в rtc_buf
	ldi		REG_TMP, RTC_ALARM_ADDR				; регистры ОЗУ
	ldi		REG_TMP1, RTC_BUF_LENGTH			; RTC_BUF_LENGTH байт
	rcall	rtc_read							; прочитать
												; Проверяю, установлено ли время выключения

	cbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; сбрасываю флаг того, что время выключения не установлено
	ldi		YL, low(rtc_buf_off)				; Для этого сравниваю первый байт
	ld		REG_TMP, Y							; считанного времени выключения
	cpi		REG_TMP, RTC_UNSET_MARK				; с константой 0xFF
	brne	proc_rtc_ss_cmp						; установлено - дальше к сравнению времени
	rcall	rtc_reset_off						; не установлено - записываю 23:59:59
proc_rtc_ss_cmp:								; сравнение текущего времени с временем выключения

												; сначала сравню с 23:59:59 и,
												; если равно, установлю флаг RTC_OFF_UNDEF - он нужен для корректной индикации
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
	sbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; равно - устанавливаю флаг того, что время выключения не установлено

proc_rtc_ss_cmp_start:
												; а тут нужна процедура сравнения времени...
												; TODO: напишу пока в лоб
	ldi		REG_TMP2, RTC_BUF_LENGTH			; счетчик
	ldi		YL, LOW(rtc_buf_current+RTC_BUF_LENGTH)
	ldi		ZL, LOW(rtc_buf_off+RTC_BUF_LENGTH)

proc_rtc_ss_cmp_loop:
	ld		REG_TMP, -Y
	ld		REG_TMP1, -Z
	cp		REG_TMP, REG_TMP1
	breq	proc_rtc_ss_cmp_cont
	brpl	proc_rtc_ss_night					; часов текущего таймера больше, чем часов времени выключения - установить статус RTC_ST_NIGHT
	brlo	proc_rtc_ss_day						; часов текущего таймера меньше, чем часов времени выключения - установить статус RTC_ST_DAY
proc_rtc_ss_cmp_cont:
	dec		REG_TMP2
	brne	proc_rtc_ss_cmp_loop
proc_rtc_ss_day:								; устанавливаю состояние DAY
	sbr		REG_FLAGS, (1<<RTC_ON_BIT)
	rjmp	proc_rtc_ss_end
proc_rtc_ss_night:								; устанавливаю состояние NIGHT
	cbr		REG_FLAGS, (1<<RTC_ON_BIT)
proc_rtc_ss_end:
;	ret											; закомментировал - пусть устанавливает режим светодиода часов каждый цикл обмена с часами

; Изменение состояния индикатора часов (мигает/не мигает)
rtc_switch_mode:
	set
	sbrc	REG_FLAGS, MANUAL_MODE_BIT
	clt
												; в автоматическом режиме нужно включить импульсы от RTC раз в секунду
												; в ручном режиме - выключить
												; индикатор режима - во флаге T: 1 - автоматический
	clr		REG_TMP1
	bld		REG_TMP1, RTC_CONTROL_BIT			; флаг Т заношу в 4-ый бит REG_TMP1
	sts		rtc_buf_control, REG_TMP1			; сохраняю в буфере
	ldi		YL, LOW(rtc_buf_control)			; указатель на буфер
	ldi		REG_TMP, RTC_CONTROL_ADDR			; регистр управления RTC
	ldi		REG_TMP1, RTC_CONTROL_LEHGTH		; длина буфера
	rcall	rtc_write							; записать
	ret

; процедура установки будильника в RTC в 23:59:59
rtc_reset_off:
	ldi		REG_TMP1, RTC_BUF_LENGTH
	ldi		ZL, low(rtc_235959*2)				; из константы rtc_235959
	ldi		YL, low(rtc_buf_off)				; в rtc_buf_off
rtc_reset_off_loop:
	lpm		REG_TMP, Z+
	st		Y+, REG_TMP							; копирую
	dec		REG_TMP1
	brne	rtc_reset_off_loop					; RTC_BUF_LENGTH байт 
	ldi		REG_TMP1, RTC_BUF_LENGTH
	ldi		YL, low(rtc_buf_off)				; из rtc_buf_off
	ldi		REG_TMP, RTC_ALARM_ADDR				; в ОЗУ часов
	rcall	rtc_write							; записываю
	sbr		REG_FLAGS, (1<<RTC_OFF_UNDEF)		; устанавливаю флаг того, что время выключения не установлено
	ret

; Обработка изменения состояния нагрузки
proc_load:
	bst		REG_FLAGS, LOAD_ON_BIT
	rcall	proc_load_hw
	ret

; Подпрограмма драйвера управления нагрузкой
; в зависимости от флага T зажигает (Т=1) или гасит (T=0) нагрузку
proc_load_hw:
	brts	proc_load_hw_on
	sbi		PORTB, PORTB2						; гашу светодиод
	ret
proc_load_hw_on:
	cbi		PORTB, PORTB2						; зажигаю светодиод
	ret

; Подпрограмма драйвера управления индикацией - один светодиод
; в зависимости от флага T зажигает (Т=1) или гасит (T=0) светодиод
proc_indc_hw:
	brts	proc_indc_hw_ledon
	sbi		PORTB, PORTB0						; гашу светодиод
	ret
proc_indc_hw_ledon:
	cbi		PORTB, PORTB0						; зажигаю светодиод
	ret

; Подпрограмма драйвера управления вводом - кнопка
; Если кнопка нажата - устанавливает флаг Т; в противном случае - сбрасывает
proc_inp_hw:
	clt
	sbis	PINB, PINB1							; если кнопка отпущена (на pin = 1), пропустить след. строку
	set
	ret

; Подпрограмма обработки флагов
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
											; по короткому нажатию:
											; если установлен manual mode, то инвертировать LOAD_ON_BIT
											; если manual mode не установлен - 
											; установить флаг RTC_SET_BIT
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
											; по длинному нажатию:
											; инвертировать manual mode
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
	rjmp	indc_upd_auto_off					; переход на индикацию в авто режиме, когда нагрузка выключена
												; индикация в авто режиме, когда нагрузка включена
	ldi		REG_TMP, ST_IND_A_ON_DEF			; загрузить режим, соответствующий авто с установленным временем выключения
	sbrc	REG_FLAGS, RTC_OFF_UNDEF
	ldi		REG_TMP, ST_IND_A_ON_UNDEF			; загрузить режим, соответствующий авто с неустановленным временем выключения
	rjmp	indc_upd_end
indc_upd_auto_off:								; индикация в авто режиме, когда нагрузка выключена
	ldi		REG_TMP, ST_IND_A_OFF_DEF			; загрузить режим, соответствующий авто с установленным временем выключения
	sbrc	REG_FLAGS, RTC_OFF_UNDEF
	ldi		REG_TMP, ST_IND_A_OFF_UNDEF			; загрузить режим, соответствующий авто с неустановленным временем выключения
	rjmp	indc_upd_end
indc_upd_manual:								; индикация в ручном режиме
	ldi		REG_TMP, ST_IND_M_OFF				; загрузить режим, соответствующий выключено в ручном режиме
	sbrc	REG_FLAGS, LOAD_ON_BIT
	ldi		REG_TMP, ST_IND_M_ON				; загрузить режим, соответствующий включено в ручном режиме
indc_upd_end:
	cp		REG_TMP, REG_IND_STATE
	breq	indc_upd_end_end
	sbr		REG_FLAGS, (1<<IND_ST_CH_BIT)
indc_upd_end_end:
	mov		REG_IND_STATE, REG_TMP
	ret
	

#define _AT_SUBROUTINE_SECTION_
; Подключаю реализацию протокола I2C
.include "I2C.inc"
.include "timers.inc"
.include "I2C_RTC.inc"
#undef _AT_SUBROUTINE_SECTION_
; Конец подпрограмм ===========================================================

; EEPROM ======================================================================
.ESEG
; EEPROM Конец ================================================================
