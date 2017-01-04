; куски кода для инициализации памяти и регистров

; инициализация ОЗУ
RAM_Flush:
	ldi		ZL, Low(SRAM_START)	; Адрес начала ОЗУ
	ldi		ZH, High(SRAM_START)
	clr		REG_TMP				; Очищаем REG_TMP
Flush:
	st		Z+,REG_TMP			; Записываем в ячейку 0
	cpi		ZH, High(RAMEND)	; Достигли конца ОЗУ?
	brne	Flush				; Нет - продолжить
	cpi		ZL, Low(RAMEND)		; Младший байт?
	brne	Flush

; инициализация регистров нулями
	ldi		ZL, 30				; 30 = 32 (кол-во РОН) - ZL - ZH
	clr		ZH
	dec		ZL
	st		Z, ZH
	brne	PC-2
