; ����� ���� ��� ������������� ������ � ���������

; ������������� ���
RAM_Flush:
	ldi		ZL, Low(SRAM_START)	; ����� ������ ���
	ldi		ZH, High(SRAM_START)
	clr		REG_TMP				; ������� REG_TMP
Flush:
	st		Z+,REG_TMP			; ���������� � ������ 0
	cpi		ZH, High(RAMEND)	; �������� ����� ���?
	brne	Flush				; ��� - ����������
	cpi		ZL, Low(RAMEND)		; ������� ����?
	brne	Flush

; ������������� ��������� ������
	ldi		ZL, 30				; 30 = 32 (���-�� ���) - ZL - ZH
	clr		ZH
	dec		ZL
	st		Z, ZH
	brne	PC-2
