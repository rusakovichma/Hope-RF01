Sleep_INIT:
	;��������� ������� ������
	in 		r16,MCUCR
	sbr		r16,SM0		;power save
   	sbr		r16,SM1		;
	out		MCUCR,r16
ret

;���������:
;in 		temp,MCUCR
;sbr		temp,(1<<SE)
;out		MCUCR,temp
;������� � ������ �����
;sleep
