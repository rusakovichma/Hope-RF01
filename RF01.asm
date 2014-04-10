.include "m8535def.inc"

.def	OSRG = r17
.def	ChkSum = r22
.def  	comm = r23
.def  	Flag = r24

.equ	PORT_SCK	= PORTB
.equ	PIN_SCK		= PINB
.equ	DDR_SCK		= DDRB

.equ	PORT_SDI	= PORTB
.equ	PIN_SDI		= PINB
.equ	DDR_SDI		= DDRB

.equ	PORT_SEL	= PORTB
.equ	PIN_SEL		= PINB
.equ	DDR_SEL		= DDRB

.equ	PORT_nIRQ	= PORTD
.equ	PIN_nIRQ	= PIND
.equ	DDR_nIRQ	= DDRD

.equ	PORT_SDO	= PORTB
.equ	PIN_SDO		= PINB
.equ	DDR_SDO		= DDRB

.equ	RFXX_SCK	= 3
.equ	RFXX_SDI	= 4
.equ	RFXX_SEL	= 2
.equ	RFXX_nIRQ	= 2
.equ	RFXX_SDO	= 1

.DSEG

Rf01Buffer:	.db	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

.CSEG 
; Interrupts;===================================================================
			.ORG 	0x0000
		    rjmp RESET

			.ORG	INT0addr		; External Interrupt Request 0
			RETI
			.ORG	INT1addr		; External Interrupt Request 1
				rjmp IRQn_INT
			.ORG	OC2addr			; Timer/Counter2 Compare Match
			RETI
			.ORG	OVF2addr		; Timer/Counter2 Overflow
			RETI
			.ORG	ICP1addr		; Timer/Counter1 Capture Event
			RETI
			.ORG	OC1Aaddr		; Timer/Counter1 Compare Match A
			RETI
			.ORG	OC1Baddr		; Timer/Counter1 Compare Match B
			RETI
			.ORG	OVF1addr		; Timer/Counter1 Overflow
			RETI
			.ORG	OVF0addr		; Timer/Counter0 Overflow
			RETI
			.ORG	SPIaddr			; Serial Transfer Complete
			RETI
			.ORG	URXCaddr		; USART, Rx Complete
			RETI
			.ORG	UDREaddr		; USART Data Register Empty
			RETI
			.ORG	UTXCaddr		; USART, Tx Complete
			RETI
			.ORG	ADCCaddr		; ADC Conversion Complete
			RETI 
			.ORG	ERDYaddr		; EEPROM Ready
			RETI
			.ORG	ACIaddr			; Analog Comparator
			RETI
			.ORG	TWIaddr			; 2-wire Serial Interface
			RETI
			.ORG	SPMRaddr		; Store Program Memory Ready
			RETI
; End Interrupts ==========================================

.org INT_VECTORS_SIZE
;=============================================================================
IRQn_INT:
	clr comm
	ldi r18, 1
RF01_buf:
	rcall RF01_RDFIFO
	mov comm, r17
	dec r18
	brne RF01_buf

	cpi comm, $F8			;CHECK COMMAND
	breq resiev_OK
	rjmp resiev_err

resiev_OK:
	sbr Flag, (1<<6)

	mov ChkSum, comm
	andi ChkSum, $0F

	rcall delay_mks
	rcall RF01_RDFIFO
	cp  ChkSum, r17
	brne resiev_err
	sbr Flag, (1<<4)
	
resiev_err:
	ldi		r19,$48
	ldi		r20,$ce
	rcall	RFXX_WRT_CMD	

	ldi		r19,$87
	ldi		r20,$ce
	rcall	RFXX_WRT_CMD
reti
;=============================================================================
RESET:
		LDI R16,Low(RAMEND)		
	  	OUT SPL,R16			
 
	  	LDI R16,High(RAMEND)
	  	OUT SPH,R16
RAM_Flush:	
		LDI	ZL,Low(SRAM_START)	
		LDI	ZH,High(SRAM_START)
		CLR	R16			
Flush:		
		ST 	Z+,R16			
		CPI	ZH,High(RAMEND+1)	
		BRNE	Flush			
 
		CPI	ZL,Low(RAMEND+1)	
		BRNE	Flush
 
		CLR	ZL			
		CLR	ZH

		LDI	ZL, 30		
		CLR	ZH		
		DEC	ZL		
		ST	Z, ZH		
		BRNE	PC-2
;------------------------------------------------------
	ldi	r16,$ff		
	out	DDRB,r16
	ldi	r16,$f0
	out	DDRD,r16	

	ldi	r16,00
	out	portB,r16
;------------------------------------------------------
	rcall IniOfRf01

	ldi r16, (0<<ISC11)|(0<<ISC10)
	out MCUCR, r16
	rcall Sleep_INIT

	ldi r16, (1<<INT1)
	out GIMSK, r16

	in 		r16,MCUCR
	sbr		r16,(1<<SE)
	out		MCUCR,r16
	sei
	
Loop:
	sleep
	nop
	nop
	nop
	rcall delay_mks

	cli
	sbrs Flag, 6
	rjmp no_resieve
	ldi		r16,(1<<6)
	out		portD,r16
	rcall delay_big
no_resieve:

	sbrs Flag, 4
	rjmp sum_error
	ldi		r16,(1<<4)
	out		portD,r16
	rcall delay_big
sum_error:

	clr Flag
	clr		r16
	out		portD,r16

	sei
rjmp Loop

;-------------------------------------------------------------------
IniOfRf01:
	sbi		PORT_SEL,RFXX_SEL
	sbi		PORT_SDI,RFXX_SDI
	cbi		PORT_SCK,RFXX_SCK

	sbi		DDR_SEL,RFXX_SEL
	sbi		DDR_SDI,RFXX_SDI
	cbi		DDR_SDO,RFXX_SDO
	cbi		DDR_nIRQ,RFXX_nIRQ
	sbi		DDR_SCK,RFXX_SCK

	nop
	nop
	nop

	ldi		r19,$00
	ldi		r20,$00
	rcall	RFXX_WRT_CMD

	ldi		r19,$8C
	ldi		r20,$89
	rcall	RFXX_WRT_CMD	;band=433MHz, frequency deviation = 67kHz

	ldi		r19,$40
	ldi		r20,$a6
	rcall	RFXX_WRT_CMD	;f=434MHz

	ldi		r19,$47
	ldi		r20,$c8
	rcall	RFXX_WRT_CMD	;4,8kbps	

	ldi		r19,$9B
	ldi		r20,$c6
	rcall	RFXX_WRT_CMD	;AFC setting

	ldi		r19,$2a
	ldi		r20,$c4
	rcall	RFXX_WRT_CMD	;Clock recovery manual control,Digital filter

	ldi		r19,$40
	ldi		r20,$C2
	rcall	RFXX_WRT_CMD	;output 1MHz

	ldi		r19,$80
	ldi		r20,$c0
	rcall	RFXX_WRT_CMD

	ldi		r19,$84
	ldi		r20,$ce
	rcall	RFXX_WRT_CMD	;use FIFO

	ldi		r19,$87
	ldi		r20,$ce
	rcall	RFXX_WRT_CMD

	ldi		r19,$81
	ldi		r20,$c0
	rcall	RFXX_WRT_CMD	;OPEN RX

ret
;-------------------------------------------------------------------
RFXX_WRT_CMD:
	push	r16
	cbi		PORT_SCK,RFXX_SCK
	cbi		PORT_SEL,RFXX_SEL
	ldi		r16,16
R_W_C1:

	cbi		PORT_SCK,RFXX_SCK
	nop
	nop

	sbrc	r20,7
	sbi		PORT_SDI,RFXX_SDI
	sbrs	r20,7
	cbi		PORT_SDI,RFXX_SDI
	nop
	nop

	sbi		PORT_SCK,RFXX_SCK
	nop
	nop

	lsl		r19
	rol		r20
	dec		r16
	brne	R_W_C1

	cbi		PORT_SCK,RFXX_SCK
	sbi		PORT_SEL,RFXX_SEL
	pop		r16
ret
;-------------------------------------------------------------------
RF01_RDFIFO:
		push	r16

		cbi		PORT_SCK,RFXX_SCK
		cbi		PORT_SDI,RFXX_SDI
		cbi		PORT_SEL,RFXX_SEL

		nop
		nop

		ldi		r16,16
skip_status:
		sbi		PORT_SCK,RFXX_SCK
		nop

		cbi		PORT_SCK,RFXX_SCK
		nop

		dec		r16
		brne	skip_status

		clr r17
		ldi	r16,8
read_data_byte:
		lsl		r17
		sbic	PIN_SDO,RFXX_SDO
		ori 	r17,$01

		sbi		PORT_SCK,RFXX_SCK
		nop

		cbi		PORT_SCK,RFXX_SCK
		nop

		dec		r16
		brne	read_data_byte


		sbi		PORT_SEL,RFXX_SEL

		pop r16
ret
;-------------------------------------------------------------------
.include "delays_4M.asm"
.include "sleep_mode.asm"
