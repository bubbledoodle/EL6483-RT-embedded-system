  .section .text
  .syntax unified
  .global LED_Off, LED_On, read_button, max

  .weak LED_On
  .weak LED_Off 
  .weak read_button
  .weak max


LED_On:
  PUSH {LR}
  LDR R1, =#0x40020c18
  LDR R2, [R1]      /* load from address 0x40020c18 */
  MOV R3, R0
  ADD R4, R3, #12    /* R4 got a */
  MOV R3, #1
  LSL R5, R3, R4
  ORR R4, R5, R2
  STR R4, [R1]
  POP {LR}
  BX LR

LED_Off:
  PUSH {LR}
  LDR R1, =#0x40020c18
  LDR R2, [R1]
  MOV R3, R0
  ADD R4, R3, #28
  MOV R3, #1
  LSL R5, R3, R4
  ORR R4, R5, R2
  STR R4, [R1]
  POP {LR}
  BX LR
  
read_button:
  PUSH {LR}
  LDR R1, =#0x40020010
  LDR R2, [R1]
  AND R3, R2, #0x01
  MOV R0, R3
  POP {LR}
  BX  LR


max:
	PUSH {LR}
  SUB R1, #1
	LDR R3, [R0]
  MOV R2, #0
  MOV R5, R2
  MOV R6, R0

MAX:
	ADD R5, #1
	ADD R6, #4
	LDR R4, [R6]
	CMP R4, R3
	BGT UPDATE
	CMP R5, R1
  BEQ END
	BNE MAX

UPDATE:
	MOV R3, R4
	MOV R2, R5
	B MAX

END:
  MOV R0, R3
  MOV R1, R2
  POP {LR}
  BX LR