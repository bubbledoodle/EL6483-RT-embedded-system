.section .text
.syntax unified
.global LED_On, LED_Off, read_button, max, FLOOR_NUM


FLOOR_NUM:
  PUSH {LR}
  VCMP.F32 S0,#0.0
  BLT STOP
  MOV R0,#0
  VMOV S1,#1.0

LOOP:
  VCMP.F32 S0,S1
  VMRS APSR_nzvc,FPSCR
  BLT STOP
  VSUB.F32 S0,S1
  ADD R0,#1
  BL LOOP

STOP:
  POP {LR}
  BX LR