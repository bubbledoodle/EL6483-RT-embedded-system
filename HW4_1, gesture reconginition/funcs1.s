  .section .text
  .syntax unified
  .global calc_average

calc_average:
  PUSH {LR}
  VLDR S0, [R0]
  MOV R8, #0
  VLDR S1, [R1]
  VLDR S2, [R2]
  VMOV S0, R8
  VMOV S1, R8
  VMOV S2, R8
  MOV R4, #0

 FOR_LOOP: 
  CMP R4, #5
  BNE ADD
  BEQ END

 ADD:
  ADD R4, #1
  LDR R5, =accel_data_window
  VLDR S3, [R5]
  VLDR S4, [R5, #4]
  VLDR S5, [R5, #8]
  VADD.F32 S0, S3
  VADD.F32 S1, S4
  VADD.F32 S2, S5
  B FOR_LOOP

END:
  MOV R6, #5
  VMOV S6, R6
  VCVT.F32.S32 S6, S6
  VDIV.F32 S0, S0, S6
  VDIV.F32 S1, S1, S6
  VDIV.F32 S2, S2, S6
  
  VSTR S0, [R0]
  VSTR S1, [R1]
  VSTR S2, [R2]
  POP {LR}
  BX LR
  



