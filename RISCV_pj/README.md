# GEMM8x8 연산을 위한 RISCV 모델 설계


## 설계 목표
  8x8 행렬 곱에 특화된 RISCV 프로세서를 설계한다.

## 사용 툴
  synopsys dc_shell을 사용하여 합성, 타이밍, 전력, 면적 분석을 한다.

## 조건
  기본적으로 lw,sw,add,addi,and,beq,jal,mul을 사용한다.
  
  표준 ISA 명령어에 해당하는 다른 명령어를 추가할 수 있다.
  
  CPU 밖에서 별도의 연산 가속기를 설계할 수 없다.
  
  assembly 코드의 배열 및 구조 변경을 할 수 있다. 단 IMEM의 크기는 최대 192이며 이를 초과해서는 안된다.
  
  PPA를 고려한 최적의 설계 방안을 제시한다. <br><br><br>


## 기본 assembly 코드

```assembly
# gemm8x8.s
# 8x8 8-bit Matrix Multiplication (Inlined Multiply Version)
# Hardware support: RV32I (No JALR required - Safe for your DATAPATH)

.text
        .globl main

main:
        addi x10, x0, 0
        addi x11, x0, 0x100
        addi x12, x0, 0x200
        addi x13, x0, 0x300

        addi x28, x0, 32
        addi x18, x0, 8
        add  x19, x10, x0
        add  x20, x12, x0
        
        addi x9, x0, 255

loop_i:
        addi x21, x0, 8
        addi x22, x0, 0

loop_j:
        addi x27, x0, 0
        addi x23, x0, 8
        add  x24, x19, x0
        add  x26, x11, x22

loop_k:
        lw   x29, 0(x24)
        and  x29, x29, x9

        lw   x30, 0(x26)
        and  x30, x30, x9

        addi x5, x0, 0
        beq  x30, x0, mul_end_k  

mul_loop_inner:
        add  x5,  x5, x29
        addi x30, x30, -1
        beq  x30, x0, mul_end_k
        jal  x0,  mul_loop_inner
        
mul_end_k:
        add  x27, x27, x5

        addi x24, x24, 4
        addi x26, x26, 32

        addi x23, x23, -1
        beq  x23, x0, done_k
        jal  x0,  loop_k

done_k:
        add  x31, x20, x22
        sw   x27, 0(x31)

        addi x21, x21, -1
        beq  x21, x0, done_j
        addi x22, x22, 4
        jal  x0,  loop_j

done_j:
        addi x18, x18, -1
        beq  x18, x0, done_i
        addi x19, x19, 32
        addi x20, x20, 32
        jal  x0,  loop_i

done_i:
        addi x2,  x0, 1
        sw   x2,  0(x13)

done:
        beq  x0,  x0, done
```


## 설계 방법

### 1. assembly 코드 수정

시도 1.
- GEMM8x8 연산의 경우 lw를 사용하여 데이터를 로드 하여 8bit 마스킹을 진행 후 사용한다. 이 때문에 데이터 하나를
   로드 할 때마다 2개의 명령어가 사용된다. 배열을 바꿔 stall을 없애본다.
  
- IMEM의 크기 제약이 있기에 loop 문을 없애기 위해 모든 loop를 해제할 수는 없다.
  
- 8x8 연산 수행 1900 cycle 소모

```assembly
# gemm8x8.s
# 8x8 8-bit Matrix Multiplication (Inlined Multiply Version)
# Hardware support: RV32I (No JALR required - Safe for your DATAPATH)

.text
        .globl main

main:
        addi x10, x0, 0          # baseA = 0
        addi x11, x0, 0x100      # baseB = 256
        addi x12, x0, 0x200      # baseC = 512
        addi x13, x0, 0x300      # STATUS = 768
        addi x9,  x0, 255        # Mask = 0xFF

        # i loop initialization
        addi x18, x0, 8          # i_count = 8
        add  x19, x10, x0        # rowA_base = baseA
        add  x20, x12, x0        # rowC_base = baseC

loop_i:
        # j loop initialization
        addi x21, x0, 8          # j_count = 8
        addi x22, x0, 0          # col_offset = 0

loop_j:
        # Accumulator Init
        addi x27, x0, 0          # sum = 0

        # Pointers for inner loop (k)
        add  x24, x19, x0        # addrA = rowA_base
        add  x26, x11, x22       # addrB = baseB + col_offset

        # --- k=0 ---
        lw   x29, 0(x24)         # Load A[0]
        lw   x30, 0(x26)         # Load B[0]
        and  x29, x29, x9        # Mask A
        and  x30, x30, x9        # Mask B
        mul  x5,  x29, x30       # Multiply
        add  x27, x27, x5        # Accumulate

        # --- k=1 ---
        lw   x29, 4(x24)         # Load A[1]
        lw   x30, 32(x26)        # Load B[1]
        and  x29, x29, x9
        and  x30, x30, x9
        mul  x5,  x29, x30
        add  x27, x27, x5


        #...... 반복 생략


        # --- k=7 ---
        lw   x29, 28(x24)
        lw   x30, 224(x26)
        and  x29, x29, x9
        and  x30, x30, x9
        mul  x5,  x29, x30
        add  x27, x27, x5

        # Store C[i][j]
        add  x31, x20, x22
        sw   x27, 0(x31)

        # j loop check
        addi x21, x21, -1        # j_count--
        beq  x21, x0, done_j
        addi x22, x22, 4         # col_offset += 4
        jal  x0,  loop_j

done_j:
        # i loop check
        addi x18, x18, -1        # i_count--
        beq  x18, x0, done_i
        addi x19, x19, 32        # rowA_curr += 32
        addi x20, x20, 32        # rowC_curr += 32
        jal  x0,  loop_i

done_i:
        # Set Status
        addi x2,  x0, 1
        sw   x2,  0(x13)         # STATUS = 1

done:
        jal  x0, done            # Infinite Loop
```

시도 2.
- lbu 명령어를 사용함으로써 lw-and 마스킹 명령어를 한 줄로 줄인다. 이를 통해 loop를 더 많이 해제할 수 있다.
- 한번에 4개의 값을 뽑아서 연산을 진행한다. 연산 진행 방법은 아래 그림처럼 진행한다.
- 8x8 연산 수행 1001 cycle 소모 => 최대 성능

### 명령어 배열 - 연산 방식
<img width="1195" height="493" alt="image" src="https://github.com/user-attachments/assets/e0c48aa5-76b5-4cab-a6df-0bc76b6dc0eb" />
<img width="1194" height="500" alt="image" src="https://github.com/user-attachments/assets/262bf282-7274-451e-a269-67f130030730" />
<img width="1151" height="522" alt="image" src="https://github.com/user-attachments/assets/6cfce948-8af6-4ce4-9583-2bc081791383" />
<img width="1126" height="530" alt="image" src="https://github.com/user-attachments/assets/fc26183c-3731-4752-acaa-db340639ac01" />


```assembly
.text
        .globl main

main:
        addi x10, x0, 0
        addi x11, x0, 0x100
        addi x12, x0, 0x200
        addi x13, x0, 0x300

        addi x18, x0, 8
        add  x20, x11, x0

loop_j:
        addi x19, x0, 2
        add  x21, x10, x0
        add  x22, x12, x0

loop_i:
        addi x14, x0, 0
        addi x15, x0, 0
        addi x16, x0, 0
        addi x5, x0, 0

        lbu  x6,  0(x20)
        lbu  x7,  0(x21)
        lbu  x8,  32(x21)
        lbu  x9,  64(x21)
        lbu  x23, 96(x21)
        
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        lbu  x6,  32(x20)
        lbu  x7,  4(x21)
        lbu  x8,  36(x21)
        lbu  x9,  68(x21)
        lbu  x23, 100(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        #........ 반복 생략


        lbu  x6,  224(x20)
        lbu  x7,  28(x21)
        lbu  x8,  60(x21)
        lbu  x9,  92(x21)
        lbu  x23, 124(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        sw   x14, 0(x22)
        sw   x15, 32(x22)
        sw   x16, 64(x22)
        sw   x5, 96(x22)

        addi x19, x19, -1
        
        addi x21, x21, 128
        addi x22, x22, 128
        
        beq  x19, x0, loop_i_end
        jal  x0, loop_i
        
loop_i_end:

        addi x18, x18, -1
        
        addi x20, x20, 4
        addi x12, x12, 4
        
        beq  x18, x0, done_j
        jal  x0, loop_j

done_j:

done:
        addi x30, x0, 1
        sw   x30, 0(x13)
end_loop:
        jal  x0, end_loop
```
### 2. RISC-V 프로세서 설계
RV32I 표준 규격에 준수하는 5단계 파이프라인 CPU를 설계한다.

Stage는 IF -> ID -> EX -> MEM -> WB로 구성되며 Hazard 판별, Data Stall 방지를 위한 Forwarding을 구현한다.

lbu 명령어 추가에 따른 로직 수정을 한다.

[코드](RISCV_pj/rtl)

<br><br>

### 전체 시스템 아키텍처
<img width="661" height="433" alt="image" src="https://github.com/user-attachments/assets/ce3f8628-1831-44f2-b0da-e4d6d0317838" />
<br><br>

### SDC 파일
<img width="931" height="400" alt="image" src="https://github.com/user-attachments/assets/b791dcf6-15cb-4087-9b1a-c1114c5686f3" />
<br><br>

### TCL 코드
```
set TOPDESIGN RISCV_CPU
set RTL_FILES [list "./../rtl/ALU.v" \
		"./../rtl/ALUDEC.v"\
		"./../rtl/CONTROLLER.v"\
		"./../rtl/DATAPATH.v"\
		"./../rtl/EXTEND.v"\
		"./../rtl/HAZARD_UNIT.v"\
		"./../rtl/MAINDEC.v"\
		"./../rtl/PC.v"\
		"./../rtl/REGFILE.v"\
		"./../rtl/${TOPDESIGN}.v"]
read_file -format verilog ${RTL_FILES}
current_design ${TOPDESIGN}
link
check_design
source ./sdc/RISCVSINGLE-3.sdc -verbose
check_timing
write_file -format ddc -output ./outputs/${TOPDESIGN}_unmapped.ddc
compile_ultra
report_constraint -all_violators
write_file -format verilog -hierarchy -output ./outputs/${TOPDESIGN}_gate.v
write_file -format ddc -output ./outputs/${TOPDESIGN}_gate.ddc
write_sdf ./outputs/${TOPDESIGN}_gate.sdf
report_timing
report_constraint -all_violators
```
<br><br>

### 결과
1. 주기 및 critical path 경로

critical path : 3.65ns

<img width="408" height="398" alt="image" src="https://github.com/user-attachments/assets/c03dd7bb-e7d4-46bf-9657-1a0e8b224d29" />

<img width="502" height="599" alt="image" src="https://github.com/user-attachments/assets/b3bc7abf-745f-4e33-a113-a6c262730d24" />

<br><br>

2. 총 전력 및 면적
   
Total power : 2.6764e + 03uW

area : 36135

<img width="601" height="434" alt="image" src="https://github.com/user-attachments/assets/b94b508c-00a5-4505-9edb-f764ec2952fb" />
<img width="415" height="464" alt="image" src="https://github.com/user-attachments/assets/f7a21289-a216-4025-93bf-d59ed0f9a963" />

