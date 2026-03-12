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

        lbu  x6,  64(x20)
        lbu  x7,  8(x21)
        lbu  x8,  40(x21)
        lbu  x9,  72(x21)
        lbu  x23, 104(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        lbu  x6,  96(x20)
        lbu  x7,  12(x21)
        lbu  x8,  44(x21)
        lbu  x9,  76(x21)
        lbu  x23, 108(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        lbu  x6,  128(x20)
        lbu  x7,  16(x21)
        lbu  x8,  48(x21)
        lbu  x9,  80(x21)
        lbu  x23, 112(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        lbu  x6,  160(x20)
        lbu  x7,  20(x21)
        lbu  x8,  52(x21)
        lbu  x9,  84(x21)
        lbu  x23, 116(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

        lbu  x6,  192(x20)
        lbu  x7,  24(x21)
        lbu  x8,  56(x21)
        lbu  x9,  88(x21)
        lbu  x23, 120(x21)
        mul  x7, x7, x6
        mul  x8, x8, x6
        mul  x9, x9, x6
        mul  x23, x23, x6
        add  x14, x14, x7
        add  x15, x15, x8
        add  x16, x16, x9
        add  x5, x5, x23

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