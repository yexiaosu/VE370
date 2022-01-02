.data
testArray: .word 47,40,4,48,0,0,21,26,0,-18,26,0,0,-46,27,32,-45,-19,0,41,0,14,-21,28,-37,0,38,21,0,-18,0,0,0,-12,18,-39,-29,26,0,-1
size: .word 40
pos: .word 1
zero: .word 0
neg: .word -1

.text
main:
    auipc x18 0x10000
    addi x18 x18 0
    lw x19 size
    lw x27 pos
    lw x26 zero
    lw x25 neg
    # cntType: x6
    addi x6 x0 1
    jal x1 countArray
    add x24 x10 x0
    addi x6 x0 -1
    jal x1 countArray
    add x23 x10 x0
    addi x6 x0 0
    jal x1 countArray
    add x22 x10 x0
    jal x1 sumArray
    beq x0 x0 end

countArray:
    # return value: x10
    addi sp sp -4
    sw x1 0(sp)
    addi x5 x0 0
    addi x10 x0 0
    beq x6 x27 posCount
    beq x6 x26 zeroCount
    beq x6 x25 negCount
exit:
    lw x1 0(sp)
    addi sp sp 4
    jalr x0 x1(0)
    
posCount:
    # i: x5; offset: x7; return value: x10
    slli x7 x5 2
    add x7 x7 x18
    lw x8 0(x7)
    jal x1 isPos
    add x10 x10 x11
    addi x5 x5 1
    blt x5 x19 posCount
    bge x5 x19 exit
isPos:
    # x: x8; return value: x11
    addi x11 x0 0
    bgt x8 x0 posTrue
    ble x8 x0 posFalse
posTrue:
    addi x11 x0 1
posFalse:
    jalr x0 x1(0)
    
zeroCount:
    # i: x5; offset: x7; return value: x10
    slli x7 x5 2
    add x7 x7 x18
    lw x8 0(x7)
    jal x1 isZero
    add x10 x10 x11
    addi x5 x5 1
    blt x5 x19 zeroCount
    bge x5 x19 exit
isZero:
    # x: x8; return value: x11
    addi x11 x0 0
    beq x8 x0 zeroTrue
    bne x8 x0 zeroFalse
zeroTrue:
    addi x11 x0 1
zeroFalse:
    jalr x0 x1(0)

negCount:
    # i: x5; offset: x7; return value: x10
    slli x7 x5 2
    add x7 x7 x18
    lw x8 0(x7)
    jal x1 isNeg
    add x10 x10 x11
    addi x5 x5 1
    blt x5 x19 negCount
    bge x5 x19 exit
isNeg:
    # x: x8; return value: x11
    addi x11 x0 0
    blt x8 x0 negTrue
    bge x8 x0 negFalse
negTrue:
    addi x11 x0 1
negFalse:
    jalr x0 x1(0)
    
sumArray:
    # i: x5; return value: x10
    addi sp sp -4
    sw x1 0(sp)
    addi x5 x0 0
    addi x10 x0 0
sumLoop:
    slli x7 x5 2
    add x7 x7 x18
    lw x8 0(x7)
    add x10 x10 x8
    addi x5 x5 1
    blt x5 x19 sumLoop
    bge x5 x19 exitLoop
exitLoop:
    lw x1 0(sp)
    addi sp sp 4
    jalr x0 x1(0)

end:
    # PosCnt: x24, NegCnt: x23, ZeroCnt: x22, Sum: x21
    add x21 x10 x0