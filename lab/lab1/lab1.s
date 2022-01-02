.data
str: .string "lab1"

.text
main:
    auipc a0, str
    addi a0, a0, 0
    lui a1, 0x10000
    addi a1, a1, 0x100
    lw a2 0(a0)
    sw a2 0(a1)