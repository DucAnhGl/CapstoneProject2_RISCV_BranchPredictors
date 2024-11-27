addi x1, x1, 5
addi x2, x2, 6
add  x3, x1, x2
and  x4, x1, x2
andi x5, x5, 0
addi x6, x6, 10

loop:
addi x5, x5, 1
bne  x5, x6, loop

#Load after Store
sw   x5, 0(x0)
lw   x2, 0(x0)

#Store after Load
sw   x2, 4(x0)

lw   x7, 4(x0)

jal  x1, func
add  x3, x1, x2
addi x0, x0, 0
addi x0, x0, 0
addi x0, x0, 0
addi x0, x0, 0

func:
jalr x0, x1, 0