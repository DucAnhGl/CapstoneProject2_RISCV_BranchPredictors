    addi x3, x0, 10

loop:
    li   x2, 0x0000FFFF
    jal  x1, delay_1s

    li   x2, 0xFFFF0000
    jal  x5, delay_1s

    addi x3, x3, -1
    beq  x3, x0, exit

    j loop

delay_1s:
    li   x10, 5                 # Load 50 million into t0 (1 second delay for 50 MHz clock)
delay_loop: 
    addi x10, x10, -1           # Decrement the counter
    bne  x10, x0, delay_loop   # If t0 is not zero, branch back to delay_loop
    jalr x0,x1,0                # Return from the function

exit:
    nop
    nop
    nop
    nop
    nop
