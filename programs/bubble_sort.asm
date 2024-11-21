# Array values: [5, 3, 4, 1, 2] are loaded into registers directly.
# Array is stored in registers t0 to t4 for simplicity.
# Registers t0-t4 represent the array elements array[0] to array[4].

    li t0, 5        # array[0]
    li t1, 3        # array[1]
    li t2, 4        # array[2]
    li t3, 1        # array[3]
    li t4, 2        # array[4]

    # Outer loop counter i (fixed at 4 iterations since array size is 5)
    li s0, 4        # Set outer loop counter to 4 (number of passes)

outer_loop:
    li s1, 0        # Inner loop counter j, reset each outer loop iteration

inner_loop:
    # Set up comparisons based on j using a0 as a helper

    mv t5, t0           # Default: compare array[0] and array[1]
    mv t6, t1           # Default: compare array[0] and array[1]

    li a0, 1
    beq s1, a0, cmp_12  # If j == 1, set up to compare t1 and t2
    li a0, 2
    beq s1, a0, cmp_23  # If j == 2, set up to compare t2 and t3
    li a0, 3
    beq s1, a0, cmp_34  # If j == 3, set up to compare t3 and t4
    j compare           # Jump to comparison logic

cmp_12:
    mv t5, t1           # Set up t5 for array[1]
    mv t6, t2           # Set up t6 for array[2]
    j compare

cmp_23:
    mv t5, t2           # Set up t5 for array[2]
    mv t6, t3           # Set up t6 for array[3]
    j compare

cmp_34:
    mv t5, t3           # Set up t5 for array[3]
    mv t6, t4           # Set up t6 for array[4]

compare:
    # Compare t5 (array[j]) and t6 (array[j+1])
    blt t5, t6, no_swap  # If array[j] < array[j+1], skip the swap

    # Swap array[j] and array[j+1]
    beq s1, x0, swap_01  # If j == 0, swap t0 and t1
    li a0, 1
    beq s1, a0, swap_12  # If j == 1, swap t1 and t2
    li a0, 2
    beq s1, a0, swap_23  # If j == 2, swap t2 and t3
    li a0, 3
    beq s1, a0, swap_34  # If j == 3, swap t3 and t4

swap_01:
    mv t0, t6
    mv t1, t5
    j next_inner

swap_12:
    mv t1, t6
    mv t2, t5
    j next_inner

swap_23:
    mv t2, t6
    mv t3, t5
    j next_inner

swap_34:
    mv t3, t6
    mv t4, t5

no_swap:
next_inner:
    addi s1, s1, 1         # j++
    li a0, 4               # Inner loop limit (4 iterations)
    blt s1, a0, inner_loop # If j < 4, continue inner loop

    # Decrement outer loop counter
    addi s0, s0, -1
    bnez s0, outer_loop    # Continue outer loop if s0 != 0

# Infinite loop to "exit" the program
end:
    j end                  # Jump to itself, creating an infinite loop