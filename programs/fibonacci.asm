_start:
    li a0, 15          # Calculate Fibonacci of 10 (replace 10 with desired number)
    li t0, 0            # Initialize t0 to store fib(0)
    li t1, 1            # Initialize t1 to store fib(1)
    li t3, 1

    # Loop until we reach the nth Fibonacci number
fibonacci_loop:
    beqz a0, end        # If n == 0, exit the loop and result is in t0
    beq a0, t3, end      # If n == 1, exit the loop and result is in t1

    add t2, t0, t1      # t2 = t0 + t1 (fib(n) = fib(n-1) + fib(n-2))
    mv t0, t1           # Move fib(n-1) to fib(n-2) position (t0 = t1)
    mv t1, t2           # Move fib(n) to fib(n-1) position (t1 = t2)
    addi a0, a0, -1     # Decrement n (a0 = a0 - 1)
    j fibonacci_loop    # Repeat the loop

end:
    mv a0, t1           # Result of fib(n) is now in a0

    # Exit program (simulate exit in bare-metal by looping indefinitely)
halt:
    j halt              # Infinite loop to end the program