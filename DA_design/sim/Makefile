#*******************************************************************************
# Creator:        Hai Cao Xuan (cxhai.sdh221@hcmut.edu.vn)
# Description:    makefile
#*******************************************************************************
# Copyright (c) 2022 Hai Cao Xuan
#*******************************************************************************

FILES := -f filelist #$(shell cat filelist)

VERILATOR = verilator
VERILATOR_COV = verilator_coverage

### LINT FLAGS
LINT_FLAGS = -Wall --lint-only -sv

### VERILATE FLAGS
# Generate C++ in executable form
VERILATOR_FLAGS += -sv -cc
# Optimize
VERILATOR_FLAGS += --x-initial unique
VERILATOR_FLAGS += --x-assign unique
VERILATOR_FLAGS += -O3 -CFLAGS -DCOVERAGE
# Warn abount lint issues; may not want this on less solid designs
VERILATOR_FLAGS += -Wall
#Disable all style warnings
VERILATOR_FLAGS += -Wno-style
# Enable assertion
VERILATOR_FLAGS += --assert
# Make waveforms
VERILATOR_FLAGS += --trace-fst
VERILATOR_FLAGS += --build -j

# Input files for Verilator
VERILATOR_INPUT = -f filelist ../tb/top.sv --top-module top

.PHONY: help
help:
	@echo "make [option]"
	@echo "\tlint    run lint checks for syntax and violations"
	@echo "\tbuild   verilate and build modules"
	@echo "\tsim     run simulation"
	@echo "\twave    open waveforms"
	@echo "\tclean   clean the working folder"
	@echo "\thelp    print this help"

.PHONY: lint
lint:
	@echo
	@echo "<>---------------- LINT CHECK ----------------"
	@$(VERILATOR) $(LINT_FLAGS) $(FILES)

.PHONY: build
build: ./obj_dir/Vtop
./obj_dir/Vtop: ../tb/tb_top.cpp ../tb/top.sv
	@echo "<>---------------- BUILD ----------------"
	@$(VERILATOR) $(VERILATOR_FLAGS) $(VERILATOR_INPUT) --exe ../tb/tb_top.cpp

.PHONY: sim
sim: ./obj_dir/Vtop
	@echo "<>---------------- SIMULATING ----------------"
	@./obj_dir/Vtop +verilator+rand+reset+2

.PHONY: wave
wave: wave.fst
	@echo "<>---------------- WAVEFORMS ----------------"
	@gtkwave wave.fst wave.gtkw

.PHONY: clean
clean:
	@rm -rf obj_dir
	@rm -f *.fst
	@echo "<> CLEAN ---------------------"

.PHONY: all
all: 
	make clean
	make sim

.PHONY: hex
hex:
	riscv32-unknown-elf-as -o test/out.o ../../programs/bubble_sort.asm
	riscv32-unknown-elf-objcopy -O verilog test/out.o InstructionMem.mem
	riscv32-unknown-elf-objdump -d test/out.o > test/output.asm