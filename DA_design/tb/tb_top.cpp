#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <iomanip>
#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vtop.h"

#define MAX_SIM_TIME 3000
vluint64_t sim_time = 0;

vluint64_t br_instr_counter = 0;
vluint64_t br_misses_counter = 0;
vluint64_t sim_cycle = 0;

//reset assertion function 
void dut_reset (Vtop *dut, vluint64_t &sim_time){
    dut->rst_i = 0;
    if(sim_time >= 3 && sim_time < 6){
        dut->rst_i = 1;
    }
}

int main(int argc, char** argv, char** env) {
    Verilated::commandArgs(argc, argv);
    Vtop *dut = new Vtop;

    Verilated::traceEverOn(true);
    VerilatedFstC *m_trace = new VerilatedFstC;
    dut->trace(m_trace, 4);
    m_trace->open("wave.fst");

    while (/*(dut->instr != 0) &&*/ (sim_time < MAX_SIM_TIME)) {
        dut_reset(dut, sim_time);         // Call reset function
        dut->clk_i ^= 1;
        dut->eval();
        m_trace->dump(sim_time);

        //counting values update at every posedge clk:
        if (dut->clk_i == 1) {
            sim_cycle = sim_cycle + 1;

        // Check instruction is a branch and increment counter
        if (dut->br_instr == 1) br_instr_counter++;

        // Check if branch misprediction occurred and increment the counter
        if (dut->br_misses == 1) br_misses_counter++;

        //increment cycle counter
        sim_cycle = sim_cycle + 1;

    }

        sim_time++;

    }

    double penalty_percent = 0.0;
    if (br_instr_counter > 0) {
        penalty_percent = ((double)br_misses_counter / (double)br_instr_counter) * 100.0;
    }

    double cycle_penalty_percent = 0.0;
    if (br_instr_counter > 0) {
        cycle_penalty_percent = ((double)br_misses_counter*2 / (double)sim_time) * 100.0;
    }

    // Output the results to the terminal
    std::cout << std::left;
    std::cout << "============================================================="                   << std::endl;

    std::cout << "                \033[32mBranch Prediction Statistics\033[0m"                     << std::endl;
    std::cout << "-------------------------------------------------------------"                   << std::endl;
    std::cout << std::setw(30) << "Total Branch Instructions: "    << br_instr_counter             << std::endl;
    std::cout << std::setw(30) << "Total Branch Mispredictions: "  << br_misses_counter            << std::endl;
    std::cout << std::setw(30) << "Percentage predicted: "         << 100 - penalty_percent << "%" << std::endl;
    std::cout << std::endl;
    std::cout << std::setw(30) << "Total Cycles: "                 << sim_cycle                    << std::endl;
    std::cout << std::setw(30) << "Cycle penalties: "              << br_misses_counter*2          << std::endl;
    std::cout << std::setw(30) << "Cycle penalty percentage: "     << cycle_penalty_percent << "%" << std::endl;

    std::cout << "============================================================="                   << std::endl;

    m_trace->close();
    delete dut;

    std::cout << std::endl;
    std::cout << "\033[32mSimulation done\033[0m" << std::endl;

    exit(EXIT_SUCCESS);
}
