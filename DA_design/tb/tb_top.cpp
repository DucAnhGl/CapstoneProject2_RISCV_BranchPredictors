#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vtop.h"

#define MAX_SIM_TIME 3000
vluint64_t sim_time = 0;

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

    while (sim_time < MAX_SIM_TIME) {
      dut_reset(dut, sim_time);         // Call reset function
      dut->clk_i ^= 1;
      dut->eval();
      m_trace->dump(sim_time);
      sim_time++;
    }

    m_trace->close();
    delete dut;

    std::cout << "Simulation done" << std::endl;

    exit(EXIT_SUCCESS);
}
