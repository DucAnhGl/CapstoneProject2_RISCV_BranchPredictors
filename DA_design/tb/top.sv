`default_nettype none

module top
(
  input logic  clk_i,
  input logic  rst_i,
  output logic br_misses,    
  output logic br_instr,
  output logic [31:0] instr   
);

  // Pipelined_always_taken Pipelined_always_taken_inst (
  //   .clk_i (clk_i),
  //   .rst_i (rst_i)
  // );

  // assign br_misses = Pipelined_always_taken_inst.IF_flush;
  // assign br_instr  = Pipelined_always_taken_inst.EX_is_jmp;
  // assign instr     = Pipelined_always_taken_inst.IF_Instr;


  Pipelined_two_bit_predictor Pipelined_two_bit_predictor_inst (
    .clk_i (clk_i),
    .rst_i (rst_i)
  );

  assign br_misses = Pipelined_two_bit_predictor_inst.IF_flush;
  assign br_instr  = Pipelined_two_bit_predictor_inst.EX_is_jmp;
  assign instr     = Pipelined_two_bit_predictor_inst.IF_Instr;

endmodule : top
