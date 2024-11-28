`default_nettype none

module top
(
  input logic  clk_i,
  input logic  rst_i    
);

  Pipelined_two_bit_predictor Pipelined_two_bit_predictor_inst (
    .clk_i (clk_i),
    .rst_i (rst_i)
  );

endmodule : top
