`default_nettype none

module top
(
  input logic  clk_i,
  input logic  rst_i    
);

  Pipelined_top topdesign (
    .clk_i (clk_i),
    .rst_i (rst_i)
  );

endmodule : top
