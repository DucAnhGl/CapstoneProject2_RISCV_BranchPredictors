module Adder_32bit (
	input [31:0] A_i,
	input [31:0] B_i,
	output [31:0] Sum_o
);

assign Sum_o = A_i + B_i;

endmodule
