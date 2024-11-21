module PC (
	input clk_i,
    input rst_i,
	input wren_i,
	input [31:0] NextAddr_i,
	output reg [31:0] CurrentAddr_o
);

always @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin 
		CurrentAddr_o <= 32'h0000_0000;
	end
	else begin 
		if (wren_i) CurrentAddr_o <= NextAddr_i;
	end
end

endmodule
