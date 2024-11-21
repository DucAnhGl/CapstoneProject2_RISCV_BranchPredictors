module ALU (
	input [31:0] OperandA_i,
	input [31:0] OperandB_i,
	input [3:0] ALUCtrl_i,

	output reg [31:0] Result_o
);

	always @(*) begin
		case (ALUCtrl_i)
			4'b0000: Result_o = OperandA_i	+	OperandB_i;
			4'b0001: Result_o = OperandA_i	-	OperandB_i;
			4'b0010: Result_o = OperandA_i	<<	OperandB_i;
		    4'b0011: Result_o = OperandA_i	^	OperandB_i;
			4'b0100: Result_o = OperandA_i	>>	OperandB_i;
			4'b0101: Result_o = $signed(OperandA_i)	>>>	OperandB_i;
			4'b0110: Result_o = OperandA_i	|	OperandB_i;
			4'b0111: Result_o = OperandA_i	&	OperandB_i;
			4'b1000: Result_o = OperandB_i;  // Modified on 6/10/2024
			default: Result_o = 32'hxxxx_xxxx;	
		endcase
		
	end

endmodule 
