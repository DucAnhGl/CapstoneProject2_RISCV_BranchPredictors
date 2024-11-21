module ImmGen (
	input [31:0] Instruction_i,
    output reg [31:0] Immediate_o	
);


parameter R_type       = 7'b0110011,
	      I_type_LOAD  = 7'b0000011,
	      I_type_IMM   = 7'b0010011,
	      I_type_JALR  = 7'b1100111,
	      S_type       = 7'b0100011,
	      B_type       = 7'b1100011,
	      U_type_LUI   = 7'b0110111,
		  U_type_AUIPC = 7'b0010111,
	      J_type       = 7'b1101111;

//wire opcode [4:0];

//assign opode = Instruction_i[6:2];

always @(*) begin
	case (Instruction_i[6:0])
		R_type        : Immediate_o = 32'h00000000;

		I_type_LOAD   : Immediate_o = { {20{Instruction_i[31]}}, Instruction_i[31:20] };
		I_type_IMM    : Immediate_o = { {20{Instruction_i[31]}}, Instruction_i[31:20] };
		I_type_JALR   : Immediate_o = { {20{Instruction_i[31]}}, Instruction_i[31:20] };

		S_type        : Immediate_o = { {20{Instruction_i[31]}}, Instruction_i[31:25], Instruction_i[11:7] };
		B_type        : Immediate_o = { {20{Instruction_i[31]}}, Instruction_i[7], Instruction_i[30:25], Instruction_i[11:8], 1'b0 };
		U_type_LUI    : Immediate_o = { Instruction_i[31:12], 12'b000000000000 };
		U_type_AUIPC  : Immediate_o = { Instruction_i[31:12], 12'b000000000000 };
		J_type        : Immediate_o = { {11{Instruction_i[31]}}, Instruction_i[31], Instruction_i[19:12], Instruction_i[20], Instruction_i[30:21], 1'b0 };	
		default       : Immediate_o = 32'h00000000;
	endcase
end

endmodule
