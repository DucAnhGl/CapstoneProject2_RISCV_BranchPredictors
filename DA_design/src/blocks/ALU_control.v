module ALU_control (
	input [1:0] ALUOp_i,
	input [2:0] Funct3_i,
	input [6:0] Funct7_i,
	
	output reg [3:0] ALUCtrl_o
);

	always @(*) begin
		case (ALUOp_i)
			2'b00: begin
				     if ( (Funct3_i==3'b000) && (Funct7_i==7'b0000000) ) ALUCtrl_o = 4'b0000; // Add
				else if ( (Funct3_i==3'b000) && (Funct7_i==7'b0100000) ) ALUCtrl_o = 4'b0001; // Sub

				else if ( (Funct3_i==3'b001) && (Funct7_i==7'b0000000) ) ALUCtrl_o = 4'b0010; // Sll
				else if ( (Funct3_i==3'b100) && (Funct7_i==7'b0000000) ) ALUCtrl_o = 4'b0011; // Xor
				else if ( (Funct3_i==3'b101) && (Funct7_i==7'b0000000) ) ALUCtrl_o = 4'b0100; // Srl
				else if ( (Funct3_i==3'b101) && (Funct7_i==7'b0100000) ) ALUCtrl_o = 4'b0101; // Sra
				else if ( (Funct3_i==3'b110) && (Funct7_i==7'b0000000) ) ALUCtrl_o = 4'b0110; // Or
				else if ( (Funct3_i==3'b111) && (Funct7_i==7'b0000000) ) ALUCtrl_o = 4'b0111; // And
			end

			2'b01:   ALUCtrl_o = 4'b0000; // Load-Store: Add
			2'b10:   ALUCtrl_o = 4'b1000; // LUI: out OperandB
			default: ALUCtrl_o = 4'b0000; 

		endcase
	end

endmodule
