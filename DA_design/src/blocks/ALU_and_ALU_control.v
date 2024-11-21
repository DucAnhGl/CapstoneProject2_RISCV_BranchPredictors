module ALU_and_ALU_control (
	input [31:0] OperandA_i,
        input [31:0] OperandB_i,
	input [6:0] Funct7_i,
	input [2:0] Funct3_i,
	input [1:0] ALUOp_i,

	output [31:0] Result_o
);


 
          wire [3:0] ALUCtrl_wire;
  
          ALU ALU_dut (
                  .OperandA_i (OperandA_i),
                  .OperandB_i (OperandB_i),
                  .ALUCtrl_i (ALUCtrl_wire),
                  .Result_o (Result_o)
          );
  
          ALU_control ALU_control_dut (
                  .ALUOp_i (ALUOp_i),
                  .Funct3_i (Funct3_i),
                  .Funct7_i (Funct7_i),
 
                  .ALUCtrl_o (ALUCtrl_wire)
          );



endmodule
