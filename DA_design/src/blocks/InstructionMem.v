module InstructionMem (
	input [31:0] PC_i,
	output reg  [31:0] Instruction_o
);

reg [7:0] InstrMem [0:1023];

initial begin 
	$readmemh("InstructionMem.mem", InstrMem);
end

always @(*) begin
	Instruction_o ={ InstrMem[PC_i+3], InstrMem[PC_i+2], InstrMem[PC_i+1], InstrMem[PC_i] }; //Little Endian 
end

 
endmodule
