module Comparator (
    input [31:0] OperandA_i, OperandB_i,
    input [2:0] Flagsel_i, //Funct3
    output reg Flag_o
);

    parameter   EQ_case	 = 3'b000,
		        NE_case	 = 3'b001,
	  	        LT_case	 = 3'b100,
		        GE_case	 = 3'b101,
		        LTU_case = 3'b110,
		        GEU_case = 3'b111;	  

	reg EQ, NE, LT, GE, LTU, GEU;

    always @(*) begin
        EQ   = (OperandA_i == OperandB_i) ? 1'b1 : 1'b0;
		NE	 = (OperandA_i == OperandB_i) ? 1'b0 : 1'b1;
		LT	 = ($signed(OperandA_i) < $signed(OperandB_i)) ? 1'b1 : 1'b0;
		GE	 = ($signed(OperandA_i) >= $signed(OperandB_i)) ? 1'b1 : 1'b0;
		LTU	 = (OperandA_i < OperandB_i) ? 1'b1 : 1'b0;
		GEU	 = (OperandA_i >= OperandB_i) ? 1'b1 : 1'b0;
    end

	//Different way to implement:
	

	// Flag selection
    always @(*) begin 
		case (Flagsel_i)
			EQ_case	: Flag_o = EQ;
			NE_case	: Flag_o = NE;
			LT_case	: Flag_o = LT;
			GE_case	: Flag_o = GE;
			LTU_case: Flag_o = LTU;
			GEU_case: Flag_o = GEU;
			default : Flag_o = 1'b0;			
		endcase
	end

endmodule: Comparator
