module CmpForwardingUnit (
    input [4:0] EXMEM_Rd, MEMWB_Rd, IFID_Rs1, IFID_Rs2,
    input EXMEM_RegWrEn, MEMWB_RegWrEn,
    output reg [1:0] FrwD, FrwE
);

    always @(*) begin 
        if ((EXMEM_RegWrEn) && (EXMEM_Rd != 0) && (EXMEM_Rd == IFID_Rs1)) FrwD = 2'b01;
        else if ( MEMWB_RegWrEn && (MEMWB_Rd != 0)
            && (MEMWB_Rd == IFID_Rs1) ) FrwD = 2'b10;
        else FrwD = 2'b00;

        if ((EXMEM_RegWrEn) && (EXMEM_Rd != 0) && (EXMEM_Rd == IFID_Rs2)) FrwE = 2'b01;
        else if ( MEMWB_RegWrEn && (MEMWB_Rd != 0)
            && (MEMWB_Rd == IFID_Rs2) ) FrwE = 2'b10;
        else FrwE = 2'b00;
    end

endmodule
