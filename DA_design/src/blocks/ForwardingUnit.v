module ForwardingUnit (
    input [4:0] EXMEM_Rd, MEMWB_Rd, IDEX_Rs1, IDEX_Rs2,
    input EXMEM_RegWrEn, MEMWB_RegWrEn,
    output reg [1:0] FrwA, FrwB
);

    always @(*) begin 
        if ((EXMEM_RegWrEn) && (EXMEM_Rd != 0) && (EXMEM_Rd == IDEX_Rs1)) FrwA = 2'b01;
        else if ( MEMWB_RegWrEn && (MEMWB_Rd != 0) && (MEMWB_Rd == IDEX_Rs1) ) FrwA = 2'b10;
        else FrwA = 2'b00;

        if ((EXMEM_RegWrEn) && (EXMEM_Rd != 0) && (EXMEM_Rd == IDEX_Rs2)) FrwB = 2'b01;
        else if ( MEMWB_RegWrEn && (MEMWB_Rd != 0) && (MEMWB_Rd == IDEX_Rs2) ) FrwB = 2'b10;
        else FrwB = 2'b00;
    end
endmodule
