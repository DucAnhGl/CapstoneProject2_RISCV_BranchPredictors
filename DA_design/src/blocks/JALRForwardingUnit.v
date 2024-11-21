module JALRForwardingUnit (
    input [4:0] EXMEM_Rd, MEMWB_Rd, IFID_Rs1,
    input EXMEM_RegWrEn, MEMWB_RegWrEn,
    output reg [1:0] FrwF
);

    always @(*) begin 
        if ( EXMEM_RegWrEn && (EXMEM_Rd != 0) && (EXMEM_Rd == IFID_Rs1) )  FrwF = 2'b01;
        else if ( MEMWB_RegWrEn && (MEMWB_Rd != 0) && (MEMWB_Rd == IFID_Rs1) ) FrwF = 2'b10;
        else begin
            FrwF = 2'b00;
        end
    end

endmodule
