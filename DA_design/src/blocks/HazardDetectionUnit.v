module HazardDetectionUnit (
    input ID_BrEn, is_jalr,
    input [4:0] IFID_Rs1, IFID_Rs2, IDEX_Rd, EXMEM_Rd,
    input IDEX_RegWrEn, IDEX_MemRdEn, EXMEM_MemRdEn,

    output reg IFID_Write, PC_Write
);

    always @(*) begin
        if ( ( IDEX_MemRdEn && ((IDEX_Rd == IFID_Rs1)||(IDEX_Rd == IFID_Rs2)) )    //Resolving AL after Load

        || ( (ID_BrEn) &&    ( (IDEX_RegWrEn && ((IDEX_Rd == IFID_Rs1) || (IDEX_Rd == IFID_Rs2)))        // Resolving
                            || ((EXMEM_MemRdEn) && ((EXMEM_Rd == IFID_Rs1)||(EXMEM_Rd == IFID_Rs2))) ) )  // cmp data hazard

        || ( (is_jalr) && ((IDEX_Rd == IFID_Rs1) || ((EXMEM_MemRdEn) && (EXMEM_Rd == IFID_Rs1))) ) // Resolving jalr hazards

        ) begin
            IFID_Write = 1'b0;
            PC_Write = 1'b0;
        end else begin
            IFID_Write = 1'b1;
            PC_Write = 1'b1;
        end
    end

endmodule
