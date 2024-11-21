/* verilator lint_off UNUSED */
module ControlUnit (
    input [31:0] Instruction_i, 
    output reg ALU_A_Src_o, BrBase_o, BrEn_o, UncBr_o, MemWrEn_o, MemRdEn_o, WB_Src_o, RegWrEn_o,
    output reg [1:0] ALUOp_o, ALU_B_Src_o
    );

    parameter R_type    = 7'b0110011,
              Imm_A_L   = 7'b0010011,
              Load      = 7'b0000011,
              Store     = 7'b0100011,
              SB_type   = 7'b1100011,
              Lui       = 7'b0110111,
              jal       = 7'b1101111,
              jalr      = 7'b1100111,
              auipc     = 7'b0010111;

    wire [6:0] Opcode;
    assign Opcode = Instruction_i[6:0];

    always@ (*) begin 
        case (Opcode)
            R_type: begin
               ALUOp_o = 2'b00; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b00; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b1; 
            end

            Imm_A_L: begin
               ALUOp_o = 2'b00; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b01; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b1; 
            end

            Load: begin
               ALUOp_o = 2'b01; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b01; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b1; WB_Src_o = 1'b1; RegWrEn_o = 1'b1;
            end

            Store: begin
               ALUOp_o = 2'b01; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b01; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b1; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b0; 
            end

            SB_type: begin
               ALUOp_o = 2'b00; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b00; BrBase_o = 1'b0; BrEn_o = 1'b1; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b0;
            end
            
            Lui: begin
               ALUOp_o = 2'b10; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b01; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b1;
            end

            jal: begin
               ALUOp_o = 2'b01; ALU_A_Src_o = 1'b1; ALU_B_Src_o = 2'b10; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b1;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b1;
            end

            jalr: begin
               ALUOp_o = 2'b01; ALU_A_Src_o = 1'b1; ALU_B_Src_o = 2'b10; BrBase_o = 1'b1; BrEn_o = 1'b0; UncBr_o = 1'b1;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b1;
            end

            auipc: begin
               ALUOp_o = 2'b01; ALU_A_Src_o = 1'b1; ALU_B_Src_o = 2'b01; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b1;
            end

            default: begin
               ALUOp_o = 2'b00; ALU_A_Src_o = 1'b0; ALU_B_Src_o = 2'b00; BrBase_o = 1'b0; BrEn_o = 1'b0; UncBr_o = 1'b0;
               MemWrEn_o = 1'b0; MemRdEn_o = 1'b0; WB_Src_o = 1'b0; RegWrEn_o = 1'b0;
            end

           
        endcase
    end

endmodule
