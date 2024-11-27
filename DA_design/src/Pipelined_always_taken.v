module Pipelined_always_taken (
    input clk_i,
    input rst_i
);
    //Local params
    localparam INDEX_WIDTH = 12;

    ///////////////////// Control signals////////////////////////
    ///// ID stage
    wire       ID_BrBase, ID_BrEn, ID_UncBr, ID_ALU_A_Src, ID_MemWrEn, ID_MemRdEn, ID_RegWrEn, ID_WB_Src;  
    wire [1:0] ID_ALU_B_Src, ID_ALUOp;
    reg        ID_BrEn_true, ID_UncBr_true, ID_ALU_A_Src_true, ID_MemWrEn_true, ID_MemRdEn_true, ID_RegWrEn_true, ID_WB_Src_true;  
    reg  [1:0] ID_ALU_B_Src_true, ID_ALUOp_true;

    ///// EX stage
    reg       IDEX_ALU_A_Src, IDEX_MemWrEn, IDEX_MemRdEn, IDEX_RegWrEn, IDEX_WB_Src, IDEX_BrEn, IDEX_UncBr; 
    reg [1:0] IDEX_ALU_B_Src, IDEX_ALUOp;

    ///// MEM stage
    reg       EXMEM_MemWrEn, EXMEM_MemRdEn, EXMEM_RegWrEn, EXMEM_WB_Src;

    ///// WB stage
    reg       MEMWB_RegWrEn, MEMWB_WB_Src;

    //////////////////// Datapath signals ///////////////////////
    // IF stage signals
    wire [31:0]                   IF_CurrentPC, IF_PCplus4, IF_NextPC, IF_Instr;
    wire                          IF_IFIDFlush, IF_IDEXFlush;
    wire [(INDEX_WIDTH-1):0]      IF_btb_rd_index;
    wire [(32-INDEX_WIDTH-2)-1:0] IF_PC_tag;
    wire                          IF_btb_hit;
    wire [31:0]                   IF_btb_target;
    wire [1:0]                    IF_PCnext_sel;
    wire                          IF_flush;       

    // ID stage signals
    reg  [31:0] IFID_Instr, IFID_CurrentPC, IFID_PCplus4;
    wire [4:0]  IFID_Rs1, IFID_Rs2, IFID_Rd;
    wire [6:0]  IFID_Funct7;
    wire [2:0]  IFID_Funct3;
    wire [31:0] ID_Imm, ID_ReadData1, ID_ReadData2, ID_BrAddr;
    wire        ID_flag; //ID_True_Br_Decision;
    wire        ID_HDU_IFID_Write, ID_HDU_PC_Write; // Write-enable signals for IFID reg and PC
    wire [1:0]  ID_FrwD, ID_FrwE, ID_FrwF;
    wire [31:0] ID_BrBase_value, ID_BrBase_from_reg;
    reg         IFID_btb_hit;

    reg         IFID_Flush;
    wire        ID_FlushMuxSel;
    wire        ID_is_jalr;

    wire [31:0] ID_compdata_i_1, ID_compdata_i_2; 

    assign IFID_Rs1 = IFID_Instr[19:15];
    assign IFID_Rs2 = IFID_Instr[24:20];
    assign IFID_Rd  = IFID_Instr[11:7];

    assign IFID_Funct7 = IFID_Instr[31:25];
    assign IFID_Funct3 = IFID_Instr[14:12];

    //assign ID_True_Br_Decision = ID_UncBr | (ID_BrEn & ID_flag);

    // EX stage signals
    reg  [4:0]  IDEX_Rs1, IDEX_Rs2, IDEX_Rd;
    reg  [31:0] IDEX_CurrentPC, IDEX_PCplus4, IDEX_ReadData1, IDEX_ReadData2, IDEX_Imm; 
    reg  [2:0]  IDEX_Funct3;
    reg  [6:0]  IDEX_Funct7;
    wire [31:0] EX_ALU_operandA, EX_ALU_operandB, EX_ALU_result, EX_MemWrData;
    wire [31:0] EX_frwA_mux_out, EX_frwB_mux_out;
    wire [1:0]  EX_FrwA, EX_FrwB;
    reg         IDEX_flag;
    wire        IDEX_True_Br_Decision;
    reg  [31:0] IDEX_BrAddr;
    reg         IDEX_btb_hit;
    wire        EX_is_jmp;
    wire [(INDEX_WIDTH-1):0]      IDEX_btb_wr_index;
    wire [(32-INDEX_WIDTH-2)-1:0] IDEX_btb_wr_tag;
    
    
    // MEM stage signals
    reg  [31:0]  EXMEM_ALU_result, EXMEM_MemWrData; 
    wire [31:0]  MEM_Dmem_data;
    reg  [4:0]   EXMEM_Rd;

    // WB stage signals
    reg  [4:0]  MEMWB_Rd;
    reg  [31:0] MEMWB_ALU_result, MEMWB_Dmem_data;
    wire [31:0] WB_data;



    // -----------------------------IF stage module instantiation and connections ----------------------------------

    //Datapath:
    PC PC_inst (
        .clk_i         (clk_i),
        .rst_i         (rst_i),
        .wren_i        (ID_HDU_PC_Write),
        .NextAddr_i    (IF_NextPC),
        .CurrentAddr_o (IF_CurrentPC)
    );

    Adder_32bit PCplus4_adder (
        .A_i   (IF_CurrentPC),
        .B_i   (32'h0000_0004),
        .Sum_o (IF_PCplus4)
    );

    always_taken_predictor #(
        .INDEX_WIDTH (INDEX_WIDTH)
    ) always_taken_predictor_inst (
        .clk_i                 (clk_i),
        .rst_i                 (rst_i),
        .IF_PC_tag_i           (IF_PC_tag),              
        .IF_btb_rd_index_i     (IF_btb_rd_index),
        .EXMEM_btb_wr_index_i  (IDEX_btb_wr_index),
        .EXMEM_btb_wr_tag_i    (IDEX_btb_wr_tag),  
        .EXMEM_btb_wr_target_i (IDEX_BrAddr),
        .EXMEM_btb_hit_i       (IDEX_btb_hit),    
        .EXMEM_br_decision_i   (IDEX_True_Br_Decision),
        .EXMEM_is_jmp_i        (EX_is_jmp),  
        
        .IF_btb_hit_o          (IF_btb_hit),        
        .IF_PCnext_sel_o       (IF_PCnext_sel),       
        .IF_btb_rd_target_o    (IF_btb_target),
        .IF_flush_o            (IF_flush)
    );

    assign IF_btb_rd_index = IF_CurrentPC[(INDEX_WIDTH+1):2];
    assign IF_PC_tag       = IF_CurrentPC[31:(INDEX_WIDTH+2)];

    assign IF_NextPC    = (IF_PCnext_sel == 2'b00) ? IF_PCplus4    : 
                          (IF_PCnext_sel == 2'b01) ? IDEX_PCplus4  :
                          (IF_PCnext_sel == 2'b10) ? IF_btb_target :
                                                     IDEX_BrAddr;

    // Flush for branch mispredict penalty
    assign IF_IFIDFlush = IF_flush;
    assign IF_IDEXFlush = IF_flush;

    InstructionMem Imem_inst (
        .PC_i          (IF_CurrentPC),
        .Instruction_o (IF_Instr)
    ); 

    // IFID Datapath pipeline registers: async reset, sync write
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            IFID_Instr     <= 32'h0000_0000;
            IFID_CurrentPC <= 32'h0000_0000;
            IFID_PCplus4   <= 32'h0000_0000;
            IFID_btb_hit   <= 1'b0;
        end else begin
            if (ID_HDU_IFID_Write) begin
                IFID_Instr     <= IF_Instr;
                IFID_CurrentPC <= IF_CurrentPC;
                IFID_PCplus4   <= IF_PCplus4;
                IFID_btb_hit   <= IF_btb_hit;
            end
        end
    end

    // IFID Control pipeline registers:
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            IFID_Flush <= 1'b0;
        end else begin
            IFID_Flush <= IF_IFIDFlush;
        end
    end

    // -----------------------------ID stage module instantiation and connections ----------------------------------

    // Datapath:
    RegFile Regfile_inst (
        .clk_i       (clk_i),
        .rst_i       (rst_i),
        .ReadReg1_i  (IFID_Rs1),
        .ReadReg2_i  (IFID_Rs2),
        .WriteReg_i  (MEMWB_Rd),
        .WriteData_i (WB_data),
        .RegWrEn_i   (MEMWB_RegWrEn),
        .ReadData1_o (ID_ReadData1),
        .ReadData2_o (ID_ReadData2)
    );

    ImmGen Immgen_inst (
        .Instruction_i (IFID_Instr),
        .Immediate_o   (ID_Imm)
    );

    Adder_32bit BrAddr_calulation_adder (
        .A_i   (ID_BrBase_value),
        .B_i   (ID_Imm),
        .Sum_o (ID_BrAddr)
    );

    assign ID_BrBase_value = (!ID_BrBase) ? IFID_CurrentPC : ID_BrBase_from_reg; // Branch address calculation base selection
    assign ID_BrBase_from_reg = (ID_FrwF == 2'b00) ? ID_ReadData1 : 
                                (ID_FrwF == 2'b01) ? EXMEM_ALU_result : (ID_FrwF == 2'b10) ? WB_data : ID_ReadData1; // JALR Forwarding MUX

    Comparator Comparator_inst (
        .OperandA_i (ID_compdata_i_1),
        .OperandB_i (ID_compdata_i_2),
        .Flagsel_i  (IFID_Funct3),
        .Flag_o     (ID_flag)
    );

    ControlUnit control_unit_inst (
        .Instruction_i (IFID_Instr),
        .ALU_A_Src_o   (ID_ALU_A_Src),
        .BrBase_o      (ID_BrBase),
        .BrEn_o        (ID_BrEn),
        .UncBr_o       (ID_UncBr),
        .MemWrEn_o     (ID_MemWrEn),
        .MemRdEn_o     (ID_MemRdEn),
        .WB_Src_o      (ID_WB_Src),
        .RegWrEn_o     (ID_RegWrEn),
        .ALUOp_o       (ID_ALUOp),
        .ALU_B_Src_o   (ID_ALU_B_Src)
    );

    assign ID_compdata_i_1 = (ID_FrwD == 2'b00) ? ID_ReadData1 : 
                             (ID_FrwD == 2'b01) ? EXMEM_ALU_result : (ID_FrwD == 2'b10) ? WB_data : ID_ReadData1; // Cmp Forwarding MUX 1
    assign ID_compdata_i_2 = (ID_FrwE == 2'b00) ? ID_ReadData2 : 
                             (ID_FrwE == 2'b01) ? EXMEM_ALU_result : (ID_FrwE == 2'b10) ? WB_data : ID_ReadData2; // Cmp Forwarding MUX 2

    assign ID_FlushMuxSel = (~ID_HDU_IFID_Write) | IFID_Flush;
    assign ID_is_jalr = ID_BrBase & ID_UncBr;
    //assign ID_True_Br_Decision = ID_UncBr_true | (ID_BrEn_true & ID_flag); // Branch decision logic

    HazardDetectionUnit hazard_detection_unit_inst (
        .ID_BrEn       (ID_BrEn),
        .is_jalr       (ID_is_jalr),
        .IFID_Rs1      (IFID_Rs1),
        .IFID_Rs2      (IFID_Rs2),
        .IDEX_Rd       (IDEX_Rd),
        .EXMEM_Rd      (EXMEM_Rd),
        .IDEX_RegWrEn  (IDEX_RegWrEn),
        .IDEX_MemRdEn  (IDEX_MemRdEn),
        .EXMEM_MemRdEn (EXMEM_MemRdEn),
        .IFID_Write    (ID_HDU_IFID_Write),
        .PC_Write      (ID_HDU_PC_Write)
    );

    CmpForwardingUnit cmp_forwarding_unit_inst (
        .EXMEM_Rd      (EXMEM_Rd),
        .MEMWB_Rd      (MEMWB_Rd),
        .IFID_Rs1      (IFID_Rs1),
        .IFID_Rs2      (IFID_Rs2),
        .EXMEM_RegWrEn (EXMEM_RegWrEn),
        .MEMWB_RegWrEn (MEMWB_RegWrEn),
        .FrwD          (ID_FrwD),
        .FrwE          (ID_FrwE)
    );

    JALRForwardingUnit jalr_forwarding_unit_inst (
        .EXMEM_Rd      (EXMEM_Rd),
        .MEMWB_Rd      (MEMWB_Rd),
        .IFID_Rs1      (IFID_Rs1),
        .EXMEM_RegWrEn (EXMEM_RegWrEn),
        .MEMWB_RegWrEn (MEMWB_RegWrEn),
        .FrwF          (ID_FrwF)
    );

    // Flush MUX:
    always @(*) begin
        if (!ID_FlushMuxSel) begin
            ID_BrEn_true      = ID_BrEn;
            ID_UncBr_true     = ID_UncBr;
            ID_ALU_A_Src_true = ID_ALU_A_Src; 
            ID_MemWrEn_true   = ID_MemWrEn; 
            ID_MemRdEn_true   = ID_MemRdEn; 
            ID_RegWrEn_true   = ID_RegWrEn; 
            ID_WB_Src_true    = ID_WB_Src;
            ID_ALU_B_Src_true = ID_ALU_B_Src;
            ID_ALUOp_true     = ID_ALUOp;
        end else begin
            ID_BrEn_true      = 1'b0;
            ID_UncBr_true     = 1'b0;
            ID_ALU_A_Src_true = 1'b0;
            ID_MemWrEn_true   = 1'b0;
            ID_MemRdEn_true   = 1'b0;
            ID_RegWrEn_true   = 1'b0;
            ID_WB_Src_true    = 1'b0;
            ID_ALU_B_Src_true = 2'b00;
            ID_ALUOp_true     = 2'b00;
        end
    end

    // IDEX Datapath pipeline registers: async reset
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            IDEX_CurrentPC        <= 32'h0000_0000;
            IDEX_ReadData1        <= 32'h0000_0000;
            IDEX_ReadData2        <= 32'h0000_0000;
            IDEX_Funct7           <= 7'b000_0000;
            IDEX_Funct3           <= 3'b000;
            IDEX_Imm              <= 32'h0000_0000;
            IDEX_Rs1              <= 5'b00000;
            IDEX_Rs2              <= 5'b00000;
            IDEX_Rd               <= 5'b00000;
            IDEX_flag             <= 1'b0;
            IDEX_BrAddr           <= 32'h0000_0000;
            IDEX_PCplus4          <= 32'h0000_0000;
            IDEX_btb_hit          <= 1'b0;
        end else begin
            IDEX_CurrentPC        <= IFID_CurrentPC;
            IDEX_ReadData1        <= ID_ReadData1;
            IDEX_ReadData2        <= ID_ReadData2;
            IDEX_Imm              <= ID_Imm;
            IDEX_Funct7           <= IFID_Funct7;
            IDEX_Funct3           <= IFID_Funct3;
            IDEX_Rs1              <= IFID_Rs1;
            IDEX_Rs2              <= IFID_Rs2;
            IDEX_Rd               <= IFID_Rd;
            IDEX_flag             <= ID_flag;
            IDEX_BrAddr           <= ID_BrAddr;
            IDEX_PCplus4          <= IFID_PCplus4;
            IDEX_btb_hit          <= IFID_btb_hit;
        end
    end

    // IDEX Control pipeline registers: async reset; sync IF_IDEXFlush signal: when IF_IDEXFlush is 1, the outputs are 0.
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            IDEX_BrEn      <= 1'b0;
            IDEX_UncBr     <= 1'b0;
            IDEX_ALU_A_Src <= 1'b0;
            IDEX_ALU_B_Src <= 2'b00;
            IDEX_ALUOp     <= 2'b00;
            IDEX_MemWrEn   <= 1'b0;
            IDEX_MemRdEn   <= 1'b0;
            IDEX_WB_Src    <= 1'b0;
            IDEX_RegWrEn   <= 1'b0;
        end else begin
            if (IF_IDEXFlush) begin
                IDEX_BrEn      <= 1'b0;
                IDEX_UncBr     <= 1'b0;
                IDEX_ALU_A_Src <= 1'b0;
                IDEX_ALU_B_Src <= 2'b00;
                IDEX_ALUOp     <= 2'b00;
                IDEX_MemWrEn   <= 1'b0;
                IDEX_MemRdEn   <= 1'b0;
                IDEX_WB_Src    <= 1'b0;
                IDEX_RegWrEn   <= 1'b0;
            end else begin
                IDEX_BrEn      <= ID_BrEn_true;
                IDEX_UncBr     <= ID_UncBr_true;
                IDEX_ALU_A_Src <= ID_ALU_A_Src_true;
                IDEX_ALU_B_Src <= ID_ALU_B_Src_true;
                IDEX_ALUOp     <= ID_ALUOp_true;
                IDEX_MemWrEn   <= ID_MemWrEn_true;
                IDEX_MemRdEn   <= ID_MemRdEn_true;
                IDEX_WB_Src    <= ID_WB_Src_true;
                IDEX_RegWrEn   <= ID_RegWrEn_true;
            end
        end
    end

    // -----------------------------EX stage module instantiation and connections ----------------------------------
    //Datapath:
    ALU_and_ALU_control alu_and_alu_control_inst (
        .OperandA_i (EX_ALU_operandA),
        .OperandB_i (EX_ALU_operandB),
        .Funct7_i   (IDEX_Funct7),
        .Funct3_i   (IDEX_Funct3),
        .ALUOp_i    (IDEX_ALUOp),
        .Result_o   (EX_ALU_result)
    );

    ForwardingUnit forwarding_unit_inst (
        .EXMEM_Rd      (EXMEM_Rd),
        .MEMWB_Rd      (MEMWB_Rd),
        .IDEX_Rs1      (IDEX_Rs1),
        .IDEX_Rs2      (IDEX_Rs2),
        .EXMEM_RegWrEn (EXMEM_RegWrEn),
        .MEMWB_RegWrEn (MEMWB_RegWrEn),
        .FrwA          (EX_FrwA),
        .FrwB          (EX_FrwB)
    );

    assign EX_ALU_operandA = (!IDEX_ALU_A_Src) ? EX_frwA_mux_out : IDEX_CurrentPC;
    assign EX_ALU_operandB = (IDEX_ALU_B_Src == 2'b00) ? EX_frwB_mux_out : (IDEX_ALU_B_Src == 2'b01) ? IDEX_Imm : 
                             (IDEX_ALU_B_Src == 2'b10) ? 32'h0000_0004   : EX_frwB_mux_out;

    assign EX_frwA_mux_out = (EX_FrwA == 2'b00) ? IDEX_ReadData1 : (EX_FrwA == 2'b01) ? EXMEM_ALU_result :        // FrwA MUX
                             (EX_FrwA == 2'b10) ? WB_data : IDEX_ReadData1; 
    assign EX_frwB_mux_out = (EX_FrwB == 2'b00) ? IDEX_ReadData2 : (EX_FrwB == 2'b01) ? EXMEM_ALU_result :        // FrwB MUX
                             (EX_FrwB == 2'b10) ? WB_data : IDEX_ReadData2;

    assign EX_MemWrData = EX_frwB_mux_out;

    assign EX_is_jmp = IDEX_BrEn | IDEX_UncBr; 

    assign IDEX_True_Br_Decision = IDEX_UncBr | (IDEX_BrEn & IDEX_flag); // Branch decision logic

    assign IDEX_btb_wr_index = IDEX_CurrentPC[(INDEX_WIDTH+1):2];
    assign IDEX_btb_wr_tag   = IDEX_CurrentPC[31:(INDEX_WIDTH+2)];

    // EXMEM Datapath pipeline registers: async reset
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            EXMEM_ALU_result <= 32'h0000_0000;
            EXMEM_Rd         <= 5'b0_0000;
            EXMEM_MemWrData  <= 32'h0000_0000;

        end else begin
            EXMEM_ALU_result <= EX_ALU_result;
            EXMEM_Rd         <= IDEX_Rd;
            EXMEM_MemWrData  <= EX_MemWrData;
        end
    end

    // EXMEM Control pipeline registers: async reset
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            EXMEM_MemWrEn   <= 1'b0;
            EXMEM_MemRdEn   <= 1'b0;
            EXMEM_WB_Src    <= 1'b0;
            EXMEM_RegWrEn   <= 1'b0;
        end else begin
            EXMEM_MemWrEn   <= IDEX_MemWrEn;
            EXMEM_MemRdEn   <= IDEX_MemRdEn;
            EXMEM_WB_Src    <= IDEX_WB_Src;
            EXMEM_RegWrEn   <= IDEX_RegWrEn;
        end
    end

    // -----------------------------MEM stage module instantiation and connections ----------------------------------
    //Datapath:
    DataMem data_memory_inst (
        .clk_i       (clk_i),
        .Address_i   (EXMEM_ALU_result),
        .WriteData_i (EXMEM_MemWrData),
        .ReadEn_i    (EXMEM_MemRdEn),
        .WriteEn_i   (EXMEM_MemWrEn),
        .Data_o      (MEM_Dmem_data)
    );

    // MEMWB Datapath pipeline registers: async reset
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            MEMWB_Rd         <= 5'b00000;
            MEMWB_ALU_result <= 32'h0000_0000;
            MEMWB_Dmem_data  <= 32'h0000_0000;
        end else begin
            MEMWB_Rd         <= EXMEM_Rd;
            MEMWB_ALU_result <= EXMEM_ALU_result;
            MEMWB_Dmem_data  <= MEM_Dmem_data;
        end
    end

    // MEMWB Control pipeline registers: async reset
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            MEMWB_WB_Src    <= 1'b0;
            MEMWB_RegWrEn   <= 1'b0;
        end else begin
            MEMWB_WB_Src    <= EXMEM_WB_Src;
            MEMWB_RegWrEn   <= EXMEM_RegWrEn;
        end
    end

    // -----------------------------WB stage module instantiation and connections ----------------------------------
    //Datapath:
    assign WB_data = (!MEMWB_WB_Src) ? MEMWB_ALU_result : MEMWB_Dmem_data;
    
endmodule
