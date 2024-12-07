/*
Signals with "EXMEM" prefix indicates that they come from the branch commit stage,
which is in default the MEM stage.
*/

module gshare_predictor #(
    parameter INDEX_WIDTH = 12,
    parameter HISTORY_WIDTH = 4
) (
    input  logic                          clk_i, rst_i,
    input  logic [(32-INDEX_WIDTH-2)-1:0] IF_PC_tag_i,             // Tag field of Fetch stage's PC

    input  logic [(INDEX_WIDTH-1):0]      IF_btb_rd_index_i,       // Read index of btb
    input  logic [(HISTORY_WIDTH-1):0]    IF_pht_rd_index_i,       // read index of pht NEW

    input  logic [(INDEX_WIDTH-1):0]      EXMEM_btb_wr_index_i,    // Write index of btb
    input  logic [(32-INDEX_WIDTH-2)-1:0] EXMEM_btb_wr_tag_i,      // New tag to write to btb
    input  logic [31:0]                   EXMEM_btb_wr_target_i,   // New target PC to write to btb

    input  logic [(HISTORY_WIDTH-1):0]    EXMEM_pht_wr_index_i,    // NEW

    input  logic                          EXMEM_btb_hit_i,         // Whether there was a hit in the btb 
    input  logic                          EXMEM_br_decision_i,     // Branch decision in the branch commit stage
    input  logic                          EXMEM_is_br_i,           // Whether the instruction is a conditional branch
    input  logic [1:0]                    EXMEM_is_uncbr_i,        // Whether the instruction is an unconditional branch
    input  logic                          EXMEM_prediction_i,      // Prediction the predictor has made for this branch back at Fetch  
    
    output logic                          IF_btb_hit_o,            // Whether the instruction in the Fetch stage "hit"
    output logic                          IF_prediction_o,         // The prediction the predictor make
    output logic [1:0]                    IF_PCnext_sel_o,         // Selection for the PCnext MUX:
                                                                   // 2'b00: IF_PCplus4;    2'b01: EXMEM_PCplus4, 
                                                                   // 2'b10: IF_btb_target; 2'b11: EXMEM_br_target
    output logic [31:0]                   IF_btb_rd_target_o,      // Target read from btb in Fetch stage
    output logic                          IF_flush_o               // Flush signal for penalty when prediction is wrong
);


    logic btb_wren;
    logic pht_update_en;
    logic ghr_update_en;
    logic btb_valid;
    logic pht_predictor_bit;
    logic [(HISTORY_WIDTH-1):0] ghr_data;
    logic [(32-INDEX_WIDTH-2)-1:0] IF_btb_rd_tag;

    assign btb_wren        = (!EXMEM_btb_hit_i) && (EXMEM_is_br_i || (EXMEM_is_uncbr_i == 2'b10)); // btb update condition: If the instruction was a branch and it was a miss
    assign pht_update_en   = (EXMEM_is_br_i || (EXMEM_is_uncbr_i == 2'b10));                       // pht update condition: If the instruction was a branch or JAL
    assign ghr_update_en   = (EXMEM_is_br_i || (EXMEM_is_uncbr_i == 2'b10));                       // ghr update condition: If the instruction was a branch or JAL
    assign IF_btb_hit_o    = ((IF_PC_tag_i == IF_btb_rd_tag) && (btb_valid)) ? 1'b1 : 1'b0;
    assign IF_prediction_o = IF_btb_hit_o & pht_predictor_bit;

    btb #(
        .INDEX_WIDTH(INDEX_WIDTH)
    ) u_btb (
        .clk_i       (clk_i),
        .rst_i       (rst_i),
        .rd_index_i  (IF_btb_rd_index_i),
        .wr_index_i  (EXMEM_btb_wr_index_i),
        .wren_i      (btb_wren),
        .wr_target_i (EXMEM_btb_wr_target_i),
        .wr_tag_i    (EXMEM_btb_wr_tag_i),

        .valid_o     (btb_valid),
        .rd_target_o (IF_btb_rd_target_o),
        .rd_tag_o    (IF_btb_rd_tag)
    );

    pht #(
        .INDEX_WIDTH(HISTORY_WIDTH)
    ) pht_inst (
        .clk_i           (clk_i),
        .rst_i           (rst_i),
        .update_en_i     (pht_update_en),
        .update_index_i  (EXMEM_pht_wr_index_i ^ ghr_data),
        .br_taken_i      (EXMEM_br_decision_i),
        .rd_index_i      (IF_pht_rd_index_i ^ ghr_data),
        .br_prediction_o (pht_predictor_bit)
    );

    ghr #(
        .HISTORY_WIDTH(HISTORY_WIDTH)  
    ) ghr_inst (
        .clk_i       (clk_i),          
        .rst_i       (rst_i),          
        .update_en_i (ghr_update_en),  
        .br_taken_i  (EXMEM_br_decision_i),    
        .ghr_data_o  (ghr_data)     
    );


    //Next PC selection decoder: 
    always @(*) begin
        if (EXMEM_is_uncbr_i == 2'b11) begin                                // check if instruction was a JALR
            IF_PCnext_sel_o = 2'b11;                                        // Recover to calculated target
            IF_flush_o      = 1'b1;
        end else if (EXMEM_is_br_i || (EXMEM_is_uncbr_i == 2'b10)) begin    // check if instruction was conditional branch or JAL
            case ({EXMEM_prediction_i, EXMEM_br_decision_i})                // check if prediction was correct: Query btb to predict
                2'b00, 2'b11: begin                                         // prediction was correct
                    if (IF_prediction_o) begin                                 // If it's a hit in btb
                        IF_PCnext_sel_o = 2'b10;                            // take stored target as next PC
                        IF_flush_o      = 1'b0;
                    end else begin
                        IF_PCnext_sel_o = 2'b00;                            // If it's a miss in btb, take PC+4 as next address
                        IF_flush_o      = 1'b0;
                    end
                end
                2'b10: begin                                                // predicted branch, but actually not branch
                    IF_PCnext_sel_o = 2'b01;                                // recover to PCplus4
                    IF_flush_o      = 1'b1;
                end
                2'b01: begin                                                // predicted not branch, but actually branch
                    IF_PCnext_sel_o = 2'b11;                                // recover to correct branch target
                    IF_flush_o      = 1'b1;
                end
            endcase
        end else if (!(EXMEM_is_br_i || (EXMEM_is_uncbr_i == 2'b10))) begin // if instruction was not a conditional branch or JAL: query BTB
            if (IF_prediction_o) begin                                      // If it's a hit in btb
                IF_PCnext_sel_o = 2'b10;                                    // take stored target as next PC
                IF_flush_o      = 1'b0;
            end else begin
                IF_PCnext_sel_o = 2'b00;                                    // If it's a miss in btb, take PC+4 as next address
                IF_flush_o      = 1'b0;
            end
        end else begin
            IF_PCnext_sel_o = 2'b00;                                        // If it's a miss in btb, take PC+4 as next address
            IF_flush_o      = 1'b0;
        end
    end
    
endmodule
