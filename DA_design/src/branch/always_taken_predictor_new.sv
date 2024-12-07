/*
Signals with "EXMEM" prefix indicates that they come from the branch commit stage,
which is in default the MEM stage.
*/

module always_taken_predictor #(
    parameter INDEX_WIDTH = 12
) (
    input logic                          clk_i, rst_i,
    input logic [(32-INDEX_WIDTH-2)-1:0] IF_PC_tag_i,              // Tag field of Fetch stage's PC
    input logic [(INDEX_WIDTH-1):0]      IF_btb_rd_index_i,        // Read index of btb
    input logic [(INDEX_WIDTH-1):0]      EXMEM_btb_wr_index_i,     // Write index of btb
    input logic [(32-INDEX_WIDTH-2)-1:0] EXMEM_btb_wr_tag_i,       // New tag to write to btb
    input logic [31:0]                   EXMEM_btb_wr_target_i,    // New target PC to write to btb
    input logic                          EXMEM_btb_hit_i,          // Whether there was a hit in the btb 
    input logic                          EXMEM_br_decision_i,      // Branch decision in the branch commit stage
    input logic                          EXMEM_is_br_i,            // Whether the instruction is a conditional branch
    input logic [1:0]                    EXMEM_is_uncbr_i,         // Whether the instruction is an unconditional branch:
                                                                   //   2'b10: JAL; 2'b11: JALR 
    
    output logic                         IF_btb_hit_o,             // Whether the instruction in the Fetch stage "hit"
    output logic [1:0]                   IF_PCnext_sel_o,          // Selection for the PCnext MUX:
                                                                   //   2'b00: IF_PCplus4;    2'b01: EXMEM_PCplus4, 
                                                                   //   2'b10: IF_btb_target; 2'b11: EXMEM_br_target 
    output logic [31:0]                  IF_btb_rd_target_o,       // Target read from btb in Fetch stage
    output logic                         IF_flush_o                // Flush signal for penalty when prediction is wrong
);

    localparam IS_JAL              = 2'b10;
    localparam IS_JALR             = 2'b11;
    localparam IS_BR               = 1'b1;
    localparam IS_NOT_BR           = 1'b0;     
    localparam BTB_HIT             = 1'b1;
    localparam BTB_MISS            = 1'b0;
    localparam DECISION_BRANCH     = 1'b1;
    localparam DECISION_NOT_BRANCH = 1'b0;

    logic btb_wren;
    logic btb_valid;
    logic [(32-INDEX_WIDTH-2)-1:0] IF_btb_rd_tag;

    assign btb_wren     = (!EXMEM_btb_hit_i) && (EXMEM_is_br_i || (EXMEM_is_uncbr_i==2'b10)); // btb update condition: If the instruction was a 
                                                                                              // conditional branch or 
                                                                                              // JAL and it was a miss
    assign IF_btb_hit_o = ((IF_PC_tag_i == IF_btb_rd_tag) && (btb_valid)) ? 1'b1 : 1'b0;

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

    //Next PC selection decoder: 
    always @(*) begin
        case ({EXMEM_is_br_i, EXMEM_is_uncbr_i})
            {IS_NOT_BR, IS_JALR}: begin
                IF_PCnext_sel_o = 2'b11;
                IF_flush_o      = 1'b1;
            end
            {IS_BR, IS_NOT_JAL}, {IS_NOT_BR, IS_JAL}: begin
                case ({EXMEM_btb_hit_i, EXMEM_br_decision_i})
                    {BTB_HIT, DECISION_BRANCH}, {BTB_MISS, DECISION_NOT_BRANCH}: begin
                        case (IF_btb_hit_o)
                            BTB_HIT: begin
                                IF_PCnext_sel_o = 2'b10;
                                IF_flush_o      = 1'b0;
                            end
                            BTB_MISS: begin
                                IF_PCnext_sel_o = 2'b00;
                                IF_flush_o      = 1'b0;
                            end
                        endcase
                    end
                    {BTB_MISS, DECISION_BRANCH}: begin
                        IF_PCnext_sel_o = 2'b11;
                        IF_flush_o      = 1'b1;
                    end
                    {BTB_HIT, DECISION_NOT_BRANCH}: begin
                        IF_PCnext_sel_o = 2'b01;
                        IF_flush_o      = 1'b1;
                    end
                endcase
            end 
            default: 
        endcase
    end
    
endmodule
