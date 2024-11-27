module btb #(
    parameter INDEX_WIDTH = 12
)(
    input  logic                          clk_i, rst_i,  // clock and async active-high reset
    input  logic [(INDEX_WIDTH-1):0]      rd_index_i,    // Index to read out tag and target
    input  logic [(INDEX_WIDTH-1):0]      wr_index_i,    // Index to write to tag table and target table
    input  logic                          wren_i,        // Enable writing to tag table and target table
    input  logic [31:0]                   wr_target_i,   // Target to write
    input  logic [(32-INDEX_WIDTH-2)-1:0] wr_tag_i,      // Tag to write

    output logic                          valid_o,       // Whether the data in btb is valid (updated by the CPU)
    output logic [31:0]                   rd_target_o,   // Branch target saved in the btb indexed by part of PC
    output logic [(32-INDEX_WIDTH-2)-1:0] rd_tag_o       // Tag saved in the btb indexed by part of PC
                                                         // (32-INDEX_WIDTH-2): width of tag field
                                                         //    32: Width of PC
                                                         //    -2: the last 2 bits of PC are not used for indexing
);

    localparam TABLE_SIZE = 2**(INDEX_WIDTH); // size of the table (number of rows)
    integer i;

    logic [31:0]                   target_table [0:(TABLE_SIZE-1)];
    logic [(32-INDEX_WIDTH-2)-1:0] tag_table    [0:(TABLE_SIZE-1)];
    logic                          valid_table  [0:(TABLE_SIZE-1)];

    //Sync write:
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (i=0; i<TABLE_SIZE; i=i+1) begin
                // target_table[i] <= 0;
                // tag_table   [i] <= 0;
                valid_table [i] <= 0;    
            end
        end else begin
            if (wren_i) begin
                target_table[wr_index_i] <= wr_target_i;
                tag_table   [wr_index_i] <= wr_tag_i;
                valid_table [wr_index_i] <= 1'b1;        
            end
        end
    end

    //Async read:
    assign rd_target_o = target_table[rd_index_i];
    assign rd_tag_o    = tag_table   [rd_index_i];
    assign valid_o     = valid_table [rd_index_i];


endmodule
