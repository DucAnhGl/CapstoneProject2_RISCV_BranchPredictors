module pht #(
    INDEX_WIDTH = 12
)(
    input  logic                     clk_i, rst_i,       // Clock and active-high reset
    input  logic                     update_en_i,        // Decide whether to enable updating the table or not
    input  logic [(INDEX_WIDTH-1):0] update_index_i,     // The index of row that needs to be updated
    input  logic                     br_taken_i,         // The true branch decision, used to decide the updating of pht
    input  logic [(INDEX_WIDTH-1):0] rd_index_i,         // The index of prediction bit to read out

    output logic                     br_prediction_o     // The prediction read bit
);  

    localparam TABLE_SIZE = 2**(INDEX_WIDTH);
    integer i;

    localparam STRONGLY_NOT_TAKEN = 2'b00;
    localparam WEAKLY_NOT_TAKEN   = 2'b01;
    localparam WEAKLY_TAKEN       = 2'b10;
    localparam STRONGLY_TAKEN     = 2'b11;

    logic [1:0] pht_table [0:(TABLE_SIZE-1)];

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            for (i=0; i<TABLE_SIZE; i=i+1) begin
                pht_table[i] <= STRONGLY_NOT_TAKEN; //Initial state is strongly not taken
            end
        end else begin
            if (update_en_i) begin
                case (pht_table[update_index_i])
                    STRONGLY_NOT_TAKEN: pht_table[update_index_i] <= br_taken_i ? WEAKLY_NOT_TAKEN : STRONGLY_NOT_TAKEN;
                    WEAKLY_NOT_TAKEN  : pht_table[update_index_i] <= br_taken_i ? WEAKLY_TAKEN     : STRONGLY_NOT_TAKEN;
                    WEAKLY_TAKEN      : pht_table[update_index_i] <= br_taken_i ? STRONGLY_TAKEN   : WEAKLY_NOT_TAKEN;
                    STRONGLY_TAKEN    : pht_table[update_index_i] <= br_taken_i ? STRONGLY_TAKEN   : WEAKLY_TAKEN;
                endcase
            end
        end
    end

    // Async read
    assign br_prediction_o = pht_table[rd_index_i][1]; //Read MSB of pattern history bits.

endmodule
