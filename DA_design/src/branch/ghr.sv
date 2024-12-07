module ghr #(
    HISTORY_WIDTH = 4
)(
    input  logic                     clk_i, rst_i,       // Clock and active-high reset
    input  logic                     update_en_i,        // Decide whether to enable updating the table or not
    input  logic                     br_taken_i,         // The true branch decision to be updated into the ghr
    output logic [HISTORY_WIDTH-1:0] ghr_data_o          // The global history bits
);  

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            ghr_data_o <= 0;
        end else begin
            if (update_en_i) begin
                ghr_data_o <= {ghr_data_o[HISTORY_WIDTH-2:0], br_taken_i};
            end
        end
    end

endmodule
