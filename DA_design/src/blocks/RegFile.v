module RegFile (
    input         clk_i,
    input         rst_i,
	input  [4:0]  ReadReg1_i,
	input  [4:0]  ReadReg2_i,
	input  [4:0]  WriteReg_i,
	input  [31:0] WriteData_i,
	input         RegWrEn_i,
	output [31:0] ReadData1_o,
	output [31:0] ReadData2_o
);

    reg [31:0] Registers[0:31];

    ////////////////////////////////////////////////////////////////////////
    // Write on posedge clk_i
    always @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            Registers[0] <= 32'd0;
            Registers[1] <= 32'd0;
            Registers[2] <= 32'd0;
            Registers[3] <= 32'd0;
            Registers[4] <= 32'd0;
            Registers[5] <= 32'd0;
            Registers[6] <= 32'd0;
            Registers[7] <= 32'd0;
            Registers[8] <= 32'd0;
            Registers[9] <= 32'd0;
            Registers[10] <= 32'd0;
            Registers[11] <= 32'd0;
            Registers[12] <= 32'd0;
            Registers[13] <= 32'd0;
            Registers[14] <= 32'd0;
            Registers[15] <= 32'd0;
            Registers[16] <= 32'd0;
            Registers[17] <= 32'd0;
            Registers[18] <= 32'd0;
            Registers[19] <= 32'd0;
            Registers[20] <= 32'd0;
            Registers[21] <= 32'd0;
            Registers[22] <= 32'd0;
            Registers[23] <= 32'd0;
            Registers[24] <= 32'd0;
            Registers[25] <= 32'd0;
            Registers[26] <= 32'd0;
            Registers[27] <= 32'd0;
            Registers[28] <= 32'd0;
            Registers[29] <= 32'd0;
            Registers[30] <= 32'd0;
            Registers[31] <= 32'd0;
        end
        else begin
            if ((RegWrEn_i) && (!(WriteReg_i == 5'b0_0000))) Registers[WriteReg_i] <= WriteData_i;
        end
    end

    // Output pins with forwarding network
    assign ReadData1_o = (RegWrEn_i && (WriteReg_i == ReadReg1_i)) ? WriteData_i : Registers[ReadReg1_i];
    assign ReadData2_o = (RegWrEn_i && (WriteReg_i == ReadReg2_i)) ? WriteData_i : Registers[ReadReg2_i];

endmodule
