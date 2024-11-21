module DataMem (
    input clk_i,

    input [31:0] Address_i,
    input [31:0] WriteData_i,

    input ReadEn_i,
    input WriteEn_i,

    output reg [31:0] Data_o
);

reg [7:0] DataMemory[0:1023];

// Write on negedge clk_i
// Little-endian
always @(negedge clk_i) begin
    if (WriteEn_i==1'b1) begin
        DataMemory[Address_i+3] <= WriteData_i[31:24];
        DataMemory[Address_i+2] <= WriteData_i[23:16];
        DataMemory[Address_i+1] <= WriteData_i[15:8];
        DataMemory[Address_i] <= WriteData_i[7:0];
    end
    $writememh("DataMem.mem", DataMemory);  // For simulation only
end
  
always @(*) begin
    if (ReadEn_i==1'b1) Data_o = {DataMemory[Address_i+3], DataMemory[Address_i+2], DataMemory[Address_i+1], DataMemory[Address_i]};
    else Data_o = 32'h00000000;
end

endmodule
