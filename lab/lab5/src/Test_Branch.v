`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/11/02 20:40:21
// Design Name: 
// Module Name: Test_Branch
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Test_Branch;
    reg clk; reg PC_en;
    reg [4:0] reg_address;
    wire [31:0] data_out;
    
	Pipeline_Plus test (
		.clk(clk), .Display_in(reg_address), .PC_en(PC_en), .Display_out(data_out)
	);

	initial begin
		clk = 0; PC_en = 1; reg_address = 5'b00001; // Initialize Inputs
	end

    initial begin
        while ($time < 60) @(posedge clk)begin
            $display("===============================================");
            $display("Clock cycle %d, PC = %H", $time/2, test.PC);
            $display("ra = %H, t0 = %H, t1 = %H", test.RF.data[1], test.RF.data[5], test.RF.data[6]);
            $display("t2 = %H, t3 = %H, t4 = %H", test.RF.data[7], test.RF.data[28], test.RF.data[29]);
            $display("===============================================");
        end
        $finish();
    end
    always #1 clk = ~clk;

endmodule
