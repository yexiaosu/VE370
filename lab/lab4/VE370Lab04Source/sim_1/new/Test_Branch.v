`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UMJI
// Engineer: CWQ
// 
// Create Date: 2021/10/29 21:20:40
// Design Name: 
// Module Name: Test Branch
// Project Name: Lab04 Pipeline Processor
// Target Devices: FPGA Basys3
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
//    parameter half_period = 25;
//    reg clock;
    
//    Pipeline_Processor PP (clock); 
    
//    initial begin
//        $display ("*********************************");
//        $display (" The textual simulation results: ");
//        $display ("*********************************");
//        $monitor ($time, " clock = %d, SCP.PC = %d, SCP.data_1 = %h, SCP.data_2 = %h", clock, SCP.PC, SCP.data_1, SCP.data_2);
//    end

//    initial begin clock = 0; end
//    always  #half_period clock = ~clock;
//    initial #900 $stop;
    reg clk;
	Pipeline_Processor test (
		clk
	);

	initial begin
		clk = 0; // Initialize Inputs
	end

    initial begin
        while ($time < 200) @(posedge clk)begin
            $display("===============================================");
            $display("Time: %d, CLK = %d, PC = %H", $time/2, clk, test.PC);
            $display("[ra] = %H, [sp] = %H, [$t0] = %H", test.RF.data[1], test.RF.data[2], test.RF.data[5]);
            $display("[$t1] = %H, [$t2] = %H, [$t3] = %H", test.RF.data[6], test.RF.data[7], test.RF.data[28]);
            $display("[$t4] = %H, [$t5] = %H, [$t6] = %H", test.RF.data[29], test.RF.data[30], test.RF.data[31]);
            $display("[$s0] = %H, [$s1] = %H, [$s2] = %H", test.RF.data[9], test.RF.data[18], test.RF.data[19]);
            $display("[$s3] = %H, [$s4] = %H, [$s5] = %H", test.RF.data[20], test.RF.data[21], test.RF.data[22]);
            $display("[$a0] = %H, [$a1] = %H, [$a2] = %H", test.RF.data[10], test.RF.data[11], test.RF.data[12]);
            $display("===============================================");
        end
        $finish();
    end
    always #1 clk = ~clk;
    
endmodule
