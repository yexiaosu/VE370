`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/11/22 16:06:48
// Design Name: 
// Module Name: tb
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


module tb_cache;
    reg          clk;
    
    // interface between cache and CPU
    wire         write_in; /* 1 if write, 0 if read */
    wire [2:0]   funct;
    wire [9:0]   rqst_addr;
    wire [31:0]  write_data_in, read_data_out;
    // interface between cache and main memory
    wire done, write_out;
    wire [31:0] read_data_in, write_data_out;
    wire [9:0] addr_out;
    
    Cache cache (.done(done),
                 .write_in(write_in),
                 .funct(funct),
                 .rqst_addr(rqst_addr),
                 .read_data_in(read_data_in),
                 .write_data_in(write_data_in),
                 .hit(hit),
                 .write_out(write_out),
                 .read_data_out(read_data_out), 
                 .write_data_out(write_data_out),
                 .addr_out(addr_out)
                 );
    main_mem mem (.read_or_write(write_out),
                  .address(addr_out),
                  .write_data(write_data_out),
                  .read_data(read_data_in),
                  .done(done)
                  );
    CPU cpu (.hit_miss(hit),
                .clock(clk),
                .read_write(write_in),
                .funct(funct),
                .address(rqst_addr),
                .write_data(write_data_in)
                );
    
    // CPU
    wire [4:0]  request_num=cpu.request_num;
    wire        read_write_test[4:0]=cpu.read_write_test;
    wire [2:0]  funct_test[4:0]=cpu.funct_test;
    wire [9:0]  address_test[4:0]=cpu.address_test;
    wire [31:0] write_data_test[4:0]=cpu.write_data_test;
    
    // cache
    wire        [133:0]   cacheA       [3:0]=cache.cacheA;
    wire [1:0]  blockIndex=cache.blockIndex;
    wire        valid=cache.valid, dirty=cache.dirty;
    wire                            equal_A=cache.equal_A;
    wire        [31:0]              lw_out=cache.lw_out, lb_out=cache.lb_out;
    wire        [133:0]   block_A=cache.block_A;
    
    // memory
    wire 	[7:0] 		main_memory [1023:0]=mem.main_memory;
//	wire read_or_write_withdelay=mem.read_or_write_withdelay;
    
    initial begin
        clk = 0;
    end
    
    initial begin
        while ($time < 200) @(request_num)begin
            $display("===============================================");
            $display("Cache status:");
            $display("cache[0] = %H", cacheA[0]);
            $display("cache[1] = %H", cacheA[1]);
            $display("cache[2] = %H", cacheA[2]);
            $display("cache[3] = %H", cacheA[3]);
            $display("Read/Write data of last request: %H", read_data_out);
            $display("RequestNum: %d, do: %s %s", request_num, (write_data_test[request_num]==0)?"load":"store", (funct_test[request_num] == 3'b010)?"word":"byte");
            #1
            $display("From/To: %b", rqst_addr);
            $display("Block Index = %d", blockIndex);
            $display("hit = %d", hit);
            $display("Dirty? %s", (dirty==0)?"No":"Yes");
            #5
            $display("Write back? %s", (write_out==0)?"No":"Yes");
            $display("Write data = %H", write_data_in);
            $display("Block data form memory = %H%H%H%H %H%H%H%H %H%H%H%H %H%H%H%H", main_memory[{rqst_addr[9:4],4'b1111}], main_memory[{rqst_addr[9:4],4'b1110}], main_memory[{rqst_addr[9:4],4'b1101}], main_memory[{rqst_addr[9:4],4'b1100}], main_memory[{rqst_addr[9:4],4'b1011}], main_memory[{rqst_addr[9:4],4'b1010}], main_memory[{rqst_addr[9:4],4'b1001}], main_memory[{rqst_addr[9:4],4'b1000}], main_memory[{rqst_addr[9:4],4'b0111}], main_memory[{rqst_addr[9:4],4'b0110}], main_memory[{rqst_addr[9:4],4'b0101}], main_memory[{rqst_addr[9:4],4'b0100}], main_memory[{rqst_addr[9:4],4'b0011}], main_memory[{rqst_addr[9:4],4'b0010}], main_memory[{rqst_addr[9:4],4'b0001}], main_memory[{rqst_addr[9:4],{4{1'b0}}}]);
            $display("===============================================");
            $display("===============================================");
            $display("Change in memory:");
            $display("(word, byte index ignored)Memory[01100101] = %H%H%H%H", main_memory[407], main_memory[406], main_memory[405], main_memory[404]);
            $display("(word, byte index ignored)Memory[01010101] = %H%H%H%H", main_memory[343], main_memory[342], main_memory[341], main_memory[340]);
            $display("===============================================");
        end
        $finish();
    end
    
    always #1 clk = ~clk;
endmodule
