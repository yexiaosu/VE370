`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/11/21 14:57:03
// Design Name: 
// Module Name: cache
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


module Cache #(
    parameter   index_exp  = 2,                         // Total blocks in cache = 2^index_exp
    parameter   words_exp  = 2,                         // Size (words) in each block = 2^words_exp
    parameter   addr_width = 10,                        // Width of address
    ////////////////////////////////////////////////////// You don't need to modify the following.~
    parameter   index_rows = 2 ** index_exp,                            // 4
    parameter   tag_length = addr_width - index_exp - words_exp  - 2,   // 4
    parameter   cache_width = 32 * (2 ** words_exp) + tag_length + 2,   // 134
    parameter   n = cache_width, r = addr_width, tag = tag_length
)(
    input                           done,               // From main memory
    input                           write_in,           // From CPU request, 1'b1 for write, 1'b0 for read 
    input       [2:0]               funct,              // From CPU request, for differentiating lw,lb (,lbu )
                                                        // 000 for lb/sb, 010 for lw/sw
    input       [addr_width-1:0]    rqst_addr,          // From CPU request
    input       [31:0]              read_data_in,       // From main memory
    input       [31:0]              write_data_in,      // From CPU request
    
    output                          hit,                // To CPU request, 1'b1 for hit,   1'b0 for miss 
    output reg                      write_out,          // To main memory, 1'b1 for write, 1'b0 for read 
    output      [31:0]              read_data_out,      // To CPU request
    output reg  [31:0]              write_data_out,     // To main memory
    output reg  [addr_width-1:0]    addr_out            // To main memory
);
    
    reg         [cache_width-1:0]   cacheA       [index_rows-1:0];
    ////////////////////////////////////////////////////// 6 MSB of cache is V(1'b)+D(1'b)+Tag(4'b)

    wire                            equal_A;
    wire        [31:0]              lw_out, lb_out;  // , lbu_out;
    wire        [cache_width-1:0]   block_A;
    
    ////////////////////////////////////////////////////// for simplicity
    wire        [1:0]               blockIndex;
    wire        [3:0]               tagContent;
    wire                            valid, dirty;

    integer i;
    initial begin
        write_out = 0; write_data_out = 0; addr_out = 0;
        for (i = 0; i < index_rows; i = i + 1) begin
            cacheA[i] = 0;
        end
    end
    
    assign  blockIndex = rqst_addr[5:4];
    assign  block_A = cacheA[blockIndex];
    assign  tagContent = block_A[131:128];
    assign  equal_A = (block_A[(n-3)-: tag] == rqst_addr[(r-1)-: tag]);
    assign  valid = block_A[n-1];
    assign  dirty = block_A[n-2];
    
    ////////////////////////////////////////////////////// To CPU request
    and (hit, valid, equal_A);

    _N_bit_4to1_MUX #(.N(32)) Mux_CacheOut1(.data1(block_A[31-: 32]), .data2(block_A[63-: 32]), 
    .data3(block_A[95-: 32]), .data4(block_A[127-:32]), .sel(rqst_addr[3:2]), .result(lw_out));

    _N_bit_4to1_MUX #(.N(32)) Mux_Cache_lbu(.data1({{24{1'b0}}, lw_out[7:0]}), 
    .data2({{24{1'b0}}, lw_out[15:8]}), .data3({{24{1'b0}}, lw_out[23:16]}), 
    .data4({{24{1'b0}}, lw_out[31:24]}), .sel(rqst_addr[1:0]), .result(lb_out));

//    _N_bit_4to1_MUX #(.N(32)) Mux_Cache_lb (.data1({{24{lw_out[7]}}, lw_out[7:0]}), 
//    .data2({{24{lw_out[7]}}, lw_out[15:8]}), .data3({{24{lw_out[7]}}, lw_out[23:16]}), 
//    .data4({{24{lw_out[7]}}, lw_out[31:24]}), .sel(rqst_addr[1:0]), .result(lbu_out));

    _N_bit_8to1_MUX #(.N(32)) Mux_Data_Out (.data1(lb_out), .data3(lw_out), .data4(), //.data4(lbu_out), 
    .data2(), .data5(), .data6(), .data7(), .data8(),  .sel(funct), .result(read_data_out));

    ////////////////////////////////////////////////////// To main memory
    always @ (*) begin
        #2
        if (write_in) begin // sw/sb
            if (!hit) begin // miss
                // Write data from cache to memory if dirty
                if (dirty) begin i = 31;
                    addr_out = {{tagContent,blockIndex},{4{1'b0}}};
                    write_out = 1'b1;
//                    addr_out = {rqst_addr[9:4],{4{1'b0}}};
                    for (i = 31; i <128; i = i + 32) begin
                        write_data_out = block_A[i-: 32];
                        @(posedge done) begin
                        addr_out = addr_out + 4;
                        end
                    end
                    #5;
//                    write_data_out = block_A[i-: 32];
//                    repeat (3) begin i = i + 32; 
//                        @(posedge done) begin
//                            #4 addr_out = addr_out + 4;
//                            write_data_out = block_A[i-: 32];
//                        end
//                    end 
                end i = 31;
                // Read data from memory to cache anyway
                addr_out = {rqst_addr[9:4],{4{1'b0}}};
                write_out = 1'b0;
                for (i = 31; i <128; i = i + 32) begin
                    @(posedge done) begin
                    cacheA[blockIndex][i-: 32] = read_data_in;
                    addr_out = addr_out + 4;
                    end
                end
//                #4 cacheA[blockIndex][i-: 32] = read_data_in;
//                repeat (3) begin i = i + 32; 
//                    @(posedge done) begin
//                        #4 addr_out = addr_out + 4;
//                        cacheA[blockIndex][i-: 32] = read_data_in;
//                    end
//                end 
                cacheA[blockIndex][n-1] = 1'b1; // Set Valid to True 
                if (funct == 3'b000) begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                           cacheA[blockIndex][i-: 8] = write_data_in[7:0]; end  // sb
                else if (funct == 3'b010) begin i = 31 + 32 * rqst_addr[3:2]; 
                                                cacheA[blockIndex][i-: 32] = write_data_in; end  // sw
                else begin i = 31 + 32 * rqst_addr[3:2]; 
                           cacheA[blockIndex][i-: 32] = write_data_in; end
                cacheA[blockIndex][n-2] = 1'b1; // Mark cacheA dirty
                cacheA[blockIndex][(n-3)-: tag] = rqst_addr[9-: tag];
            end
            else if (hit) begin
                if (funct == 3'b000) begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                           cacheA[blockIndex][i-: 8] = write_data_in[7:0]; end  // sb
                else if (funct == 3'b010) begin i = 31 + 32 * rqst_addr[3:2]; 
                                                cacheA[blockIndex][i-: 32] = write_data_in; end  // sw
                else begin i = 31 + 32 * rqst_addr[3:2]; 
                           cacheA[blockIndex][i-: 32] = write_data_in; end
                cacheA[blockIndex][n-2] = 1'b1; // Mark cacheA dirty
                cacheA[blockIndex][(n-3)-: tag] = rqst_addr[9-: tag];
            end
        end
        else begin // read_out  // lw/lb
            if (!hit) begin 
                // Write data from cache to memory if dirty
                if (dirty) begin i = 31;
                    addr_out = {{tagContent,blockIndex},{4{1'b0}}};
                    write_out = 1'b1;
//                    addr_out = {rqst_addr[9:4],{4{1'b0}}};
                    for (i = 31; i <128; i = i + 32) begin
                        write_data_out = block_A[i-: 32];
                        @(posedge done) begin
                        addr_out = addr_out + 4;
                        end
                    end
                    #5;
//                    write_data_out = block_A[i-: 32];
//                    repeat (3) begin i = i + 32; 
//                        @(posedge done) begin
//                            #4 addr_out = addr_out + 4;
//                            write_data_out = block_A[i-: 32];
//                        end
//                    end 
                end i = 31;
                // Read data from memory to cache anyway
                addr_out = {rqst_addr[9:4],{4{1'b0}}};
                write_out = 1'b0;
                for (i = 31; i <128; i = i + 32) begin
                    @(posedge done) begin
                    cacheA[blockIndex][i-: 32] = read_data_in;
                    addr_out = addr_out + 4;
                    end
                end
//                #4 cacheA[blockIndex][i-: 32] = read_data_in;
//                repeat (3) begin i = i + 32; 
//                    @(posedge done) begin
//                        #4 addr_out = addr_out + 4;
//                        cacheA[blockIndex][i-: 32] = read_data_in;
//                    end
//                end 
                cacheA[blockIndex][n-2] = 1'b0; // Mark as NOT dirty
                cacheA[blockIndex][n-1] = 1'b1; // Set Valid to True 
                cacheA[blockIndex][(n-3)-: tag] = rqst_addr[9-: tag];
            end
            else if (hit) begin
                cacheA[blockIndex][(n-3)-: tag] = rqst_addr[9-: tag];
            end
        end
    end        

endmodule

module _N_bit_2to1_MUX #(
    parameter   N = 32
)(
    input                   sel,
    input       [N-1:0]     data1, 
    input       [N-1:0]     data2,
    
    output reg  [N-1:0]     result
);

    initial begin
        result = 0;
    end

    always @ (*) begin
        case (sel)
            1'b0:   result = data1;
            1'b1:   result = data2;
            default:    result = 0;
        endcase
    end
    
endmodule

module _N_bit_4to1_MUX #(
    parameter   N = 32
)(
    input       [1:0]       sel,
    input       [N-1:0]     data1, 
    input       [N-1:0]     data2, 
    input       [N-1:0]     data3, 
    input       [N-1:0]     data4,
    
    output reg  [N-1:0]     result
);

    initial begin
        result = 0;
    end

    always @ (*) begin
        case (sel)
            2'b00:  result = data1;
            2'b01:  result = data2;
            2'b10:  result = data3;
            2'b11:  result = data4;
            default:    result = 0;
        endcase
    end
    
endmodule

module _N_bit_8to1_MUX #(
    parameter   N = 32
)(
    input       [1:0]       sel,
    input       [N-1:0]     data1, 
    input       [N-1:0]     data2, 
    input       [N-1:0]     data3, 
    input       [N-1:0]     data4,
    input       [N-1:0]     data5, 
    input       [N-1:0]     data6, 
    input       [N-1:0]     data7, 
    input       [N-1:0]     data8,
    
    output reg  [N-1:0]     result
);

    initial begin
        result = 0;
    end

    always @ (*) begin
        case (sel)
            3'b000: result = data1;
            3'b001: result = data2;
            3'b010: result = data3;
            3'b011: result = data4;
            3'b100: result = data5;
            3'b101: result = data6;
            3'b110: result = data7;
            3'b111: result = data8;
            default:    result = 0;
        endcase
    end
    
endmodule


module main_mem (
	input					read_or_write,//1'b1 for read, 1'b0 for write
	input			[9:0]	address,
	input			[31:0]	write_data,
	output reg  	[31:0]	read_data,
	output reg             done
);
	reg 	[7:0] 	main_memory [1023:0];
//	wire read_or_write_withdelay=read_or_write;
	integer i;
	
	initial begin
		//wait for initial data
		for(i=0;i<1024;i=i+1) main_memory[i]=i;
		done=1'b0;
	end
	
	//first read
	always @(read_or_write or address) begin
	    #3
	    repeat (4) begin
        if (read_or_write == 1'b1) begin
			main_memory[{address[9:2],2'b00}] = write_data[7:0];
			main_memory[{address[9:2],2'b01}] = write_data[15:8];
			main_memory[{address[9:2],2'b10}] = write_data[23:16];
			main_memory[{address[9:2],2'b11}] = write_data[31:24];
		end
		
	
    //then write
		if (read_or_write == 1'b0) begin
			  read_data[7:0]   = main_memory[{address[9:2], 2'b00}];
              read_data[15:8]  = main_memory[{address[9:2], 2'b01}];
              read_data[23:16] = main_memory[{address[9:2], 2'b10}];
              read_data[31:24] = main_memory[{address[9:2], 2'b11}]; 
		end
		#2 done=1'b1;
		#2 done=1'b0;
        end 
	end
endmodule

module CPU (
    input  hit_miss,
    input  clock,
    output read_write,
    output [2:0] funct,
    output [9:0] address,
    output [31:0] write_data
);
    parameter  request_total = 6; // change this number to how many requests you want in your testbench
    reg [4:0]  request_num;
    reg        read_write_test[request_total-1:0];
    reg [2:0]  funct_test[request_total-1:0];
    reg [9:0]  address_test[request_total-1:0];
    reg [31:0] write_data_test[request_total-1:0]; 
    initial begin
        request_num = 0;
        read_write_test[0] = 0; funct_test[0] = 3'b010; address_test[0] = 10'b0110101001; write_data_test[0] = 0; // lw
        read_write_test[1] = 1; funct_test[1] = 3'b010; address_test[1] = 10'b0110010101; write_data_test[1] = 10'hfac; // sw
        read_write_test[2] = 0; funct_test[2] = 3'b000; address_test[2] = 10'b0110011001; write_data_test[2] = 0; // lb
        read_write_test[3] = 1; funct_test[3] = 3'b000; address_test[3] = 10'b0101010100; write_data_test[3] = 10'hfff; // sb
        read_write_test[4] = 1; funct_test[4] = 3'b010; address_test[4] = 10'b0101011100; write_data_test[4] = 10'haaa; // sw
        read_write_test[5] = 0; funct_test[5] = 3'b010; address_test[5] = 10'b0110010101; write_data_test[5] = 0; // lw
      
        /* add lines if necessary */
        
        
    end
    always @(posedge clock) begin
        if (hit_miss == 1) request_num = request_num + 1;
        else request_num = request_num;
    end
    assign address      = address_test[request_num];
    assign funct        = funct_test[request_num];
    assign read_write   = read_write_test[request_num];
    assign write_data   = write_data_test[request_num]; 
endmodule
