`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Group Four
// Engineer: CWQ, WL, & LJY
// 
// Create Date: 2021/11/26 13:41:01
// Design Name: 
// Module Name: Cache_Final
// Project Name: Lab 7 Virtual Memory
// Target Devices: FPGA
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

module New_Cache #(
    parameter   index_exp  = 1,                         // Total blocks in cache = 2^index_exp
    parameter   words_exp  = 2,                         // Size (words) in each block = 2^words_exp
    parameter   addr_width = 10,                        // Width of address
    ////////////////////////////////////////////////////// You don't need to modify the following.~
    parameter   index_rows = 2 ** index_exp,                            // 2
    parameter   tag_length = addr_width - index_exp - words_exp  - 2,   // 5
    parameter   cache_width = 32 * (2 ** words_exp) + tag_length + 2,   // 135
    parameter   n = cache_width, r = addr_width, tag = tag_length
)(
    input                           done,               // From main memory
    input                           write_in,           // From CPU request, 1'b1 for write, 1'b0 for read
    input                           addr_prepared,      // From TLB address
    input       [2:0]               funct,              // From CPU request, for differentiating lw,lb (,lbu )
                                                        // 000 for lb/sb, 010 for lw/sw
    input       [addr_width-1:0]    rqst_addr,          // From physicalTLB 
    input       [31:0]              read_data_in,       // From main memory
    input       [31:0]              write_data_in,      // From CPU request
    
    output                          hit,                // To CPU request, 1'b1 for hit,   1'b0 for miss 
    output reg                      write_out,          // To main memory, 1'b1 for write, 1'b0 for read 
    output      [31:0]              read_data_out,      // To CPU request
    output reg  [31:0]              write_data_out,     // To main memory
    output reg  [addr_width-1:0]    addr_out            // To main memory
);
    
    reg                             pos_done;           // NEW
    reg         [index_rows -1:0]   LRU;
    reg         [cache_width-1:0]   cache_setA          [index_rows-1:0];
    reg         [cache_width-1:0]   cache_setB          [index_rows-1:0];
    ////////////////////////////////////////////////////// 7 MSB of cache is V(1'b)+D(1'b)+Tag(5'b) // NEW

    wire                            hit_setA, hit_setB, equal_A, equal_B; // NEW
    wire        [31:0]              data_A, data_B, lw_out, lb_out; //, lbu_out;
    wire        [cache_width-1:0]   block_A, block_B; // NEW
    
    ////////////////////////////////////////////////////// for simplicity by WL and CWQ
    wire                            setIndex; // NEW (Change From 2 bits To 1 bit)
    wire        [tag-1:0]           tagContent_A, tagContent_B; // NEW (Change From 4 bits To 5 bits)
    wire                            valid_A, dirty_A, valid_B, dirty_B; // NEW

    integer i;
    initial begin
        pos_done = 0; addr_out = 0; 
        write_out = 0; write_data_out = 0; 
        for (i = 0; i < index_rows; i = i + 1) begin
            cache_setA[i] = 0; cache_setB[i] = 0; LRU[i] = 0;
        end
    end
    
    assign  setIndex = rqst_addr[4];
    assign  block_A = cache_setA[setIndex];
    assign  block_B = cache_setB[setIndex];
    assign  tagContent_A = block_A[(n-3)-: tag];
    assign  tagContent_B = block_B[(n-3)-: tag];
    assign  equal_A = (tagContent_A == rqst_addr[(r-1)-: tag]);
    assign  equal_B = (tagContent_B == rqst_addr[(r-1)-: tag]);
    assign  valid_A = block_A[n-1];
    assign  valid_B = block_B[n-1];
    assign  dirty_A = block_A[n-2];
    assign  dirty_B = block_B[n-2];
    
    ////////////////////////////////////////////////////// To CPU request
    or  (hit, hit_setA, hit_setB);
    and (hit_setA, valid_A, equal_A);
    and (hit_setB, valid_B, equal_B);

    _N_bit_4to1_MUX #(.N(32)) Mux_CacheOut1(.data1(block_A[31-: 32]), .data2(block_A[63-: 32]), 
    .data3(block_A[95-: 32]), .data4(block_A[127-:32]), .sel(rqst_addr[3:2]), .result(data_A));

    _N_bit_4to1_MUX #(.N(32)) Mux_CacheOut2(.data1(block_B[31-: 32]), .data2(block_B[63-: 32]), 
    .data3(block_B[95-: 32]), .data4(block_B[127-:32]), .sel(rqst_addr[3:2]), .result(data_B));
    
    _N_bit_4to1_MUX #(.N(32)) Mux_CacheData(.data1(lw_out), .data2(data_A), .data3(data_B), .data4(lw_out), 
    .sel({hit_setB, hit_setA}), .result(lw_out));
    
//    _N_bit_4to1_MUX #(.N(32)) Mux_Cache_lbu(.data1({{24{1'b0}}, lw_out[7:0]}), 
//    .data2({{24{1'b0}}, lw_out[15:8]}), .data3({{24{1'b0}}, lw_out[23:16]}), 
//    .data4({{24{1'b0}}, lw_out[31:24]}), .sel(rqst_addr[1:0]), .result(lbu_out));

    _N_bit_4to1_MUX #(.N(32)) Mux_Cache_lb (.data1({{24{lw_out[7]}}, lw_out[7:0]}), 
    .data2({{24{lw_out[7]}}, lw_out[15:8]}), .data3({{24{lw_out[7]}}, lw_out[23:16]}), 
    .data4({{24{lw_out[7]}}, lw_out[31:24]}), .sel(rqst_addr[1:0]), .result(lb_out));

    _N_bit_8to1_MUX #(.N(32)) Mux_Data_Out (.data1(lb_out), .data3(lw_out), .data4(), //.data4(lbu_out), 
    .data2(), .data5(), .data6(), .data7(), .data8(),  .sel(funct), .result(read_data_out));//////

    ////////////////////////////////////////////////////// To main memory
//    always @(posedge done) begin // NEW // PLEASE take care of this delay
//        #1
//        pos_done = 1'b1;
//        #2
//        pos_done = 1'b0;
//    end

    always @ (*) begin
        #2
        if (addr_prepared)begin
            if (write_in) begin // sw/sb
                if (!hit) begin // miss
                    if (LRU[setIndex]==1'b0) begin
                        // Write data from cache to memory if dirty
                        if (dirty_A) begin i = 31;
                            addr_out = {{tagContent_A,setIndex},{4{1'b0}}};
                            write_out = 1'b1;
                            for (i = 31; i <128; i = i + 32) begin
                                write_data_out = block_A[i-: 32];
                                @(posedge done) begin
                                    addr_out = addr_out + 4;
                                end
                            end
                            #5;
                        end i = 31;
                        // Read data from memory to cache anyway
                        addr_out = {rqst_addr[9:4],{4{1'b0}}};
                        write_out = 1'b0;
                        for (i = 31; i <128; i = i + 32) begin
                            @(posedge done) begin
                                cache_setA[setIndex][i-: 32] = read_data_in;
                                addr_out = addr_out + 4;
                            end
                        end
                        cache_setA[setIndex][n-1] = 1'b1; // Set Valid to True 
                        LRU[setIndex] = 1'b1; // Reset Least Resently Used 
                        case(funct)
                            3'b000: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setA[setIndex][i-: 8] = write_data_in[7:0]; end // sb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setA[setIndex][i-: 32] = write_data_in; end // sw
                        endcase
                        cache_setA[setIndex][n-2] = 1'b1; // Mark cache_setA dirty
                        cache_setA[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                    end 
                    else begin
                        // Write data from cache to memory if dirty
                        if (dirty_B) begin i = 31;
                            #1
                            addr_out = {{tagContent_B,setIndex},{4{1'b0}}};
                            write_out = 1'b1;
                            for (i = 31; i <128; i = i + 32) begin
                                write_data_out = block_B[i-: 32];
                                @(posedge done) begin
                                    addr_out = addr_out + 4;
                                end
                            end
                            #5;
                        end i = 31;
                        // Read data from memory to cache anyway
                        #1
                        addr_out = {rqst_addr[9:4],{4{1'b0}}};
                        write_out = 1'b0;
                        for (i = 31; i <128; i = i + 32) begin
                            @(posedge done) begin
                                cache_setB[setIndex][i-: 32] = read_data_in;
                                addr_out = addr_out + 4;
                            end
                        end
                        cache_setB[setIndex][n-1] = 1'b1; // Set Valid to True
                        LRU[setIndex] = 1'b0; // Reset Least Resently Used 
                        case(funct)
                            3'b000: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setB[setIndex][i-: 8] = write_data_in[7:0]; end // sb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setB[setIndex][i-: 32] = write_data_in; end // sw
                        endcase
                        cache_setB[setIndex][n-2] = 1'b1; // Mark cache_setB dirty
                        cache_setB[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                    end
                end
                else if (hit_setA) begin
                    case(funct)
                        3'b000: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                cache_setA[setIndex][i-: 8] = write_data_in[7:0]; end // sb
                        default:begin i = 31 + 32 * rqst_addr[3:2];
                                cache_setA[setIndex][i-: 32] = write_data_in; end // sw
                    endcase
                    cache_setA[setIndex][n-2] = 1'b1; // Mark cache_setA dirty
                    LRU[setIndex] = 1'b1; // Reset Least Resently Used to setB
                    cache_setA[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                end
                else if (hit_setB) begin
                    case(funct)
                        3'b000: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                cache_setB[setIndex][i-: 8] = write_data_in[7:0]; end // sb
                        default:begin i = 31 + 32 * rqst_addr[3:2];
                                cache_setB[setIndex][i-: 32] = write_data_in; end // sw
                    endcase
                    cache_setB[setIndex][n-2] = 1'b1; // Mark cache_setA dirty
                    LRU[setIndex] = 1'b0; // Reset Least Resently Used to setA
                    cache_setB[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                end
            end
            else begin // read_out  // lw/lb
                if (!hit) begin 
                    if (LRU[setIndex]==1'b0) begin
                        // Write data from cache to memory if dirty
                        if (dirty_A) begin i = 31;
                            addr_out = {{tagContent_A,setIndex},{4{1'b0}}};
                            write_out = 1'b1;
                            for (i = 31; i <128; i = i + 32) begin
                                write_data_out = block_A[i-: 32];
                                @(posedge done) begin
                                    addr_out = addr_out + 4;
                                end
                            end
                            #5;
                        end i = 31;
                        // Read data from memory to cache anyway
                        addr_out = {rqst_addr[9:4],{4{1'b0}}};
                        write_out = 1'b0;
                        for (i = 31; i <128; i = i + 32) begin
                            @(posedge done) begin
                                cache_setA[setIndex][i-: 32] = read_data_in;
                                addr_out = addr_out + 4;
                            end
                        end
                        cache_setA[setIndex][n-2] = 1'b0; // Mark as NOT dirty
                        cache_setA[setIndex][n-1] = 1'b1; // Set Valid to True 
                        LRU[setIndex] = 1'b1; // Reset Least Resently Used = B
                        cache_setA[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                    end
                    else begin
                        // Write data from cache to memory if dirty
                        if (dirty_B) begin i = 31;
                            addr_out = {{tagContent_B,setIndex},{4{1'b0}}};
                            write_out = 1'b1;
                            for (i = 31; i <128; i = i + 32) begin
                                write_data_out = block_B[i-: 32];
                                @(posedge done) begin
                                    addr_out = addr_out + 4;
                                end
                            end
                            #5;
                        end i = 31;
                        // Read data from memory to cache anyway
                        addr_out = {rqst_addr[9:4],{4{1'b0}}};
                        write_out = 1'b0;
                        for (i = 31; i <128; i = i + 32) begin
                            @(posedge done) begin
                                cache_setB[setIndex][i-: 32] = read_data_in;
                                addr_out = addr_out + 4;
                            end
                        end
                        cache_setB[setIndex][n-2] = 1'b0; // Mark as NOT dirty
                        cache_setB[setIndex][n-1] = 1'b1; // Set Valid to True 
                        LRU[setIndex] = 1'b0; // Reset Least Resently Used = A
                        cache_setB[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                    end
                end
                else if (hit_setA) begin
                    cache_setA[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                    LRU[setIndex] = 1'b1; // Reset Least Resently Used = B
                end
                else if (hit_setB) begin
                    cache_setB[setIndex][(n-3)-: tag] = rqst_addr[9-: tag];
                    LRU[setIndex] = 1'b0; // Reset Least Resently Used = A
                end
            end
        end
    end        

endmodule

module New_TLB #(
    parameter   V_addr_width = 14,
    parameter   P_addr_width = 10,
    parameter   Page_offset  = 8,
    parameter   TLB_row_num  = 4,
    ////////////////////////////////////////////////////// You don't need to modify the following.~
    parameter   VPN_size = V_addr_width - Page_offset,  // 6
    parameter   PPN_size = P_addr_width - Page_offset,  // 2
    parameter   V = VPN_size, P = PPN_size, VA = V_addr_width, PA = P_addr_width
)(
    input                           done,               // From PageTable, 1'b1 for done, 1'b0 for unfinished
    input       [V_addr_width-1:0]  Virtual_addr,       // From Processor
    input       [PPN_size-1:0]      P_addr_PT_in,       // From PageTable

    output                          TLB_hit,            // To Processor, 1'b1 for hit, 1'b0 for miss 
    output reg                      write_to_table,     // To PageTable, 1'b1 for write, 1'b0 for read
    output reg  [VPN_size-1:0]      V_addr_PT_out,      // To PageTable
    output reg  [PPN_size-1:0]      P_addr_PT_out,      // To PageTable
    output reg  [P_addr_width-1:0]  Physical_addr,      // To Cache Mem
    output reg                      Addr_prepared       // To Cache Mem
);

    reg         [15:0]      TLB     [TLB_row_num-1:0];  // 1bit_Valid, 1bit_Dirty, 2bit_Ref = TLB[15:12];
                                                        // Tag = TLB[9:4]; Physical Page Addr = TLB[1:0];
    ////////////////////////////////////////////////////// 4X2-Byte TLB
    
    wire        [TLB_row_num-1:0]   Hit;                // whether [Tag == VPN] && Valid in this row
    wire        [VPN_size-1:0]      VPN;                // Data of Virtual Page Number, in Processor
    
    ////////////////////////////////////////////////////// Initialization
   
    integer i, j, k;
    initial begin
        V_addr_PT_out = 0; P_addr_PT_out = 0; 
        Physical_addr = 0; Addr_prepared = 0;
        write_to_table = 0;
        for (i = 0; i < TLB_row_num; i = i + 1) begin///
            TLB[i] = 0;
            TLB[i][13:12]=i;
        end
    end
    
    assign VPN = Virtual_addr[(VA-1)-: V];
    assign Hit[0] = (VPN == TLB[0][9:4]) && TLB[0][15];
    assign Hit[1] = (VPN == TLB[1][9:4]) && TLB[1][15];
    assign Hit[2] = (VPN == TLB[2][9:4]) && TLB[2][15];
    assign Hit[3] = (VPN == TLB[3][9:4]) && TLB[3][15];

    ////////////////////////////////////////////////////// To PageTable
    or (TLB_hit, Hit[0], Hit[1], Hit[2], Hit[3]);
    
    always @(*) begin
        if (!TLB_hit) begin 
            Addr_prepared = 1'b0;
            for (i = 0; i < TLB_row_num; i = i + 1) begin // LRU_sel
                if (TLB[i][13:12] == 2'b11) begin
                    // Write physical_addr from TLB to Page Table if dirty
                    if (TLB[i][14]) begin // If the certain block is dirty
                        V_addr_PT_out = TLB[i][9:4];
                        P_addr_PT_out = TLB[i][1:0];
                        write_to_table = 1'b1;
                    end 
                    // Read physical_addr from Page Table to TLB anyway
                    write_to_table = 1'b0;
                    V_addr_PT_out = VPN;
                    # 1
                    if (done) begin
                        TLB[i][1:0] = P_addr_PT_in;
                        TLB[i][9:4] = VPN; // Reset P_addr in TLB[i]
                    end
                    TLB[i][14] = 1'b0; // Mark as NOT dirty
                    TLB[i][15] = 1'b1; // Set Valid to True 
//                    for (j = 0; j < TLB_row_num; j = j + 1) begin // LRU_sel
//                        TLB[j][13:12] = TLB[j][13:12] + 2'b01;
//                    end
//                    TLB[i][13:12] = 2'b00; // Reset 2-bit LRU 
                end
            end
        end
        ////////////////////////////////////////////////////// To Cache Mem
        else begin
            Physical_addr[Page_offset-1:0] = Virtual_addr[Page_offset-1:0];
            case (Hit)
                4'b0001:  begin Physical_addr[(PA-1)-: P] = TLB[0][1:0];
                    k = 0;
                end
                4'b0010:  begin Physical_addr[(PA-1)-: P] = TLB[1][1:0];
                    k = 1;
                end
                4'b0100:  begin Physical_addr[(PA-1)-: P] = TLB[2][1:0];
                    k = 2;
                end
                4'b1000:  begin Physical_addr[(PA-1)-: P] = TLB[3][1:0];
                    k = 3;
                end
                default:  Physical_addr[(PA-1)-: P] = Physical_addr[(PA-1)-: P];
            endcase
            Addr_prepared = 1'b1;
            for (j = 0; j < 4; j = j + 1) begin
                if (TLB[j][13:12] < TLB[k][13:12]) begin
                    TLB[j][13:12] = TLB[j][13:12] + 2'b01;
                end
            end
            TLB[k][13:12] = 2'b00;
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
    input       [2:0]       sel,
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
    //1'b1 for read, 1'b0 for write
	input			    read_or_write,
	input       [9:0]	address,
	input       [31:0]	write_data,
                   
    output reg          done,
	output reg  [31:0]  read_data

);
	reg 	    [31:0] 	main_memory [255:0];

	integer i;
	initial begin
		//wait for initial data
		for(i = 0; i < 256; i = i + 1) begin 
            main_memory[i] = 4*i+1; 
        end
		done = 1'b0;
	end
	
	//first read
	always @(read_or_write or address) begin
	    #3
	    repeat (4) begin
            if (read_or_write == 1'b1) begin
                main_memory[address[9:2]] = write_data[31:0];
            end
	
    //then write
            if (read_or_write == 1'b0) begin
                read_data[31:0] = main_memory[address[9:2]];
            end
		    #2 done = 1'b1;
		    #2 done = 1'b0;
        end 
	end

endmodule

module page_table (
    //1'b1 for read, 1'b0 for write
	input			    read_from_TLB,
	input       [1:0]	phy_page_num_in,
    input       [5:0]	vir_page_num_in,
                   
    output reg          done,
	output reg  [1:0]   phy_page_num,

    output reg          page_fault
);
	reg 	    [4:0] 	page_table [63:0];

	integer i;
	initial begin
		//wait for initial data
		for(i = 0; i < 64; i = i + 1) begin 
            page_table[i][2] = 0;  // ref
            page_table[i][3] = 0;  // dirty
            page_table[i][4] = 0;  // valid
        end
        
        page_table[4][1:0] = 2;
        page_table[0][1:0] = 1;
        page_table[1][1:0] = 3;
        page_table[7][1:0] = 1;
        page_table[8][1:0] = 1;
        page_table[10][1:0] = 1;
        
        page_table[4][4] = 1;
        page_table[0][4] = 1;
        page_table[1][4] = 1;
        page_table[7][4] = 1;
        page_table[8][4] = 1;
        page_table[10][4] = 1;

		done = 1'b0; page_fault = 1'b0;
	end
	
	//always @(read_from_TLB or address) begin
	always @(*) begin
	    # 0.75
            if (read_from_TLB == 1'b1) begin
                page_table[vir_page_num_in][1:0] = phy_page_num_in;
                page_table[vir_page_num_in][2] = 1;  // ref
                page_table[vir_page_num_in][3] = 1;  // dirty
            end
	
            if (read_from_TLB == 1'b0) begin
                //phy_page_num = main_memory[vir_page_num_in][1:0];
                phy_page_num = page_table[vir_page_num_in][1:0];
                page_table[vir_page_num_in][2] = 1;  // ref
                page_fault=1-page_table[vir_page_num_in][4];//
            end
		    done = 1'b1;
		    #1 done = 1'b0;
	end

endmodule

module CPU # (
    // change this number to how many requests you want in your testbench
    parameter      request_total = 13
)(
    input           clock,
    input           hit_miss,
    output          read_write,
    output [2:0]    funct,
    output [13:0]   address,
    output [31:0]   write_data
);
    
    reg    [4:0]    request_num;
    reg    [2:0]    funct_test[request_total-1:0];
    reg    [13:0]   address_test[request_total-1:0]; //change to 14 bits
    reg             read_write_test[request_total-1:0];
    reg    [31:0]   write_data_test[request_total-1:0];
    
    assign funct        = funct_test[request_num];
    assign address      = address_test[request_num];
    assign read_write   = read_write_test[request_num];
    assign write_data   = write_data_test[request_num]; 
    
    initial begin
    //change the num of address test
        request_num = 0;
        read_write_test[0]  = 1; funct_test[0] = 3'b010;address_test[0]  = 14'b000100_100_0_1000; write_data_test[0]  = 1;       // sw, virtual page  4, TLB miss, mapped to physical page 2, physical tag 10100, cache miss in set 0 block 0,
        read_write_test[1]  = 1; funct_test[1] = 3'b010;address_test[1]  = 14'b000000_100_1_1100; write_data_test[1]  = 12'hdac; // sw, virtual page  0, TLB miss, mapped to physical page 1, physical tag 01100, cache miss in set 1 block 0,
        read_write_test[2]  = 1; funct_test[2] = 3'b010;address_test[2]  = 14'b000001_100_1_1000; write_data_test[2]  = 12'hfac; // sw, virtual page  1, TLB miss, mapped to physical page 3, physical tag 11100, cache miss in set 1 block 1,
        read_write_test[3]  = 1; funct_test[3] = 3'b000;address_test[3]  = 14'b000000_100_1_0101; write_data_test[3]  = 12'hfac; // sb, virtual page  0, TLB hit,  mapped to physical page 1, physical tag 01100, cache hit  in set 1 block 0,
        read_write_test[4]  = 0; funct_test[4] = 3'b000;address_test[4]  = 14'b000111_100_1_0101; write_data_test[4]  = 0;       // lb, virtual page  7, TLB miss, mapped to physical page 1, physical tag 01100, cache hit  in set 1 block 0,
        read_write_test[5]  = 0; funct_test[5] = 3'b000;address_test[5]  = 14'b001000_110_1_0101; write_data_test[5]  = 0;       // lb, virtual page  8, TLB miss, mapped to physical page 1, virtual page 4 replaced, write back entry with virtual tag 4,
                                                                                                         //                                                           physical tag 01110, cache miss in set 1, set 1 block 1 replaced and write back
        read_write_test[6]  = 0; funct_test[6] = 3'b010;address_test[6]  = 14'b000001_110_1_0100; write_data_test[6]  = 0;       // lw, virtual page  1, TLB hit,  mapped to physical page 3, physical tag 11110, cache miss in set 1, set 1 block 0 replaced and write back
        read_write_test[7]  = 1; funct_test[7] = 3'b000;address_test[7]  = 14'b000111_100_1_0111; write_data_test[7]  = 12'h148; // sb, virtual page  7, TLB hit,  mapped to physical page 1, physical tag 01100, cache miss in set 1, set 1 block 1 replaced
        read_write_test[8]  = 0; funct_test[8] = 3'b010;address_test[8]  = 14'b000000_100_1_1000; write_data_test[8]  = 0;       // lw, virtual page  0, TLB hit,  mapped to physical page 1, physical tag 01100, cache hit  in set 1 block 1,
        read_write_test[9]  = 0; funct_test[9] = 3'b010;address_test[9]  = 14'b001010_100_1_0100; write_data_test[9]  = 0;       // lw, virtual page 10, TLB miss, mapped to physical page 1, virtual page 8 replaced, write back entry with virtual tag 8,
                                                                                                                                                                  //  physical tag 01100, cache hit  in set 1 block 1,
        read_write_test[10] = 0; funct_test[10] = 3'b010;address_test[10] = 14'b000000_110_1_0100; write_data_test[10] = 0;       // lw, virtual page  0, TLB hit,  mapped to physical page 1, physical tag 01110, cache miss in set 1, set 1 block 1 replaced
        read_write_test[11] = 0; funct_test[11] = 3'b010;address_test[11] = 14'b000100_100_0_1000; write_data_test[11] = 0;       // lw, virtual page  4, TLB miss, mapped to physical page 2, virtual page 1 replaced, write back entry with virtual tag 1,
                                                                                                                                                                  //   physical tag 10100, cache hit  in set 1 block 0
        read_write_test[12] = 0; funct_test[12] = 3'b010;address_test[12] = 14'b000010_110_1_0100; write_data_test[12] = 0;       // lw, virtual page  2, TLB miss, page fault
        read_write_test[13] = 0; funct_test[13] = 3'b010;address_test[13] = 14'b000111_100_1_1100; write_data_test[13] = 0;       // lw, virtual page  10, TLB hit, mapped to physical page 1, physcial tag 01100, cache hit in set 1 block 1
        
        /* add lines if necessary */
           
    end
    
    always @(posedge clock) begin
        if (hit_miss == 1) request_num = request_num + 1;
        else request_num = request_num;
    end

endmodule