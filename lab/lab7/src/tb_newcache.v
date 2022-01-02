`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/12/07 16:00:21
// Design Name: 
// Module Name: tb_newcache
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

module tb_newcache;

    reg             clk;
    
    // interface between cache and CPU
    wire            write_in; /* 1 if write, 0 if read */
    wire [2:0]      funct;
    wire [31:0]     write_data_in, read_data_out;
    
    // interface between cache and main memory
    wire            done_tlb, done_cache, write_out;
    wire [31:0]     read_data_in, write_data_out;
    wire [9:0]      addr_out;
    
    //other interface
    //tlb
    wire            Addr_prepared;
    wire            write_to_table;
    wire [5:0]      V_addr_PT_out;
    wire [1:0]      P_addr_PT_out;
    wire [9:0]      Physical_addr;
    //page table
    wire [1:0]      phy_page_num;
    //cpu
    wire [13:0]     virtual_address;  
    
    New_Cache cache (.done(done_cache),
                 .write_in(write_in),
                 .addr_prepared(Addr_prepared),//tlb
                 .funct(funct),
                 .rqst_addr(Physical_addr),//tlb
                 .read_data_in(read_data_in),
                 .write_data_in(write_data_in),
                 
                 .hit(hit),
                 .write_out(write_out),
                 .read_data_out(read_data_out), 
                 .write_data_out(write_data_out),
                 .addr_out(addr_out)
                 );
                 
    New_TLB tlb (.done(done_tlb),
                 .Virtual_addr(virtual_address),
                 .P_addr_PT_in(phy_page_num),
                 
                 .TLB_hit(TLB_hit),
                 .write_to_table(write_to_table),
                 .V_addr_PT_out(V_addr_PT_out),
                 .P_addr_PT_out(P_addr_PT_out),
                 .Physical_addr(Physical_addr),
                 .Addr_prepared(Addr_prepared)
                 );
                 
    main_mem mem (.read_or_write(write_out),
                  .address(addr_out),
                  .write_data(write_data_out),
                  
                  .read_data(read_data_in),
                  .done(done_cache)
                  );
                  
    page_table pag_tb(.read_from_TLB(write_to_table),
                      .phy_page_num_in(P_addr_PT_out),
                      .vir_page_num_in(V_addr_PT_out),
                      
                      .done(done_tlb),
                      .phy_page_num(phy_page_num),
                      .page_fault(page_fault)
                      );
                  
    CPU cpu (.hit_miss(hit),
                .clock(clk),
                
                .read_write(write_in),
                .funct(funct),
                .address(virtual_address),
                .write_data(write_data_in)
                );
    
    // CPU 
    wire    [4:0]       request_num=cpu.request_num;
    wire                read_write_test[5:0]=cpu.read_write_test;
    wire    [2:0]       funct_test[5:0]=cpu.funct_test;
    wire    [13:0]      address_test[5:0]=cpu.address_test;
    wire    [31:0]      write_data_test[5:0]=cpu.write_data_test;
    
    // cache
    wire                pos_done=cache.done;           
    wire    [1:0]       LRU=cache.LRU;
    wire    [134:0]     cacheA[1:0]=cache.cache_setA;
    wire    [134:0]     cacheB[1:0]=cache.cache_setB;
    wire                setIndex=cache.setIndex;
    wire                validA=cache.valid_A, dirtyA=cache.dirty_A, validB=cache.valid_B, dirtyB=cache.dirty_B;
    wire                equal_A=cache.equal_A, equal_B=cache.equal_B;
    wire    [31:0]      lw_out=cache.lw_out, lb_out=cache.lb_out;
    wire    [134:0]     block_A=cache.block_A;
    wire    [134:0]     block_B=cache.block_B;
    
    // memory
    wire 	[31:0]       main_memory[255:0]=mem.main_memory;

   //TLB
    wire    [15:0]       TLB[3:0]=tlb.TLB;  
    wire    [3:0]        Hit=tlb.Hit;              
    wire    [5:0]        VPN=tlb.VPN;   

    //page table
    wire    [4:0] 	     page_table[63:0]=pag_tb.page_table;
   
    always #30 clk = ~clk;

    always @(posedge clk) begin
        $display("Request %d: ", cpu.request_num);
        $display("page fault: %b", pag_tb.page_fault);
        $display("data read posedge: %H", read_data_out);
        $display("contents in TLB: ");
        $display("block 00: tag: %2d, valid: %b, dirty: %b, reference: %b, PPN: %1d", TLB[3][9:4], TLB[3][15], TLB[3][14], TLB[3][13:12],TLB[3][1:0]);
        $display("block 01: tag: %2d, valid: %b, dirty: %b, reference: %b, PPN: %1d", TLB[2][9:4], TLB[2][15], TLB[2][14], TLB[2][13:12],TLB[2][1:0]);
        $display("block 10: tag: %2d, valid: %b, dirty: %b, reference: %b, PPN: %1d", TLB[1][9:4], TLB[1][15], TLB[1][14], TLB[1][13:12],TLB[1][1:0]);
        $display("block 11: tag: %2d, valid: %b, dirty: %b, reference: %b, PPN: %1d", TLB[0][9:4], TLB[0][15], TLB[0][14], TLB[0][13:12],TLB[0][1:0]);
        $display("contents in cache: ");
        $display("block 00: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setA[0][132-: 5], cache.cache_setA[0][134], cache.cache_setA[0][133],  cache.cache_setA[0][127:96],  cache.cache_setA[0][95:64],  cache.cache_setA[0][63:32],  cache.cache_setA[0][31:0]);
        $display("block 01: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setB[0][132-: 5], cache.cache_setB[0][134], cache.cache_setB[0][133],  cache.cache_setB[0][127:96],  cache.cache_setB[0][95:64],  cache.cache_setB[0][63:32],  cache.cache_setB[0][31:0]);
        $display("block 10: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setA[1][132-: 5], cache.cache_setA[1][134], cache.cache_setA[1][133],  cache.cache_setA[1][127:96],  cache.cache_setA[1][95:64],  cache.cache_setA[1][63:32],  cache.cache_setA[1][31:0]);
        $display("block 11: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setB[1][132-: 5], cache.cache_setB[1][134], cache.cache_setB[1][133],  cache.cache_setB[1][127:96],  cache.cache_setB[1][95:64],  cache.cache_setB[1][63:32],  cache.cache_setB[1][31:0]);
    end
    
    initial begin
        clk = 0;
    end
    
    initial #1000 $stop;
    
endmodule
