`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UMJI
// Engineer: CWQ, WL, and LJY
// 
// Create Date: 2021/11/02 20:35:58
// Design Name: CPU
// Module Name: Pipeline_Plus
// Project Name: Lab05 Pipeline Processor (Hazard Resolved)
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

module Pipeline_Plus(
    input                    clk, PC_en,
    input       [4:0]        Display_in,
    output reg  [31:0]       Display_out
);

    wire        [31:0]          Instrct, Imm, Imm_shift, data_1, data_2, MUX_ALU, loadPC, W_data, Add_1, Add_2, Add_3;
    wire        [31:0]          ALUResult, Read_Mem, JumpTo, ALUInput1, ALUInput2;
    wire        [31:0]          IF_ID_nextPC, IF_ID_crntPC, IF_ID_Instrct, Display_RF;
    wire        [31:0]          ID_EX_nextPC, ID_EX_crntPC, ID_EX_data_1, ID_EX_data_2, ID_EX_Imm, EX_MEM_Imm, MEM_WB_Imm;
    wire        [31:0]          EX_MEM_nextPC, EX_MEM_JumpTo, EX_MEM_ALUResult, EX_MEM_data_2, mem_write_content;
    wire        [31:0]          MEM_WB_nextPC, MEM_WB_Read_Mem, MEM_WB_ALUResult;
    wire        [31:0]          mux2_out, mux3_out; //NewBug_18
    wire        [4:0]           ID_EX_Reg_rd, ID_EX_Rs1_Addr, ID_EX_Rs2_Addr, EX_MEM_Rs2_Addr, EX_MEM_Reg_rd, MEM_WB_Reg_rd;
    wire        [3:0]           ALUControl, ID_EX_ALUInstrct;
    wire        [2:0]           EX_MEM_Funct3;
    wire        [1:0]           ALUOp, MemtoReg, ID_EX_ALUOp, ID_EX_MemtoReg, EX_MEM_MemtoReg, MEM_WB_MemtoReg;
    wire                        Branch, isBranch, MemRead, MemWrite, ALUSrc, isJump, RegWrite, isZero;
    wire                        IF_Flush, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_isJump, ID_EX_RegWrite;
    wire                        EX_MEM_Branch, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, MEM_WB_RegWrite, MEM_WB_MemRead;
    wire                        PC_write, IFID_write, mux_con, Forward_MemSrc; 
    wire        [1:0]           ForwardA, ForwardB; //Bug_03
    wire                        Forward_Eq1, Forward_Eq2;
    wire        [9:0]           allControlIn, allControlOut;
    reg         [31:0]          PC;
    
    initial PC = 0;   
      
    assign Imm_shift = {Imm[30:0],1'b0}; //NewBug_22
//  allControl = {ALUOp, MemtoReg, Branch, MemRead, MemWrite, ALUSrc, Jump, RegWrite}
    assign RegWrite = allControlOut[0];
    assign isJump   = allControlOut[1]; //Bug_11
    assign ALUSrc   = allControlOut[2];
    assign MemWrite = allControlOut[3];
    assign MemRead  = allControlOut[4];
    assign Branch   = allControlOut[5];
    assign MemtoReg = allControlOut[7:6];
    assign ALUOp    = allControlOut[9:8];
    
    always @ (posedge clk) 
        if (PC_write == 1'b1) PC <= loadPC;
        
    always @ (*) 
        if (PC_en == 1'b1) Display_out <= PC;
        else Display_out <= Display_RF;
    
    
    Hazard_detection HD (
        .IFID_r1 (IF_ID_Instrct[19:15]),
        .IFID_r2 (IF_ID_Instrct[24:20]),
//      .IDEX_r2 (ID_EX_Rs2_Addr), //NewBug_10
        .IDEX_RegRd (ID_EX_Reg_rd),
        .EXMEM_RegRd (EX_MEM_Reg_rd),
        .IDEX_MemRead (ID_EX_MemRead),
        .EXMEM_MemRead (EX_MEM_MemRead),
        .ID_branch (allControlIn[5]), //NewBug_17
        .ID_MemWrite (allControlIn[3]), //NewBug_17
        .IDEX_RegWrite (ID_EX_RegWrite), //NewBug_17
        .PC_write (PC_write),
        .IFID_write (IFID_write),
        .mux_con (mux_con)
    );
    
    Forwarding_unit FU (
        .IDEX_r1 (ID_EX_Rs1_Addr),
        .IDEX_r2 (ID_EX_Rs2_Addr),
        .IFID_r1 (IF_ID_Instrct[19:15]),
        .IFID_r2 (IF_ID_Instrct[24:20]),
        .EXMEM_r2 (EX_MEM_Rs2_Addr),
        .MEMWB_RegRd (MEM_WB_Reg_rd),
        .EXMEM_RegRd (EX_MEM_Reg_rd),
        .IDEX_RegRd (ID_EX_Reg_rd),
        .MEMWB_RegWrite (MEM_WB_RegWrite),
        .EXMEM_RegWrite (EX_MEM_RegWrite),
        .EXMEM_MemWrite (EX_MEM_MemWrite),
        .MEMWB_MemRead (MEM_WB_MemRead),
        .EXMEM_MemRead (EX_MEM_MemRead),
        .IDEX_MemRead (ID_EX_MemRead),
        .IDEX_MemWrite (ID_EX_MemWrite),
        .IDEX_RegWrite (ID_EX_RegWrite),
        .IFID_branch (Branch),
        .Forwarding_A (ForwardA),
        .Forwarding_B (ForwardB),
        .Forwarding_Eq1 (Forward_Eq1),
        .Forwarding_Eq2 (Forward_Eq2),
        .MemSrc (Forward_MemSrc)
    );
    
    Instruction_Memory IM (
        .instrct_out (Instrct),
        .addr (PC)
    );
    Central_Control_Unit CU (
        .opcode (IF_ID_Instrct[6:0]),
        .allControl (allControlIn)
    );
    Register_File RF (
        .clock (clk),
        .W_en (MEM_WB_RegWrite),
        .W_data (W_data),
        .R_addr_1 (IF_ID_Instrct[19:15]),
        .R_addr_2 (IF_ID_Instrct[24:20]),
        .Display_addr (Display_in),
        .W_addr (MEM_WB_Reg_rd),
        .R_data_1 (data_1),
        .R_data_2 (data_2),
        .Display_output (Display_RF)
    );
    Comparator cmp (
        .func ({IF_ID_Instrct[2], IF_ID_Instrct[14:12]}),
        .in1 (mux2_out),
        .in2 (mux3_out),
        .isZero (isZero)
    );
    Immediate_Generator ImmGen (
        .In (IF_ID_Instrct),
        .Out (Imm)
    );
    N_bit_ALU ALU (     
        .data1 (ALUInput1),
        .data2 (ALUInput2),
        .sel (ALUControl),
        .result (ALUResult)
    ); 
    ALU_Control ALU_Ctrl (
        .ALU_op (ID_EX_ALUOp),
        .instrct (ID_EX_ALUInstrct),
        .ALU_sel (ALUControl)
    );
    Data_Memory DM (
        .clock (clk),
        .W_en (EX_MEM_MemWrite),
        .R_en (EX_MEM_MemRead),
        .Funct3 (EX_MEM_Funct3),
        .addr (EX_MEM_ALUResult),
        .W_data (mem_write_content), //NewBug_15
        .R_data_out (Read_Mem)        
    );
    
    IF_ID_State_Reg IF_ID (
        .clock (clk),
        .IF_Flush (IF_Flush),
        .IF_ID_Write (IFID_write),
        .crntPC (PC),
        .nextPC (Add_1),
        .Instrct (Instrct),
        .crntPC_out (IF_ID_crntPC),
        .nextPC_out (IF_ID_nextPC),
        .Instrct_out (IF_ID_Instrct)
    );
    ID_EX_State_Reg ID_EX (
        .clock (clk),
        .RegWrite (RegWrite),
        .MemtoReg (MemtoReg),
        .MemRead (MemRead),
        .MemWrite (MemWrite),
        .Jump (isJump),
        .ALUSrc (ALUSrc),
        .ALUOp (ALUOp),
        .crntPC (IF_ID_crntPC),
        .nextPC (IF_ID_nextPC),
        .Reg_rs1 (data_1),
        .Reg_rs2 (data_2),
        .Reg_rs1_addr (IF_ID_Instrct[19:15]),
        .Reg_rs2_addr (IF_ID_Instrct[24:20]),
        .Imm_Gen (Imm),
        .Reg_rd (IF_ID_Instrct[11:7]),
        .ALU_Instrct ({IF_ID_Instrct[30],IF_ID_Instrct[14:12]}),
        .RegWrite_out (ID_EX_RegWrite),
        .MemtoReg_out (ID_EX_MemtoReg),
        .MemRead_out (ID_EX_MemRead),
        .MemWrite_out (ID_EX_MemWrite),
        .Jump_out (ID_EX_isJump),
        .ALUSrc_out (ID_EX_ALUSrc),
        .ALUOp_out (ID_EX_ALUOp),
        .crntPC_out (ID_EX_crntPC),
        .nextPC_out (ID_EX_nextPC),
        .Reg_rs1_out (ID_EX_data_1),
        .Reg_rs2_out (ID_EX_data_2),
        .Reg_rs1_addr_out (ID_EX_Rs1_Addr),
        .Reg_rs2_addr_out (ID_EX_Rs2_Addr),
        .Imm_Gen_out (ID_EX_Imm),
        .Reg_rd_out (ID_EX_Reg_rd),
        .ALU_Instrct_out (ID_EX_ALUInstrct)
    );
    EX_MEM_State_Reg EX_MEM (
        .clock (clk),
        .RegWrite (ID_EX_RegWrite),
        .MemtoReg (ID_EX_MemtoReg),
        .MemRead (ID_EX_MemRead),
        .MemWrite (ID_EX_MemWrite),
        .imm (ID_EX_Imm),
        .nextPC (ID_EX_nextPC),
        .ALUResult (ALUResult),
        .Reg_rs2 (ID_EX_data_2),
        .Funct3 (ID_EX_ALUInstrct[2:0]),
        .Reg_rd (ID_EX_Reg_rd),
        .Reg_rs2_addr (ID_EX_Rs2_Addr),
        .RegWrite_out (EX_MEM_RegWrite),
        .MemtoReg_out (EX_MEM_MemtoReg),
        .MemRead_out (EX_MEM_MemRead),
        .MemWrite_out (EX_MEM_MemWrite),
        .imm_out (EX_MEM_Imm),
        .nextPC_out (EX_MEM_nextPC),
        .ALUResult_out (EX_MEM_ALUResult),
        .Reg_rs2_out (EX_MEM_data_2),
        .Funct3_out (EX_MEM_Funct3),
        .Reg_rd_out (EX_MEM_Reg_rd),
        .Reg_rs2_addr_out (EX_MEM_Rs2_Addr)
    );
    MEM_WB_State_Reg MEM_WB (
        .clock (clk),
        .MemRead (EX_MEM_MemRead),
        .RegWrite (EX_MEM_RegWrite),
        .MemtoReg (EX_MEM_MemtoReg),
        .imm (EX_MEM_Imm),
        .nextPC (EX_MEM_nextPC),
        .ReadData (Read_Mem),
        .ALUResult (EX_MEM_ALUResult),
        .Reg_rd (EX_MEM_Reg_rd),
        .MemRead_out (MEM_WB_MemRead),
        .RegWrite_out (MEM_WB_RegWrite),
        .MemtoReg_out (MEM_WB_MemtoReg),
        .imm_out (MEM_WB_Imm),
        .nextPC_out (MEM_WB_nextPC),
        .ReadData_out (MEM_WB_Read_Mem),
        .ALUResult_out (MEM_WB_ALUResult),
        .Reg_rd_out (MEM_WB_Reg_rd)
    );
    
    //Mux: Input_1, Input_2, (Input_3, Input_4,) Select, Output.
    _N_bit_4to1_MUX #(.N(32)) Mux_1(.data1(Add_1), .data2(Add_2), .data3(Add_3), .data4(Add_2),
                                    .sel({isJump,isBranch}), .result(loadPC)); //NewBug_12
    _N_bit_2to1_MUX #(.N(32)) Mux_2(.data1(data_1), .data2(EX_MEM_ALUResult), .sel(Forward_Eq1), .result(mux2_out)); 
    _N_bit_2to1_MUX #(.N(32)) Mux_3(.data1(data_2), .data2(EX_MEM_ALUResult), .sel(Forward_Eq2), .result(mux3_out));
    _N_bit_4to1_MUX #(.N(32)) Mux_4(.data1(MEM_WB_Read_Mem), .data2(MEM_WB_nextPC), .data3(MEM_WB_Imm), .data4(MEM_WB_ALUResult),
                                    .sel(MEM_WB_MemtoReg), .result(W_data));
    _N_bit_2to1_MUX #(.N(10)) Mux_5(.data1(10'b0000000000), .data2(allControlIn), .sel(mux_con), .result(allControlOut)); //Bug_10
    _N_bit_4to1_MUX #(.N(32)) Mux_6(.data1(ID_EX_data_1), .data2(W_data), .data3(EX_MEM_ALUResult), .data4(0),
                                    .sel(ForwardA), .result(ALUInput1));
    _N_bit_4to1_MUX #(.N(32)) Mux_7(.data1(MUX_ALU), .data2(W_data), .data3(EX_MEM_ALUResult), .data4(0),
                                    .sel(ForwardB), .result(ALUInput2)); 
    _N_bit_2to1_MUX #(.N(32)) Mux_8(.data1(EX_MEM_data_2), .data2(W_data), .sel(Forward_MemSrc), .result(mem_write_content)); //NewBug_14
    _N_bit_2to1_MUX #(.N(32)) Mux_9(.data1(ID_EX_data_2), .data2(ID_EX_Imm), .sel(ID_EX_ALUSrc), .result(MUX_ALU)); //NewBug_11

    //Adder: Input_1, Input_2, Output.
    _32_bit_Adder Adder_1(.data1(PC), .data2(32'h00000004), .result(Add_1)); //PC+4
    _32_bit_Adder Adder_2(.data1(IF_ID_crntPC), .data2(Imm_shift), .result(Add_2)); //PC+Imm //NewBug_21
    _32_bit_Adder Adder_3(.data1(data_1), .data2(Imm), .result(Add_3)); //Addr+Imm //To_Be_Modified
    
    and (isBranch, Branch, isZero); 
    or  (IF_Flush, isBranch, isJump); //NewBug_19
    
endmodule

module Hazard_detection
(
    input [4:0] IFID_r1,
    input [4:0] IFID_r2,
    
//  input [4:0] IDEX_r2, //NewBug_10
    input [4:0] IDEX_RegRd,
    input [4:0] EXMEM_RegRd,
    
    input IDEX_MemRead,
    input EXMEM_MemRead,
    
    input ID_branch,
    input ID_MemWrite,
    input IDEX_RegWrite,

    output reg PC_write, IFID_write, mux_con 
    );

    initial begin
        PC_write=1'b1; IFID_write=1'b1; mux_con=1'b1;
    end

    always @(*) begin //NewBug_09 in here
        if (IDEX_MemRead && !ID_MemWrite) begin
            if (IDEX_RegRd == IFID_r1 || IDEX_RegRd == IFID_r2) begin
                PC_write=1'b0; IFID_write=1'b0; mux_con=1'b0;
            end
            else begin
                PC_write=1'b1; IFID_write=1'b1; mux_con=1'b1;
            end  
        end
        else if (ID_branch && IDEX_RegWrite) begin
            if ((IDEX_RegRd!=0) && (IFID_r1 == IDEX_RegRd || IFID_r2 == IDEX_RegRd)) begin
                PC_write=1'b0; IFID_write=1'b0; mux_con=1'b0;
            end
            else begin
                PC_write=1'b1; IFID_write=1'b1; mux_con=1'b1;
            end  
        end
        else if (ID_branch && EXMEM_MemRead) begin
            if ((EXMEM_RegRd!=0) && (IFID_r1 == EXMEM_RegRd || IFID_r2 == EXMEM_RegRd)) begin
                PC_write=1'b0; IFID_write=1'b0; mux_con=1'b0;
            end
            else begin
                PC_write=1'b1; IFID_write=1'b1; mux_con=1'b1;
            end  
        end
        else begin
            PC_write=1'b1; IFID_write=1'b1; mux_con=1'b1;
        end
    end
endmodule

module Forwarding_unit
(
    input [4:0] IDEX_r1,
    input [4:0] IDEX_r2,
    input [4:0] IFID_r1,
    input [4:0] IFID_r2,
    input [4:0] EXMEM_r2, //New_Bug_05
    
    input [4:0] MEMWB_RegRd,
    input [4:0] EXMEM_RegRd,
    input [4:0] IDEX_RegRd,
    
    input MEMWB_RegWrite, 
    input EXMEM_RegWrite,
    input EXMEM_MemWrite,

    input EXMEM_MemRead,
    input MEMWB_MemRead,
    
    input IDEX_MemRead,
    input IDEX_MemWrite,
    input IDEX_RegWrite,
    input IFID_branch,
    
    output reg [1:0] Forwarding_A,
    output reg [1:0] Forwarding_B,
    output reg Forwarding_Eq1,
    output reg Forwarding_Eq2,
    output reg MemSrc
    );
    
    initial begin
        Forwarding_A = 2'b00;
        Forwarding_B = 2'b00;
        Forwarding_Eq1 = 1'b0;
        Forwarding_Eq2 = 1'b0;
        MemSrc = 1'b0; //NewBug_08
    end
    
    always @(*) begin
    //ForwardingA
        //EX
        if (EXMEM_RegWrite && (EXMEM_RegRd!=0) && EXMEM_RegRd == IDEX_r1 && !EXMEM_MemRead && !IDEX_MemRead && !IDEX_MemWrite) //NewBug_16
            Forwarding_A = 2'b10;
        //MEM
        else if (MEMWB_RegWrite && (MEMWB_RegRd!=0) && MEMWB_RegRd == IDEX_r1)
            Forwarding_A = 2'b01;
        else
            Forwarding_A = 2'b00;
    
    //ForwardingB
         //EX
        if (EXMEM_RegWrite && (EXMEM_RegRd!=0) && EXMEM_RegRd == IDEX_r2 && !EXMEM_MemRead && !IDEX_MemRead && !IDEX_MemWrite) //NewBug_06
            Forwarding_B = 2'b10;
         //MEM
        else if (MEMWB_RegWrite && (MEMWB_RegRd!=0) && MEMWB_RegRd == IDEX_r2) 
            Forwarding_B = 2'b01;
        else
            Forwarding_B = 2'b00;
   
    //Memsrc
        if (MEMWB_RegRd == EXMEM_r2 && EXMEM_MemWrite) //NewBug_03
            MemSrc = 1'b1;
        else
            MemSrc = 1'b0;

    //ForwardingEq
        if (EXMEM_RegWrite && (EXMEM_RegRd!=0) && EXMEM_RegRd == IFID_r1 && IFID_branch)
            Forwarding_Eq1 = 1'b1;
        else
            Forwarding_Eq1 = 1'b0;
        
        if (EXMEM_RegWrite && (EXMEM_RegRd!=0) && EXMEM_RegRd == IFID_r2 && IFID_branch)
            Forwarding_Eq2 = 1'b1; //NewBug_07
        else
            Forwarding_Eq2 = 1'b0; //NewBug_07

    end

endmodule

module Register_File #
(
    parameter width = 32,
    parameter addr_width = 5,
    parameter reg_rows = 2 ** addr_width
)
(
    input                           clock, W_en,
    input       [width-1:0]         W_data,
    input       [addr_width-1:0]    R_addr_1, R_addr_2, W_addr,
    input       [addr_width-1:0]    Display_addr,
    output      [width-1:0]         R_data_1, R_data_2,
    output      [width-1:0]         Display_output
);
    reg         [width-1:0]         data  [reg_rows-1:0];
    
    integer i;
    initial begin  
        for(i=0; i<32; i=i+1)  
            data[i] <= 0;
    end 
    
    always @ (negedge clock) begin
        if (W_en && W_addr!= 0) begin
            data[W_addr] = W_data;
        end  
    end
     
    assign R_data_1 = data[R_addr_1];
    assign R_data_2 = data[R_addr_2]; 
    assign Display_output = data[Display_addr]; 
    
endmodule

module Comparator #
(
    parameter SUB_BEQ = 4'b0000, parameter SUB_NEQ = 4'b0001,
    parameter SUB_BLT = 4'b0100, parameter SUB_BGE = 4'b0101
)
(
    input       [3:0]           func,
    input       [31:0]          in1, in2,
    output reg                  isZero
);
    initial isZero = 1'b1;
    always @ (*) begin
        case (func)
                SUB_BEQ: isZero = (in1==in2)?1'b1:1'b0;
                SUB_NEQ: isZero = (in1==in2)?1'b0:1'b1;
                SUB_BLT: isZero = ($signed(in1) < $signed(in2))?1'b1:1'b0; //NewBug_02
                SUB_BGE: isZero = ($signed(in1) < $signed(in2))?1'b0:1'b1; //NewBug_02
                default: isZero = 1'b1;
        endcase
    end
endmodule

module N_bit_ALU #
(
    parameter N = 32,
    parameter ADD = 4'b0010, parameter SUB = 4'b1000, parameter SUB_NEQ = 4'b1001,
    parameter XOR = 4'b0100, parameter  OR = 4'b0110, parameter AND = 4'b0111, parameter SUB_BLT = 4'b1100,
    parameter SLL = 4'b0001, parameter SRL = 4'b0101, parameter SRA = 4'b1101, parameter SUB_BGE = 4'b1110
)
(
    input       [3:0]           sel,
    input       [N-1:0]         data1, data2,
    output reg  [N-1:0]         result
);

    always @ (*) begin
        case (sel)
            ADD:    result =  data1 + data2; //add
            SUB,SUB_NEQ: 
                    result =  data1 - data2; //sub //NewBug_01
            XOR:    result =  data1 ^ data2; //xor
             OR:    result =  data1 | data2; //or
            AND:    result =  data1 & data2; //and
            SLL:    result =  data1 <<data2; //sll
            SRL:    result =  data1 >>data2; //srl
            SRA:    result =  $signed(($signed(data1))>>>data2); //sra
            default:begin result = 0; end
        endcase
    end
endmodule

module ALU_Control 
(
    input       [1:0]         ALU_op,
    input       [3:0]         instrct,
    output reg  [3:0]         ALU_sel 
);
    
    always @ (*) begin
        case (ALU_op)
            2'b00:  begin
                        ALU_sel = 4'b0010; //Addition
                    end
            2'b01:  begin
                        if (instrct[2:0] == 3'b101) begin ALU_sel = 4'b1110; end //bge
                        else begin ALU_sel = {1'b1, instrct[2:0]}; end //beq,bne,blt
                    end
            2'b10:  begin
                        if (instrct == 0) begin ALU_sel = 4'b0010; end //add
                        else begin ALU_sel = instrct; end //sub,and,or,xor,sll,srl,sra
                    end
            2'b11:  begin
                        if (instrct[2:0] == 0) begin ALU_sel = 4'b0010; end //addi
                        else begin ALU_sel = {1'b0, instrct[2:0]}; end //andi,slli,srli
                    end
            default:ALU_sel = 4'b0000;
        endcase
    end
endmodule

module _32_bit_Adder
(
    input       [31:0]          data1, data2,
    output reg  [31:0]          result
);
    
    always @ (*) begin
        result = data1 + data2;
    end
endmodule

module Immediate_Generator 
(
    input       [31:0]          In,
    output reg  [31:0]          Out
);

    always @ (*) begin
        case (In[6:0])
            7'b0000011: begin Out = {{20{In[31]}}, In[31:20]}; end //I-type: lw
            7'b0001111: begin Out = {{20{In[31]}}, In[31:20]}; end //I-type: fence
            7'b0010011: begin Out = {{20{In[31]}}, In[31:20]}; end //I-type: addi
            7'b0010111: begin Out = {In[31:12], {12{1'b0}}}; end //U-type: auipc
            7'b0100011: begin Out = {{20{In[31]}}, In[31:25], In[11:7]}; end //S-type: sw
            7'b0110111: begin Out = {In[31:12], {12{1'b0}}}; end //U-type: lui
            7'b1100011: begin Out = {{21{In[31]}}, In[7], In[30:25], In[11:8]}; end //B-type: beq, bne
            7'b1100111: begin Out = {{20{In[31]}}, In[31:20]}; end //I-type: jalr
            7'b1101111: begin Out = {{21{In[31]}}, In[19:12], In[20], In[30:21]}; end //J-type: jal
            7'b1110011: begin Out = {{20{In[31]}}, In[31:20]}; end //I-type: ecall
            default:    begin Out[31:0]  = 0; end
        endcase
    end 
endmodule

module Central_Control_Unit 
(
    input       [6:0]           opcode,
    
    output reg  [9:0]           allControl
//  output reg  [1:0]           ALUOp, MemtoReg, //MemtoReg to select among ALUResult, DataMemo, and PC+4.
//  output reg                  Branch, MemRead, MemWrite, ALUSrc, Jump, RegWrite //Jump for jalr.
);

    initial begin //Bug_09
        allControl <= 0;
    end

//  allControl = {ALUOp, MemtoReg, Branch, MemRead, MemWrite, ALUSrc, Jump, RegWrite}
    always @ (opcode) begin //NewBug_13 about MemtoReg
        case (opcode)
        //I-type,load
            7'b0000011: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 1; allControl[7:6] <= 2'b00; allControl[1] <= 0; allControl[0] <= 1; end 
//          7'b0001111: 
        //I-type,Imm
            7'b0010011: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b11; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 1; end 
//          7'b0010111: 
        //S-type,save
            7'b0100011: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 1; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 0; end 
//          7'b0110111: 
        //B-type,brnch
            7'b1100011: begin allControl[5] <= 1; allControl[2] <= 0; allControl[9:8] <= 2'b01; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 0; end 
        //I-type,jalr
            7'b1100111: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b01; allControl[1] <= 1; allControl[0] <= 1; end 
        //J-type,jal
            7'b1101111: begin allControl[5] <= 1; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b01; allControl[1] <= 1; allControl[0] <= 1; end 
//          7'b1110011: 
        //R-type,calc
            7'b0110011: begin allControl[5] <= 0; allControl[2] <= 0; allControl[9:8] <= 2'b10; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 1; end 
            default:    begin allControl    <= 0; end
        endcase
    end
endmodule

module _N_bit_2to1_MUX #
(
    parameter   N = 32
)
(
    input                       sel,
    input       [N-1:0]         data1, data2,
    output reg  [N-1:0]         result
);

    always @ (*) begin
        case (sel)
            1'b0:   result = data1;
            1'b1:   result = data2;
            default:    result = 0;
        endcase
    end
endmodule

module _N_bit_4to1_MUX #
(
    parameter   N = 32
)
(
    input       [1:0]           sel,
    input       [N-1:0]         data1, data2, data3, data4,
    output reg  [N-1:0]         result
);

    always @ (*) begin
        case (sel)
            2'b00:   result = data1;
            2'b01:   result = data2;
            2'b10:   result = data3;
            2'b11:   result = data4;
            default:    result = 0;
        endcase
    end
endmodule

module Instruction_Memory #
(
    parameter N = 32,
    parameter reg_rows = 32
)(
    input       [N-1:0]         addr,
    output reg  [N-1:0]         instrct_out
);
    reg         [N-1:0]         Instruction [reg_rows-1:0];
   
    initial begin
        Instruction[0] <= 32'b00111001100100000000001100010011;
        Instruction[1] <= 32'b00000000011000000010001000100011;
        Instruction[2] <= 32'b00000000010000000000001010000011;
        Instruction[3] <= 32'b00000000010100000010000000100011;
        Instruction[4] <= 32'b00000010000000110000000001100011;
        Instruction[5] <= 32'b00000000000000000010111000000011;
        Instruction[6] <= 32'b00000001110000101001110001100011;
        Instruction[7] <= 32'b00000001110000101000001110110011;
        Instruction[8] <= 32'b00000001110000111111001100110011;
        Instruction[9] <= 32'b00000000000000111111001100010011;
        Instruction[10] <= 32'b01000000000000110000001010110011;
        Instruction[11] <= 32'b00000000011000101101010001100011;
        Instruction[12] <= 32'b00000000000000000000001110110011;
        Instruction[13] <= 32'b00000000110000000000000011101111;
        Instruction[14] <= 32'b00000001010000000000000011101111;
        Instruction[15] <= 32'b00000000000000000000111000110011;
        Instruction[16] <= 32'b00000000011111100110111000110011;
        Instruction[17] <= 32'b00000000000000001000000001100111;
        Instruction[18] <= 32'b00000100100000000000001100010011;
        Instruction[19] <= 32'b00000000000000000000000000110011;
        Instruction[20] <= 32'b00000000000000000000000000110011;
        Instruction[21] <= 32'b00000000000000000000000000110011;
        Instruction[22] <= 32'b00000000000000000000000000110011;
        Instruction[23] <= 32'b00000000000000000000000000110011;
        Instruction[24] <= 32'b00000000000000000000000000110011;
        Instruction[25] <= 32'b00000000000000000000000000110011;
        Instruction[26] <= 32'b00000000000000000000000000110011;
        Instruction[27] <= 32'b00000000000000000000000000110011;
        Instruction[28] <= 32'b00000000000000000000000000110011;
        Instruction[29] <= 32'b00000000000000000000000000110011;
    end   
    always @ (*) begin
        instrct_out = Instruction[(addr>>2)];
    end
endmodule

module Data_Memory #
(
    parameter N = 32, parameter LSW = 3'b010, parameter LSH = 3'b001, parameter LSB = 3'b000, parameter LBU = 3'b100,
    parameter reg_rows = 32'h00000010
)
(
    input                       clock,
    input                       W_en, R_en,
    input       [2:0]           Funct3,
    input       [N-1:0]         addr, W_data,
    output reg  [N-1:0]         R_data_out
);
    reg         [7:0]           data[reg_rows-1:0];
    
    //Write to Data Memory
    always @ (negedge clock) begin
        if (W_en) begin
            case (Funct3)
                LSW: begin //save word
                    data[addr] <= W_data[7:0]; data[addr+1] <= W_data[15:8]; data[addr+2] <= W_data[23:16]; data[addr+3] <= W_data[31:24]; 
                end
                LSH: begin //save half word 
                    data[addr] <= W_data[7:0]; data[addr+1] <= W_data[15:8]; 
                end 
                LSB: begin //save byte
                    data[addr] <= W_data[7:0];
                end
                default: data[addr] <= data[addr];
            endcase
        end
    end
    //Read from Data Memory
    always @ (*) begin    
        if (R_en) begin
            case (Funct3)
                LSW: begin //load word
                    R_data_out <= {data[addr+3], data[addr+2], data[addr+1], data[addr]};
                end
                LSH: begin //load half word
                    R_data_out <= {{16{data[addr+1][7]}}, data[addr+1], data[addr]};
                end
                LSB: begin //load byte
                    R_data_out <= {{24{data[addr][7]}}, data[addr]};
                end
                LBU: begin //load byte unsigned
                    R_data_out <= {{24{1'b0}}, data[addr]};
                end
                default: R_data_out <= R_data_out;
            endcase
        end
    end
endmodule

module IF_ID_State_Reg
(
    input                   clock, IF_Flush,
    input                   IF_ID_Write,
    input       [31:0]      crntPC,
    input       [31:0]      nextPC,
    input       [31:0]      Instrct,
    output reg  [31:0]      crntPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Instrct_out
);

    initial begin //Bug_05
        crntPC_out = 0; nextPC_out = 0; Instrct_out = 0;
    end

    always @ (posedge clock) begin
        if (IF_Flush == 1'b1) begin //Newbug_20 is here
//          crntPC_out      <= 0;
//          nextPC_out      <= 0;
            Instrct_out     <= 0;
        end
        else if (IF_ID_Write == 1'b1) begin
            crntPC_out      <= crntPC;
            nextPC_out      <= nextPC;
            Instrct_out     <= Instrct;
        end
    end
    
endmodule

module ID_EX_State_Reg
(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input                   Jump,       //EX
    input                   ALUSrc,     //EX
    input       [1:0]       ALUOp,      //EX
    input       [31:0]      crntPC,
    input       [31:0]      nextPC,
    input       [31:0]      Reg_rs1,
    input       [31:0]      Reg_rs2,
    input       [4:0]       Reg_rs1_addr,
    input       [4:0]       Reg_rs2_addr,
    input       [31:0]      Imm_Gen,
    input       [4:0]       Reg_rd,
    input       [3:0]       ALU_Instrct,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg              Jump_out,       //EX
    output reg              ALUSrc_out,     //EX
    output reg  [1:0]       ALUOp_out,      //EX
    output reg  [31:0]      crntPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Reg_rs1_out,
    output reg  [31:0]      Reg_rs2_out,
    output reg  [4:0]       Reg_rs1_addr_out, //Bug_01
    output reg  [4:0]       Reg_rs2_addr_out, //Bug_01
    output reg  [31:0]      Imm_Gen_out,
    output reg  [4:0]       Reg_rd_out,
    output reg  [3:0]       ALU_Instrct_out
);

    initial begin //Bug_06
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; Jump_out = 0; ALUSrc_out = 0; 
        ALUOp_out = 0; crntPC_out = 0; nextPC_out = 0; Reg_rs1_out = 0; Reg_rs2_out = 0; Imm_Gen_out = 0; Reg_rd_out = 0; ALU_Instrct_out = 0;
        Reg_rs1_addr_out = 0; Reg_rs2_addr_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Jump_out        <= Jump;
        ALUSrc_out      <= ALUSrc;
        ALUOp_out       <= ALUOp;
        crntPC_out      <= crntPC;
        nextPC_out      <= nextPC;
        Reg_rs1_out     <= Reg_rs1;
        Reg_rs2_out     <= Reg_rs2;
        Reg_rs1_addr_out<= Reg_rs1_addr;
        Reg_rs2_addr_out<= Reg_rs2_addr;
        Reg_rd_out      <= Reg_rd;
        Imm_Gen_out     <= Imm_Gen;
        ALU_Instrct_out <= ALU_Instrct;
    end

endmodule

module EX_MEM_State_Reg
(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input       [31:0]      imm,
    input       [31:0]      nextPC,
    input       [31:0]      ALUResult,
    input       [31:0]      Reg_rs2,
    input       [2:0]       Funct3,
    input       [4:0]       Reg_rd,
    input       [4:0]       Reg_rs2_addr, //NewBug_04
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg  [31:0]      imm_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [31:0]      Reg_rs2_out,
    output reg  [2:0]       Funct3_out,
    output reg  [4:0]       Reg_rd_out,
    output reg  [4:0]       Reg_rs2_addr_out //NewBug_04
);

    initial begin //Bug_07
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; nextPC_out = 0; 
        ALUResult_out = 0; Reg_rs2_out = 0; Funct3_out = 0; Reg_rd_out = 0; imm_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        imm_out         <= imm;
        nextPC_out      <= nextPC;
        ALUResult_out   <= ALUResult;
        Reg_rs2_out     <= Reg_rs2;
        Funct3_out      <= Funct3;
        Reg_rd_out      <= Reg_rd;
        Reg_rs2_addr_out<= Reg_rs2_addr;
    end

endmodule

module MEM_WB_State_Reg
(
    input                   clock,
    input                   MemRead,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input       [31:0]      imm,
    input       [31:0]      nextPC,
    input       [31:0]      ReadData,
    input       [31:0]      ALUResult,
    input       [4:0]       Reg_rd,
    output reg              MemRead_out,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg  [31:0]      imm_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ReadData_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [4:0]       Reg_rd_out
);

    initial begin //Bug_08
        MemRead_out = 0; RegWrite_out = 0; MemtoReg_out = 0; nextPC_out = 0; ReadData_out = 0; ALUResult_out = 0; Reg_rd_out = 0; imm_out = 0;
    end

    always @ (posedge clock) begin
        MemRead_out     <= MemRead;
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        imm_out         <= imm;
        nextPC_out      <= nextPC;
        ReadData_out    <= ReadData;
        ALUResult_out   <= ALUResult;
        Reg_rd_out      <= Reg_rd;
    end

endmodule


//Counter with CE and reset but without load
module N_bit_Counter #
(
    parameter   N = 4
)
(
    input  clock, CE, reset,
    output reg   [N-1:0]  Q
);

    always @ (posedge clock or posedge reset) begin
        if (reset == 1'b1)
            Q <= 0;
        else if (CE == 1'b1)
            Q <= Q + 1;
        else
            Q <= Q;
    end
    
endmodule

//Ring Counter with reset and CE
module Ring_Counter_4_bit
(
    input   clock, CE, reset,
    output reg  [3:0]  anode
);
    wire    [1:0]   sel;

    N_bit_Counter #(.N(2)) counter (.clock (clock), .CE (CE), .reset (reset), .Q(sel));
    always @ (posedge clock or posedge reset) begin
        if (reset == 1'b1) //When the clock is reseting, display no digits.
            anode <= 4'b1111;
        else
            case(sel)
                2'b00: anode = 4'b1110; // "1"  
                2'b01: anode = 4'b1101; // "2" 
                2'b10: anode = 4'b1011; // "3" 
                2'b11: anode = 4'b0111; // "4"
            endcase     
    end
    
endmodule

//Divide the clock by N
module Clock_divider #
(
    parameter   bit = 18, 
    parameter   N = 18'd200000 //2x10^5 in decimal number
)
(
    input       clock, CE,
    output reg    divided
);
    wire    [bit-1:0]   Q;
    
    N_bit_Counter #(.N(bit)) counter (.clock (clock), .CE (CE), .reset (divided), .Q(Q));
    always @ (posedge clock) begin
        if (Q == N)
            divided <= 1'b1;
        else
            divided <= 1'b0;
    end
    
endmodule

//Seven Segment Display Driver
module SSD_Driver 
(
    input                   clock,
    input       [3:0]       LED_in,
    output reg  [7:0]       LED_out
);

    always @(posedge clock) begin
        case(LED_in)
            4'b0000: LED_out = 8'b11000000; // "0"  
            4'b0001: LED_out = 8'b11111001; // "1" 
            4'b0010: LED_out = 8'b10100100; // "2" 
            4'b0011: LED_out = 8'b10110000; // "3" 
            4'b0100: LED_out = 8'b10011001; // "4"
            4'b0101: LED_out = 8'b10010010; // "5" 
            4'b0110: LED_out = 8'b10000010; // "6" 
            4'b0111: LED_out = 8'b11111000; // "7" 
            4'b1000: LED_out = 8'b10000000; // "8"  
            4'b1001: LED_out = 8'b10010000; // "9" 
            4'b1010: LED_out = 8'b10001000; // "A" 
            4'b1011: LED_out = 8'b10000011; // "b" 
            4'b1100: LED_out = 8'b11000110; // "C" 
            4'b1101: LED_out = 8'b10100001; // "d" 
            4'b1110: LED_out = 8'b10000110; // "E" 
            4'b1111: LED_out = 8'b10001110; // "F" 
            default: LED_out = 8'b11000000; // "0"
        endcase 
    end
    
endmodule

//4 Digit Displayer
module Digit_Displayer_4_digit
(
    input                   clock, CE, reset,
    input       [31:0]      Display_out,
    output reg  [3:0]       Q0, Q1, Q2, Q3
);
    always @ (*) begin
        Q0 = Display_out[3:0];
        Q1 = Display_out[7:4];
        Q2 = Display_out[11:8];
        Q3 = Display_out[15:12];
    end

endmodule

module Digital_Display_Block
(
    input                   clock, clk, reset, PC_en,
    input       [4:0]       Display_in,
    output      [3:0]       anode,
    output reg  [7:0]       cathode
);

    wire                ring_clock;
    wire    [3:0]       Q_L, Q_R, Q_LL, Q_RR;
    wire    [7:0]       LED_L, LED_R, LED_LL, LED_RR;
    wire    [31:0]      PC, Display_out;

    Clock_divider #(.bit(18),.N(18'd200000)) clock_divider_500Hz (.clock(clock), .CE(1'b1), .divided(ring_clock));
    Pipeline_Plus PP (.clk(clk), .PC_en(PC_en), .Display_in(Display_in), .Display_out(Display_out));
    Digit_Displayer_4_digit DD (.clock(clock), .CE(1'b1), .reset(reset), .Display_out(Display_out), .Q0(Q_RR), .Q1(Q_R), .Q2(Q_L), .Q3(Q_LL));
    
    SSD_Driver driver_RR (.clock(clock), .LED_in(Q_RR), .LED_out(LED_RR));
    SSD_Driver driver_R  (.clock(clock), .LED_in(Q_R),  .LED_out(LED_R));
    SSD_Driver driver_L  (.clock(clock), .LED_in(Q_L),  .LED_out(LED_L));
    SSD_Driver driver_LL (.clock(clock), .LED_in(Q_LL), .LED_out(LED_LL));
    Ring_Counter_4_bit ring_counter (.clock(ring_clock), .CE(1'b1), .reset(reset), .anode(anode));
    //Tri-state buffer or MUX to control cathode
    always @ (*) begin //DO NOT connect ring_clock here! Because you don't need a flip-flop here!
        case(anode)
             4'b1110: cathode = LED_RR; // ones  
             4'b1101: cathode = LED_R;  // tens
             4'b1011: cathode = LED_L;  // hundreds
             4'b0111: cathode = LED_LL; // thousands
             default: cathode = 8'b11111111; // others
        endcase
    end
    
endmodule
