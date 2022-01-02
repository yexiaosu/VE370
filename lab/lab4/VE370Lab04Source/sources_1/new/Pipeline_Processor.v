`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: UMJI
// Engineer: CWQ
// 
// Create Date: 2021/10/29 21:20:40
// Design Name: 
// Module Name: State Register
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

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Wenqi Cao
// 
// Create Date: 2021/10/10 11:07:29
// Design Name: 
// Module Name: Single_Cycle_Processor
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

module Register_File #
(
    parameter width = 32,
    parameter addr_width = 5,
    parameter reg_rows = 2 ** addr_width
)(
    input                           clock, W_en,
    input       [width-1:0]         W_data,
    input       [addr_width-1:0]    R_addr_1, R_addr_2, W_addr,
    output      [width-1:0]         R_data_1, R_data_2
);
    reg         [width-1:0]         data  [reg_rows-1:0];
    
    integer i;
    initial begin  
        for(i=0; i<32; i=i+1)  
            data[i] <= 0;
    end 
    
    always @ (*) begin
        if (W_en && W_addr!= 0) begin
            data[W_addr] = W_data;
        end  
    end
     
    assign R_data_1 = data[R_addr_1];
    assign R_data_2 = data[R_addr_2]; 
    
endmodule

module N_bit_ALU #
(
    parameter N = 32,
    parameter ADD = 4'b0010, parameter SUB = 4'b1000, parameter SUB_NEQ = 4'b1001,
    parameter XOR = 4'b0100, parameter  OR = 4'b0110, parameter AND = 4'b0111, parameter SUB_BLT = 4'b1100,
    parameter SLL = 4'b0001, parameter SRL = 4'b0101, parameter SRA = 4'b1101, parameter SUB_BGE = 4'b1110
)(
    input       [3:0]           sel,
    input       [N-1:0]         data1, data2,
    output reg                  Zero,
    output reg  [N-1:0]         result
);

    always @ (*) begin
        case (sel)
            ADD:
                begin
                    result =  data1 + data2; //add
                    Zero = 1'b1; //jal & jalr
                end
            SUB,SUB_NEQ: 
                begin
                     if (data1 == data2) begin
                         Zero = ~sel[0]; result =  0;
                     end 
                     else begin
                         Zero =  sel[0]; result =  data1 - data2; //sub
                     end
                 end
            SUB_BLT,SUB_BGE:
                begin
                    if ($signed(data1) < $signed(data2)) begin
                        Zero = ~sel[1]; result =  0;
                    end 
                    else begin
                        Zero =  sel[1]; result =  0;
                    end
                end
            XOR:     result =  data1 ^ data2; //xor
             OR:     result =  data1 | data2; //or
            AND:     result =  data1 & data2; //and
            SLL:     result =  data1 <<data2; //sll
            SRL:     result =  data1 >>data2; //srl
            SRA:     result =  $signed(($signed(data1))>>>data2); //sra
            default: begin result = 0; Zero = 0; end
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
                        else begin ALU_sel = instrct; end //aub,and,or,xor,sll,srl,sra
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
    output reg  [1:0]           ALUOp, MemtoReg, //MemtoReg to select among ALUResult, DataMemo, and PC+4.
    output reg                  Branch, MemRead, MemWrite, ALUSrc, Jump, RegWrite //Jump for jalr.
);
    initial begin
        ALUOp = 0; MemtoReg = 0; Branch = 0; MemRead = 0; MemWrite = 0; ALUSrc = 0; Jump = 0; RegWrite = 0;
    end
   
    always @ (opcode) begin
        case (opcode)
        //I-type,load
            7'b0000011: begin Branch <= 0; ALUSrc <= 1; ALUOp <= 2'b00; MemWrite <= 0; MemRead <= 1; MemtoReg <= 2'b01; Jump <= 0; RegWrite <= 1; end 
//          7'b0001111: 
        //I-type,Imm
            7'b0010011: begin Branch <= 0; ALUSrc <= 1; ALUOp <= 2'b11; MemWrite <= 0; MemRead <= 0; MemtoReg <= 2'b00; Jump <= 0; RegWrite <= 1; end 
//          7'b0010111: 
        //S-type,save
            7'b0100011: begin Branch <= 0; ALUSrc <= 1; ALUOp <= 2'b00; MemWrite <= 1; MemRead <= 0; MemtoReg <= 2'b00; Jump <= 0; RegWrite <= 0; end 
//          7'b0110111: 
        //B-type,brnch
            7'b1100011: begin Branch <= 1; ALUSrc <= 0; ALUOp <= 2'b01; MemWrite <= 0; MemRead <= 0; MemtoReg <= 2'b00; Jump <= 0; RegWrite <= 0; end 
        //I-type,jalr
            7'b1100111: begin Branch <= 1; ALUSrc <= 1; ALUOp <= 2'b00; MemWrite <= 0; MemRead <= 0; MemtoReg <= 2'b10; Jump <= 1; RegWrite <= 1; end 
        //J-type,jal
            7'b1101111: begin Branch <= 1; ALUSrc <= 1; ALUOp <= 2'b00; MemWrite <= 0; MemRead <= 0; MemtoReg <= 2'b10; Jump <= 0; RegWrite <= 1; end 
//          7'b1110011: 
        //R-type,calc
            7'b0110011: begin Branch <= 0; ALUSrc <= 0; ALUOp <= 2'b10; MemWrite <= 0; MemRead <= 0; MemtoReg <= 2'b00; Jump <= 0; RegWrite <= 1; end 
            default:    begin Branch <= 0; ALUSrc <= 0; ALUOp <= 2'b00; MemWrite <= 0; MemRead <= 0; MemtoReg <= 2'b00; Jump <= 0; RegWrite <= 0; end
        endcase
    end
endmodule

module _32_bit_2to1_MUX 
(
    input                      sel,
    input       [31:0]         data1, data2,
    output reg  [31:0]         result
);

    always @ (*) begin
        case (sel)
            1'b0:   result = data1;
            1'b1:   result = data2;
            default:    result = 0;
        endcase
    end
endmodule

module _32_bit_4to2_MUX 
(
    input       [1:0]          sel,
    input       [31:0]         data1, data2, data3, data4,
    output reg  [31:0]         result
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
    parameter reg_rows = 128
)(
    input       [N-1:0]         addr,
    output reg  [N-1:0]         instrct_out
);
    reg         [N-1:0]         Instruction [reg_rows-1:0];
   
    initial begin
        Instruction[0] <= 32'b00011001001100000000001010010011;
        Instruction[1] <= 32'b00000000000000000000000000010011;
        Instruction[2] <= 32'b00000000000000000000000000010011;
        Instruction[3] <= 32'b00000000010100101000001100110011;
        Instruction[4] <= 32'b00000000000000000000000000010011;
        Instruction[5] <= 32'b00000000000000000000000000010011;
        Instruction[6] <= 32'b01000000011000000000001110110011;
        Instruction[7] <= 32'b00000000011000101111111000110011;
        Instruction[8] <= 32'b00000000001000000000111000010011;
        Instruction[9] <= 32'b00000000000000000000000000010011;
        Instruction[10] <= 32'b00000000000000000000000000010011;
        Instruction[11] <= 32'b00000001110000110001001100110011;
        Instruction[12] <= 32'b00000000000000000000000000010011;
        Instruction[13] <= 32'b00000000000000000000000000010011;
        Instruction[14] <= 32'b00000000011000111110001110110011;
        Instruction[15] <= 32'b00000000000000000000000000010011;
        Instruction[16] <= 32'b00000000000000000000000000010011;
        Instruction[17] <= 32'b01110011001000111111111010010011;
        Instruction[18] <= 32'b00000000000000000000000000010011;
        Instruction[19] <= 32'b00000000000000000000000000010011;
        Instruction[20] <= 32'b00000000010111101101111010010011;
        Instruction[21] <= 32'b00000000000000000000000000010011;
        Instruction[22] <= 32'b00000000000000000000000000010011;
        Instruction[23] <= 32'b00000001110000111101001110110011;
        Instruction[24] <= 32'b00000000000000000000000000010011;
        Instruction[25] <= 32'b00000000000000000000000000010011;
        Instruction[26] <= 32'b00000001000000111001001110010011;
        Instruction[27] <= 32'b00000000000000000000000000010011;
        Instruction[28] <= 32'b00000000000000000000000000010011;
        Instruction[29] <= 32'b01000001110000111101001110110011;
        Instruction[30] <= 32'b00000000011000101001100001100011;
        Instruction[31] <= 32'b00000000000000000000000000010011;
        Instruction[32] <= 32'b00000000000000000000000000010011;
        Instruction[33] <= 32'b00000000000000000000001110110011;
        Instruction[34] <= 32'b00000010000000111000100001100011;
        Instruction[35] <= 32'b00000000000000000000000000010011;
        Instruction[36] <= 32'b00000000000000000000000000010011;
        Instruction[37] <= 32'b00000011110100111101001001100011;
        Instruction[38] <= 32'b00000000000000000000000000010011;
        Instruction[39] <= 32'b00000000000000000000000000010011;
        Instruction[40] <= 32'b00000000000000111000001010110011;
        Instruction[41] <= 32'b00000000000000000000000000010011;
        Instruction[42] <= 32'b00000000000000000000000000010011;
        Instruction[43] <= 32'b00000001110100111100100001100011;
        Instruction[44] <= 32'b00000000000000000000000000010011;
        Instruction[45] <= 32'b00000000000000000000000000010011;
        Instruction[46] <= 32'b00000000000000000000001010110011;
        Instruction[47] <= 32'b00000000010100111001110001100011;
        Instruction[48] <= 32'b00000000000000000000000000010011;
        Instruction[49] <= 32'b00000000000000000000000000010011;
        Instruction[50] <= 32'b00000000010100111000100001100011;
        Instruction[51] <= 32'b00000000000000000000000000010011;
        Instruction[52] <= 32'b00000000000000000000000000010011;
        Instruction[53] <= 32'b00000000000000000000001100110011;
        Instruction[54] <= 32'b00000001110100110100110001100011;
        Instruction[55] <= 32'b00000000000000000000000000010011;
        Instruction[56] <= 32'b00000000000000000000000000010011;
        Instruction[57] <= 32'b00000001110000110101100001100011;
        Instruction[58] <= 32'b00000000000000000000000000010011;
        Instruction[59] <= 32'b00000000000000000000000000010011;
        Instruction[60] <= 32'b00000000000000000000111000110011;
        Instruction[61] <= 32'b00000000000000000000111010110011;
        Instruction[62] <= 32'b00000001100000000000000011101111;
        Instruction[63] <= 32'b00000000000000000000000000010011;
        Instruction[64] <= 32'b00000000000000000000000000010011;
        Instruction[65] <= 32'b00000010011100101000100001100011;
        Instruction[66] <= 32'b00000000000000000000000000010011;
        Instruction[67] <= 32'b00000000000000000000000000010011;
        Instruction[68] <= 32'b00000000010100010010000000100011;
        Instruction[69] <= 32'b00000000011000010000001000100011;
        Instruction[70] <= 32'b00000000000000010010111010000011;
        Instruction[71] <= 32'b00000000010000010000111010000011;
        Instruction[72] <= 32'b00000000010000010100111010000011;
        Instruction[73] <= 32'b00000000000000001000000001100111;
        Instruction[74] <= 32'b00000000000000000000000000010011;
        Instruction[75] <= 32'b00000000000000000000000000010011;
        Instruction[76] <= 32'b00000000000000000000000010110011;
        Instruction[77] <= 32'b00000000000000000000001110110011;
        Instruction[78] <= 32'b00000000000000000000000000010011;
        Instruction[79] <= 32'b00000000000000000000000000010011;
        Instruction[80] <= 32'b00000000000000000000000000010011;
        
    end   
    always @ (*) begin
        instrct_out = Instruction[(addr>>2)];
    end
endmodule

module Data_Memory #
(
    parameter N = 32, parameter LSW = 3'b010, parameter LSH = 3'b001, parameter LSB = 3'b000, parameter LBU = 3'b100,
    parameter reg_rows = 32'h00000010
)(
    input                       W_en, R_en,
    input       [2:0]           Funct3,
    input       [N-1:0]         addr, W_data,
    output reg  [N-1:0]         R_data_out
);
    reg         [7:0]           data[reg_rows-1:0];
    
    //Write to Data Memory
    always @ (*) begin
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

module IF_ID_State_Reg(
    input                   clock,
    input       [31:0]      crntPC,
    input       [31:0]      nextPC,
    input       [31:0]      Instrct,
    output reg  [31:0]      crntPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Instrct_out
);

    initial begin
        crntPC_out = 0; nextPC_out = 0; Instrct_out = 0;
    end

    always @ (posedge clock) begin
        crntPC_out      <= crntPC;
        nextPC_out      <= nextPC;
        Instrct_out     <= Instrct;
    end
    
endmodule

module ID_EX_State_Reg(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input                   Branch,     //MEM
    input                   Jump,       //EX
    input                   ALUSrc,     //EX
    input       [1:0]       ALUOp,      //EX
    input       [31:0]      crntPC,
    input       [31:0]      nextPC,
    input       [31:0]      Reg_rs1,
    input       [31:0]      Reg_rs2,
    input       [31:0]      Imm_Gen,
    input       [4:0]       Reg_rd,
    input       [3:0]       ALU_Instrct,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg              Branch_out,     //MEM
    output reg              Jump_out,       //EX
    output reg              ALUSrc_out,     //EX
    output reg  [1:0]       ALUOp_out,      //EX
    output reg  [31:0]      crntPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Reg_rs1_out,
    output reg  [31:0]      Reg_rs2_out,
    output reg  [31:0]      Imm_Gen_out,
    output reg  [4:0]       Reg_rd_out,
    output reg  [3:0]       ALU_Instrct_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; Branch_out = 0; Jump_out = 0; ALUSrc_out = 0; 
        ALUOp_out = 0; crntPC_out = 0; nextPC_out = 0; Reg_rs1_out = 0; Reg_rs2_out = 0; Imm_Gen_out = 0; Reg_rd_out = 0; ALU_Instrct_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Branch_out      <= Branch;
        Jump_out        <= Jump;
        ALUSrc_out      <= ALUSrc;
        ALUOp_out       <= ALUOp;
        crntPC_out      <= crntPC;
        nextPC_out      <= nextPC;
        Reg_rs1_out     <= Reg_rs1;
        Reg_rs2_out     <= Reg_rs2;
        Reg_rd_out      <= Reg_rd;
        Imm_Gen_out     <= Imm_Gen;
        ALU_Instrct_out <= ALU_Instrct;
    end

endmodule

module EX_MEM_State_Reg(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input                   Branch,     //MEM
    input                   Zero,
    input       [31:0]      nextPC,
    input       [31:0]      ALUResult,
    input       [31:0]      AddSum,
    input       [31:0]      Reg_rs2,
    input       [2:0]       Funct3,
    input       [4:0]       Reg_rd,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg              Branch_out,     //MEM
    output reg              Zero_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [31:0]      AddSum_out,
    output reg  [31:0]      Reg_rs2_out,
    output reg  [2:0]       Funct3_out,
    output reg  [4:0]       Reg_rd_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; Branch_out = 0; Zero_out = 0; nextPC_out = 0; 
        ALUResult_out = 0; AddSum_out = 0; Reg_rs2_out = 0; Funct3_out = 0; Reg_rd_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Branch_out      <= Branch;
        Zero_out        <= Zero;
        nextPC_out      <= nextPC;
        ALUResult_out   <= ALUResult;
        AddSum_out      <= AddSum;
        Reg_rs2_out     <= Reg_rs2;
        Funct3_out      <= Funct3;
        Reg_rd_out      <= Reg_rd;
    end

endmodule

module MEM_WB_State_Reg(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input       [31:0]      nextPC,
    input       [31:0]      ReadData,
    input       [31:0]      ALUResult,
    input       [4:0]       Reg_rd,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ReadData_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [4:0]       Reg_rd_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; nextPC_out = 0; ReadData_out = 0; ALUResult_out = 0; Reg_rd_out = 0;
    end
    
    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        nextPC_out      <= nextPC;
        ReadData_out    <= ReadData;
        ALUResult_out   <= ALUResult;
        Reg_rd_out      <= Reg_rd;
    end

endmodule

module Pipeline_Processor(input clock); //Top Module

    wire            [31:0]          Instrct, Imm, Imm_shift, data_1, data_2, MUX_ALU, loadPC, W_data, Add_1, Add_2, ALUResult, Read_Mem, JumpTo;
    wire            [31:0]          IF_ID_nextPC, IF_ID_crntPC, IF_ID_Instrct;
    wire            [31:0]          ID_EX_nextPC, ID_EX_crntPC, ID_EX_data_1, ID_EX_data_2, ID_EX_Imm;
    wire            [31:0]          EX_MEM_nextPC, EX_MEM_JumpTo, EX_MEM_ALUResult, EX_MEM_data_2;
    wire            [31:0]          MEM_WB_nextPC, MEM_WB_Read_Mem, MEM_WB_ALUResult;
    wire            [4:0]           ID_EX_Reg_rd, EX_MEM_Reg_rd, MEM_WB_Reg_rd;
    wire            [3:0]           ALUControl, ID_EX_ALUInstrct;
    wire            [2:0]           EX_MEM_Funct3;
    wire            [1:0]           ALUOp, MemtoReg, ID_EX_ALUOp, ID_EX_MemtoReg, EX_MEM_MemtoReg, MEM_WB_MemtoReg;
    wire                            Branch, isBranch, MemRead, MemWrite, ALUSrc, isJump, RegWrite, isZero;
    wire                            ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_isJump, ID_EX_RegWrite;
    wire                            EX_MEM_Branch, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_isZero, MEM_WB_RegWrite;                         
    reg             [31:0]          PC;  
        
    initial PC = 0;   
      
    assign Imm_shift = {ID_EX_Imm[30:0],1'b0};
    always @ (posedge clock ) PC <= loadPC;
    
    Instruction_Memory IM (
        .instrct_out (Instrct),
        .addr (PC)
    );
    Central_Control_Unit CU (
        .opcode (IF_ID_Instrct[6:0]),
        .Branch (Branch),
        .MemRead (MemRead),
        .MemWrite (MemWrite),
        .MemtoReg (MemtoReg),
        .ALUOp (ALUOp),
        .ALUSrc (ALUSrc),
        .Jump (isJump),
        .RegWrite (RegWrite)
    );
    Register_File RF (
        .clock (clock),
        .W_en (MEM_WB_RegWrite),
        .W_data (W_data),
        .R_addr_1 (IF_ID_Instrct[19:15]),
        .R_addr_2 (IF_ID_Instrct[24:20]),
        .W_addr (MEM_WB_Reg_rd),
        .R_data_1 (data_1),
        .R_data_2 (data_2)
    );
    Immediate_Generator ImmGen (
        .In (IF_ID_Instrct),
        .Out (Imm)
    );
    N_bit_ALU ALU (     
        .data1 (ID_EX_data_1),
        .data2 (MUX_ALU),
        .sel (ALUControl),
        .Zero (isZero),
        .result (ALUResult)
    ); 
    ALU_Control ALU_Ctrl (
        .ALU_op (ID_EX_ALUOp),
        .instrct (ID_EX_ALUInstrct),
        .ALU_sel (ALUControl)
    );
    Data_Memory DM (
        .W_en (EX_MEM_MemWrite),
        .R_en (EX_MEM_MemRead),
        .Funct3 (EX_MEM_Funct3),
        .addr (EX_MEM_ALUResult),
        .W_data (EX_MEM_data_2),
        .R_data_out (Read_Mem)        
    );
    
    IF_ID_State_Reg IF_ID(
        .clock (clock),
        .crntPC (PC),
        .nextPC (Add_1),
        .Instrct (Instrct),
        .crntPC_out (IF_ID_crntPC),
        .nextPC_out (IF_ID_nextPC),
        .Instrct_out (IF_ID_Instrct)
    );
    ID_EX_State_Reg ID_EX(
        .clock (clock),
        .RegWrite (RegWrite),
        .MemtoReg (MemtoReg),
        .MemRead (MemRead),
        .MemWrite (MemWrite),
        .Branch (Branch),
        .Jump (isJump),
        .ALUSrc (ALUSrc),
        .ALUOp (ALUOp),
        .crntPC (IF_ID_crntPC),
        .nextPC (IF_ID_nextPC),
        .Reg_rs1 (data_1),
        .Reg_rs2 (data_2),
        .Imm_Gen (Imm),
        .Reg_rd (IF_ID_Instrct[11:7]),
        .ALU_Instrct ({IF_ID_Instrct[30],IF_ID_Instrct[14:12]}),
        .RegWrite_out (ID_EX_RegWrite),
        .MemtoReg_out (ID_EX_MemtoReg),
        .MemRead_out (ID_EX_MemRead),
        .MemWrite_out (ID_EX_MemWrite),
        .Branch_out (ID_EX_Branch),
        .Jump_out (ID_EX_isJump),
        .ALUSrc_out (ID_EX_ALUSrc),
        .ALUOp_out (ID_EX_ALUOp),
        .crntPC_out (ID_EX_crntPC),
        .nextPC_out (ID_EX_nextPC),
        .Reg_rs1_out (ID_EX_data_1),
        .Reg_rs2_out (ID_EX_data_2),
        .Imm_Gen_out (ID_EX_Imm),
        .Reg_rd_out (ID_EX_Reg_rd),
        .ALU_Instrct_out (ID_EX_ALUInstrct)
    );
    EX_MEM_State_Reg EX_MEM(
        .clock (clock),
        .RegWrite (ID_EX_RegWrite),
        .MemtoReg (ID_EX_MemtoReg),
        .MemRead (ID_EX_MemRead),
        .MemWrite (ID_EX_MemWrite),
        .Branch (ID_EX_Branch),
        .Zero (isZero),
        .nextPC (ID_EX_nextPC),
        .ALUResult (ALUResult),
        .AddSum (JumpTo),
        .Reg_rs2 (ID_EX_data_2),
        .Funct3 (ID_EX_ALUInstrct[2:0]),
        .Reg_rd (ID_EX_Reg_rd),
        .RegWrite_out (EX_MEM_RegWrite),
        .MemtoReg_out (EX_MEM_MemtoReg),
        .MemRead_out (EX_MEM_MemRead),
        .MemWrite_out (EX_MEM_MemWrite),
        .Branch_out (EX_MEM_Branch),
        .Zero_out (EX_MEM_isZero),
        .nextPC_out (EX_MEM_nextPC),
        .ALUResult_out (EX_MEM_ALUResult),
        .AddSum_out (EX_MEM_JumpTo),
        .Reg_rs2_out (EX_MEM_data_2),
        .Funct3_out (EX_MEM_Funct3),
        .Reg_rd_out (EX_MEM_Reg_rd)
    );
    MEM_WB_State_Reg MEM_WB(
        .clock (clock),
        .RegWrite (EX_MEM_RegWrite),
        .MemtoReg (EX_MEM_MemtoReg),
        .nextPC (EX_MEM_nextPC),
        .ReadData (Read_Mem),
        .ALUResult (EX_MEM_ALUResult),
        .Reg_rd (EX_MEM_Reg_rd),
        .RegWrite_out (MEM_WB_RegWrite),
        .MemtoReg_out (MEM_WB_MemtoReg),
        .nextPC_out (MEM_WB_nextPC),
        .ReadData_out (MEM_WB_Read_Mem),
        .ALUResult_out (MEM_WB_ALUResult),
        .Reg_rd_out (MEM_WB_Reg_rd)
    );
    
    //Mux: Input_1, Input_2, (Input_3, Input_4,) Select, Output.
//    _32_bit_2to1_MUX  Mux_1(.data1(Add_1), .data2(EX_MEM_JumpTo), .sel(isBranch), .result(loadPC));
    _32_bit_2to1_MUX  Mux_1(.data1(Add_1), .data2(JumpTo), .sel(isBranch), .result(loadPC));
    _32_bit_2to1_MUX  Mux_2(.data1(ID_EX_data_2), .data2(ID_EX_Imm), .sel(ID_EX_ALUSrc), .result(MUX_ALU));
    _32_bit_2to1_MUX  Mux_3(.data1(Add_2), .data2(ALUResult), .sel(ID_EX_isJump), .result(JumpTo)); //Lab4_latest
    _32_bit_4to2_MUX  Mux_4(.data1(MEM_WB_ALUResult), .data2(MEM_WB_Read_Mem), .data3(MEM_WB_nextPC), .data4(MEM_WB_nextPC),
                            .sel(MEM_WB_MemtoReg), .result(W_data)); //Lab4_latest
    
    //Adder: Input_1, Input_2, Output.
    _32_bit_Adder Adder_1(.data1(PC), .data2(32'h00000004), .result(Add_1)); //PC+4
    _32_bit_Adder Adder_2(.data1(ID_EX_crntPC), .data2(Imm_shift), .result(Add_2)); //PC+Imm
    
//    and (isBranch, EX_MEM_Branch, EX_MEM_isZero);
    and (isBranch, ID_EX_Branch, isZero); 
    
endmodule
