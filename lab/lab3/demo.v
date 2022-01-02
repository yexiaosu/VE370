`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/10/15 13:37:31
// Design Name: 
// Module Name: lab3
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


module top(clk);
    input clk;
    
    reg[31:0] pc_current=0;
    wire [31:0] pc_next,pc4,pcb;
    wire [31:0] inst,writeData,readData1,readData2,imm,ALUin,ALUresult,readDataMem,immout;
    wire branch,memRead,memToReg,memWrite,ALUSrc,regWrite,zero,PCselect;
    wire [1:0] ALUOp;
    wire [3:0] ALUCon;
    
    always @(posedge clk)  
        pc_current <= pc_next;

    // instruction memory
    instrMem instrucionMemory(pc_current,inst);
    control Control(inst[6:0],branch,memRead,memToReg,ALUOp,memWrite,ALUSrc,regWrite);
    regFile Registers(inst[19:15], inst[24:20], inst[11:7], writeData, regWrite, readData1, readData2,clk);
    immGen immGenerator(inst,imm);
    ALUcontrol ALUControl({inst[30],inst[14:12]},ALUOp,ALUCon);
    mux mux1(readData2,imm,ALUSrc,ALUin);
    ALU ALU1(readData1, ALUin, ALUCon, ALUresult, zero);
    dataMem dataMemory(ALUresult,memWrite,memRead,readData2,readDataMem,clk);
    mux mux2(ALUresult,readDataMem,memToReg,writeData);
    shift shifter(imm,immout);
    assign PCselect = (branch & zero);
    adder add1(pc_current,32'b00000000000000000000000000000100,pc4);
    adder add2(pc_current,immout,pcb);
    mux mux3(pc4,pcb,PCselect,pc_next);
    
endmodule

module adder(in1,in2,sum);
    input [31:0] in1, in2;
    output [31:0] sum;
    
    assign sum = in1+in2;
endmodule

module mux(in1,in2,s,out);
    input [31:0] in1, in2;
    input s;
    output [31:0] out;
    
    reg [31:0] out;

    always @(in1, in2, s) begin
        case (s)
            1'b0: out = in1;
            1'b1: out = in2;
            default out = 0;
        endcase
    end
endmodule

module ALU(in1, in2, s, out, zero);
    input [31:0] in1, in2;
    input [3:0] s;
    output [31:0] out;
    output zero;
    
    reg [31:0] out;
    reg zero;

    always @(in1, in2, s) begin
        case (s)
            4'b0000: begin out = in1&in2; zero = 1'b0; end
            4'b0001: begin out = in1|in2; zero = 1'b0; end
            4'b0010: begin out = in1+in2; zero = 1'b0; end
            4'b0110: begin
                        out = in1-in2;
                        if (out==0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            4'b0111: begin // bne
                        out = in1-in2;
                        if (out!=0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            default begin out = 0; zero = 1'b0; end
        endcase
    end
endmodule

module ALUcontrol (inst,aluop,alucon);
    input [3:0] inst;
    input [1:0] aluop;
    output [3:0] alucon;
    
    reg [3:0] alucon;

    always @(inst, aluop) begin
        case (aluop)
            2'b00: begin
                    case (inst)
                        4'b0010: alucon = 4'b0010;
                        4'b1010: alucon = 4'b0010;
                        default alucon = 1;
                    endcase
                   end
            2'b01: begin
                    case (inst)
                        4'b0000: alucon = 4'b0110;
                        4'b1000: alucon = 4'b0110;
                        4'b0001: alucon = 4'b0111;
                        4'b1001: alucon = 4'b0111;
                        default alucon = 1;
                    endcase
                   end
            2'b10: begin
                    case (inst)
                        4'b0000: alucon = 4'b0010;
                        4'b1000: alucon = 4'b0110;
                        4'b0111: alucon = 4'b0000;
                        4'b0110: alucon = 4'b0001;
                        default alucon = 1;
                    endcase
                   end
            2'b11: begin
                    case (inst)
                        4'b0000: alucon = 4'b0010;
                        4'b1000: alucon = 4'b0010;
                        default alucon = 1;
                    endcase
                   end
            default alucon = 1;
        endcase
    end
endmodule

module regFile(readReg1, readReg2, writeReg, writeData, regWrite, readData1, readData2,clk);
    input [4:0] readReg1, readReg2, writeReg;
    input [31:0] writeData;
    input regWrite,clk;
    output [31:0] readData1, readData2;
    
    integer i;
    wire [31:0] readData1, readData2;
    reg [31:0] regs [31:0];
    
    initial 
        begin  
           for(i=0;i<32;i=i+1)  
                regs[i] <= 0;  
        end  
    
    always @ (posedge clk) begin
        if (regWrite) regs[writeReg] <= writeData;
    end
    assign readData1 = ( readReg1 == 0)? 32'b0 : regs[readReg1];  
    assign readData2 = ( readReg2 == 0)? 32'b0 : regs[readReg2];   
endmodule

module immGen(instr,imm);
    input [31:0] instr;
    output [31:0] imm;
    
    reg [31:0] imm;
    integer j;
    
    always @ (instr)
        if (instr[6:0] == 7'b0010111 || instr[6:0] == 7'b0110111) // U
        begin
            for(j=0;j<20;j=j+1)
                imm[j]=instr[j+12];
            for(j=20;j<32;j=j+1)
                imm[j]=0;
        end
        else if (instr[6:0] == 7'b1101111) // J
        begin
            imm[19]=instr[31];
            imm[9:0]=instr[30:21];
            imm[10]=instr[20];
            imm[18:11]=instr[19:12];
            for(j=20;j<32;j=j+1)
                imm[j]=imm[19];
        end
        else if (instr[6:4] == 3'b000 || instr[6:4] == 3'b001 || instr[6:4] == 3'b111 || instr[6:0] == 7'b1100111) // I
        begin
            imm[11:0]=instr[31:20];
            for(j=12;j<32;j=j+1)
                imm[j]=imm[11];
        end
        else if (instr[6:4] == 3'b010) // S
        begin
            imm[11:5]=instr[31:25];
            imm[4:0]=instr[11:7];
            for(j=12;j<32;j=j+1)
                imm[j]=imm[11];
        end
        else if (instr[6:4] == 3'b110) // B
        begin
            imm[11]=instr[31];
            imm[9:4]=instr[30:25];
            imm[3:0]=instr[11:8];
            imm[10]=instr[7];
            for(j=12;j<32;j=j+1)
                imm[j]=imm[11];
        end
endmodule

module control(inst,branch,memRead,memToReg,ALUOp,memWrite,ALUSrc,regWrite);
    input [6:0] inst;
    output branch,memRead,memToReg,memWrite,ALUSrc,regWrite;
    output [1:0] ALUOp;
    
    reg branch,memRead,memToReg,memWrite,ALUSrc,regWrite;
    reg [1:0] ALUOp;
    
    always @ (inst)
        case(inst)
            7'b0110011: begin // add, sub, and, or
                            ALUOp = 2'b10;
                            branch = 0;
                            memRead = 0;
                            memToReg = 0;
                            memWrite = 0;
                            ALUSrc = 0;
                            regWrite = 1;
                        end
            7'b0010011: begin // addi
                            ALUOp = 2'b11;
                            branch = 0;
                            memRead = 0;
                            memToReg = 0;
                            memWrite = 0;
                            ALUSrc = 1;
                            regWrite = 1;
                        end
            7'b0000011: begin // lw
                            ALUOp = 2'b00;
                            branch = 0;
                            memRead = 1;
                            memToReg = 1;
                            memWrite = 0;
                            ALUSrc = 1;
                            regWrite = 1;
                        end
            7'b0100011: begin // sw
                            ALUOp = 2'b00;
                            branch = 0;
                            memRead = 0;
                            memToReg = 0;
                            memWrite = 1;
                            ALUSrc = 1;
                            regWrite = 0;
                        end
            7'b1100011: begin // beq, bne
                            ALUOp = 2'b01;
                            branch = 1;
                            memRead = 0;
                            memToReg = 0;
                            memWrite = 0;
                            ALUSrc = 0;
                            regWrite = 0;
                        end
        endcase
endmodule

module shift(in,out);
    input [31:0] in;
    output [31:0] out;
    
    reg [31:0] out;

    always @ (in)
        begin
            out[31:1] = in[30:0];
            out[0] = 0;
        end
endmodule

module instrMem(pc,inst);
    input [31:0] pc;
    output [31:0] inst;
    
    reg [31:0] rom[31:0];
    
    initial  
      begin  
                rom[0] = 32'b11111111011000000000001010010011;
                rom[1] = 32'b00000000010100101000001100110011;
                rom[2] = 32'b01000000011000101000001110110011;
                rom[3] = 32'b00000000000000110111111000110011;
                rom[4] = 32'b00000000010100110110111010110011;
                rom[5] = 32'b00000001110100000010000000100011;
                rom[6] = 32'b00000000010100000010001000100011;
                rom[7] = 32'b00000000000000101000010001100011;
                rom[8] = 32'b00000000000000110000111010110011;
                rom[9] = 32'b00000001110100110001010001100011;
                rom[10] = 32'b00000001110000110001010001100011;
                rom[11] = 32'b00000000000000000000001110110011;
                rom[12] = 32'b00000000000000000010010000000011;
                rom[13] = 32'b00000000010000000010010010000011;
                rom[14] = 32'b00000000100001001000010010010011;
                rom[15] = 32'b00000000100101000000010001100011;
                rom[16] = 32'b00000000000000000000001110110011;
                rom[17] = 32'b00000000011100111000001110110011;
      end
    assign inst = (pc[31:2] < 32 )? rom[pc[31:2]]: 0;  
endmodule

module dataMem(addr,memWrite,memRead,writeData,readData,clk);
    input memWrite,memRead,clk;
    input [31:0] addr,writeData;
    output [31:0] readData;
    
    integer i;
    reg [7:0] ram [127:0];
    initial 
        begin  
           for(i=0;i<128;i=i+1)  
                ram[i] <= 4'b0000;  
        end  
    always @(posedge clk)
        begin  
           if (memWrite)  begin
                ram[addr] <= writeData[7:0];  
                ram[addr+1] <= writeData[15:8];  
                ram[addr+2] <= writeData[23:16];  
                ram[addr+3] <= writeData[31:14];  end
        end  
    assign readData = (memRead==1'b1) ? {ram[addr+3],ram[addr+2],ram[addr+1],ram[addr]}: 0;   
endmodule
