`timescale 1ns/1ns

module MIPSprocessor (input clk, rst);
    
    wire PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA, ZeroFlag;
    wire [1:0] ALUSrcB, PCSrc;
    wire [2:0] ALUoperation;
    wire [31:0] Instruction;
    MIPSDatapath DPUnit(PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA, rst, clk, ALUSrcB, PCSrc, ALUoperation, ZeroFlag, Instruction);
    Controller CUnit(ZeroFlag, Instruction, clk, rst, PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA, ALUSrcB, PCSrc, ALUoperation);

endmodule
