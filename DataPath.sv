`timescale 1ns/1ns

module MUX32Bit2input (input[31:0]ZEROsel, ONEsel, input selector, output reg [32:0]out);
    always @(ZEROsel, ONEsel, selector)begin
        out = 32'b0;
        if (selector == 1'b0)
            out = ZEROsel
        else if (selector == 1'b1)
            out = ONEsel
    end
endmodule

module  MUX5Bit2input(input [4:0]ZEROsel, ONEsel, input selector, output reg [4:0] out);
    always @(ZEROsel, ONEsel, selector) begin
        out = 5'b0;
        if(selector == 1'b0) 
            out = ZEROsel;
        else if (selector == 1'b1)
            out = ONEsel;
    end
endmodule

module MUX32Bit4input (input [31:0]ZEROsel, ONEsel, TWOsel, THREEsel, input [1:0]selector, output reg [31:0]out);
    always @(ZEROsel, ONEsel, TWOsel, THREEsel, selector)begin
        out = 32'b0;
        if (selector == 2'b00)
            out = ZEROsel;
        else if (selector == 2'b01)
            out = ONEsel;
        else if (selector == 2'b10)
            out = TWOsel;
        else if (selector == 2'b11)
            out = THREEsel;
    end
endmodule

module  SignExtender(input [15:0]in, output reg [31:0] out);
    always @(in) begin
        out = 32'b0;
        out = 32'(signed'(in));
    end
endmodule
    
module ZEROExtender(input [25:0]in, output reg [27:0]out);
    always @(in) begin
        out = 28'b0000000000000000000000000000; 
        out = {in, 2'b00};
    end
endmodule

module ShiftLeft2bit(input [31:0]in, output reg [31:0]out);
    always @(in) begin
        out = 32'b0;
        out = {in[29:0], 2'b00};
    end
endmodule

module ALU32Bit (input [31:0]A, B, input [2:0]ALUop, output reg[31:0] ALUout, output reg ZeroFlag);
    always @(A, B, ALUop) begin
        ALUout = 32'b0;
        ZeroFlag = 1'b0;
        if (ALUop == 3'b000) // case AND
            ALUout = A & B;
        else if (ALUop == 3'b001) // case OR
            ALUout = A | B;
        else if (ALUop == 3'b010) // case ADD
            ALUout = A + B;
        else if (ALUop == 3'b110) // case SUB
            ALUout = A - B;
        else if (ALUop == 3'b111)begin // case SLT
            if ($signed(A) < $signed(B))  // supposed that A and B are signed check whether comparison happens right?
                ALUout = 32'b00000000000000000000000000000001;
            else
                ALUout = 32'b00000000000000000000000000000000;
        end
        if(ALUout == 32'b0)
            ZeroFlag = 1'b1;
    end
endmodule

module Register32BitWithLoad(input [31:0]in, input rst, clk, ldin, initPC, output reg[31:0]out);
    always @(posedge clk, posedge rst) begin
        if (rst)
            out <= 32'b0;
        else if (initPC)
            out <= 32'b0;
        else if (ldin)
            out <= in;
    end
endmodule

// Note that I didn't put init property for registers without load property, if you think that's needed Please add it!!!
module Register32BitWithoutLoad (input [31:0]in, input rst, clk, output reg [31:0]out);
    always@(posedge clk, posedge rst) begin
        if (rst)
            out <= 32'b0;
        else 
            out <= in;
    end
endmodule

module RegFile (input [4:0]ReadReg1, ReadReg2, input [4:0]WriteReg, input [31:0]Writedata, input clk, rst, regWriteSignal, output reg [31:0]ReadData1, ReadData2);
    reg [31:0] REGFILE [0:31];

    always @(ReadReg1, ReadReg2) begin
        {ReadData1, ReadData2} = 64'b0;
        ReadData1 = REGFILE[ReadReg1];
        ReadData2 = REGFILE[ReadReg2];
    end
    always @(posedge clk, posedge rst)begin
        {ReadData1, ReadData2} = 64'b0;
        if (rst) begin
            for(integer i = 0; i < 32; i++) begin
                REGFILE[i] = 32'b0;
            end
        end
        if (regWriteSignal)
            REGFILE[WriteReg] = Writedata;
    end
endmodule

module Memory (input [31:0]Address, WriteData, input MemRead, MemWrite, clk, rst, output reg [31:0]ReadData);
    // I've defined Memory size as below if you think it should be more, modify it
    reg [31:0] Mem [0:512];
    always @(negedge rst) begin
        $readmemb("Memory.mem", Mem);
    end
    // Check this memory difference with CA#2 memories, I merged all always statements with each other
    always @(posedge clk, posedge rst, Address, MemRead) begin
        ReadData = 32'b00000000000000000000000000000000;
        if (rst) begin
            // Check that this for statement works well, cause I think it should go exactly through 512
            for (integer i = 0; i < 512; i++) begin
                Mem[i] = 32'b00000000000000000000000000000000;
            end
        end
        else if (MemRead)
            ReadData = Mem[Address[31:2]];
        else if (MemWrite)
            Mem[Address[31:2]] = WriteData;
    end
endmodule




