`timescale 1ns/1ns

module MUX32Bit2input (input[31:0]ZEROsel, ONEsel, input selector, output reg [32:0]out);
    always @(ZEROsel, ONEsel, selector)begin
        out = 32'b0;
        if (selector == 1'b0)
            out = ZEROsel;
        else if (selector == 1'b1)
            out = ONEsel;
    end
endmodule

module MUX5Bit2input(input [4:0]ZEROsel, ONEsel, input selector, output reg [4:0] out);
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

module SignExtender(input [15:0]in, output reg [31:0] out);
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

module Register32BitWithLoad(input [31:0]in, input rst, clk, ldin, output reg[31:0]out);
    always @(posedge clk, posedge rst) begin
        if (rst)
            out <= 32'b0;
        // Init property has been removed
        else if (ldin)
            out <= in;
    end
endmodule

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
    reg [31:0] Mem [0:512];
    always @(negedge rst) begin
        $readmemb("Memory2.mem", Mem); // Change it to Memory1.mem or Memory2.mem for 1 and 2!
    end
    always @(Address, MemRead, negedge rst) begin
        ReadData = 32'b00000000000000000000000000000000;
        if (MemRead)
            ReadData = Mem[Address[31:2]];
    end
    always@( posedge clk, posedge rst) begin
        if (rst)
            for(integer i = 0; i < 512; i++)begin
                Mem[i] = 32'b00000000000000000000000000000000;
            end
        else if (MemWrite)
            Mem[Address[31:2]] = WriteData;
    end
endmodule

module MIPSDatapath (input PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA, rst, clk, input [1:0]ALUSrcB, PCSrc, input [2:0]ALUoperation, output ZeroFlag, output [31:0]Instruction);
    wire PCload, Andout, zeroflagout;
    wire [31:0] PCWire, AddressWire, DataWire, IRout, MDRout, MemtoRegWire, WDataInput, ReadData1Wire, ReadData2Wire, RegAout, RegBout, container4, SEXTout, Shl2Sextout, SrcAin, SrcBin, mainALUout, ALUoutRegWire, PCinWire, ZeroExtout2;
    wire [4:0] RegDstWire, container31, WriteRegInput;
    wire [27:0] ZeroExtout;
    
    assign PCload = PCWrite | Andout;
    assign Andout = PCWriteCond & zeroflagout;
    assign container31 = 5'b11111; // 5bit 31 value
    assign container4 = 32'b00000000000000000000000000000100; // 32bit 4 value
    assign ZeroExtout2 = {PCWire[31:28], ZeroExtout};

    MUX32Bit2input MUX1(PCWire, ALUoutRegWire, IorD, AddressWire);
    MUX5Bit2input MUX2(IRout[20:16], IRout[15:11], RegDst, RegDstWire);
    MUX5Bit2input MUX3(RegDstWire, container31, WriteRegSel, WriteRegInput);

    MUX32Bit2input MUX4(ALUoutRegWire, MDRout, MemtoReg, MemtoRegWire);
    MUX32Bit2input MUX5(MemtoRegWire, PCWire, WriteDataSel, WDataInput);
    MUX32Bit2input MUX6(PCWire, RegAout, ALUSrcA, SrcAin);

    MUX32Bit4input MUX7(RegBout, container4, SEXTout, Shl2Sextout, ALUSrcB, SrcBin);
    MUX32Bit4input MUX8(mainALUout, ZeroExtout2, ALUoutRegWire, RegAout, PCSrc, PCinWire);

    SignExtender SignExtend(IRout[15:0], SEXTout);
    ShiftLeft2bit Shl2sext(SEXTout, Shl2Sextout);
    ZEROExtender ZeroExt(IRout[25:0], ZeroExtout);

    Register32BitWithLoad PC(PCinWire, rst, clk, PCload, PCWire);
    Register32BitWithLoad IR(DataWire, rst, clk, IRWrite, IRout);

    Register32BitWithoutLoad MDR(DataWire, rst, clk, MDRout);
    Register32BitWithoutLoad A(ReadData1Wire, rst, clk, RegAout);
    Register32BitWithoutLoad B(ReadData2Wire, rst, clk, RegBout);
    Register32BitWithoutLoad ALUout(mainALUout, rst, clk, ALUoutRegWire);

    ALU32Bit MainALU (SrcAin, SrcBin, ALUoperation, mainALUout, zeroflagout);

    Memory MainMemory(AddressWire, RegBout, MemRead, MemWrite, clk, rst, DataWire);
    RegFile MainRegFile(IRout[25:21], IRout[20:16], WriteRegInput, WDataInput,  clk, rst, RegWrite, ReadData1Wire, ReadData2Wire);

    // output assignment
    assign ZeroFlag = zeroflagout;
    assign Instruction = IRout;


endmodule


