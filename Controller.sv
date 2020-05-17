`timescale 1ns/1ns

module Controller (input zeroflag, input [31:0]instruction, input clk, rst, output reg PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA, output reg [1:0]ALUSrcB, PCSrc, output reg [2:0]ALUoperation);
    parameter [3:0] IF = 4'b0000, ID = 4'b0001, JumpComplete = 4'b0010, branchComplete = 4'b0011, RTstart = 4'b0100, RTcomplete = 4'b0101, MemRefStart = 4'b0110, SWcomplete = 4'b0111, LWstart = 4'1000, LWcomplete = 4'b1001, JumpRcomplete = 4'b1010, JALcomplete = 4'b1011;
    // parameter [3:0] add = 4'b0000, sub = 4'b0001, and = 4'b0010, or = 4'b0011, slt = 4'b0100, addi = 4'b0101, andi = 4'b0110,  lw = 4'b0111, sw = 4'b1000, j = 4'b1001, jal = 4'b1010, jr = 4'b1011, beq = 4'b1100, bne = 4'b1101;

    reg [3:0]ps, ns;
    wire [5:0]functionType;
    wire [5:0]instructionType;
    assign Opcode = instruction[31:26];
    assign functionType = instruction[5:0];
    always @(zeroflag, instruction, ps)begin
        {PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA} = 12'b0;
        ALUSrcB = 2'b00;
        PCSrc = 2'b00;
        ALUoperation = 3'b000;
        ns = IF;
        case(ps)
            IF : begin
                IorD = 1'b0;
                MemRead = 1'b1;
                IRWrite = 1'b1;
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b01;
                ALUoperation = 3'b010; // Add command
                PCSrc = 2'b00;
                PCWrite = 1'b1;
                ns = ID;
            end
            ID : begin
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b11;
                ALUoperation = 3'b010; // Add command
                case(Opcode)
                    6'b000000 : begin //.. case R-type
                        ns = RTstart;
                    end
                    6'b000010 : begin //.. case j
                        ns = JumpComplete;
                    end
                    6'b000011 : begin //.. case JAL
                        ns = JALcomplete;
                    end
                    6'b000100 : begin //.. case beq
                        ns = branchComplete;
                    end
                    6'b000101 : begin //.. case bne
                        ns = branchComplete;
                    end
                    6'b001000 : begin //.. case addi
                        ns = RTstart;
                    end
                    6'b001100 : begin //.. case andi
                        ns = RTstart;
                    end
                    6'b100011 : begin //.. case lw
                        ns = MemRefStart;
                    end
                    6'b101011 : begin //.. case sw
                        ns = MemRefStart;
                    end
                    6'b100000 : begin //.. case jr
                        ns = JumpRcomplete;
                    end
                endcase
            end
            JumpComplete : begin
                PCSrc = 2'b01;
                PCWrite = 1'b1;
                ns = 
            end
            branchComplete : begin
                
            end
            RTstart : begin
                
            end
            RTcomplete : begin
                
            end
            MemRefStart : begin
                
            end
            SWcomplete : begin
                
            end
            LWstart : begin
                
            end
            LWcomplete : begin
                
            end
            JumpRcomplete : begin
                
            end
            JALcomplete : begin
                
            end
        endcase
    end





    always @(posedge clk, posedge rst)begin
        if(rst)
            ps <= IF;
        else 
            ps <= ns;
    end

endmodule