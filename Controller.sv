`timescale 1ns/1ns

module Controller (input zeroflag, input [31:0]instruction, input clk, rst, output reg PCWrite, PCWriteCond, IorD, MemWrite, MemRead, IRWrite, RegDst, WriteRegSel, MemtoReg, WriteDataSel, RegWrite, ALUSrcA, output reg [1:0]ALUSrcB, PCSrc, output reg [2:0]ALUoperation);
    parameter [3:0] IF = 4'b0000, ID = 4'b0001, JumpComplete = 4'b0010, branchComplete = 4'b0011, RTstart = 4'b0100, RTcomplete = 4'b0101, MemRefStart = 4'b0110, SWcomplete = 4'b0111, LWstart = 4'1000, LWcomplete = 4'b1001, JumpRcomplete = 4'b1010, JALcomplete = 4'b1011;

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
                ns = IF;
            end
            branchComplete : begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b00;
                ALUoperation = 3'b110; //.. subtract command
                PCWriteCond = 1'b1;
                PCSrc = 2'b10;
                ns = IF;
            end
            RTstart : begin
                case(Opcode)
                    6'b000000 : begin //.. ordinary rtype
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b00;
                        case(functionType)
                            6'b100000 : begin //.. case add function
                                ALUoperation = 3'b010;
                            end
                            6'b100010 : begin //.. case sub function
                                ALUoperation = 3'b110;
                            end
                            6'b100100 : begin //.. case and function 
                                ALUoperation = 3'b000;
                            end
                            6'b100101 : begin //.. case or function
                                ALUoperation = 3'b001;
                            end
                            6'b101010 : begin //.. case slt function
                                ALUoperation = 3'b111;
                            end
                        endcase
                    end
                    6'b001000 : begin //.. addi
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUoperation = 3'b010;
                    end
                    6'b001100 : begin //.. andi
                        ALUSrcA = 1'b1;
                        ALUSrcB = 2'b10;
                        ALUoperation = 3'b000;
                    end
                endcase
                ns = RTcomplete;
            end
            RTcomplete : begin
                RegDst = 1'b0;
                WriteRegSel = 1'b0;
                MemtoReg = 1'b0;
                WriteDataSel = 1'b0;
                RegWrite = 1'b1;
            end
            MemRefStart : begin
                ALUSrcA = 1'b1;
                ALUSrcB = 2'b10;
                ALUoperation = 3'b010; //.. add command
                case(Opcode)
                    6'b100011 : begin //.. case lw
                        ns = LWstart;
                    end
                    6'b101011 : begin //.. case sw
                        ns = SWcomplete;
                    end
                endcase
            end
            SWcomplete : begin
                IorD = 1'b1;
                MemWrite = 1'b1;
                ns = IF;
            end
            LWstart : begin
                IorD = 1'b1;
                MemRead = 1'b1;
                ns = LWcomplete;
            end
            LWcomplete : begin
                RegDst = 1'b0;
                WriteRegSel = 1'b0;
                MemtoReg = 1'b1;
                WriteDataSel = 1'b0;
                RegWrite = 1'b1;
            end
            JumpRcomplete : begin
                // Please check that this signals are enough to set for this instruction 
                PCSrc = 2'b11;
                PCWrite = 1'b1;
                ns = IF;
            end
            JALcomplete : begin
                WriteRegSel = 1'b1;
                WriteDataSel = 1'b1;
                RegWrite = 1'b1;
                PCSrc = 1'b1;
                PCWrite = 1'b1;
                ns = IF;
            end
            default : ns = IF;
        endcase
    end

    always @(posedge clk, posedge rst)begin
        if(rst)
            ps <= IF;
        else 
            ps <= ns;
    end

endmodule