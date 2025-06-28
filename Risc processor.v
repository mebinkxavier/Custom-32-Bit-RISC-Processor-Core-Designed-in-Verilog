module ProgramCounter(input clk, reset, input [31:0] PC_in, output reg [31:0] PC_out);
  always @(posedge clk or posedge reset)
    if (reset) PC_out <= 0;
    else       PC_out <= PC_in;
endmodule


//================ InstructionMemory.v =================
module InstructionMemory(input [31:0] address, output [31:0] instruction);
  reg [31:0] memory [0:255]; // 1KB instruction memory
  initial $readmemh("instructions.mem", memory); // Load instructions
  assign instruction = memory[address[9:2]]; // Word-aligned (PC ÷ 4)
endmodule


//================ RegisterFile.v =================
module RegisterFile(input clk, regWrite,
                    input [4:0] rs1, rs2, rd,
                    input [31:0] writeData,
                    output [31:0] readData1, readData2);
  reg [31:0] regfile[0:31];
  assign readData1 = regfile[rs1];
  assign readData2 = regfile[rs2];
  always @(posedge clk) if (regWrite && rd != 0) regfile[rd] <= writeData;
endmodule


//================ ALUControl.v =================
module ALUControl(
    input [1:0] ALUOp,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] ALU_control
);
    always @(*) begin
        case(ALUOp)
            2'b00: ALU_control = 4'b0010; // lw/sw/addi -> ADD
            2'b01: ALU_control = 4'b0110; // beq/bne -> SUB
            2'b10: begin // R-type decoding
                case({funct7[5], funct3})
                    4'b0000: ALU_control = 4'b0010; // ADD
                    4'b1000: ALU_control = 4'b0110; // SUB
                    4'b0111: ALU_control = 4'b0000; // AND
                    4'b0110: ALU_control = 4'b0001; // OR
                    4'b0010: ALU_control = 4'b0111; // SLT
                    default: ALU_control = 4'b1111; // Invalid
                endcase
            end
            default: ALU_control = 4'b1111;
        endcase
    end
endmodule


//================ ALU.v =================
module ALU(
    input [31:0] A,
    input [31:0] B,
    input [3:0] ALU_control,
    output reg [31:0] Result,
    output Zero
);
    always @(*) begin
        case(ALU_control)
            4'b0000: Result = A & B;                     // AND
            4'b0001: Result = A | B;                     // OR
            4'b0010: Result = A + B;                     // ADD
            4'b0110: Result = A - B;                     // SUB
            4'b0111: Result = ($signed(A) < $signed(B)) ? 1 : 0; // SLT
            4'b1100: Result = ~(A | B);                  // NOR (optional)
            default: Result = 32'h00000000;
        endcase
    end

    assign Zero = (Result == 0);
endmodule


//================ ControlUnit.v =================
module ControlUnit(
    input [6:0] opcode,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemRead,
    output reg MemWrite,
    output reg MemToReg,
    output reg Branch,
    output reg [1:0] ALUOp
);
    always @(*) begin
        case(opcode)
            7'b0110011: begin // R-type
                RegWrite = 1;
                ALUSrc   = 0;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b10;
            end
            7'b0010011: begin // addi, andi, ori
                RegWrite = 1;
                ALUSrc   = 1;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b00;
            end
            7'b0000011: begin // lw
                RegWrite = 1;
                ALUSrc   = 1;
                MemRead  = 1;
                MemWrite = 0;
                MemToReg = 1;
                Branch   = 0;
                ALUOp    = 2'b00;
            end
            7'b0100011: begin // sw
                RegWrite = 0;
                ALUSrc   = 1;
                MemRead  = 0;
                MemWrite = 1;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b00;
            end
            7'b1100011: begin // beq, bne
                RegWrite = 0;
                ALUSrc   = 0;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 1;
                ALUOp    = 2'b01;
            end
            default: begin
                RegWrite = 0;
                ALUSrc   = 0;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b00;
            end
        endcase
    end
endmodule
