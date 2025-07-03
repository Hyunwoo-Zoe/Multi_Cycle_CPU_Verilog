///////////////////////////////////////////////////////////////////////////
// MODULE: Multi-cycle CPU for TSC microcomputer
// Author: 
// Description: Multi-cycle implementation with FSM control
`timescale 1ns/100ps
// DEFINITIONS
`include "constants.v"
`include "opcodes.v"

// MODULE DECLARATION
module cpu (
    output       readM, writeM,                   // 메모리 제어
    output [`WORD_SIZE-1:0] address,              // 메모리 주소
    inout  [`WORD_SIZE-1:0] data,                 // 메모리 데이터 버스
    input        inputReady,                      // 메모리 data 준비
    input        reset_n, clk,                    // 리셋 / 클럭
    output [`WORD_SIZE-1:0] num_inst,             // 실행한 명령어 수
    output [`WORD_SIZE-1:0] output_port,          // WWD 결과 (wire)
    output       is_halted                        // HLT 플래그 (wire)
);

// State definitions
`define STATE_IF   3'b000
`define STATE_ID   3'b001
`define STATE_EX   3'b010
`define STATE_MEM  3'b011
`define STATE_WB   3'b100

// Internal registers
reg [`WORD_SIZE-1:0] PC, PC_next;
reg [`WORD_SIZE-1:0] IR, MDR, A, B, ALUOut;
reg [2:0] state, next_state;
reg [`WORD_SIZE-1:0] num_inst_reg;
reg [`WORD_SIZE-1:0] output_port_reg;
reg is_halted_reg;

// Memory interface control
reg readM_reg, writeM_reg;
reg [`WORD_SIZE-1:0] address_reg;

// Control signals
reg PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite;
reg MemtoReg, RegDst, RegWrite, ALUSrcA;
reg [1:0] ALUSrcB, PCSource;
reg [3:0] ALUOp;
reg Branch;

// Instruction fields
wire [3:0] opcode = IR[15:12];
wire [5:0] func = IR[5:0];
wire [1:0] rs = IR[11:10];
wire [1:0] rt = IR[9:8];
wire [1:0] rd = IR[7:6];
wire [7:0] imm = IR[7:0];
wire [11:0] target = IR[11:0];
wire [`WORD_SIZE-1:0] sign_extended = {{8{imm[7]}}, imm};
wire [`WORD_SIZE-1:0] zero_extended = {8'b0, imm};
wire [`WORD_SIZE-1:0] imm_ext = (opcode == `OPCODE_ORI)
                                ? zero_extended
                                : sign_extended;
// J-format(JAL) 또는 R-format(JRL)인가?
wire link_inst = (opcode == `OPCODE_JAL) ||
                 (opcode == 4'hF && func == `FUNC_JRL);
// Output assignments
assign readM = readM_reg;
assign writeM = writeM_reg;
assign address = address_reg;
assign num_inst = num_inst_reg;
assign output_port = output_port_reg;
assign is_halted = is_halted_reg;

// Bidirectional data bus
assign data = (writeM_reg) ? B : 16'bz;

// Register File
reg rf_write_enable;
wire [1:0] write_addr = link_inst ? 2'b10        // ★ 항상 $2
                                  : (RegDst ? rd : rt);
reg [`WORD_SIZE-1:0] write_data_mux;
wire [`WORD_SIZE-1:0] read_data1, read_data2;

// Write data selection
always @(*) begin
    if (MemtoReg)                       write_data_mux = MDR;
    else if (link_inst)                 write_data_mux = PC;   // ★ PC (ID단=next inst)
    else                                write_data_mux = ALUOut;
end

RF rf_unit (
    .write(rf_write_enable),
    .clk(clk),
    .reset(~reset_n),
    .addr1(rs),
    .addr2(rt),
    .addr3(write_addr),
    .data1(read_data1),
    .data2(read_data2),
    .data3(write_data_mux)
);

// ALU
wire [`WORD_SIZE-1:0] alu_input_A = ALUSrcA ? A : PC;
reg [`WORD_SIZE-1:0] alu_input_B;
wire [`WORD_SIZE-1:0] alu_result;
wire alu_zero = (alu_result == 16'b0);

always @(*) begin
    case (ALUSrcB)
        2'b00: alu_input_B = B;
        2'b01: alu_input_B = 16'd1;
        2'b10: alu_input_B = imm_ext;
        2'b11: alu_input_B = {imm, 8'b0};  // For LHI
    endcase
end

ALU alu_unit (
    .A(alu_input_A),
    .B(alu_input_B),
    .Cin(1'b0),
    .OP(ALUOp),
    .C(alu_result),
    .Cout()
);

// Branch logic
reg bcond;
always @(*) begin
    case (opcode)
        `OPCODE_BNE: bcond = (A != B);                      // rs ≠ rt
        `OPCODE_BEQ: bcond = (A == B);                      // rs = rt
        `OPCODE_BGZ: bcond = (~A[15]) && (A != 16'd0);      // rs  > 0
        `OPCODE_BLZ: bcond =  A[15];                        // rs  < 0
        default     : bcond = 1'b0;
    endcase
end

// PC update logic
wire pc_enable = PCWrite || (PCWriteCond && bcond);

always @(*) begin
    case (PCSource)
        2'b00: PC_next = alu_result;
        2'b01: PC_next = ALUOut;
        2'b10: PC_next = {PC[15:12], target};
        2'b11: PC_next = read_data1;
    endcase
end

// State machine
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= `STATE_IF;
        PC <= 16'b0;
        IR <= 16'b0;
        MDR <= 16'b0;
        A <= 16'b0;
        B <= 16'b0;
        ALUOut <= 16'b0;
        num_inst_reg <= 16'b0;
        output_port_reg <= 16'b0;
        is_halted_reg <= 1'b0;
    end
    else begin
        state <= next_state;
        
        // PC update
        if (pc_enable)
            PC <= PC_next;
            
        // IR update
        if (IRWrite && inputReady)
            IR <= data;
            
        // MDR update
        if (MemRead && inputReady)
            MDR <= data;
            
        // Register A, B update
        if (state == `STATE_ID) begin
            A <= read_data1;
            B <= read_data2;
        end
        
        // ALUOut update
        if (state == `STATE_IF || state == `STATE_ID || state == `STATE_EX)
            ALUOut <= alu_result;
            
        // Instruction count
        if ((state == `STATE_ID && (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL || 
             (opcode == 4'hF && (func == `FUNC_JPR || func == `FUNC_JRL || 
              func == `FUNC_WWD || func == `FUNC_HLT)))) ||
            (state == `STATE_EX && (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || 
             opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ)) ||
            (state == `STATE_MEM && opcode == `OPCODE_SWD) ||
            (state == `STATE_WB))
            num_inst_reg <= num_inst_reg + 1;
            
        // WWD instruction
        if (state == `STATE_ID && opcode == 4'hF && func == `FUNC_WWD)
            output_port_reg <= read_data1;
            
        // HLT instruction
        if (state == `STATE_ID && opcode == 4'hF && func == `FUNC_HLT)
            is_halted_reg <= 1'b1;
    end
end

// Next state logic
always @(*) begin
    case (state)
        `STATE_IF: begin
            if (inputReady)
                next_state = `STATE_ID;
            else
                next_state = `STATE_IF;
        end
        
        `STATE_ID: begin
            case (opcode)
                `OPCODE_JMP, `OPCODE_JAL: next_state = `STATE_IF;
                4'hF: begin
                    case (func)
                        `FUNC_JPR, `FUNC_JRL, `FUNC_WWD, `FUNC_HLT: next_state = `STATE_IF;
                        default: next_state = `STATE_EX;
                    endcase
                end
                default: next_state = `STATE_EX;
            endcase
        end
        
        `STATE_EX: begin
            case (opcode)
                `OPCODE_BNE, `OPCODE_BEQ, `OPCODE_BGZ, `OPCODE_BLZ: next_state = `STATE_IF;
                `OPCODE_LWD, `OPCODE_SWD: next_state = `STATE_MEM;
                default: next_state = `STATE_WB;
            endcase
        end
        
        `STATE_MEM: begin
            if (opcode == `OPCODE_LWD) begin
                if (inputReady)
                    next_state = `STATE_WB;
                else
                    next_state = `STATE_MEM;
            end
            else  // SWD
                next_state = `STATE_IF;
        end
        
        `STATE_WB: next_state = `STATE_IF;
        
        default: next_state = `STATE_IF;
    endcase
end

// Control signal generation
always @(*) begin
    // Default values
    PCWrite = 0; PCWriteCond = 0; IorD = 0;
    MemRead = 0; MemWrite = 0; IRWrite = 0;
    MemtoReg = 0; PCSource = 2'b00;
    ALUOp = 4'b0000; ALUSrcA = 0; ALUSrcB = 2'b00;
    RegWrite = 0; RegDst = 0;
    rf_write_enable = 0;
    readM_reg = 0; writeM_reg = 0;
    
    case (state)
        `STATE_IF: begin
            readM_reg = 1;
            MemRead = 1;
            IRWrite = 1;
            ALUSrcA = 0;  // PC
            ALUSrcB = 2'b01;  // 1
            ALUOp = 4'b0000;  // ADD
            PCWrite = 1;
            PCSource = 2'b00;
        end
        
        `STATE_ID: begin
            ALUSrcA = 0;  // PC
            ALUSrcB = 2'b10;  // sign-extended
            ALUOp = 4'b0000;  // ADD
            
            case (opcode)
                `OPCODE_JMP: begin
                    PCWrite = 1;
                    PCSource = 2'b10;
                end
                `OPCODE_JAL: begin
                    PCWrite = 1;
                    PCSource = 2'b10;
                    RegWrite = 1;
                    rf_write_enable = 1;
                end
                4'hF: begin
                    case (func)
                        `FUNC_JPR: begin
                            PCWrite = 1;
                            PCSource = 2'b11;
                        end
                        `FUNC_JRL: begin
                            PCWrite = 1;
                            PCSource = 2'b11;
                            RegWrite = 1;
                            rf_write_enable = 1;
                        end
                    endcase
                end
            endcase
        end
        
        `STATE_EX: begin
            case (opcode)
                `OPCODE_BNE, `OPCODE_BEQ, `OPCODE_BGZ, `OPCODE_BLZ: begin
                    ALUSrcA = 1;  // A
                    ALUSrcB = 2'b00;  // B
                    ALUOp = 4'b0001;  // SUB
                    PCWriteCond = 1;
                    PCSource = 2'b01;  // ALUOut (from ID stage)
                end
                
                `OPCODE_ADI: begin
                    ALUSrcA = 1;
                    ALUSrcB = 2'b10;
                    ALUOp = 4'b0000;  // ADD
                end
                
                `OPCODE_ORI: begin
                    ALUSrcA = 1;
                    ALUSrcB = 2'b10;
                    ALUOp = 4'b1000;  // OR
                end
                
                `OPCODE_LHI: begin
                    ALUSrcA = 1;
                    ALUSrcB = 2'b11;  // {imm, 8'b0}
                    ALUOp = 4'b0010;  // PASS B
                end
                
                `OPCODE_LWD, `OPCODE_SWD: begin
                    ALUSrcA = 1;
                    ALUSrcB = 2'b10;
                    ALUOp = 4'b0000;  // ADD
                end
                
                4'hF: begin  // R-type
                    ALUSrcA = 1;
                    ALUSrcB = 2'b00;
                    case (func)
                        `FUNC_ADD: ALUOp = 4'b0000;
                        `FUNC_SUB: ALUOp = 4'b0001;
                        `FUNC_AND: ALUOp = 4'b0111;
                        `FUNC_ORR: ALUOp = 4'b1000;
                        `FUNC_NOT: ALUOp = 4'b0110;
                        `FUNC_TCP: ALUOp = 4'b1110;
                        `FUNC_SHL: ALUOp = 4'b1101;
                        `FUNC_SHR: ALUOp = 4'b1011;
                    endcase
                end
            endcase
        end
        
        `STATE_MEM: begin
            IorD = 1;
            if (opcode == `OPCODE_LWD) begin
                readM_reg = 1;
                MemRead = 1;
            end
            else begin  // SWD
                writeM_reg = 1;
                MemWrite = 1;
            end
        end
        
        `STATE_WB: begin
            RegWrite = 1;
            rf_write_enable = 1;
            
            if (opcode == `OPCODE_LWD) begin
                MemtoReg = 1;
                RegDst = 0;  // rt
            end
            else if (opcode == 4'hF) begin
                MemtoReg = 0;
                RegDst = 1;  // rd
            end
            else begin  // I-type ALU
                MemtoReg = 0;
                RegDst = 0;  // rt
            end
        end
    endcase
end

// Address mux
always @(*) begin
    if (IorD)
        address_reg = ALUOut;
    else
        address_reg = PC;
end

endmodule

// Register File module
module RF (
    input [1:0]  addr1, addr2, addr3,
    input [15:0] data3,
    input write,
    input clk,
    input reset,
    output reg [15:0] data1, data2
);
    reg [15:0] regFile [3:0];

    always @ (posedge clk) begin
        if (reset) begin        
            regFile[0] <= 16'd0;
            regFile[1] <= 16'd0;
            regFile[2] <= 16'd0;
            regFile[3] <= 16'd0;
        end
        else if (write)
            regFile[addr3] <= data3;
    end

    always @ * begin
        data1 = regFile[addr1];
        data2 = regFile[addr2];
    end   
endmodule

// ALU module - Fixed NOT operation
module ALU (
    input [15:0] A, B,   
    input Cin,
    input [3:0] OP,
    output reg Cout,     
    output reg [15:0] C
);
    always @ * begin
        {Cout, C} = 17'd0;
        case (OP)
            4'b0000: {Cout, C} = A + B + Cin;          // ADD
            4'b0001: {Cout, C} = {1'b0, A} - ({1'b0, B} + Cin); // SUB
            4'b0010: C = B;  // PASS B (for LHI)
            4'b0011: C = ~(A & B);  // NAND
            4'b0100: C = ~(A | B);  // NOR
            4'b0101: C = ~(A ^ B);  // XNOR
            4'b0110: C = ~A; // NOT
            4'b0111: C = A & B;  // AND
            4'b1000: C = A | B;  // OR
            4'b1001: C = A ^ B;  // XOR
            4'b1010: C = A >> 1;  // SHR (logical)
            4'b1011: C = $signed(A) >>> 1;  // SHR (arithmetic)
            4'b1100: C = {A[0], A[15:1]};  // Rotate right
            4'b1101: C = A << 1;  // SHL
            4'b1110: C = ~A + 16'd1; // TCP
            4'b1111: C = {A[14:0], A[15]};  // Rotate left
        endcase
    end
endmodule