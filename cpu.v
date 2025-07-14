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
    output readM,
    output writeM,
    output [`WORD_SIZE-1:0] address,
    inout [`WORD_SIZE-1:0] data,
    input inputReady,
    input reset_n,
    input clk,
    
    output [`WORD_SIZE-1:0] num_inst,
    output [`WORD_SIZE-1:0] output_port,
    output is_halted
);

// State definitions - 5단계 파이프라인 상태
`define STATE_IF   3'b000    // Instruction Fetch
`define STATE_ID   3'b001    // Instruction Decode
`define STATE_EX   3'b010    // Execute
`define STATE_MEM  3'b011    // Memory Access
`define STATE_WB   3'b100    // Write Back

// Internal registers - CPU 내부 레지스터
reg [`WORD_SIZE-1:0] PC, PC_next;       // Program Counter와 다음 PC 값
reg [`WORD_SIZE-1:0] IR, MDR, A, B, ALUOut;  // Instruction Register, Memory Data Register, 임시 레지스터 A/B, ALU 출력
reg [2:0] state, next_state;            // 현재 상태와 다음 상태
reg [`WORD_SIZE-1:0] num_inst_reg;      // 명령어 카운터
reg [`WORD_SIZE-1:0] output_port_reg;   // WWD 출력 포트
reg is_halted_reg;                      // halt 플래그

// Memory interface control - 메모리 인터페이스 제어
reg readM_reg, writeM_reg;              // 읽기/쓰기 제어 신호
reg [`WORD_SIZE-1:0] address_reg;       // 주소 레지스터

// Control signals - 제어 신호들
reg PCWrite, PCWriteCond, IorD, MemRead, MemWrite, IRWrite;  // PC 쓰기, 조건부 PC 쓰기, I/D 선택, 메모리 읽기/쓰기, IR 쓰기
reg MemtoReg, RegDst, RegWrite, ALUSrcA;                     // 메모리→레지스터, 레지스터 목적지, 레지스터 쓰기, ALU 입력 A 선택
reg [1:0] ALUSrcB, PCSource;            // ALU 입력 B 선택, PC 소스 선택
reg [3:0] ALUOp;                        // ALU 연산 코드
reg Branch;                             // 분기 신호

// Instruction fields - 명령어 필드 분해
wire [3:0] opcode = IR[15:12];         // Operation code
wire [5:0] func = IR[5:0];             // R-type function code
wire [1:0] rs = IR[11:10];             // Source register 1
wire [1:0] rt = IR[9:8];               // Source register 2
wire [1:0] rd = IR[7:6];               // Destination register
wire [7:0] imm = IR[7:0];              // Immediate value
wire [11:0] target = IR[11:0];         // Jump target address

// Immediate 확장 - sign extension vs zero extension
wire [`WORD_SIZE-1:0] sign_extended = {{8{imm[7]}}, imm};    // 부호 확장
wire [`WORD_SIZE-1:0] zero_extended = {8'b0, imm};           // 0 확장
wire [`WORD_SIZE-1:0] imm_ext = (opcode == `OPCODE_ORI)      // ORI는 zero extension, 나머지는 sign extension
                                ? zero_extended
                                : sign_extended;

// Link instruction 판별 - JAL 또는 JRL인지 확인
wire link_inst = (opcode == `OPCODE_JAL) ||                  // Jump And Link
                 (opcode == 4'hF && func == `FUNC_JRL);      // Jump Register And Link

// Output assignments - 출력 신호 연결
assign readM = readM_reg;
assign writeM = writeM_reg;
assign address = address_reg;
assign num_inst = num_inst_reg;
assign output_port = output_port_reg;
assign is_halted = is_halted_reg;

// Bidirectional data bus - 양방향 데이터 버스 제어
assign data = (writeM_reg) ? B : 16'bz;  // 쓰기 시에는 B 레지스터 값, 아니면 high-Z

// Register File 인터페이스
reg rf_write_enable;                     // Register file 쓰기 활성화
wire [1:0] write_addr = link_inst ? 2'b10        // Link 명령어는 항상 $2에 저장
                                  : (RegDst ? rd : rt);  // RegDst에 따라 rd 또는 rt 선택
reg [`WORD_SIZE-1:0] write_data_mux;    // Register file에 쓸 데이터
wire [`WORD_SIZE-1:0] read_data1, read_data2;  // Register file에서 읽은 데이터

// Write data selection - Register file에 쓸 데이터 선택
always @(*) begin
    if (MemtoReg)                       write_data_mux = MDR;      // Load 명령어: 메모리 데이터
    else if (link_inst)                 write_data_mux = PC;       // Link 명령어: 현재 PC (다음 명령어 주소)
    else                                write_data_mux = ALUOut;   // ALU 연산 결과
end

// Register File 인스턴스
RF rf_unit (
    .write(rf_write_enable),
    .clk(clk),
    .reset(~reset_n),
    .addr1(rs),                         // 읽기 주소 1
    .addr2(rt),                         // 읽기 주소 2
    .addr3(write_addr),                 // 쓰기 주소
    .data1(read_data1),                 // 읽기 데이터 1
    .data2(read_data2),                 // 읽기 데이터 2
    .data3(write_data_mux)              // 쓰기 데이터
);

// ALU 입력 선택
wire [`WORD_SIZE-1:0] alu_input_A = ALUSrcA ? A : PC;  // ALUSrcA=0: PC, ALUSrcA=1: A 레지스터
reg [`WORD_SIZE-1:0] alu_input_B;      // ALU 두 번째 입력
wire [`WORD_SIZE-1:0] alu_result;      // ALU 연산 결과
wire alu_zero = (alu_result == 16'b0); // Zero flag

// ALU 입력 B 선택 - 4가지 소스 중 선택
always @(*) begin
    case (ALUSrcB)
        2'b00: alu_input_B = B;              // B 레지스터
        2'b01: alu_input_B = 16'd1;         // 상수 1 (PC+1 계산용)
        2'b10: alu_input_B = imm_ext;       // 확장된 immediate
        2'b11: alu_input_B = {imm, 8'b0};   // LHI용 - immediate를 상위 8비트로
    endcase
end

// ALU 인스턴스
ALU alu_unit (
    .A(alu_input_A),
    .B(alu_input_B),
    .Cin(1'b0),
    .OP(ALUOp),
    .C(alu_result),
    .Cout()
);

// Branch logic - 분기 조건 계산
reg bcond;  // Branch condition
always @(*) begin
    case (opcode)
        `OPCODE_BNE: bcond = (A != B);                      // Branch Not Equal: rs ≠ rt
        `OPCODE_BEQ: bcond = (A == B);                      // Branch Equal: rs = rt
        `OPCODE_BGZ: bcond = (~A[15]) && (A != 16'd0);     // Branch Greater than Zero: rs > 0
        `OPCODE_BLZ: bcond = A[15];                        // Branch Less than Zero: rs < 0
        default: bcond = 1'b0;
    endcase
end

// PC update logic - PC 업데이트 활성화
wire pc_enable = PCWrite || (PCWriteCond && bcond);  // 무조건 쓰기 또는 조건부 쓰기

// PC 다음 값 선택
always @(*) begin
    case (PCSource)
        2'b00: PC_next = alu_result;         // ALU 결과 (PC+1)
        2'b01: PC_next = ALUOut;            // 저장된 ALU 결과 (branch target)
        2'b10: PC_next = {PC[15:12], target}; // Jump target (J-format)
        2'b11: PC_next = read_data1;        // Register 값 (JR, JRL)
    endcase
end

// State machine - 상태 머신과 레지스터 업데이트
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        // 리셋 시 모든 레지스터 초기화
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
        state <= next_state;  // 상태 전이
        
        // PC update - PC 업데이트
        if (pc_enable)
            PC <= PC_next;
            
        // IR update - 명령어 레지스터 업데이트 (IF 단계에서 메모리 읽기 완료 시)
        if (IRWrite && inputReady)
            IR <= data;
            
        // MDR update - 메모리 데이터 레지스터 업데이트 (MEM 단계에서 읽기 완료 시)
        if (MemRead && inputReady)
            MDR <= data;
            
        // Register A, B update - ID 단계에서 레지스터 값 래치
        if (state == `STATE_ID) begin
            A <= read_data1;
            B <= read_data2;
        end
        
        // ALUOut update - ALU 결과 저장 (IF, ID, EX 단계에서)
        if (state == `STATE_IF || state == `STATE_ID || state == `STATE_EX)
            ALUOut <= alu_result;
            
        // Instruction count - 명령어 완료 시점에 카운터 증가
        if ((state == `STATE_ID && (opcode == `OPCODE_JMP || opcode == `OPCODE_JAL || 
             (opcode == 4'hF && (func == `FUNC_JPR || func == `FUNC_JRL || 
              func == `FUNC_WWD || func == `FUNC_HLT)))) ||    // Jump, WWD, HLT는 ID에서 완료
            (state == `STATE_EX && (opcode == `OPCODE_BNE || opcode == `OPCODE_BEQ || 
             opcode == `OPCODE_BGZ || opcode == `OPCODE_BLZ)) ||  // Branch는 EX에서 완료
            (state == `STATE_MEM && opcode == `OPCODE_SWD) ||     // Store는 MEM에서 완료
            (state == `STATE_WB))                                  // 나머지는 WB에서 완료
            num_inst_reg <= num_inst_reg + 1;
            
        // WWD instruction - ID 단계에서 rs 값을 출력 포트로
        if (state == `STATE_ID && opcode == 4'hF && func == `FUNC_WWD)
            output_port_reg <= read_data1;
            
        // HLT instruction - ID 단계에서 halt 플래그 설정
        if (state == `STATE_ID && opcode == 4'hF && func == `FUNC_HLT)
            is_halted_reg <= 1'b1;
    end
end

// Next state logic - 다음 상태 결정
always @(*) begin
    case (state)
        `STATE_IF: begin
            // Instruction Fetch: 메모리 준비되면 ID로 전이
            if (inputReady)
                next_state = `STATE_ID;
            else
                next_state = `STATE_IF;  // 대기
        end
        
        `STATE_ID: begin
            // Instruction Decode: 명령어 종류에 따라 다음 상태 결정
            case (opcode)
                `OPCODE_JMP, `OPCODE_JAL: next_state = `STATE_IF;  // Jump는 ID에서 완료
                4'hF: begin  // R-type
                    case (func)
                        `FUNC_JPR, `FUNC_JRL, `FUNC_WWD, `FUNC_HLT: 
                            next_state = `STATE_IF;  // 이 명령어들은 ID에서 완료
                        default: 
                            next_state = `STATE_EX;  // 나머지 R-type은 EX로
                    endcase
                end
                default: next_state = `STATE_EX;  // 대부분의 명령어는 EX로
            endcase
        end
        
        `STATE_EX: begin
            // Execute: 명령어 종류에 따라 다음 상태 결정
            case (opcode)
                `OPCODE_BNE, `OPCODE_BEQ, `OPCODE_BGZ, `OPCODE_BLZ: 
                    next_state = `STATE_IF;  // Branch는 EX에서 완료
                `OPCODE_LWD, `OPCODE_SWD: 
                    next_state = `STATE_MEM;  // Load/Store는 메모리 접근 필요
                default: 
                    next_state = `STATE_WB;  // ALU 연산은 WB로
            endcase
        end
        
        `STATE_MEM: begin
            // Memory Access
            if (opcode == `OPCODE_LWD) begin
                // Load: 메모리 준비되면 WB로
                if (inputReady)
                    next_state = `STATE_WB;
                else
                    next_state = `STATE_MEM;  // 대기
            end
            else  // SWD
                next_state = `STATE_IF;  // Store는 MEM에서 완료
        end
        
        `STATE_WB: 
            next_state = `STATE_IF;  // Write Back 후 다음 명령어로
        
        default: 
            next_state = `STATE_IF;
    endcase
end

// Control signal generation - 제어 신호 생성 (상태별)
always @(*) begin
    // 기본값 설정 (모든 신호 비활성화)
    PCWrite = 0; PCWriteCond = 0; IorD = 0;
    MemRead = 0; MemWrite = 0; IRWrite = 0;
    MemtoReg = 0; PCSource = 2'b00;
    ALUOp = 4'b0000; ALUSrcA = 0; ALUSrcB = 2'b00;
    RegWrite = 0; RegDst = 0;
    rf_write_enable = 0;
    readM_reg = 0; writeM_reg = 0;
    
    case (state)
        `STATE_IF: begin
            // Instruction Fetch: 메모리에서 명령어 읽고 PC 증가
            readM_reg = 1;      // 메모리 읽기 활성화
            MemRead = 1;        // MDR 업데이트 활성화
            IRWrite = 1;        // IR 업데이트 활성화
            ALUSrcA = 0;        // ALU 입력 A = PC
            ALUSrcB = 2'b01;    // ALU 입력 B = 1
            ALUOp = 4'b0000;    // ADD (PC + 1)
            PCWrite = 1;        // PC 업데이트 활성화
            PCSource = 2'b00;   // PC = ALU 결과
        end
        
        `STATE_ID: begin
            // Instruction Decode: 레지스터 읽고 branch target 계산
            ALUSrcA = 0;        // ALU 입력 A = PC
            ALUSrcB = 2'b10;    // ALU 입력 B = sign-extended immediate
            ALUOp = 4'b0000;    // ADD (PC + offset)
            
            // Jump 명령어 처리
            case (opcode)
                `OPCODE_JMP: begin
                    PCWrite = 1;
                    PCSource = 2'b10;  // PC = {PC[15:12], target}
                end
                `OPCODE_JAL: begin
                    PCWrite = 1;
                    PCSource = 2'b10;  // PC = {PC[15:12], target}
                    RegWrite = 1;      // $2 = PC (return address)
                    rf_write_enable = 1;
                end
                4'hF: begin  // R-type
                    case (func)
                        `FUNC_JPR: begin
                            PCWrite = 1;
                            PCSource = 2'b11;  // PC = rs
                        end
                        `FUNC_JRL: begin
                            PCWrite = 1;
                            PCSource = 2'b11;  // PC = rs
                            RegWrite = 1;      // $2 = PC (return address)
                            rf_write_enable = 1;
                        end
                    endcase
                end
            endcase
        end
        
        `STATE_EX: begin
            // Execute: ALU 연산 수행
            case (opcode)
                // Branch 명령어들
                `OPCODE_BNE, `OPCODE_BEQ, `OPCODE_BGZ, `OPCODE_BLZ: begin
                    ALUSrcA = 1;        // ALU 입력 A = A 레지스터
                    ALUSrcB = 2'b00;    // ALU 입력 B = B 레지스터
                    ALUOp = 4'b0001;    // SUB (비교를 위해)
                    PCWriteCond = 1;    // 조건부 PC 쓰기
                    PCSource = 2'b01;   // PC = ALUOut (ID에서 계산한 target)
                end
                
                // I-type ALU 명령어들
                `OPCODE_ADI: begin
                    ALUSrcA = 1;        // A = rs
                    ALUSrcB = 2'b10;    // B = sign-extended immediate
                    ALUOp = 4'b0000;    // ADD
                end
                
                `OPCODE_ORI: begin
                    ALUSrcA = 1;        // A = rs
                    ALUSrcB = 2'b10;    // B = zero-extended immediate
                    ALUOp = 4'b1000;    // OR
                end
                
                `OPCODE_LHI: begin
                    ALUSrcA = 1;        // A = rs (사용 안 함)
                    ALUSrcB = 2'b11;    // B = {imm, 8'b0}
                    ALUOp = 4'b0010;    // PASS B
                end
                
                // Load/Store 주소 계산
                `OPCODE_LWD, `OPCODE_SWD: begin
                    ALUSrcA = 1;        // A = rs
                    ALUSrcB = 2'b10;    // B = sign-extended offset
                    ALUOp = 4'b0000;    // ADD (base + offset)
                end
                
                // R-type ALU 명령어들
                4'hF: begin
                    ALUSrcA = 1;        // A = rs
                    ALUSrcB = 2'b00;    // B = rt
                    case (func)
                        `FUNC_ADD: ALUOp = 4'b0000;  // ADD
                        `FUNC_SUB: ALUOp = 4'b0001;  // SUB
                        `FUNC_AND: ALUOp = 4'b0111;  // AND
                        `FUNC_ORR: ALUOp = 4'b1000;  // OR
                        `FUNC_NOT: ALUOp = 4'b0110;  // NOT
                        `FUNC_TCP: ALUOp = 4'b1110;  // Two's Complement
                        `FUNC_SHL: ALUOp = 4'b1101;  // Shift Left
                        `FUNC_SHR: ALUOp = 4'b1011;  // Shift Right
                    endcase
                end
            endcase
        end
        
        `STATE_MEM: begin
            // Memory Access: Load/Store 수행
            IorD = 1;  // 주소 = ALUOut (계산된 주소)
            if (opcode == `OPCODE_LWD) begin
                // Load
                readM_reg = 1;
                MemRead = 1;
            end
            else begin  // SWD
                // Store
                writeM_reg = 1;
                MemWrite = 1;
            end
        end
        
        `STATE_WB: begin
            // Write Back: 결과를 레지스터에 저장
            RegWrite = 1;
            rf_write_enable = 1;
            
            if (opcode == `OPCODE_LWD) begin
                MemtoReg = 1;   // 데이터 = MDR (메모리에서 읽은 값)
                RegDst = 0;     // 목적지 = rt
            end
            else if (opcode == 4'hF) begin  // R-type
                MemtoReg = 0;   // 데이터 = ALUOut
                RegDst = 1;     // 목적지 = rd
            end
            else begin  // I-type ALU
                MemtoReg = 0;   // 데이터 = ALUOut
                RegDst = 0;     // 목적지 = rt
            end
        end
    endcase
end

// Address mux - 메모리 주소 선택
always @(*) begin
    if (IorD)
        address_reg = ALUOut;  // Load/Store: 계산된 주소
    else
        address_reg = PC;      // Instruction fetch: PC
end

endmodule

// Register File module - 4개의 16비트 레지스터
module RF (
    input [1:0]  addr1, addr2, addr3,   // 읽기 주소1, 읽기 주소2, 쓰기 주소
    input [15:0] data3,                 // 쓰기 데이터
    input write,                        // 쓰기 활성화
    input clk,                          // 클럭
    input reset,                        // 리셋
    output reg [15:0] data1, data2      // 읽기 데이터1, 읽기 데이터2
);
    reg [15:0] regFile [3:0];  // 4개의 16비트 레지스터 배열

    // 쓰기 동작 (동기식)
    always @ (posedge clk) begin
        if (reset) begin        
            // 리셋 시 모든 레지스터 0으로 초기화
            regFile[0] <= 16'd0;
            regFile[1] <= 16'd0;
            regFile[2] <= 16'd0;
            regFile[3] <= 16'd0;
        end
        else if (write)
            regFile[addr3] <= data3;  // 쓰기 활성화 시 데이터 저장
    end

    // 읽기 동작 (비동기식)
    always @ * begin
        data1 = regFile[addr1];
        data2 = regFile[addr2];
    end   
endmodule

// ALU module - 산술/논리 연산 유닛
module ALU (
    input [15:0] A, B,      // 입력 피연산자
    input Cin,              // Carry 입력
    input [3:0] OP,         // 연산 코드
    output reg Cout,        // Carry 출력
    output reg [15:0] C     // 연산 결과
);
    always @ * begin
        {Cout, C} = 17'd0;  // 기본값 설정
        case (OP)
            4'b0000: {Cout, C} = A + B + Cin;                      // ADD
            4'b0001: {Cout, C} = {1'b0, A} - ({1'b0, B} + Cin);   // SUB
            4'b0010: C = B;                                        // PASS B (LHI용)
            4'b0011: C = ~(A & B);                                 // NAND
            4'b0100: C = ~(A | B);                                 // NOR
            4'b0101: C = ~(A ^ B);                                 // XNOR
            4'b0110: C = ~A;                                       // NOT
            4'b0111: C = A & B;                                    // AND
            4'b1000: C = A | B;                                    // OR
            4'b1001: C = A ^ B;                                    // XOR
            4'b1010: C = A >> 1;                                   // Logical shift right
            4'b1011: C = $signed(A) >>> 1;                        // Arithmetic shift right
            4'b1100: C = {A[0], A[15:1]};                         // Rotate right
            4'b1101: C = A << 1;                                   // Shift left
            4'b1110: C = ~A + 16'd1;                              // Two's complement
            4'b1111: C = {A[14:0], A[15]};                        // Rotate left
        endcase
    end
endmodule