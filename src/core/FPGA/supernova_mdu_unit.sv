`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_mdu_unit
 * @brief Unidade de Multiplicação/Divisão (Extensão 'M') para o Supernova.
 *
 * @details Este é um módulo multi-ciclo (FSM). Ele recebe uma 'rs_entry'
 * do scheduler (issue), a executa (levando N ciclos) e sinaliza
 * 'req_ready_out = 0' (busy) enquanto estiver em operação.
 * Ele implementa a ISA RV64M completa (64-bit e 32-bit 'W').
 */
module supernova_mdu_unit (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Emissão (Issue) ---
    input  wire                       req_valid_in,
    input  supernova_pkg::rs_entry_t  req_entry_in, // A instrução e seus dados
    output logic                      req_ready_out, // 1 = Pronto para aceitar

    // --- Interface de Saída (Writeback) ---
    output logic                      wb_valid_out,
    output logic [stu_pkg::XLEN-1:0]  wb_data_out,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] wb_gpr_tag_out,
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] wb_rob_idx_out,
    output logic                      wb_exception_out,
    output logic [stu_pkg::ADDR_WIDTH-1:0] wb_trap_cause_out
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Definições de Latência (Exemplos) ---
    localparam int LATENCY_MUL = 3;  // Multiplicação de 3 ciclos
    localparam int LATENCY_DIV = 32; // Divisão de 32 ciclos

    // --- Estruturas de Decode/Execute ---
    typedef struct packed {
        logic [6:0] opcode;
        logic [4:0] rd;
        logic [2:0] funct3;
        logic [6:0] funct7;
    } decoded_m_instr_t;
    
    typedef struct packed {
        logic [XLEN-1:0] result;
        logic            trap;
        logic [XLEN-1:0] trap_cause;
    } mdu_result_t;

    // --- Lógica da FSM Multi-Ciclo ---
    typedef enum logic [1:0] { 
        IDLE,
        CALCULATE,
        DONE
    } state_t;
    
    state_t state, next_state;

    // Registradores para armazenar a operação em andamento
    logic [stu_pkg::XLEN-1:0]  result_reg;
    logic [supernova_pkg::GPR_TAG_WIDTH-1:0] wb_gpr_tag_reg;
    logic [supernova_pkg::ROB_IDX_WIDTH-1:0] wb_rob_idx_reg;
    logic                      wb_exception_reg;
    stu_pkg::addr_t            wb_trap_cause_reg;
    
    logic [6:0] cycle_counter; // Contador para latência

    // --- Lógica de Cálculo (Combinacional) ---
    
    mdu_result_t mdu_result;
    decoded_m_instr_t decoded_instr;

    // (Função de decode interna, necessária para obter funct3/funct7)
    assign decoded_instr = decode_m_instr_function(req_entry_in.instr); 

    // (Chama a função de execução completa)
    assign mdu_result = execute_mdu_function(
        decoded_instr, 
        req_entry_in.src1_data, 
        req_entry_in.src2_data
    );

    // --- Lógica da FSM Sequencial ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cycle_counter <= '0;
            wb_valid_out <= 1'b0;
        end else begin
            state <= next_state;
            wb_valid_out <= 1'b0; // Padrão (pulso de 1 ciclo)

            case (state)
                IDLE: begin
                    if (req_valid_in) begin
                        // Captura a operação
                        wb_gpr_tag_reg <= req_entry_in.rd_phys_tag;
                        wb_rob_idx_reg <= req_entry_in.rob_idx;
                        result_reg <= mdu_result.result;
                        wb_exception_reg <= mdu_result.trap;
                        wb_trap_cause_reg <= mdu_result.trap_cause;
                        
                        // Define a latência
                        if (decoded_instr.funct3 < 3'b100) // MUL/MULW
                            cycle_counter <= LATENCY_MUL;
                        else // DIV/REM
                            cycle_counter <= LATENCY_DIV;
                    end
                end
                
                CALCULATE: begin
                    cycle_counter <= cycle_counter - 1;
                end
                
                DONE: begin
                    // Apresenta o resultado ao CDB/ROB
                    wb_valid_out <= 1'b1;
                end
            endcase
        end
    end

    // --- Lógica de Próximo Estado (Combinacional) ---
    always_comb begin
        next_state = state;
        req_ready_out = 1'b0; // Não está pronto por padrão

        case (state)
            IDLE: begin
                req_ready_out = 1'b1; // Pronto para aceitar
                if (req_valid_in) begin
                    next_state = CALCULATE;
                end
            end
            
            CALCULATE: begin
                if (cycle_counter == 1) begin // (N-1 ciclos)
                    next_state = DONE;
                end
            end
            
            DONE: begin
                next_state = IDLE; // Volta ao ocioso
            end
        endcase
    end

    // --- Saídas (Registradas) ---
    assign wb_data_out = result_reg;
    assign wb_gpr_tag_out = wb_gpr_tag_reg;
    assign wb_rob_idx_out = wb_rob_idx_reg;
    assign wb_exception_out = wb_exception_reg;
    assign wb_trap_cause_out = wb_trap_cause_reg;
    
    
    // --- Funções de Pipeline (RV64M Completo) ---
    
    function automatic decoded_m_instr_t decode_m_instr_function(input [ILEN-1:0] instr);
        decoded_m_instr_t result;
        result.opcode = instr[6:0];
        result.funct3 = instr[14:12];
        result.funct7 = instr[31:25];
        return result;
    endfunction
    
    function automatic mdu_result_t execute_mdu_function(
        input decoded_m_instr_t instr, 
        input [XLEN-1:0] rs1_data, 
        input [XLEN-1:0] rs2_data
    );
        mdu_result_t result;
        result.result = '0;
        result.trap = 1'b0;
        result.trap_cause = 2; // Causa Padrão: Instrução Ilegal
        
        logic [31:0] rs1_w, rs2_w;
        logic [63:0] result_w;
        
        // Operandos de 32 bits (para 'W' ops)
        rs1_w = rs1_data[31:0];
        rs2_w = rs2_data[31:0];
        
        // Apenas processa se for opcode 'M' (verificado pelo decode)
        if (instr.opcode == 7'b0110011 && instr.funct7 == 7'b0000001) begin
            result.trap_cause = '0; // Trap válido
            
            case (instr.funct3)
                // --- 64-bit (RV64M) ---
                3'b000: // MUL
                    result.result = rs1_data * rs2_data;
                3'b001: // MULH
                    result.result = ($signed(rs1_data) * $signed(rs2_data)) >> 64;
                3'b010: // MULHSU
                    result.result = ($signed(rs1_data) * $unsigned(rs2_data)) >> 64;
                3'b011: // MULHU
                    result.result = ($unsigned(rs1_data) * $unsigned(rs2_data)) >> 64;
                
                3'b100: // DIV
                    if (rs2_data == 0)
                        result.result = -1; // Div por zero
                    else if ($signed(rs1_data) == -($signed(1'b1) << (XLEN-1)) && rs2_data == -1)
                        result.result = rs1_data; // Overflow
                    else
                        result.result = $signed(rs1_data) / $signed(rs2_data);
                        
                3'b101: // DIVU
                    if (rs2_data == 0)
                        result.result = -1; // Div por zero
                    else
                        result.result = $unsigned(rs1_data) / $unsigned(rs2_data);

                3'b110: // REM
                    if (rs2_data == 0)
                        result.result = rs1_data; // Div por zero
                    else if ($signed(rs1_data) == -($signed(1'b1) << (XLEN-1)) && rs2_data == -1)
                        result.result = 0; // Overflow
                    else
                        result.result = $signed(rs1_data) % $signed(rs2_data);
                        
                3'b111: // REMU
                    if (rs2_data == 0)
                        result.result = rs1_data; // Div por zero
                    else
                        result.result = $unsigned(rs1_data) % $unsigned(rs2_data);
                
                default: result.trap = 1'b1;
            endcase
        end
        // Operações 'W' (RV64M)
        else if (instr.opcode == 7'b0111011 && instr.funct7 == 7'b0000001) begin
            result.trap_cause = '0;
            
            case (instr.funct3)
                3'b000: // MULW
                    result_w = $signed(rs1_w) * $signed(rs2_w);
                    result.result = $signed({ {32{result_w[31]}}, result_w[31:0] });
                
                3'b100: // DIVW
                    if (rs2_w == 0)
                        result_w = -1;
                    else if ($signed(rs1_w) == -($signed(1'b1) << 31) && rs2_w == -1)
                        result_w = rs1_w;
                    else
                        result_w = $signed(rs1_w) / $signed(rs2_w);
                    result.result = $signed({ {32{result_w[31]}}, result_w[31:0] });
                    
                3'b101: // DIVUW
                    if (rs2_w == 0)
                        result_w = -1;
                    else
                        result_w = $unsigned(rs1_w) / $unsigned(rs2_w);
                    result.result = $signed({ {32{result_w[31]}}, result_w[31:0] });
                    
                3'b110: // REMW
                    if (rs2_w == 0)
                        result_w = rs1_w;
                    else if ($signed(rs1_w) == -($signed(1'b1) << 31) && rs2_w == -1)
                        result_w = 0;
                    else
                        result_w = $signed(rs1_w) % $signed(rs2_w);
                    result.result = $signed({ {32{result_w[31]}}, result_w[31:0] });
                    
                3'b111: // REMUW
                    if (rs2_w == 0)
                        result_w = rs1_w;
                    else
                        result_w = $unsigned(rs1_w) % $unsigned(rs2_w);
                    result.result = $signed({ {32{result_w[31]}}, result_w[31:0] });
                    
                default: result.trap = 1'b1;
            endcase
        end
        else begin
            result.trap = 1'b1; // Não deveria ter sido enviado para a MDU
        end
        
        return result;
    endfunction
    
endmodule
