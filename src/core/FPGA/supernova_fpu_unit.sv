`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_fpu_unit
 * @brief Unidade de Ponto Flutuante (Extensões 'F' e 'D') para o Supernova.
 *
 * @details Este é um módulo multi-ciclo, pipelined.
 * Ele recebe uma 'rs_entry' do scheduler (issue), a executa
 * (levando N ciclos) e sinaliza 'busy' enquanto estiver
 * em operação. Ele também gerencia as flags de exceção de FP (fflags).
 */
module supernova_fpu_unit (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Emissão (Issue) ---
    input  wire                       req_valid_in,
    input  supernova_pkg::rs_entry_t  req_entry_in, // A instrução e seus dados
    output logic                      req_ready_out, // 1 = Pronto para aceitar

    // --- Interface de Saída (Writeback) ---
    output logic                      wb_valid_out,
    output logic [stu_pkg::XLEN-1:0]  wb_data_out,
    output logic [supernova_pkg::FPR_TAG_WIDTH-1:0] wb_fpr_tag_out, // Note: FPR Tag
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] wb_rob_idx_out,
    output logic                      wb_exception_out,
    output logic [stu_pkg::ADDR_WIDTH-1:0] wb_trap_cause_out
    
    // (Também precisaria de uma interface para ler/escrever o 'fcsr')
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Estruturas de Decode/Execute ---
    typedef struct packed {
        logic [6:0] opcode;
        logic [4:0] rd;
        logic [2:0] funct3;
        logic [4:0] rs1;
        logic [4:0] rs2;
        logic [6:0] funct7;
    } decoded_fp_instr_t;
    
    typedef struct packed {
        logic [XLEN-1:0] result;
        logic            trap;
        logic [4:0]      fflags; // (NV,DZ,OF,UF,NX)
        logic [XLEN-1:0] trap_cause;
    } fpu_result_t;

    // --- Lógica da FSM Multi-Ciclo ---
    typedef enum logic [1:0] { 
        IDLE,
        CALCULATE,
        DONE
    } state_t;
    
    state_t state, next_state;

    // Registradores para armazenar a operação em andamento
    logic [stu_pkg::XLEN-1:0]  result_reg;
    logic [supernova_pkg::FPR_TAG_WIDTH-1:0] wb_fpr_tag_reg; // FPR
    logic [supernova_pkg::ROB_IDX_WIDTH-1:0] wb_rob_idx_reg;
    logic                      wb_exception_reg;
    stu_pkg::addr_t            wb_trap_cause_reg;
    
    logic [6:0] cycle_counter; // Contador para latência
    logic [4:0] fflags_reg; // Flags de exceção de FP

    // --- Lógica de Cálculo (Combinacional) ---
    
    fpu_result_t fpu_result;
    decoded_fp_instr_t decoded_instr;

    // (Função de decode interna)
    assign decoded_instr = decode_fp_instr_function(req_entry_in.instr); 

    // (Chama a função de execução)
    assign fpu_result = execute_fpu_function(
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
                        wb_fpr_tag_reg <= req_entry_in.rd_phys_tag; // FPR Tag
                        wb_rob_idx_reg <= req_entry_in.rob_idx;
                        result_reg <= fpu_result.result;
                        fflags_reg <= fpu_result.fflags;
                        wb_exception_reg <= fpu_result.trap;
                        wb_trap_cause_reg <= fpu_result.trap_cause;
                        
                        // (Define a latência - FPU é rápido para ADD, lento para DIV)
                        if (decoded_instr.funct7 == 7'b0000001) // FADD.D
                            cycle_counter <= 4; // Latência de 4 ciclos (exemplo)
                        else // FMUL.D / FDIV.D
                            cycle_counter <= 8; // Latência de 8 ciclos (exemplo)
                    end
                end
                
                CALCULATE: begin
                    cycle_counter <= cycle_counter - 1;
                end
                
                DONE: begin
                    // Apresenta o resultado ao CDB/ROB
                    wb_valid_out <= 1'b1;
                    // TODO: Atualizar o fcsr (FP CSR) com as fflags_reg
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
    assign wb_fpr_tag_out = wb_fpr_tag_reg; // Saída do Tag FPR
    assign wb_rob_idx_out = wb_rob_idx_reg;
    assign wb_exception_out = wb_exception_reg;
    assign wb_trap_cause_out = wb_trap_cause_reg;
    
    
    // --- Funções de Pipeline (RV64G - F/D Stubs) ---
    
    function automatic decoded_fp_instr_t decode_fp_instr_function(input [ILEN-1:0] instr);
        decoded_fp_instr_t result;
        result.opcode = instr[6:0];
        result.funct3 = instr[14:12];
        result.funct7 = instr[31:25];
        return result;
    endfunction
    
    function automatic fpu_result_t execute_fpu_function(
        input decoded_fp_instr_t instr, 
        input [XLEN-1:0] rs1_data, 
        input [XLEN-1:0] rs2_data
    );
        fpu_result_t result;
        result.result = '0;
        result.trap = 1'b0;
        result.fflags = 5'b0;
        result.trap_cause = 2; // Causa Padrão: Instrução Ilegal
        
        // Apenas processa se for opcode FPU (R-Type)
        if (instr.opcode == 7'b1010011) begin
            result.trap_cause = '0;
            
            // --- STUB: Lógica da Extensão 'F/D' (RV64G) ---
            // (Esta é uma lógica de hardware extremamente complexa)
            
            // Exemplo: FADD.D (funct7 = 0000001, rs2/fmt=D)
            if (instr.funct7 == 7'b0000001) begin
                // fpu_result = fadd_d(rs1_data, rs2_data);
                // fpu_fflags = ...
                result.result = rs1_data + rs2_data; // Placeholder incorreto (int)
            end
            // Exemplo: FMUL.D (funct7 = 0001001)
            else if (instr.funct7 == 7'b0001001) begin
                // fpu_result = fmul_d(rs1_data, rs2_data);
                // fpu_fflags = ...
                result.result = rs1_data * rs2_data; // Placeholder incorreto (int)
            end
            // ... (Muitas outras operações: FSUB, FDIV, FSQRT, FCVT, etc.)
            else begin
                result.trap = 1'b1; // Instrução FP ilegal
            end
        end
        else begin
            // (Lida com outros opcodes de FP, como FMADD, etc.)
            result.trap = 1'b1; // Não deveria ter sido enviado para a FPU
        end
        
        return result;
    endfunction

endmodule
