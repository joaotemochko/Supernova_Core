`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_rat
 * @brief Register Alias Table (RAT) para GPRs e FPRs.
 *
 * @details Este módulo gerencia o mapeamento entre registradores
 * arquiteturais (ARF) e físicos (PRF). Ele mantém um
 * estado 'commit' (seguro) e um estado 'speculative' (de front-end).
 *
 * Ele lida com leituras/escritas paralelas do estágio de Rename
 * e restaura o estado 'speculative' para o 'commit' em
 * caso de flush (redirect).
 */
module supernova_rat #(
    parameter int HART_ID = 0
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Redirecionamento (Flush) ---
    // (Vindo do Commit (mispredict) ou da STU (SQUASH/L2_START))
    input  wire                                  redirect_valid_in,

    // --- Interface com o Rename (Leitura Especulativa) ---
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_gpr_read_addr_in,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_gpr_read_tag_out,
    output logic [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_gpr_read_ready_out,

    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_fpr_read_addr_in,
    output logic [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_fpr_read_tag_out,
    output logic [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_fpr_read_ready_out,

    // --- Interface com o Rename (Escrita Especulativa) ---
    input  wire [supernova_pkg::FETCH_WIDTH-1:0]                       rat_gpr_write_valid_in,
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_gpr_write_addr_in,
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_gpr_write_tag_in,

    input  wire [supernova_pkg::FETCH_WIDTH-1:0]                       rat_fpr_write_valid_in,
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_fpr_write_addr_in,
    input  wire [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_fpr_write_tag_in,

    // --- Interface com o Commit (Atualização Arquitetural) ---
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0] arf_write_valid_in,
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] arf_write_addr_in,
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    arf_gpr_write_tag_in,
    input  wire [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    arf_fpr_write_tag_in
    
    // (Também precisa de entradas de 'ready' do PRF, omitidas por simplicidade)
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Tabela de Mapeamento de GPR ---
    // (Mapeia x0-x31 para p0-p127)
    logic [GPR_TAG_WIDTH-1:0] commit_rat_gpr [0:NUM_ARCH_GPRS-1];
    logic [GPR_TAG_WIDTH-1:0] spec_rat_gpr   [0:NUM_ARCH_GPRS-1];
    
    // (O PRF (Physical Register File) teria 'busy bits' para 'ready_out')
    // logic prf_gpr_ready [0:NUM_PHYS_GPRS-1]; 

    // --- Tabela de Mapeamento de FPR ---
    // (Mapeia f0-f31 para p0-p127)
    logic [FPR_TAG_WIDTH-1:0] commit_rat_fpr [0:NUM_ARCH_GPRS-1];
    logic [FPR_TAG_WIDTH-1:0] spec_rat_fpr   [0:NUM_ARCH_GPRS-1];
    // logic prf_fpr_ready [0:NUM_PHYS_FPRS-1];


    // --- Lógica de Leitura (Combinacional) ---
    // O estágio de Rename *sempre* lê da RAT Especulativa.
    
    genvar i, j;
    generate
        for (i = 0; i < FETCH_WIDTH; i++) begin : gen_read_ports
            for (j = 0; j < 2; j++) begin : gen_read_src
                // GPRs
                assign rat_gpr_read_tag_out[i][j] = spec_rat_gpr[rat_gpr_read_addr_in[i][j]];
                // (Lógica de 'ready' leria o busy bit do PRF)
                assign rat_gpr_read_ready_out[i][j] = 1'b1; // STUB
                
                // FPRs
                assign rat_fpr_read_tag_out[i][j] = spec_rat_fpr[rat_fpr_read_addr_in[i][j]];
                assign rat_fpr_read_ready_out[i][j] = 1'b1; // STUB
            end
        end
    endgenerate

    // --- Lógica de Escrita e Commit (Sequencial) ---
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Inicializa ambas as tabelas
            for (int k = 0; k < NUM_ARCH_GPRS; k++) begin
                // Mapeamento 1-para-1 inicial (x0->p0, x1->p1, ...)
                commit_rat_gpr[k] <= k;
                spec_rat_gpr[k]   <= k;
                commit_rat_fpr[k] <= k;
                spec_rat_fpr[k]   <= k;
            end
        end
        // --- Cenário 1: Flush (Redirect / STU SQUASH) ---
        else if (redirect_valid_in) begin
            // Restaura o estado especulativo para o estado de commit (seguro)
            for (int k = 0; k < NUM_ARCH_GPRS; k++) begin
                spec_rat_gpr[k] <= commit_rat_gpr[k];
                spec_rat_fpr[k] <= commit_rat_fpr[k];
            end
        end
        // --- Cenário 2: Operação Normal ---
        else begin
            
            // --- Atualização da RAT de Commit ---
            // (Ocorre quando o estágio de Commit aposenta instruções)
            for (int c = 0; c < COMMIT_WIDTH; c++) begin
                if (arf_write_valid_in[c] && arf_write_addr_in[c] != 0) begin
                    if (is_fpr(arf_write_addr_in[c])) // (Função stub)
                        commit_rat_fpr[arf_write_addr_in[c]] <= arf_fpr_write_tag_in[c];
                    else
                        commit_rat_gpr[arf_write_addr_in[c]] <= arf_gpr_write_tag_in[c];
                end
            end

            // --- Atualização da RAT Especulativa ---
            // (Ocorre quando o estágio de Rename aloca instruções)
            for (int w = 0; w < FETCH_WIDTH; w++) begin
                if (rat_gpr_write_valid_in[w] && rat_gpr_write_addr_in[w] != 0) begin
                    spec_rat_gpr[rat_gpr_write_addr_in[w]] <= rat_gpr_write_tag_in[w];
                end
                
                if (rat_fpr_write_valid_in[w] && rat_fpr_write_addr_in[w] != 0) begin
                    spec_rat_fpr[rat_fpr_write_addr_in[w]] <= rat_fpr_write_tag_in[w];
                end
            end
            
        end
    end
    
    // (Função Stub)
    function automatic logic is_fpr(input [4:0] addr);
        return 1'b0; // TODO: Implementar lógica de detecção de FPR
    endfunction

endmodule
