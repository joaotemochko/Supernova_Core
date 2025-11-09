`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_free_list
 * @brief Gerencia o pool de registradores físicos (PRF) livres.
 *
 * @details Este módulo implementa uma fila circular (FIFO) que armazena
 * os 'tags' (índices) dos registradores físicos que não estão
 * em uso.
 *
 * Ele fornece tags para o estágio de Rename (alocação) e
 * aceita tags de volta do estágio de Commit (liberação).
 * Implementa lógicas separadas para GPRs e FPRs.
 */
module supernova_free_list #(
    parameter int HART_ID = 0
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Redirecionamento (Flush) ---
    // (Vindo do Commit (mispredict) ou da STU (SQUASH/L2_START))
    input  wire                                  redirect_valid_in,
    // (Em um flush, o RAT é restaurado, então o Free List também precisa ser)
    // (Esta interface de snapshot/restore é complexa e omitida aqui)
    
    // --- Interface de Alocação (Saída para o Rename) ---
    // GPRs
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] prf_gpr_alloc_tag_out,
    output logic                                     prf_gpr_alloc_ready_out,
    // FPRs
    output logic [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] prf_fpr_alloc_tag_out,
    output logic                                     prf_fpr_alloc_ready_out,

    // --- Interface de Liberação (Entrada do Commit) ---
    // GPRs
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0] prf_gpr_free_valid_in,
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] prf_gpr_free_tag_in,
    // FPRs
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0] prf_fpr_free_valid_in,
    input  wire [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] prf_fpr_free_tag_in
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Fila de GPRs Livres ---
    logic [GPR_TAG_WIDTH-1:0] gpr_free_list_q [0:NUM_PHYS_GPRS-1];
    logic [GPR_TAG_WIDTH-1:0] gpr_alloc_ptr; // Ponteiro de Leitura (Alocação)
    logic [GPR_TAG_WIDTH-1:0] gpr_free_ptr;  // Ponteiro de Escrita (Liberação)
    logic gpr_list_full, gpr_list_empty;
    logic [GPR_TAG_WIDTH:0] gpr_list_count; // Um bit extra para diferenciar full/empty

    // --- Fila de FPRs Livres ---
    logic [FPR_TAG_WIDTH-1:0] fpr_free_list_q [0:NUM_PHYS_FPRS-1];
    logic [FPR_TAG_WIDTH-1:0] fpr_alloc_ptr;
    logic [FPR_TAG_WIDTH-1:0] fpr_free_ptr;
    logic fpr_list_full, fpr_list_empty;
    logic [FPR_TAG_WIDTH:0] fpr_list_count;
    
    
    // --- Lógica da Fila de GPRs Livres (Sequencial) ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Preenche a lista com todos os tags físicos
            // (Deixando os 32 primeiros para o estado arquitetural inicial)
            for (int i = NUM_ARCH_GPRS; i < NUM_PHYS_GPRS; i++) begin
                gpr_free_list_q[i - NUM_ARCH_GPRS] <= i;
            end
            gpr_alloc_ptr <= 0;
            gpr_free_ptr <= NUM_PHYS_GPRS - NUM_ARCH_GPRS;
            gpr_list_count <= NUM_PHYS_GPRS - NUM_ARCH_GPRS;
        end
        // TODO: Lógica de 'redirect_valid_in' (restore)
        else begin
            
            // --- Alocação (Leitura da FIFO) ---
            // (O Rename pede até FETCH_WIDTH tags)
            if (prf_gpr_alloc_ready_out) begin
                // (Assumindo que o Rename só pede se houver 'ready')
                gpr_alloc_ptr <= gpr_alloc_ptr + FETCH_WIDTH;
                gpr_list_count <= gpr_list_count - FETCH_WIDTH;
            end
            
            // --- Liberação (Escrita na FIFO) ---
            // (O Commit retorna até COMMIT_WIDTH tags)
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (prf_gpr_free_valid_in[i]) begin
                    gpr_free_list_q[gpr_free_ptr + i] <= prf_gpr_free_tag_in[i];
                end
            end
            
            // (Lógica de atualização de ponteiro/contador para liberação)
            // (Simplificado)
            if (prf_gpr_free_valid_in[0]) begin
                gpr_free_ptr <= gpr_free_ptr + COMMIT_WIDTH;
                gpr_list_count <= gpr_list_count + COMMIT_WIDTH;
            end
            
        end
    end
    
    // --- Saídas de Alocação (Combinacional) ---
    
    // GPRs
    assign prf_gpr_alloc_ready_out = (gpr_list_count >= FETCH_WIDTH);
    genvar i;
    generate
        for (i = 0; i < FETCH_WIDTH; i++) begin : gen_gpr_alloc
            assign prf_gpr_alloc_tag_out[i] = gpr_free_list_q[gpr_alloc_ptr + i];
        end
    endgenerate

    // --- Lógica da Fila de FPRs Livres (Sequencial) ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // (Lógica de reset idêntica para FPRs)
            for (int j = NUM_ARCH_GPRS; j < NUM_PHYS_FPRS; j++) begin
                fpr_free_list_q[j - NUM_ARCH_GPRS] <= j;
            end
            fpr_alloc_ptr <= 0;
            fpr_free_ptr <= NUM_PHYS_FPRS - NUM_ARCH_GPRS;
            fpr_list_count <= NUM_PHYS_FPRS - NUM_ARCH_GPRS;
        end
        // TODO: Lógica de 'redirect_valid_in' (restore)
        else begin
            // (Lógica de alocação/liberação idêntica para FPRs)
            
            if (prf_fpr_alloc_ready_out) begin
                fpr_alloc_ptr <= fpr_alloc_ptr + FETCH_WIDTH;
                fpr_list_count <= fpr_list_count - FETCH_WIDTH;
            end
            
            if (prf_fpr_free_valid_in[0]) begin
                // (Lógica de escrita na fila)
                fpr_free_ptr <= fpr_free_ptr + COMMIT_WIDTH;
                fpr_list_count <= fpr_list_count + COMMIT_WIDTH;
            end
        end
    end
    
    // FPRs
    assign prf_fpr_alloc_ready_out = (fpr_list_count >= FETCH_WIDTH);
    genvar j;
    generate
        for (j = 0; j < FETCH_WIDTH; j++) begin : gen_fpr_alloc
            assign prf_fpr_alloc_tag_out[j] = fpr_free_list_q[fpr_alloc_ptr + j];
        end
    endgenerate

endmodule
