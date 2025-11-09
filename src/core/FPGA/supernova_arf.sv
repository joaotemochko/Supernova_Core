`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_arf
 * @brief Architectural Register File (ARF) para GPRs e FPRs.
 *
 * @details Este módulo armazena o estado "comitado" (não-especulativo)
 * do processador. Ele é o estado visível para o software.
 *
 * Ele é escrito pelo estágio de Commit e lido pelo
 * 'stu_context_manager' para cópias de checkpoint de Nível 2.
 */
module supernova_arf #(
    parameter int HART_ID = 0
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Escrita (do Estágio de Commit) ---
    // (Pode receber até COMMIT_WIDTH escritas por ciclo)
    // GPRs
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0] arf_gpr_write_valid_in,
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] arf_gpr_write_addr_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    arf_gpr_write_data_in,
    // FPRs
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0] arf_fpr_write_valid_in,
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] arf_fpr_write_addr_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    arf_fpr_write_data_in,

    // --- Interface de Cópia da STU (Lida pelo Context Manager) ---
    // (Assume 1 porta de leitura para cópia)
    // GPRs
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] stu_gpr_read_addr_in,
    output logic [stu_pkg::REG_WIDTH-1:0]    stu_gpr_read_data_out,
    // FPRs
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] stu_fpr_read_addr_in,
    output logic [stu_pkg::REG_WIDTH-1:0]    stu_fpr_read_data_out,
    
    // --- Interface de Escrita da STU (Para restaurar/copiar contexto) ---
    input  wire                      stu_gpr_write_valid_in,
    input  wire [supernova_pkg::ARCH_REG_WIDTH-1:0] stu_gpr_write_addr_in,
    input  wire [stu_pkg::REG_WIDTH-1:0]    stu_gpr_write_data_in
    // (Portas de escrita de FPR da STU omitidas por simplicidade)
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- O Banco de Registradores Arquitetural (GPR) ---
    logic [REG_WIDTH-1:0] gpr_reg_file [0:NUM_ARCH_GPRS-1];

    // --- O Banco de Registradores Arquitetural (FPR) ---
    logic [REG_WIDTH-1:0] fpr_reg_file [0:NUM_ARCH_GPRS-1];


    // --- Lógica de Leitura da STU (Combinacional) ---
    // (Porta de leitura dedicada para o stu_context_manager)
    
    // O registrador 'x0' (GPR) sempre lê zero
    assign stu_gpr_read_data_out = (stu_gpr_read_addr_in == 0) ? 
                                   '0 : 
                                   gpr_reg_file[stu_gpr_read_addr_in];
                                   
    assign stu_fpr_read_data_out = fpr_reg_file[stu_fpr_read_addr_in];


    // --- Lógica de Escrita (Sequencial) ---
    // (Escrita do Commit e da STU)
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reseta todos os registradores para 0
            for (int i = 0; i < NUM_ARCH_GPRS; i++) begin
                gpr_reg_file[i] <= '0;
                fpr_reg_file[i] <= '0;
            end
        end else begin
            
            // --- Escrita do Commit ---
            // (O 'Commit' tem prioridade mais baixa)
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                // GPRs
                if (arf_gpr_write_valid_in[i] && arf_gpr_write_addr_in[i] != 0) begin
                    gpr_reg_file[arf_gpr_write_addr_in[i]] <= arf_gpr_write_data_in[i];
                end
                // FPRs
                if (arf_fpr_write_valid_in[i]) begin // (FPRs não têm 'f0' como zero)
                    fpr_reg_file[arf_fpr_write_addr_in[i]] <= arf_fpr_write_data_in[i];
                end
            end
            
            // --- Escrita da STU (Cópia de Contexto) ---
            // (Tem prioridade mais alta que o Commit)
            if (stu_gpr_write_valid_in && stu_gpr_write_addr_in != 0) begin
                gpr_reg_file[stu_gpr_write_addr_in] <= stu_gpr_write_data_in;
            end
            // (Lógica de escrita de FPR da STU)
            
        end
    end

endmodule
