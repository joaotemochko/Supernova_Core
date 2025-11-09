`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_prf
 * @brief Physical Register File (PRF) para GPRs e FPRs.
 *
 * @details Este módulo é a memória SRAM real que armazena os valores
 * dos registradores físicos (ex: p0-p127).
 *
 * É uma memória multi-portada para permitir que o estágio de Issue
 * leia múltiplos operandos e o estágio de Writeback
 * escreva múltiplos resultados por ciclo.
 */
module supernova_prf #(
    parameter int HART_ID = 0,
    // (Assumindo os mesmos parâmetros de porta do Issue/Commit)
    parameter int NUM_READ_PORTS = supernova_pkg::ISSUE_WIDTH * 2, // 4-wide Issue * 2 srcs
    parameter int NUM_WRITE_PORTS = supernova_pkg::COMMIT_WIDTH      // 4-wide Commit/WB
) (
    input  wire clk,
    input  wire rst_n,

    // --- Portas de Leitura (para o Estágio de Issue) ---
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [NUM_READ_PORTS-1:0] gpr_read_addr_in,
    output logic [stu_pkg::REG_WIDTH-1:0] [NUM_READ_PORTS-1:0]    gpr_read_data_out,

    input  wire [supernova_pkg::FPR_TAG_WIDTH-1:0] [NUM_READ_PORTS-1:0] fpr_read_addr_in,
    output logic [stu_pkg::REG_WIDTH-1:0] [NUM_READ_PORTS-1:0]    fpr_read_data_out,

    // --- Portas de Escrita (do Estágio de Writeback/CDB) ---
    input  wire [NUM_WRITE_PORTS-1:0]            gpr_write_valid_in,
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [NUM_WRITE_PORTS-1:0] gpr_write_addr_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [NUM_WRITE_PORTS-1:0]    gpr_write_data_in,
    
    input  wire [NUM_WRITE_PORTS-1:0]            fpr_write_valid_in,
    input  wire [supernova_pkg::FPR_TAG_WIDTH-1:0] [NUM_WRITE_PORTS-1:0] fpr_write_addr_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [NUM_WRITE_PORTS-1:0]    fpr_write_data_in
    
    // (Também precisa de 'busy bits' para a RAT, omitidos por simplicidade)
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- O Banco de Registradores Físicos (GPR) ---
    // (Implementado como registradores, mas seria SRAM em um ASIC)
    logic [REG_WIDTH-1:0] gpr_reg_file [0:NUM_PHYS_GPRS-1];

    // --- O Banco de Registradores Físicos (FPR) ---
    logic [REG_WIDTH-1:0] fpr_reg_file [0:NUM_PHYS_FPRS-1];


    // --- Lógica de Leitura (Combinacional) ---
    // (A leitura do PRF é tipicamente combinacional para
    //  o estágio de Issue verificar a prontidão)
    
    genvar i;
    generate
        for (i = 0; i < NUM_READ_PORTS; i++) begin : gen_read_ports
            // GPRs
            assign gpr_read_data_out[i] = gpr_reg_file[gpr_read_addr_in[i]];
            
            // FPRs
            assign fpr_read_data_out[i] = fpr_reg_file[fpr_read_addr_in[i]];
        end
    endgenerate

    // --- Lógica de Escrita (Sequencial) ---
    // (A escrita ocorre no final do estágio de Writeback)
    
    always_ff @(posedge clk) begin
        // (Não é resetável para velocidade)
        
        // GPRs
        for (int j = 0; j < NUM_WRITE_PORTS; j++) begin
            if (gpr_write_valid_in[j]) begin
                // O Tag 'p0' (mapeado para x0) nunca pode ser escrito
                if (gpr_write_addr_in[j] != 0) begin 
                    gpr_reg_file[gpr_write_addr_in[j]] <= gpr_write_data_in[j];
                end
            end
        end
        
        // FPRs
        for (int k = 0; k < NUM_WRITE_PORTS; k++) begin
            if (fpr_write_valid_in[k]) begin
                fpr_reg_file[fpr_write_addr_in[k]] <= fpr_write_data_in[k];
            end
        end
    end

endmodule
