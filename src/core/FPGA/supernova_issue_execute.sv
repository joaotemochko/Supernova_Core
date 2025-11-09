`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_issue_execute
 * @brief Estágios de Emissão (Issue) e Execução (Execute) do Supernova.
 *
 * @details Este módulo contém as Reservation Stations (RS), a lógica
 * de "wakeup" (via CDB) e a lógica de "issue" (scheduler).
 * Ele seleciona instruções prontas, as executa nas ALUs
 * e transmite os resultados no CDB.
 */
module supernova_issue_execute #(
    parameter int HART_ID = 0,
    parameter int NUM_ALU_UNITS = 2,
    parameter int NUM_FPU_UNITS = 1,
    parameter int NUM_MDU_UNITS = 1
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Redirecionamento (Flush) ---
    input  wire                                  redirect_valid_in,

    // --- Interface com o Rename (Alocação na RS) ---
    input  wire [supernova_pkg::FETCH_WIDTH-1:0]                       rs_alloc_valid_in,
    input  supernova_pkg::rs_entry_t [supernova_pkg::FETCH_WIDTH-1:0]  rs_alloc_entry_in,

    // --- Interface com o Barramento de Wakeup (CDB) ---
    output logic [NUM_ALU_UNITS-1:0]            cdb_valid_out,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [NUM_ALU_UNITS-1:0] cdb_gpr_tag_out,
    output logic [stu_pkg::REG_WIDTH-1:0] [NUM_ALU_UNITS-1:0]    cdb_gpr_data_out,
    // (Sinais de FPU/MDU omitidos por simplicidade)
    // output logic [supernova_pkg::FPR_TAG_WIDTH-1:0] ... cdb_fpr_tag_out,

    // --- Interface com o ROB (Writeback) ---
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] [NUM_ALU_UNITS-1:0] rob_wb_idx_out,
    output logic [stu_pkg::REG_WIDTH-1:0] [NUM_ALU_UNITS-1:0]    rob_wb_data_out,
    output logic                                 [NUM_ALU_UNITS-1:0]    rob_wb_valid_out,
    output logic                                 [NUM_ALU_UNITS-1:0]    rob_wb_exception_out,
    
    // --- Interface com a LSQ (Load/Store Queue) ---
    output logic lsq_alloc_valid_out,
    output logic lsq_entry_t lsq_alloc_entry_out
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- As Reservation Stations (RS) ---
    rs_entry_t rs_table [0:RS_ENTRIES-1];
    
    // Ponteiros para alocação na RS (FIFO circular)
    logic [RS_IDX_WIDTH-1:0] rs_alloc_ptr;

    // --- Sinais de Emissão (Issue) ---
    logic [NUM_ALU_UNITS-1:0] alu_req_valid;
    rs_entry_t                alu_req_entry [NUM_ALU_UNITS-1:0];
    logic [RS_ENTRIES-1:0]    rs_issued; // Bitmask para marcar entradas emitidas
    
    // --- Sinais de Writeback (Vindos das ALUs) ---
    logic [NUM_ALU_UNITS-1:0] alu_wb_valid;
    logic [stu_pkg::REG_WIDTH-1:0] [NUM_ALU_UNITS-1:0] alu_wb_data;
    logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [NUM_ALU_UNITS-1:0] alu_wb_gpr_tag;
    logic [supernova_pkg::ROB_IDX_WIDTH-1:0] [NUM_ALU_UNITS-1:0] alu_wb_rob_idx;
    logic [NUM_ALU_UNITS-1:0] alu_wb_exception;
    
    
    // --- Lógica de Wakeup (CDB -> RS) e Alocação (Rename -> RS) ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < RS_ENTRIES; i++) begin
                rs_table[i].busy <= 1'b0;
            end
            rs_alloc_ptr <= '0;
        end 
        else if (redirect_valid_in) begin
            // Flush: Invalida todas as entradas da RS
            for (int i = 0; i < RS_ENTRIES; i++) begin
                rs_table[i].busy <= 1'b0;
            end
            rs_alloc_ptr <= '0;
        end 
        else begin
            // --- Lógica de Wakeup (CDB) ---
            for (int i = 0; i < RS_ENTRIES; i++) begin
                if (rs_table[i].busy) begin
                    // (Verifica Src1 GPR)
                    if (!rs_table[i].src1_ready && rs_table[i].uses_gpr_rs1) begin
                        for (int c = 0; c < NUM_ALU_UNITS; c++) begin
                            if (alu_wb_valid[c] && alu_wb_gpr_tag[c] == rs_table[i].src1_tag) begin
                                rs_table[i].src1_ready <= 1'b1;
                                rs_table[i].src1_data <= alu_wb_data[c];
                            end
                        end
                    end
                    // (Verifica Src2 GPR)
                    if (!rs_table[i].src2_ready && rs_table[i].uses_gpr_rs2) begin
                        for (int c = 0; c < NUM_ALU_UNITS; c++) begin
                            if (alu_wb_valid[c] && alu_wb_gpr_tag[c] == rs_table[i].src2_tag) begin
                                rs_table[i].src2_ready <= 1'b1;
                                rs_table[i].src2_data <= alu_wb_data[c];
                            end
                        end
                    end
                    // (Lógica similar para FPRs)
                end
            end
            
            // --- Lógica de Consumo (Issue) ---
            // Remove as entradas que foram emitidas no ciclo anterior
            for (int i = 0; i < RS_ENTRIES; i++) begin
                if (rs_issued[i]) begin
                    rs_table[i].busy <= 1'b0;
                end
            end
            
            // --- Lógica de Alocação (Rename -> RS) ---
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (rs_alloc_valid_in[i]) begin
                    // (Assume que rs_full_in impede sobreposição)
                    rs_table[rs_alloc_ptr + i] <= rs_alloc_entry_in[i];
                end
            end
            
            // (Atualiza rs_alloc_ptr)
            if (rs_alloc_valid_in[0]) // (Simplificado)
                rs_alloc_ptr <= rs_alloc_ptr + FETCH_WIDTH;
        end
    end

    // --- Lógica de Issue (Scheduler - Combinacional) ---
    // (Seleciona até NUM_ALU_UNITS instruções prontas)
    
    always_comb begin
        // Padrões
        alu_req_valid = '{default: '0};
        alu_req_entry = '{default: '0};
        rs_issued = '{default: '0};
        
        // (Lógica de seleção simples: "lowest-index-first")
        int alu_units_assigned = 0;
        
        for (int i = 0; i < RS_ENTRIES; i++) begin
            if (alu_units_assigned == NUM_ALU_UNITS) begin
                break; // Todas as ALUs estão ocupadas
            end
            
            // Verifica se a entrada está pronta (e é uma ALU op)
            if (rs_table[i].busy &&
                rs_table[i].src1_ready &&
                rs_table[i].src2_ready &&
                (rs_table[i].is_alu || rs_table[i].is_mdu)) // (Assumindo que ALU lida com MDU)
            begin
                // Emita esta instrução!
                alu_req_valid[alu_units_assigned] = 1'b1;
                alu_req_entry[alu_units_assigned] = rs_table[i];
                rs_issued[i] = 1'b1; // Marque para remoção
                
                alu_units_assigned++;
            end
        end
        // (Lógica de scheduler separada para FPU e LSQ)
        
        lsq_alloc_valid_out = 1'b0; // STUB
        lsq_alloc_entry_out = '0;   // STUB
    end

    // --- Estágio EX: Execute (Instanciação) ---
    
    genvar j;
    generate
        for (j = 0; j < NUM_ALU_UNITS; j++) begin : gen_alu
            
            // Instancia a Unidade ALU
            supernova_alu_unit u_alu (
              .clk(clk), .rst_n(rst_n),
              .req_valid_in(alu_req_valid[j]),
              .req_entry_in(alu_req_entry[j]),
              
              .wb_valid_out(alu_wb_valid[j]),
              .wb_data_out(alu_wb_data[j]),
              .wb_gpr_tag_out(alu_wb_gpr_tag[j]),
              .wb_rob_idx_out(alu_wb_rob_idx[j]),
              .wb_exception_out(alu_wb_exception[j])
            );
            
            // Conecta os resultados do WB ao CDB e ao ROB
            assign cdb_valid_out[j] = alu_wb_valid[j];
            assign cdb_gpr_tag_out[j] = alu_wb_gpr_tag[j];
            assign cdb_gpr_data_out[j] = alu_wb_data[j];
            
            assign rob_wb_valid_out[j] = alu_wb_valid[j];
            assign rob_wb_idx_out[j] = alu_wb_rob_idx[j];
            assign rob_wb_data_out[j] = alu_wb_data[j];
            assign rob_wb_exception_out[j] = alu_wb_exception[j];

        end
    endgenerate
    
    // (Instanciar FPUs e MDUs de forma similar)

endmodule