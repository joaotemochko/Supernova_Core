`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_commit
 * @brief Estágio de Commit (Aposentadoria) do Supernova.
 *
 * @details Este módulo contém o Reorder Buffer (ROB) e atua como
 * o estágio final do pipeline. Ele garante que as instruções
 * sejam validadas (comitadas) em ordem.
 *
 * Ele lida com a escrita no ARF (Banco de Registradores Arquitetural),
 * flushes por mispredict/exceção, e flushes da STU (SQUASH).
 */
module supernova_commit #(
    parameter int HART_ID = 0
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface com o Rename (Alocação no ROB) ---
    input  wire [supernova_pkg::FETCH_WIDTH-1:0]                       rob_alloc_valid_in,
    input  supernova_pkg::rob_entry_t [supernova_pkg::FETCH_WIDTH-1:0] rob_alloc_entry_in,
    input  wire [supernova_pkg::ROB_IDX_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rob_alloc_idx_in,
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0]                       rob_tail_out,
    output logic                                                          rob_full_out,

    // --- Interface com Execute/LSQ (Writeback no ROB) ---
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0]    rob_wb_valid_in,
    input  wire [supernova_pkg::ROB_IDX_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] rob_wb_idx_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    rob_wb_data_in,
    input  wire [supernova_pkg::COMMIT_WIDTH-1:0]    rob_wb_exception_in,
    input  wire [stu_pkg::ADDR_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    rob_wb_trap_cause_in,

    // --- Interface de Redirecionamento (Saída para o Fetch) ---
    output logic                            redirect_valid_out,
    output logic [stu_pkg::ADDR_WIDTH-1:0]  redirect_pc_out,
    
    // --- Saídas para o ARF (Banco de Registradores) ---
    output logic [supernova_pkg::COMMIT_WIDTH-1:0] arf_write_valid_out,
    output logic [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] arf_write_addr_out,
    output logic [stu_pkg::REG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0]    arf_write_data_out,
    // (Adicionar portas de escrita FPR)
    
    // --- Saídas para o PRF Free List ---
    output logic [supernova_pkg::COMMIT_WIDTH-1:0] prf_free_valid_out,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::COMMIT_WIDTH-1:0] prf_free_tag_out,
    // (Adicionar portas de liberação de FPR)
    
    // --- Saídas para o LSQ ---
    output logic                                  commit_store_valid_out,
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] commit_store_rob_idx_out,
    
    // --- Interface da STU (Veredito L2) ---
    input  wire [stu_pkg::NUM_CORES-1:0] squash_in,
    input  wire [stu_pkg::NUM_CORES-1:0] commit_in,
    input  wire stu_pkg::core_id_t        l2_spec_core_id_in,
    input  wire                      is_speculating_l2_in
);

    import stu_pkg::*;
    import supernova_pkg::*;

    const core_id_t MY_HART_ID = core_id_t'(HART_ID);

    // --- O Reorder Buffer (ROB) ---
    rob_entry_t rob [0:ROB_ENTRIES-1];
    logic [ROB_IDX_WIDTH-1:0] rob_head; // Ponteiro de Commit
    logic [ROB_IDX_WIDTH-1:0] rob_tail; // Ponteiro de Alocação
    logic rob_is_full;
    logic [$clog2(ROB_ENTRIES)+1:0] rob_count; // Contador de entradas

    assign rob_full_out = rob_is_full;
    assign rob_tail_out = rob_tail;
    
    // --- Lógica de Alocação e Writeback (Sequencial) ---
    
    logic [COMMIT_WIDTH-1:0] commit_valid_comb; // Sinal combinacional
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rob_head <= '0;
            rob_tail <= '0;
            rob_is_full <= 1'b0;
            rob_count <= '0;
            for (int i = 0; i < ROB_ENTRIES; i++) begin
                rob[i].valid <= 1'b0;
            end
        end 
        // --- STU SQUASH (Prioridade Máxima) ---
        else if (is_speculating_l2_in && squash_in[MY_HART_ID]) begin
            // Flush total do ROB
            rob_head <= '0;
            rob_tail <= '0;
            rob_is_full <= 1'b0;
            rob_count <= '0;
            for (int i = 0; i < ROB_ENTRIES; i++) begin
                rob[i].valid <= 1'b0;
            end
        end
        // --- Flush (Mispredict/Exceção) ---
        else if (redirect_valid_out) begin
            // (Lógica de flush: rebobina o ROB)
            rob_head <= '0;
            rob_tail <= '0;
            rob_is_full <= 1'b0;
            rob_count <= '0;
            for (int i = 0; i < ROB_ENTRIES; i++) begin
                rob[i].valid <= 1'b0;
            end
        end 
        else begin
            
            // --- Lógica de Alocação (do Rename) ---
            int alloc_count = 0;
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (rob_alloc_valid_in[i]) begin
                    rob[rob_alloc_idx_in[i]] <= rob_alloc_entry_in[i];
                    alloc_count++;
                end
            end
            rob_tail <= rob_tail + alloc_count;
                
            // --- Lógica de Writeback (do Execute/LSQ) ---
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (rob_wb_valid_in[i]) begin
                    // Escreve o resultado na entrada correta do ROB
                    rob[rob_wb_idx_in[i]].done <= 1'b1;
                    rob[rob_wb_idx_in[i]].result_data <= rob_wb_data_in[i];
                    rob[rob_wb_idx_in[i]].has_exception <= rob_wb_exception_in[i];
                    rob[rob_wb_idx_in[i]].trap_cause <= rob_wb_trap_cause_in[i];
                end
            end
            
            // --- Lógica de Commit (Avança o Head) ---
            int commit_count = 0;
            for (int i = 0; i < COMMIT_WIDTH; i++) begin
                if (commit_valid_comb[i]) begin
                    rob[rob_head + i].valid <= 1'b0; // Libera a entrada do ROB
                    commit_count++;
                end else begin
                    break; // Para no primeiro não-comitável
                end
            end
            rob_head <= rob_head + commit_count;
            
            // --- Lógica de Contagem Full/Empty ---
            rob_count <= rob_count + alloc_count - commit_count;
            rob_is_full <= (rob_count > (ROB_ENTRIES - FETCH_WIDTH));
        end
    end


    // --- Lógica de Commit (Combinacional) ---
    
    rob_entry_t commit_entry [COMMIT_WIDTH-1:0];

    always_comb begin
        // Padrões de saída
        redirect_valid_out = 1'b0;
        redirect_pc_out = '0;
        arf_write_valid_out = '{default: '0};
        arf_write_addr_out = '{default: '0};
        arf_write_data_out = '{default: '0};
        prf_free_valid_out = '{default: '0};
        prf_free_tag_out = '{default: '0};
        commit_store_valid_out = 1'b0;
        commit_store_rob_idx_out = '0;
        commit_valid_comb = '{default: '0};

        // --- Analisa até COMMIT_WIDTH instruções na cabeça do ROB ---
        for (int i = 0; i < COMMIT_WIDTH; i++) begin
            commit_entry[i] = rob[rob_head + i];
            
            // Se a entrada é válida e a execução terminou
            if (commit_entry[i].valid && commit_entry[i].done) begin
                
                // --- Cenário 1: Exceção ---
                // (Page Fault, Misaligned, ECALL)
                if (commit_entry[i].has_exception) begin
                    
                    if (is_speculating_l2_in) begin
                        // A STU (Validator) lidará com o SQUASH.
                        // Não faça nada, mas pare o commit.
                        break; // Para o loop 'i'
                    end else begin
                        // Exceção real! Dispare um flush.
                        redirect_valid_out = 1'b1;
                        // redirect_pc_out = csr_mtvec/stvec; // (Lógica de Trap)
                        // (Atualizar CSRs: mepc=pc, mcause=trap_cause)
                    end
                    
                    // Avance o ROB (descarte a instrução falha)
                    commit_valid_comb[i] = 1'b1;
                    // Libere o registrador físico *antigo*
                    prf_free_valid_out[i] = 1'b1;
                    prf_free_tag_out[i] = commit_entry[i].rd_phys_old;
                    
                    // Pare o commit aqui, só comite 1 exceção por vez
                    break; 
                end
                
                // --- Cenário 2: Branch Mispredicted ---
                // (TODO: Lógica de verificação do Branch Predictor)
                // else if (commit_entry[i].is_branch && mispredicted) begin
                //     redirect_valid_out = 1'b1;
                //     redirect_pc_out = corrected_pc;
                //     commit_valid_comb[i] = 1'b1;
                //     break; // Flush
                // end

                // --- Cenário 3: Commit Normal ---
                else begin
                    commit_valid_comb[i] = 1'b1;
                    
                    // Se for um STORE, diga ao LSQ que ele pode ir
                    if (commit_entry[i].is_store) begin
                        commit_store_valid_out = 1'b1; // (Simplificado: 1 store/ciclo)
                        commit_store_rob_idx_out = rob_head + i;
                    end
                    
                    // Se escrever em um registrador arquitetural
                    if (commit_entry[i].rd_arch != 0) begin
                        // Escreva no ARF
                        arf_write_valid_out[i] = 1'b1;
                        arf_write_addr_out[i] = commit_entry[i].rd_arch;
                        arf_write_data_out[i] = commit_entry[i].result_data;
                        
                        // Libere o registrador físico *antigo*
                        prf_free_valid_out[i] = 1'b1;
                        prf_free_tag_out[i] = commit_entry[i].rd_phys_old;
                    end
                end
                
            end // if (valid && done)
            else begin
                // Instrução não está pronta, pare o commit aqui.
                break;
            end
        end // Fim do loop
    end // always_comb

endmodule
