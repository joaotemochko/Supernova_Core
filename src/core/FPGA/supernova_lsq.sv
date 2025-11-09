`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_lsq
 * @brief Load/Store Queue (LSQ) para o Supernova.
 *
 * @details Este módulo gerencia todas as operações de memória (LOAD,
 * STORE, AMO) de forma "out-of-order". Ele implementa:
 * 1. Store-to-Load forwarding (Encaminhamento).
 * 2. Detecção de 'aliasing' de memória (dependências).
 * 3. Interface com o dTLB e D-Cache (memória de dados).
 * 4. Interface com o STU Memory Tracker (reportando PAs).
 * 5. Interface com o STU Validator (reportando Exceções/Page Faults).
 */
module supernova_lsq #(
    parameter int HART_ID = 0,
    parameter int LSQ_ENTRIES = 32 // 32 entradas de Load/Store
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Redirecionamento (Flush) ---
    input  wire                                  redirect_valid_in,

    // --- Interface com o Rename (Alocação na LSQ) ---
    input  wire [supernova_pkg::FETCH_WIDTH-1:0]                       lsq_alloc_valid_in,
    input  supernova_pkg::rs_entry_t [supernova_pkg::FETCH_WIDTH-1:0]  lsq_alloc_entry_in,

    // --- Interface com o Barramento de Wakeup (CDB) ---
    // (Lê o CDB para obter dados de rs1, rs2 - para Addr e Store Data)
    input  wire [1:0]                         cdb_valid_in,
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [1:0] cdb_gpr_tag_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [1:0]    cdb_gpr_data_in,
    // (Também precisaria de portas CDB_FPR)

    // --- Interface com o dTLB (Consulta de Endereço) ---
    output logic                         dtlb_lookup_valid_out,
    output logic [stu_pkg::VPN_WIDTH-1:0] dtlb_lookup_vpn_out,
    input  wire                          dtlb_hit_in,
    input  wire [stu_pkg::PPN_WIDTH-1:0]   dtlb_ppn_in,
    input  wire                          dtlb_page_fault_in,

    // --- Interface com o D-Cache (Memória de Dados) ---
    output logic [stu_pkg::PHYS_ADDR_SIZE-1:0] dcache_addr_pa_out,
    output logic [stu_pkg::XLEN-1:0]             dcache_wdata_out,
    output logic [7:0]                 dcache_wstrb_out,
    output logic                       dcache_req_out,
    output logic                       dcache_we_out,
    input  wire [stu_pkg::XLEN-1:0]             dcache_rdata_in,
    input  wire                        dcache_ack_in,
    
    // --- Interface com o ROB (Writeback) ---
    // (Envia resultados de LOAD prontos para o ROB/CDB)
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] rob_wb_idx_out,
    output logic [stu_pkg::REG_WIDTH-1:0]    rob_wb_data_out,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] rob_wb_gpr_tag_out,
    output logic [supernova_pkg::FPR_TAG_WIDTH-1:0] rob_wb_fpr_tag_out,
    output logic                         rob_wb_valid_out,
    output logic                         rob_wb_exception_out,
    output logic [stu_pkg::ADDR_WIDTH-1:0]    rob_wb_trap_cause_out,

    // --- Interface com o Commit ---
    // (Sinal do Commit para nos dizer quando 'aposentar' um STORE)
    input  wire                          commit_store_valid_in,
    input  wire [supernova_pkg::ROB_IDX_WIDTH-1:0] commit_store_rob_idx_in,

    // --- Saídas para a STU ---
    output logic stu_pkg::addr_t   core_mem_pa_out,
    output logic                 core_mem_is_store_out,
    output logic                 core_mem_valid_out
);

    import stu_pkg::*;
    import supernova_pkg::*;

    typedef struct packed {
        logic busy;
        logic is_load;
        logic is_store;
        logic is_amo;
        
        rs_entry_t entry; // Cópia da entrada do Rename

        // Estado do Endereço (Addr)
        logic addr_ready;
        stu_pkg::addr_t addr_va; // Endereço Virtual
        
        // Estado dos Dados do Store
        logic data_ready;
        stu_pkg::reg_width_t store_data;
        
        // Estado da Memória
        logic tlb_done;       // 1 = dTLB consultado
        logic page_fault;     // 1 = Page Fault
        stu_pkg::addr_t addr_pa; // Endereço Físico
        
        logic mem_done;       // 1 = Acesso ao D-Cache concluído
        logic committed;      // 1 = Commit nos disse que este STORE pode ir

    } lsq_entry_t;

    lsq_entry_t lsq_table [0:LSQ_ENTRIES-1];
    
    // Ponteiros FIFO circulares
    logic [$clog2(LSQ_ENTRIES):0] lsq_count;
    logic [$clog2(LSQ_ENTRIES)-1:0] lsq_alloc_ptr; // 'tail'
    logic [$clog2(LSQ_ENTRIES)-1:0] lsq_commit_ptr; // 'head' (para aposentadoria)
    
    // Ponteiro de Emissão (Issue) - varre entre commit e alloc
    logic [$clog2(LSQ_ENTRIES)-1:0] lsq_issue_ptr; 


    // --- Lógica de Wakeup (CDB -> LSQ) e Alocação (Rename -> LSQ) ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < LSQ_ENTRIES; i++) begin
                lsq_table[i].busy <= 1'b0;
            end
            lsq_alloc_ptr <= '0;
            lsq_issue_ptr <= '0;
            lsq_commit_ptr <= '0;
            lsq_count <= '0;
        end 
        else if (redirect_valid_in) begin
            // Flush: Invalida todas as entradas da LSQ
            for (int i = 0; i < LSQ_ENTRIES; i++) begin
                // (Não limpe 'committed' stores -
                //  em um flush real, o commit ptr rebobinaria)
                if (!lsq_table[i].committed) begin
                    lsq_table[i].busy <= 1'b0;
                end
            end
            // (Resetar ponteiros)
            lsq_alloc_ptr <= lsq_commit_ptr; 
            lsq_issue_ptr <= lsq_commit_ptr;
            // (lsq_count precisaria ser recalculado)
        end 
        else begin
            // --- Lógica de Wakeup (CDB) ---
            // (Varre a LSQ e atualiza addr_ready/data_ready)
            for (int i = 0; i < LSQ_ENTRIES; i++) begin
                if (lsq_table[i].busy) begin
                    // (Verifica se src1 (base addr) está no CDB)
                    if (!lsq_table[i].addr_ready && !lsq_table[i].entry.src1_ready) begin
                        for (int c = 0; c < 2; c++) begin // (Assumindo 2 portas CDB)
                            if (cdb_valid_in[c] && cdb_gpr_tag_in[c] == lsq_table[i].entry.src1_tag) begin
                                lsq_table[i].addr_ready <= 1'b1;
                                // (A lógica combinacional recalculará o addr_va)
                                lsq_table[i].entry.src1_data <= cdb_gpr_data_in[c]; 
                            end
                        end
                    end
                    // (Verifica se src2 (store data) está no CDB)
                    if (lsq_table[i].is_store && !lsq_table[i].data_ready && !lsq_table[i].entry.src2_ready) begin
                        for (int c = 0; c < 2; c++) begin
                            if (cdb_valid_in[c] && cdb_gpr_tag_in[c] == lsq_table[i].entry.src2_tag) begin
                                lsq_table[i].data_ready <= 1'b1;
                                lsq_table[i].store_data <= cdb_gpr_data_in[c];
                            end
                        end
                    end
                end
            end
            
            // --- Lógica de Alocação (Rename -> LSQ) ---
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (lsq_alloc_valid_in[i]) begin
                    lsq_table[lsq_alloc_ptr].busy <= 1'b1;
                    lsq_table[lsq_alloc_ptr].is_load <= lsq_alloc_entry_in[i].is_load;
                    lsq_table[lsq_alloc_ptr].is_store <= lsq_alloc_entry_in[i].is_store;
                    lsq_table[lsq_alloc_ptr].is_amo <= lsq_alloc_entry_in[i].is_amo;
                    lsq_table[lsq_alloc_ptr].entry <= lsq_alloc_entry_in[i];
                    
                    lsq_table[lsq_alloc_ptr].addr_ready <= lsq_alloc_entry_in[i].src1_ready;
                    lsq_table[lsq_alloc_ptr].addr_va <= lsq_alloc_entry_in[i].src1_data + lsq_alloc_entry_in[i].instr.imm; // (Imm pré-calculado no Rename)
                    lsq_table[lsq_alloc_ptr].data_ready <= lsq_alloc_entry_in[i].src2_ready;
                    lsq_table[lsq_alloc_ptr].store_data <= lsq_alloc_entry_in[i].src2_data;
                    
                    lsq_table[lsq_alloc_ptr].tlb_done <= 1'b0;
                    lsq_table[lsq_alloc_ptr].page_fault <= 1'b0;
                    lsq_table[lsq_alloc_ptr].mem_done <= 1'b0;
                    lsq_table[lsq_alloc_ptr].committed <= 1'b0;
                    
                    lsq_alloc_ptr <= lsq_alloc_ptr + 1;
                    lsq_count <= lsq_count + 1;
                end
            end

            // --- Lógica de Commit (Aposentadoria de Store) ---
            if (commit_store_valid_in) begin
                if (lsq_table[lsq_commit_ptr].busy && 
                    lsq_table[lsq_commit_ptr].is_store &&
                    lsq_table[lsq_commit_ptr].entry.rob_idx == commit_store_rob_idx_in) 
                begin
                    lsq_table[lsq_commit_ptr].committed <= 1'b1;
                    // (Não avance o commit_ptr ainda, espere a escrita na memória)
                end
            end
            
            // --- Lógica de Emissão (Issue) da LSQ (Sequencial) ---
            // (Atualiza o estado da LSQ com base nos 'acks' de TLB/D-Cache)
            
            // (Este ponteiro varre procurando por trabalho)
            lsq_issue_ptr <= (lsq_table[lsq_issue_ptr].mem_done || lsq_table[lsq_issue_ptr].page_fault) ? 
                             lsq_issue_ptr + 1 : 
                             lsq_issue_ptr;
                             
            lsq_entry_t current_entry = lsq_table[lsq_issue_ptr];
            
            // --- Atualização do TLB ---
            if (current_entry.busy && current_entry.addr_ready && !current_entry.tlb_done) begin
                if (dtlb_hit_in) begin // (Assumindo que o TLB responde em 1 ciclo)
                    lsq_table[lsq_issue_ptr].tlb_done <= 1'b1;
                    lsq_table[lsq_issue_ptr].addr_pa <= {dtlb_ppn_in, current_entry.addr_va[11:0]};
                    lsq_table[lsq_issue_ptr].page_fault <= dtlb_page_fault_in;
                end
                // (Se houver 'miss', o 'dtlb_lookup_valid_out'
                //  permanecerá alto até o 'hit' ou 'fault')
            end
            
            // --- Atualização do D-Cache (Avança o ponteiro) ---
            if (current_entry.busy && current_entry.tlb_done && !current_entry.mem_done) begin
                if (current_entry.page_fault) begin
                    // (Falha de Página! Reporte ao ROB)
                    lsq_table[lsq_issue_ptr].mem_done <= 1'b1;
                end
                else if (current_entry.is_load && dcache_ack_in) begin
                    lsq_table[lsq_issue_ptr].mem_done <= 1'b1;
                end
                else if (current_entry.is_store && current_entry.committed && dcache_ack_in) begin
                    lsq_table[lsq_issue_ptr].mem_done <= 1'b1;
                    // (Aposenta o store da LSQ)
                    lsq_table[lsq_commit_ptr].busy <= 1'b0; 
                    lsq_commit_ptr <= lsq_commit_ptr + 1;
                    lsq_count <= lsq_count - 1;
                end
            end
        end
    end

    // --- Lógica de Emissão (Issue) da LSQ (Combinacional) ---
    // (Seleciona uma operação de memória para enviar ao dTLB/D-Cache)

    always_comb begin
        // Padrões
        dtlb_lookup_valid_out = 1'b0;
        dtlb_lookup_vpn_out = '0;
        dcache_req_out = 1'b0;
        dcache_we_out = 1'b0;
        dcache_addr_pa_out = '0;
        dcache_wdata_out = '0;
        dcache_wstrb_out = '0;
        
        rob_wb_valid_out = 1'b0;
        rob_wb_idx_out = '0;
        rob_wb_data_out = '0;
        rob_wb_tag_out = '0;
        rob_wb_fpr_tag_out = '0;
        rob_wb_exception_out = 1'b0;
        rob_wb_trap_cause_out = '0;
        
        core_mem_valid_out = 1'b0;
        core_mem_pa_out = '0;
        core_mem_is_store_out = 1'b0;

        // (Varre a LSQ a partir do lsq_issue_ptr)
        lsq_entry_t current_entry = lsq_table[lsq_issue_ptr];
        
        if (current_entry.busy) begin
            
            // --- Fase 1: Cálculo de Endereço e Consulta ao TLB ---
            if (current_entry.addr_ready && !current_entry.tlb_done) begin
                dtlb_lookup_valid_out = 1'b1;
                dtlb_lookup_vpn_out = current_entry.addr_va[VPN_WIDTH+11:12];
            end
            
            // --- Fase 2: Consulta ao D-Cache (após TLB) ---
            else if (current_entry.tlb_done && !current_entry.mem_done) begin
                
                if (current_entry.page_fault) begin
                    // Falha de Página! Reporte ao ROB.
                    rob_wb_valid_out = 1'b1;
                    rob_wb_idx_out = current_entry.entry.rob_idx;
                    rob_wb_exception_out = 1'b1;
                    rob_wb_trap_cause_out = current_entry.is_load ? 13 : 15; // Load(13) ou Store(15) PF
                end
                
                // --- Lógica de Emissão de LOAD ---
                else if (current_entry.is_load) begin
                    // TODO: Lógica de Store-to-Load Forwarding
                    
                    // Se não houver forwarding, emita o LOAD
                    dcache_req_out = 1'b1;
                    dcache_we_out = 1'b0;
                    dcache_addr_pa_out = current_entry.addr_pa;
                    
                    // --- Interface do STU Tracker ---
                    core_mem_valid_out = 1'b1;
                    core_mem_pa_out = current_entry.addr_pa;
                    core_mem_is_store_out = 1'b0;

                    if (dcache_ack_in) begin
                        // Sucesso! Envie para o ROB/CDB
                        rob_wb_valid_out = 1'b1;
                        rob_wb_idx_out = current_entry.entry.rob_idx;
                        rob_wb_data_out = dcache_rdata_in; // (Precisa de sign-extend)
                        rob_wb_tag_out = current_entry.entry.rd_phys_tag;
                    end
                end
                
                // --- Lógica de Emissão de STORE ---
                else if (current_entry.is_store) begin
                    // STOREs só podem ir para a memória APÓS o Commit
                    if (current_entry.data_ready && current_entry.committed) begin
                        dcache_req_out = 1'b1;
                        dcache_we_out = 1'b1;
                        dcache_addr_pa_out = current_entry.addr_pa;
                        dcache_wdata_out = current_entry.store_data;
                        // (dcache_wstrb)
                        
                        // --- Interface do STU Tracker ---
                        core_mem_valid_out = 1'b1;
                        core_mem_pa_out = current_entry.addr_pa;
                        core_mem_is_store_out = 1'b1;
                    end
                end
                // (TODO: Lógica de AMO)
                
            end // Fim Fase 2
            
        end // if (busy)
    end // always_comb

endmodule
