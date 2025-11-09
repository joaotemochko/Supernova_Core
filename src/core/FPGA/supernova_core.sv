`include "stu_pkg.sv"
`include "supernova_pkg.svh"

// (Inclui os arquivos dos submódulos que definimos)
`include "supernova_fetch.sv"
`include "supernova_decode_rename.sv"
`include "supernova_issue_execute.sv"
`include "supernova_lsq.sv"
`include "supernova_commit.sv"
`include "supernova_rat.sv"
`include "supernova_free_list.sv"
`include "supernova_prf.sv"
`include "supernova_arf.sv"
`include "supernova_alu_unit.sv"
`include "supernova_mdu_unit.sv"
`include "supernova_fpu_unit.sv"
`include "tlb_associative.sv"
`include "ptw_sv39_full.sv"


`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_core
 * @brief Núcleo Out-of-Order (Big Core) RV64G+S, 12 estágios, 4-wide.
 *
 * @details Este é o módulo de topo do núcleo Supernova. Ele instancia
 * todos os componentes da pipeline OoO (Fetch, Rename, Issue,
 * LSQ, Commit) e as estruturas de dados centrais (RAT, PRF, ARF,
 * FreeList, TLB, PTW) e os conecta à interface da STU.
 */
module supernova_core #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int ILEN = 32,
    parameter int PHYS_ADDR_SIZE = 64,
    parameter bit ENABLE_MMU = 1,
    parameter int TLB_ENTRIES = 64,
    parameter int VPN_WIDTH = 39,
    parameter int PPN_WIDTH = 44,
    
    // Parâmetros da Microarquitetura (do supernova_pkg)
    parameter int BTB_ENTRIES = 64,
    parameter int LSQ_ENTRIES = 32,
    parameter int NUM_ALU_UNITS = 2,
    parameter int NUM_FPU_UNITS = 1,
    parameter int NUM_MDU_UNITS = 1
) (
    input wire clk,
    input wire rst_n,

    // -----------------------------------------------------------------
    // --- Interface STU (Speculative Threading Unit) ---
    // -----------------------------------------------------------------
    
    // --- Nível 1 (Conservador) ---
    // (OoO ignora L1, mas a porta é necessária para conformidade)
    input  wire [stu_pkg::NUM_CORES-1:0] l1_dispatch_valid_in,
    input  stu_pkg::instr_t [stu_pkg::NUM_CORES-1:0] [1:0] l1_dispatch_data_in,

    // --- Nível 2 (Otimista) ---
    input  wire stu_pkg::core_id_t  l2_spec_core_id_in,
    input  wire stu_pkg::addr_t     l2_spec_pc_in,
    input  wire                  l2_spec_start_in,
    input  wire stu_pkg::addr_t   l2_fork_trigger_pc_in, 

    // --- Veredito (L2) ---
    input  wire [stu_pkg::NUM_CORES-1:0] squash_in,
    input  wire [stu_pkg::NUM_CORES-1:0] commit_in,
    
    // --- Cópia de Contexto (L2) ---
    input  wire [4:0]                  core_copy_read_addr_in,
    input  wire [4:0]                  core_copy_write_addr_in,
    input  wire                      core_copy_write_en_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] core_copy_data_in,
    output logic [stu_pkg::REG_WIDTH-1:0] core_copy_data_out,

    // --- Status (Saída para STU) ---
    output logic core_busy_out,
    output logic spec_task_done_out,
    output logic spec_exception_out,
    
    // --- Tracker (Saída de Endereço Físico Pós-MMU) ---
    output logic stu_pkg::addr_t   core_mem_pa_out,
    output logic                 core_mem_is_store_out,
    output logic                 core_mem_valid_out,
    
    // -----------------------------------------------------------------
    // --- Interfaces do Núcleo (Memória, Interrupções, etc.) ---
    // -----------------------------------------------------------------
    
    output logic [PHYS_ADDR_SIZE-1:0] imem_addr_pa,
    output logic                       imem_req,
    input  wire [supernova_pkg::FETCH_WIDTH*ILEN-1:0] imem_rdata,
    input  wire                        imem_ack_in,
    input  wire                        imem_error_in,
    
    output logic [PHYS_ADDR_SIZE-1:0] dmem_addr_pa,
    output logic [XLEN-1:0]             dmem_wdata,
    output logic [7:0]                 dmem_wstrb,
    output logic                       dmem_req,
    output logic                       dmem_we,
    input  wire [XLEN-1:0]             dmem_rdata,
    input  wire                        dmem_ack_in,
    input  wire                        dmem_error_in,
    
    output logic                       ptw_mem_req,
    output logic [PHYS_ADDR_SIZE-1:0] ptw_mem_addr,
    input  wire [XLEN-1:0]             ptw_mem_rdata,
    input  wire                        ptw_mem_ack_in,
    input  wire                        ptw_mem_error_in,
    
    input wire timer_irq,
    input wire external_irq,
    input wire software_irq
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --------------------------
    // Lógica de Estado da STU
    // --------------------------
    logic is_speculating_l2; 
    const core_id_t MY_HART_ID = core_id_t'(HART_ID);
    
    // --- Sinal de Flush Global ---
    logic global_flush;
    addr_t global_flush_pc;

    // --- Checkpoint da STU (Nível 2) ---
    logic [XLEN-1:0] shadow_arf_regfile [0:NUM_ARCH_GPRS-1];
    logic [XLEN-1:0] shadow_arf_fregfile [0:NUM_ARCH_GPRS-1];
    logic [XLEN-1:0] shadow_pc;
    // (CSRs)
    logic [XLEN-1:0] csr_mstatus, csr_satp, csr_mepc, csr_sepc; // (Exemplos)


    // --------------------------
    // Sinais Internos do Pipeline (Fios)
    // --------------------------

    // Fetch -> Decode
    logic [FETCH_WIDTH-1:0][ILEN-1:0] fetch_instr_bus;
    addr_t  fetch_pc_bus;
    logic   fetch_valid_bus;
    logic   fetch_ready_bus;

    // Decode/Rename -> ROB
    logic [FETCH_WIDTH-1:0] rob_alloc_valid;
    rob_entry_t [FETCH_WIDTH-1:0] rob_alloc_entry;
    logic [ROB_IDX_WIDTH-1:0] [FETCH_WIDTH-1:0] rob_alloc_idx;
    logic [ROB_IDX_WIDTH-1:0] rob_tail_bus;
    logic rob_full_bus;
    
    // Decode/Rename -> RS
    logic [FETCH_WIDTH-1:0] rs_alloc_valid;
    rs_entry_t [FETCH_WIDTH-1:0] rs_alloc_entry;
    logic rs_full_bus;
    
    // Decode/Rename -> LSQ
    logic [FETCH_WIDTH-1:0] lsq_alloc_valid;
    rs_entry_t [FETCH_WIDTH-1:0] lsq_alloc_entry; // (Usa rs_entry_t)

    // Decode/Rename <-> RAT
    logic [ARCH_REG_WIDTH-1:0] [FETCH_WIDTH-1:0][1:0] rat_gpr_read_addr;
    logic [GPR_TAG_WIDTH-1:0] [FETCH_WIDTH-1:0][1:0] rat_gpr_read_tag;
    logic [FETCH_WIDTH-1:0][1:0] rat_gpr_read_ready;
    logic [FETCH_WIDTH-1:0] rat_gpr_write_valid;
    logic [ARCH_REG_WIDTH-1:0] [FETCH_WIDTH-1:0] rat_gpr_write_addr;
    logic [GPR_TAG_WIDTH-1:0] [FETCH_WIDTH-1:0] rat_gpr_write_tag;
    
    logic [ARCH_REG_WIDTH-1:0] [FETCH_WIDTH-1:0][1:0] rat_fpr_read_addr;
    logic [FPR_TAG_WIDTH-1:0] [FETCH_WIDTH-1:0][1:0] rat_fpr_read_tag;
    logic [FETCH_WIDTH-1:0][1:0] rat_fpr_read_ready;
    logic [FETCH_WIDTH-1:0] rat_fpr_write_valid;
    logic [ARCH_REG_WIDTH-1:0] [FETCH_WIDTH-1:0] rat_fpr_write_addr;
    logic [FPR_TAG_WIDTH-1:0] [FETCH_WIDTH-1:0] rat_fpr_write_tag;

    // Decode/Rename <-> FreeList
    logic [GPR_TAG_WIDTH-1:0] [FETCH_WIDTH-1:0] prf_gpr_alloc_tag;
    logic prf_gpr_alloc_ready;
    logic [FPR_TAG_WIDTH-1:0] [FETCH_WIDTH-1:0] prf_fpr_alloc_tag;
    logic prf_fpr_alloc_ready;
    
    // Execute/LSQ -> Commit/WB (Common Data Bus - CDB)
    // (Agregado - até COMMIT_WIDTH resultados)
    logic [COMMIT_WIDTH-1:0] cdb_valid;
    logic [GPR_TAG_WIDTH-1:0] [COMMIT_WIDTH-1:0] cdb_gpr_tag;
    logic [REG_WIDTH-1:0] [COMMIT_WIDTH-1:0]    cdb_gpr_data;
    logic [FPR_TAG_WIDTH-1:0] [COMMIT_WIDTH-1:0] cdb_fpr_tag;
    logic [REG_WIDTH-1:0] [COMMIT_WIDTH-1:0]    cdb_fpr_data;

    // Execute/LSQ -> ROB (Writeback)
    logic [COMMIT_WIDTH-1:0] rob_wb_valid;
    logic [ROB_IDX_WIDTH-1:0] [COMMIT_WIDTH-1:0] rob_wb_idx;
    logic [REG_WIDTH-1:0] [COMMIT_WIDTH-1:0] rob_wb_data;
    logic [COMMIT_WIDTH-1:0] rob_wb_exception;
    logic [ADDR_WIDTH-1:0] [COMMIT_WIDTH-1:0] rob_wb_trap_cause;

    // Commit -> ARF
    logic [COMMIT_WIDTH-1:0] arf_gpr_write_valid;
    logic [ARCH_REG_WIDTH-1:0] [COMMIT_WIDTH-1:0] arf_gpr_write_addr;
    logic [REG_WIDTH-1:0] [COMMIT_WIDTH-1:0] arf_gpr_write_data;
    logic [COMMIT_WIDTH-1:0] arf_fpr_write_valid;
    logic [ARCH_REG_WIDTH-1:0] [COMMIT_WIDTH-1:0] arf_fpr_write_addr;
    logic [REG_WIDTH-1:0] [COMMIT_WIDTH-1:0] arf_fpr_write_data;

    // Commit -> FreeList
    logic [COMMIT_WIDTH-1:0] prf_gpr_free_valid;
    logic [GPR_TAG_WIDTH-1:0] [COMMIT_WIDTH-1:0] prf_gpr_free_tag;
    logic [COMMIT_WIDTH-1:0] prf_fpr_free_valid;
    logic [FPR_TAG_WIDTH-1:0] [COMMIT_WIDTH-1:0] prf_fpr_free_tag;

    // LSQ <-> dTLB
    logic dtlb_lookup_valid;
    logic [VPN_WIDTH-1:0] dtlb_lookup_vpn;
    logic dtlb_hit;
    logic [PPN_WIDTH-1:0] dtlb_ppn;
    logic dtlb_page_fault;

    // Commit <-> LSQ
    logic commit_store_valid;
    logic [ROB_IDX_WIDTH-1:0] commit_store_rob_idx;
    
    // MMU (PTW)
    logic ptw_req_valid;
    logic [VPN_WIDTH-1:0] ptw_req_vpn;
    logic ptw_req_ready;
    logic ptw_resp_valid;
    logic [PPN_WIDTH-1:0] ptw_resp_ppn;
    logic ptw_resp_page_fault;
    logic ptw_is_itlb_fill; 
    logic [VPN_WIDTH-1:0] ptw_resp_vpn;
    logic [15:0] ptw_resp_asid;


    // --------------------------
    // Instanciação dos Módulos da Pipeline
    // --------------------------

    // --- Estágios F1-F2: Fetch & Predict ---
    supernova_fetch #(
        .HART_ID(HART_ID),
        .BTB_ENTRIES(BTB_ENTRIES)
    ) u_fetch (
        .clk(clk), .rst_n(rst_n),
        .fetch_instr_out(fetch_instr_bus),
        .fetch_pc_out(fetch_pc_bus),
        .fetch_valid_out(fetch_valid_bus),
        .fetch_ready_in(fetch_ready_bus), 
        
        .imem_addr_va_out(imem_addr_pa_va), // VA para o iTLB
        .imem_req_out(imem_req),
        .imem_rdata_in(imem_rdata),
        .imem_ack_in(imem_ack_in && itlb_hit), // AND com hit do TLB
        .imem_error_in(imem_error_in || itlb_page_fault),
        
        .redirect_valid_in(global_flush), 
        .redirect_pc_in(global_flush_pc)
    );

    // --- Estágios D1-RN2: Decode & Rename ---
    supernova_decode_rename #(
        .HART_ID(HART_ID)
    ) u_decode_rename (
        .clk(clk), .rst_n(rst_n),
        .fetch_instr_in(fetch_instr_bus),
        .fetch_pc_in(fetch_pc_bus),
        .fetch_valid_in(fetch_valid_bus),
        .fetch_ready_out(fetch_ready_bus),
        
        .redirect_valid_in(global_flush),
        
        .rob_alloc_valid_out(rob_alloc_valid),
        .rob_alloc_entry_out(rob_alloc_entry),
        .rob_alloc_idx_out(rob_alloc_idx),
        .rob_tail_in(rob_tail_bus),
        .rob_full_in(rob_full_bus),
        
        .rs_alloc_valid_out(rs_alloc_valid),
        .rs_alloc_entry_out(rs_alloc_entry),
        .rs_full_in(rs_full_bus),
        
        .cdb_valid_in(cdb_valid),
        .cdb_gpr_tag_in(cdb_gpr_tag),
        .cdb_gpr_data_in(cdb_gpr_data),
        .cdb_fpr_tag_in(cdb_fpr_tag),
        .cdb_fpr_data_in(cdb_fpr_data),
        
        .rat_gpr_read_addr_out(rat_gpr_read_addr),
        .rat_gpr_read_tag_in(rat_gpr_read_tag),
        .rat_gpr_read_ready_in(rat_gpr_read_ready),
        .rat_gpr_write_valid_out(rat_gpr_write_valid),
        .rat_gpr_write_addr_out(rat_gpr_write_addr),
        .rat_gpr_write_tag_in(rat_gpr_write_tag),
        
        .rat_fpr_read_addr_out(rat_fpr_read_addr),
        .rat_fpr_read_tag_in(rat_fpr_read_tag),
        .rat_fpr_read_ready_in(rat_fpr_read_ready),
        .rat_fpr_write_valid_out(rat_fpr_write_valid),
        .rat_fpr_write_addr_out(rat_fpr_write_addr),
        .rat_fpr_write_tag_in(rat_fpr_write_tag),
        
        .prf_gpr_alloc_tag_in(prf_gpr_alloc_tag),
        .prf_gpr_alloc_ready_in(prf_gpr_alloc_ready),
        .prf_fpr_alloc_tag_in(prf_fpr_alloc_tag),
        .prf_fpr_alloc_ready_in(prf_fpr_alloc_ready)
    );

    // --- Estágios IS-EX: Issue & Execute ---
    supernova_issue_execute #(
        .HART_ID(HART_ID),
        .NUM_ALU_UNITS(NUM_ALU_UNITS),
        .NUM_FPU_UNITS(NUM_FPU_UNITS),
        .NUM_MDU_UNITS(NUM_MDU_UNITS)
    ) u_issue_execute (
        .clk(clk), .rst_n(rst_n),
        .redirect_valid_in(global_flush),
        
        .rs_alloc_valid_in(rs_alloc_valid),
        .rs_alloc_entry_in(rs_alloc_entry),
        .rs_full_out(rs_full_bus), 
        
        .cdb_valid_out(cdb_valid),
        .cdb_gpr_tag_out(cdb_gpr_tag),
        .cdb_gpr_data_out(cdb_gpr_data),
        // (portas CDB FPR)
        
        .rob_wb_valid_out(rob_wb_valid),
        .rob_wb_idx_out(rob_wb_idx),
        .rob_wb_data_out(rob_wb_data),
        .rob_wb_exception_out(rob_wb_exception),
        .rob_wb_trap_cause_out(rob_wb_trap_cause),
        
        // (O Rename decide se vai para a RS ou LSQ)
        .lsq_alloc_valid_out(lsq_alloc_valid),
        .lsq_alloc_entry_out(lsq_alloc_entry)
    );
    
    // --- Estágos M1-M2: Load/Store Queue (LSQ) ---
    supernova_lsq #(
        .HART_ID(HART_ID),
        .LSQ_ENTRIES(LSQ_ENTRIES)
    ) u_lsq (
        .clk(clk), .rst_n(rst_n),
        .redirect_valid_in(global_flush),
        
        .lsq_alloc_valid_in(lsq_alloc_valid), 
        .lsq_alloc_entry_in(lsq_alloc_entry),
        
        .cdb_valid_in(cdb_valid),
        .cdb_gpr_tag_in(cdb_gpr_tag),
        .cdb_gpr_data_in(cdb_gpr_data),
        
        .dtlb_lookup_valid_out(dtlb_lookup_valid),
        .dtlb_lookup_vpn_out(dtlb_lookup_vpn),
        .dtlb_hit_in(dtlb_hit),
        .dtlb_ppn_in(dtlb_ppn),
        .dtlb_page_fault_in(dtlb_page_fault),
        
        .dcache_addr_pa_out(dmem_addr_pa),
        .dcache_wdata_out(dmem_wdata),
        .dcache_wstrb_out(dmem_wstrb),
        .dcache_req_out(dmem_req),
        .dcache_we_out(dmem_we),
        .dcache_rdata_in(dmem_rdata),
        .dcache_ack_in(dmem_ack_in),
        
        // (Writeback para ROB/CDB - mesclado com Issue/Execute)
        .rob_wb_idx_out(rob_wb_idx_lsq), // (Precisa de um Mux)
        .rob_wb_data_out(rob_wb_data_lsq),
        .rob_wb_gpr_tag_out(rob_wb_gpr_tag_lsq),
        .rob_wb_fpr_tag_out(rob_wb_fpr_tag_lsq),
        .rob_wb_valid_out(rob_wb_valid_lsq),
        .rob_wb_exception_out(rob_wb_exception_lsq),
        .rob_wb_trap_cause_out(rob_wb_trap_cause_lsq),
        
        .commit_store_valid_in(commit_store_valid),
        .commit_store_rob_idx_in(commit_store_rob_idx),

        // --- Saídas para a STU ---
        .core_mem_pa_out(core_mem_pa_out),
        .core_mem_is_store_out(core_mem_is_store_out),
        .core_mem_valid_out(core_mem_valid_out)
    );

    // --- Estágio C1: Commit (ROB) ---
    supernova_commit #(
        .HART_ID(HART_ID)
    ) u_commit (
        .clk(clk), .rst_n(rst_n),
        
        .rob_alloc_valid_in(rob_alloc_valid),
        .rob_alloc_entry_in(rob_alloc_entry),
        .rob_alloc_idx_in(rob_alloc_idx),
        .rob_tail_out(rob_tail_bus),
        .rob_full_out(rob_full_bus),
        
        // (Mux de Writeback do Issue e LSQ)
        .rob_wb_valid_in(rob_wb_valid), 
        .rob_wb_idx_in(rob_wb_idx),
        .rob_wb_data_in(rob_wb_data),
        .rob_wb_exception_in(rob_wb_exception),
        .rob_wb_trap_cause_in(rob_wb_trap_cause),
        
        .redirect_valid_out(redirect_valid), 
        .redirect_pc_out(redirect_pc),
        
        .arf_write_valid_out(arf_gpr_write_valid),
        .arf_write_addr_out(arf_gpr_write_addr),
        .arf_write_data_out(arf_gpr_write_data),
        .arf_fpr_write_valid_out(arf_fpr_write_valid),
        .arf_fpr_write_addr_out(arf_fpr_write_addr),
        .arf_fpr_write_data_out(arf_fpr_write_data),
        
        .prf_free_valid_out(prf_gpr_free_valid),
        .prf_free_tag_out(prf_gpr_free_tag),
        .prf_fpr_free_valid_out(prf_fpr_free_valid),
        .prf_fpr_free_tag_out(prf_fpr_free_tag),
        
        .commit_store_valid_out(commit_store_valid),
        .commit_store_rob_idx_out(commit_store_rob_idx),

        .squash_in(squash_in),
        .commit_in(commit_in),
        .l2_spec_core_id_in(l2_spec_core_id_in),
        .is_speculating_l2_in(is_speculating_l2)
    );

    // --------------------------
    // Instanciação das Estruturas de Dados
    // --------------------------

    // --- RAT (Register Alias Table) ---
    supernova_rat #(
        .HART_ID(HART_ID)
    ) u_rat (
        .clk(clk), .rst_n(rst_n),
        .redirect_valid_in(global_flush),
        
        .rat_gpr_read_addr_in(rat_gpr_read_addr),
        .rat_gpr_read_tag_out(rat_gpr_read_tag),
        .rat_gpr_read_ready_out(rat_gpr_read_ready),
        .rat_gpr_write_valid_in(rat_gpr_write_valid),
        .rat_gpr_write_addr_in(rat_gpr_write_addr),
        .rat_gpr_write_tag_in(rat_gpr_write_tag),
        
        .rat_fpr_read_addr_in(rat_fpr_read_addr),
        .rat_fpr_read_tag_out(rat_fpr_read_tag),
        .rat_fpr_read_ready_out(rat_fpr_read_ready),
        .rat_fpr_write_valid_in(rat_fpr_write_valid),
        .rat_fpr_write_addr_in(rat_fpr_write_addr),
        .rat_fpr_write_tag_in(rat_fpr_write_tag),
        
        .arf_write_valid_in(arf_gpr_write_valid),
        .arf_write_addr_in(arf_gpr_write_addr),
        .arf_gpr_write_tag_in(rob[arf_gpr_write_addr[0]].rd_phys_old), // (Simplificado)
        .arf_fpr_write_tag_in(/* ... */)
    );

    // --- PRF Free List (Lista de Livres) ---
    supernova_free_list #(
        .HART_ID(HART_ID)
    ) u_free_list (
        .clk(clk), .rst_n(rst_n),
        .redirect_valid_in(global_flush),
        
        .prf_gpr_alloc_tag_out(prf_gpr_alloc_tag),
        .prf_gpr_alloc_ready_out(prf_gpr_alloc_ready),
        .prf_fpr_alloc_tag_out(prf_fpr_alloc_tag),
        .prf_fpr_alloc_ready_out(prf_fpr_alloc_ready),
        
        .prf_gpr_free_valid_in(prf_gpr_free_valid),
        .prf_gpr_free_tag_in(prf_gpr_free_tag),
        .prf_fpr_free_valid_in(prf_fpr_free_valid),
        .prf_fpr_free_tag_in(prf_fpr_free_tag)
    );
    
    // --- PRF (Physical Register File) ---
    supernova_prf #(
        .HART_ID(HART_ID)
    ) u_prf (
        .clk(clk), .rst_n(rst_n)
        // (Conecte as portas de leitura do Issue e escrita do CDB)
    );

    // --- ARF (Architectural Register File) ---
    supernova_arf #(
        .HART_ID(HART_ID)
    ) u_arf (
        .clk(clk), .rst_n(rst_n),
        
        .arf_gpr_write_valid_in(arf_gpr_write_valid),
        .arf_gpr_write_addr_in(arf_gpr_write_addr),
        .arf_gpr_write_data_in(arf_gpr_write_data),
        .arf_fpr_write_valid_in(arf_fpr_write_valid),
        .arf_fpr_write_addr_in(arf_fpr_write_addr),
        .arf_fpr_write_data_in(arf_fpr_write_data),
        
        .stu_gpr_read_addr_in(core_copy_read_addr_in),
        .stu_gpr_read_data_out(core_copy_data_out),
        .stu_gpr_write_valid_in(core_copy_write_en_in && (MY_HART_ID == l2_spec_core_id_in)),
        .stu_gpr_write_addr_in(core_copy_write_addr_in),
        .stu_gpr_write_data_in(core_copy_data_in)
    );
    
    // --- MMU (TLB/PTW) ---
    logic [XLEN-1:0] csr_satp; // (Deve vir do banco de CSRs)
    
    tlb_associative #( .VPN_WIDTH(VPN_WIDTH), .PPN_WIDTH(PPN_WIDTH), .TLB_ENTRIES(TLB_ENTRIES) )
    u_itlb (
        .clk(clk), .rst_n(rst_n),
        .lookup_valid(imem_req),
        .lookup_vpn(imem_addr_pa_va[VPN_WIDTH+11:12]), // (Precisa do VA do Fetch)
        .lookup_asid(csr_satp[59:44]),
        .lookup_hit(itlb_hit),
        .lookup_ppn(itlb_ppn),
        .lookup_page_fault(itlb_page_fault),
        
        .insert_valid(ptw_resp_valid && !ptw_resp_page_fault && ptw_is_itlb_fill),
        .insert_vpn(ptw_resp_vpn),
        .insert_ppn(ptw_resp_ppn),
        .insert_asid(ptw_resp_asid)
    );
    
    tlb_associative #( .VPN_WIDTH(VPN_WIDTH), .PPN_WIDTH(PPN_WIDTH), .TLB_ENTRIES(TLB_ENTRIES) )
    u_dtlb (
        .clk(clk), .rst_n(rst_n),
        .lookup_valid(dtlb_lookup_valid),
        .lookup_vpn(dtlb_lookup_vpn),
        .lookup_asid(csr_satp[59:44]),
        .lookup_hit(dtlb_hit),
        .lookup_ppn(dtlb_ppn),
        .lookup_page_fault(dtlb_page_fault),

        .insert_valid(ptw_resp_valid && !ptw_resp_page_fault && !ptw_is_itlb_fill),
        .insert_vpn(ptw_resp_vpn),
        .insert_ppn(ptw_resp_ppn),
        .insert_asid(ptw_resp_asid)
    );

    ptw_sv39_full #( .XLEN(XLEN), .PADDR_WIDTH(PHYS_ADDR_SIZE), .PPN_WIDTH(PPN_WIDTH) )
    u_ptw (
        .clk(clk), .rst_n(rst_n),
        .ptw_req_valid(ptw_req_valid),
        .ptw_req_vpn(ptw_req_vpn),
        .ptw_req_asid(csr_satp[59:44]),
        .ptw_req_ready(ptw_req_ready),
        .satp(csr_satp),

        .ptw_mem_req(ptw_mem_req),
        .ptw_mem_addr(ptw_mem_addr),
        .ptw_mem_resp_valid(ptw_mem_ack_in),
        .ptw_mem_resp_data(ptw_mem_rdata),
        .ptw_mem_resp_err(ptw_mem_error_in),

        .ptw_resp_valid(ptw_resp_valid),
        .ptw_resp_ppn(ptw_resp_ppn),
        .ptw_resp_page_fault(ptw_resp_page_fault),
        .ptw_resp_vpn(ptw_resp_vpn),
        .ptw_resp_asid(ptw_resp_asid)
    );

    // --------------------------
    // Lógica Sequencial (STU e Reset)
    // --------------------------

    // Lógica de Flush Global
    assign global_flush = redirect_valid || 
                          (is_speculating_l2 && squash_in[MY_HART_ID]) ||
                          (l2_spec_start_in && (MY_HART_ID == l2_spec_core_id_in));
                          
    assign global_flush_pc = (l2_spec_start_in && (MY_HART_ID == l2_spec_core_id_in)) ? l2_spec_pc_in :
                             (is_speculating_l2 && squash_in[MY_HART_ID]) ? shadow_pc :
                             redirect_pc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_speculating_l2 <= 1'b0;
            pc <= PC_RESET_VEC;
        end else begin
        
            // --- LÓGICA STU (SEQUENCIAL) ---
            if (l2_spec_start_in && (MY_HART_ID == l2_spec_core_id_in)) begin
                is_speculating_l2 <= 1'b1;
                // CHECKPOINT: Salve o estado ARQUITETURAL
                shadow_pc <= pc;
                // (O ARF/CSRs são salvos por seus próprios módulos)
            end

            if (is_speculating_l2) begin
                if (squash_in[MY_HART_ID]) begin
                    is_speculating_l2 <= 1'b0;
                    pc <= shadow_pc; // Restaura o PC arquitetural
                end
                else if (commit_in[MY_HART_ID]) begin
                    is_speculating_l2 <= 1'b0;
                end
            end
            
            // Atualiza o PC Arquitetural (somente no commit)
            if (arf_gpr_write_valid[0] /* && é um branch comitado */) begin
                 // pc <= ... (Lógica de commit de PC)
            end
        end
    end

    // --------------------------
    // Saídas STU
    // --------------------------
    
    assign core_busy_out = (rob_head != rob_tail_bus);
    assign spec_task_done_out = 1'b0; // TODO: Lógica de detecção de fim de tarefa
    assign spec_exception_out = (rob_wb_exception[0] && is_speculating_l2); // (Simplificado)

endmodule
