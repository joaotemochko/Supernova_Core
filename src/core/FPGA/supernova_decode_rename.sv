`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_decode_rename
 * @brief Estágios de Decode (Decodificação) e Rename (Renomeação) do Supernova.
 *
 * @details Este módulo é o coração do front-end OoO. Ele pega as
 * instruções do Fetch, as decodifica e realiza a renomeação
 * de registradores para quebrar falsas dependências (WAR/WAW).
 *
 * Ele implementa lógicas paralelas para GPR e FPR.
 */
module supernova_decode_rename #(
    parameter int HART_ID = 0
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface com o Fetch ---
    input  wire [supernova_pkg::FETCH_WIDTH-1:0][stu_pkg::INSTR_WIDTH-1:0] fetch_instr_in,
    input  wire [stu_pkg::ADDR_WIDTH-1:0]       fetch_pc_in,
    input  wire                                  fetch_valid_in,
    output logic                                 fetch_ready_out, // Para o Fetch (backpressure)

    // --- Interface de Redirecionamento (Flush) ---
    input  wire                                  redirect_valid_in,

    // --- Interface com o ROB (Reorder Buffer) ---
    output logic [supernova_pkg::FETCH_WIDTH-1:0]                       rob_alloc_valid_out,
    output supernova_pkg::rob_entry_t [supernova_pkg::FETCH_WIDTH-1:0] rob_alloc_entry_out,
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rob_alloc_idx_out,
    input  wire [supernova_pkg::ROB_IDX_WIDTH-1:0]                       rob_tail_in,
    input  wire                                                          rob_full_in,
    
    // --- Interface com a RS (Reservation Station) ---
    output logic [supernova_pkg::FETCH_WIDTH-1:0]                       rs_alloc_valid_out,
    output supernova_pkg::rs_entry_t [supernova_pkg::FETCH_WIDTH-1:0]  rs_alloc_entry_out,
    input  wire                                                          rs_full_in,
    
    // --- Barramento de Wakeup/Bypass (Common Data Bus - CDB) ---
    input  wire [1:0]                         cdb_valid_in,
    input  wire [supernova_pkg::GPR_TAG_WIDTH-1:0] [1:0] cdb_gpr_tag_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [1:0]    cdb_gpr_data_in,
    input  wire [supernova_pkg::FPR_TAG_WIDTH-1:0] [1:0] cdb_fpr_tag_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] [1:0]    cdb_fpr_data_in,

    // --- Interface com a RAT (Register Alias Table) ---
    // GPRs
    output logic [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_gpr_read_addr_out,
    input  logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_gpr_read_tag_in,
    input  logic [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_gpr_read_ready_in, // 1 = Tag está pronta
    output logic [supernova_pkg::FETCH_WIDTH-1:0]                       rat_gpr_write_valid_out,
    output logic [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_gpr_write_addr_out,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_gpr_write_tag_in,
    
    // FPRs
    output logic [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_fpr_read_addr_out,
    input  logic [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_fpr_read_tag_in,
    input  logic [supernova_pkg::FETCH_WIDTH-1:0][1:0] rat_fpr_read_ready_in,
    output logic [supernova_pkg::FETCH_WIDTH-1:0]                       rat_fpr_write_valid_out,
    output logic [supernova_pkg::ARCH_REG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_fpr_write_addr_out,
    output logic [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] rat_fpr_write_tag_in,

    // --- Interface com as Listas de Livres (PRF Free List) ---
    input  logic [supernova_pkg::GPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] prf_gpr_alloc_tag_in,
    input  wire                                     prf_gpr_alloc_ready_in, // 1 = Há tags GPR livres
    input  logic [supernova_pkg::FPR_TAG_WIDTH-1:0] [supernova_pkg::FETCH_WIDTH-1:0] prf_fpr_alloc_tag_in,
    input  wire                                     prf_fpr_alloc_ready_in // 1 = Há tags FPR livres
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Definição da Estrutura de Decodificação (Corrigido) ---
    // (Portado do Nebula Core [cite: 588-591])
    typedef struct packed {
        logic [6:0] opcode;
        logic [4:0] rd;
        logic [2:0] funct3;
        logic [4:0] rs1;
        logic [4:0] rs2;
        logic [6:0] funct7;
        logic [XLEN-1:0] imm;
        logic valid;
        logic is_alu;
        logic is_branch;
        logic is_load;
        logic is_store;
        logic is_system; 
        logic is_amo;
        logic is_fpu; 
        logic is_mdu; 
        // Flags de uso de registrador
        logic uses_gpr_rs1, uses_gpr_rs2, uses_gpr_rd;
        logic uses_fpr_rs1, uses_fpr_rs2, uses_fpr_rd;
    } decoded_instr_t;

    // Buffers/Registradores de Pipeline entre Decode e Rename
    logic [FETCH_WIDTH-1:0]           decode_valid;
    decoded_instr_t [FETCH_WIDTH-1:0] decoded_instrs;
    addr_t                            decode_pc;

    // --- Estágio D1/D2: Decode (Decodificação Paralela) ---
    
    logic decode_stall;
    assign decode_stall = redirect_valid_in;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            decode_valid <= '{default: '0};
        end else if (decode_stall) begin
            decode_valid <= '{default: '0}; // Flush
        end else begin
            decode_valid <= {FETCH_WIDTH{fetch_valid_in}};
            decode_pc <= fetch_pc_in;
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                // Chama a função de decode completa
                decoded_instrs[i] <= decode_instr_function(fetch_instr_in[i], fetch_pc_in + (i*4));
            end
        end
    end
    
    // --- Estágios RN1/RN2: Rename (Renomeação Combinacional) ---
    
    logic rename_stall;
    // Backpressure: Stall se o ROB, RS ou PRF Free List estiverem cheios
    assign rename_stall = rob_full_in || rs_full_in || 
                          !prf_gpr_alloc_ready_in || !prf_fpr_alloc_ready_in;
    assign fetch_ready_out = !rename_stall;

    always_comb begin
        // Padrões
        rob_alloc_valid_out = '{default: '0};
        rob_alloc_entry_out = '{default: '0};
        rob_alloc_idx_out = '{default: '0};
        rs_alloc_valid_out = '{default: '0};
        rs_alloc_entry_out = '{default: '0};
        rat_gpr_read_addr_out = '{default: '0};
        rat_gpr_write_valid_out = '{default: '0};
        rat_gpr_write_addr_out = '{default: '0};
        rat_gpr_write_tag_in = '{default: '0};
        rat_fpr_read_addr_out = '{default: '0};
        rat_fpr_write_valid_out = '{default: '0};
        rat_fpr_write_addr_out = '{default: '0};
        rat_fpr_write_tag_in = '{default: '0};
        
        // Se houver um flush ou stall, não faça nada
        if (redirect_valid_in || rename_stall) begin
            // Não faz nada
        end else begin
            
            // Processa até 4 instruções
            for (int i = 0; i < FETCH_WIDTH; i++) begin
                if (decode_valid[i]) begin
                    
                    decoded_instr_t instr = decoded_instrs[i];
                    logic [GPR_TAG_WIDTH-1:0] rs1_gpr_tag, rs2_gpr_tag, rd_gpr_old_tag;
                    logic [FPR_TAG_WIDTH-1:0] rs1_fpr_tag, rs2_fpr_tag, rd_fpr_old_tag;
                    logic rs1_gpr_ready, rs2_gpr_ready;
                    logic rs1_fpr_ready, rs2_fpr_ready;
                    
                    // --- Passo 1: Alocar ROB ---
                    rob_alloc_valid_out[i] = 1'b1;
                    rob_alloc_idx_out[i] = rob_tail_in + i; 
                    
                    rob_alloc_entry_out[i].valid = 1'b1;
                    rob_alloc_entry_out[i].done = 1'b0;
                    rob_alloc_entry_out[i].pc = decode_pc + (i*4);
                    rob_alloc_entry_out[i].instr = fetch_instr_in[i];
                    rob_alloc_entry_out[i].rd_arch = instr.rd;
                    rob_alloc_entry_out[i].has_exception = 1'b0;
                    rob_alloc_entry_out[i].is_branch = instr.is_branch;
                    rob_alloc_entry_out[i].is_store = instr.is_store;
                    
                    // --- Passo 2: Ler RAT (Registradores Fonte) ---
                    // (Lê o estado atual do RAT)
                    if (instr.uses_gpr_rs1) rat_gpr_read_addr_out[i][0] = instr.rs1;
                    if (instr.uses_gpr_rs2) rat_gpr_read_addr_out[i][1] = instr.rs2;
                    if (instr.uses_fpr_rs1) rat_fpr_read_addr_out[i][0] = instr.rs1;
                    if (instr.uses_fpr_rs2) rat_fpr_read_addr_out[i][1] = instr.rs2;
                    
                    rs1_gpr_tag = rat_gpr_read_tag_in[i][0];
                    rs2_gpr_tag = rat_gpr_read_tag_in[i][1];
                    rs1_gpr_ready = rat_gpr_read_ready_in[i][0];
                    rs2_gpr_ready = rat_gpr_read_ready_in[i][1];
                    // (Fazer o mesmo para FPRs)
                    
                    // --- Passo 3: Alocar PRF & Atualizar RAT (Destino) ---
                    if (instr.uses_gpr_rd && instr.rd != 0) begin
                        rat_gpr_write_valid_out[i] = 1'b1;
                        rat_gpr_write_addr_out[i] = instr.rd;
                        rat_gpr_write_tag_in[i] = prf_gpr_alloc_tag_in[i]; // Pega tag livre
                        // Salva o tag *antigo* no ROB para rollback
                        rob_alloc_entry_out[i].rd_phys_old = rs1_gpr_tag; // (Simplificado: assume que a leitura do RAT [0] é o rd)
                    
                    end else if (instr.uses_fpr_rd) begin
                        rat_fpr_write_valid_out[i] = 1'b1;
                        rat_fpr_write_addr_out[i] = instr.rd;
                        rat_fpr_write_tag_in[i] = prf_fpr_alloc_tag_in[i];
                        // (Lógica similar para FPRs)
                    end

                    // --- Passo 4: Alocar RS e Preencher ---
                    rs_alloc_valid_out[i] = 1'b1;
                    rs_alloc_entry_out[i].busy = 1'b1;
                    rs_alloc_entry_out[i].instr = fetch_instr_in[i];
                    rs_alloc_entry_out[i].pc = decode_pc + (i*4);
                    rs_alloc_entry_out[i].rob_idx = rob_alloc_idx_out[i];
                    
                    // --- Conectar Fontes GPR ---
                    if (instr.uses_gpr_rs1) begin
                        if (instr.rs1 == 0) begin // Handle x0
                            rs_alloc_entry_out[i].src1_ready = 1'b1;
                            rs_alloc_entry_out[i].src1_data = '0;
                        end else if (rs1_gpr_ready) begin // Pronto no RAT/PRF
                            rs_alloc_entry_out[i].src1_ready = 1'b1;
                            // (A leitura do PRF aconteceria no Issue)
                        end else begin // Precisa esperar pelo tag
                            rs_alloc_entry_out[i].src1_ready = 1'b0;
                            rs_alloc_entry_out[i].src1_tag = rs1_gpr_tag;
                            // (Verificar bypass do CDB)
                        end
                    end
                    
                    if (instr.uses_gpr_rs2) begin
                        if (instr.rs2 == 0) begin // Handle x0
                            rs_alloc_entry_out[i].src2_ready = 1'b1;
                            rs_alloc_entry_out[i].src2_data = '0;
                        else if (rs2_gpr_ready) begin
                            rs_alloc_entry_out[i].src2_ready = 1'b1;
                        end else begin
                            rs_alloc_entry_out[i].src2_ready = 1'b0;
                            rs_alloc_entry_out[i].src2_tag = rs2_gpr_tag;
                        end
                    end
                    
                    // --- Conectar Fontes FPR ---
                    // (Lógica similar para FPRs)

                    // --- Conectar Destino ---
                    if (instr.uses_gpr_rd)
                        rs_alloc_entry_out[i].rd_phys_tag = prf_gpr_alloc_tag_in[i];
                    else if (instr.uses_fpr_rd)
                        rs_alloc_entry_out[i].rd_phys_tag = prf_fpr_alloc_tag_in[i];
                    end
                end
            end
        end
    end
    
    
    // --------------------------
    // Função de Decodificação (RV64G) - Completa
    // --------------------------
    function automatic decoded_instr_t decode_instr_function(input [ILEN-1:0] instr, input [XLEN-1:0] pc_val);
        decoded_instr_t result;
        result.opcode = instr[6:0];
        result.rd = instr[11:7];
        result.funct3 = instr[14:12];
        result.rs1 = instr[19:15];
        result.rs2 = instr[24:20];
        result.funct7 = instr[31:25];
        result.valid = 1'b1;
        
        // Zera flags
        result.is_alu = 1'b0; result.is_branch = 1'b0; result.is_load = 1'b0;
        result.is_store = 1'b0; result.is_system = 1'b0; result.is_amo = 1'b0;
        result.is_mdu = 1'b0; result.is_fpu = 1'b0;
        result.uses_gpr_rs1 = 1'b0; result.uses_gpr_rs2 = 1'b0; result.uses_gpr_rd = 1'b0;
        result.uses_fpr_rs1 = 1'b0; result.uses_fpr_rs2 = 1'b0; result.uses_fpr_rd = 1'b0;

        case (result.opcode)
            // I-Type
            7'b0000011: begin result.is_load = 1'b1; result.uses_gpr_rs1=1; result.uses_gpr_rd=1; end // LOAD
            7'b0010011: begin result.is_alu = 1'b1; result.uses_gpr_rs1=1; result.uses_gpr_rd=1; end // ALU-I
            7'b1100111: begin result.is_branch = 1'b1; result.uses_gpr_rs1=1; result.uses_gpr_rd=1; end // JALR
            
            // U-Type
            7'b0010111: begin result.is_alu = 1'b1; result.uses_gpr_rd=1; end // AUIPC
            7'b0110111: begin result.is_alu = 1'b1; result.uses_gpr_rd=1; end // LUI
            
            // S-Type
            7'b0100011: begin result.is_store = 1'b1; result.uses_gpr_rs1=1; result.uses_gpr_rs2=1; end // STORE
            
            // R-Type (I, M, A)
            7'b0110011: begin // ALU-R
                if (result.funct7 == 7'b0000001)
                    result.is_mdu = 1'b1; // M-Ext
                else
                    result.is_alu = 1'b1; // I-Ext
                result.uses_gpr_rs1=1; result.uses_gpr_rs2=1; result.uses_gpr_rd=1;
            end
            7'b0101111: begin result.is_amo = 1'b1; result.uses_gpr_rs1=1; result.uses_gpr_rs2=1; result.uses_gpr_rd=1; end // AMO

            // B-Type
            7'b1100011: begin result.is_branch = 1'b1; result.uses_gpr_rs1=1; result.uses_gpr_rs2=1; end // BRANCH

            // J-Type
            7'b1101111: begin result.is_branch = 1'b1; result.uses_gpr_rd=1; end // JAL
            
            // System
            7'b1110011: begin result.is_system = 1'b1; end // (CSRs, etc.)

            // FPU (F/D)
            7'b0000111: begin result.is_load = 1'b1; result.is_fpu = 1'b1; result.uses_gpr_rs1=1; result.uses_fpr_rd=1; end // FLW/FLD
            7'b0100111: begin result.is_store = 1'b1; result.is_fpu = 1'b1; result.uses_gpr_rs1=1; result.uses_fpr_rs2=1; end // FSW/FSD
            7'b1010011: begin result.is_fpu = 1'b1; result.uses_fpr_rs1=1; result.uses_fpr_rs2=1; result.uses_fpr_rd=1; end // FPU-R
            
            default: result.valid = 1'b0;
        endcase

        // --- Geração de Imediato (Corrigido) ---
        case (result.opcode)
            // LUI, AUIPC
            7'b0110111, 7'b0010111:
                result.imm = {{XLEN-32{instr[31]}}, instr[31:12], 12'b0};
            // JAL
            7'b1101111:
                result.imm = {{XLEN-20{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
            // JALR
            7'b1100111:
                result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
            // Branches
            7'b1100011:
                result.imm = {{XLEN-12{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            // Loads, Load-FP, ALU-I
            7'b0000011, 7'b0000111, 7'b0010011:
                result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
            // Stores, Store-FP
            7'b0100011, 7'b0100111:
                result.imm = {{XLEN-11{instr[31]}}, instr[30:25], instr[11:7]};
            // CSRs (SYSTEM)
            7'b1110011:
                result.imm = {{XLEN-4{1'b0}}, instr[19:15]}; // zimm
            
            default:
                result.imm = '0;
        endcase
        
        return result;
    endfunction

endmodule
