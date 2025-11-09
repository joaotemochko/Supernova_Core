`ifndef SUPERNOVA_PKG_SVH
`define SUPERNOVA_PKG_SVH

// Importa os tipos da STU (Speculative Threading Unit)
`include "stu_pkg.sv"

/**
 * @package supernova_pkg
 * @brief Define os tipos de dados e parâmetros da microarquitetura
 * interna do Supernova Core (OoO).
 */
package supernova_pkg;

  // --- Parâmetros da Microarquitetura OoO ---
  
  // Largura do Pipeline (4-wide superscalar)
  parameter int FETCH_WIDTH = 4;
  parameter int DECODE_WIDTH = 4;
  parameter int RENAME_WIDTH = 4;
  parameter int ISSUE_WIDTH = 4;
  parameter int COMMIT_WIDTH = 4;

  // Tamanho das Estruturas OoO
  parameter int ROB_ENTRIES = 128; // 128 entradas no Reorder Buffer
  parameter int RS_ENTRIES = 32;   // 32 entradas na Reservation Station (unificada)
  
  // Arquivos de Registradores Físicos (PRF)
  // (O RV64G tem 32 GPRs + 32 FPRs arquiteturais)
  parameter int NUM_ARCH_GPRS = 32;
  parameter int NUM_PHYS_GPRS = 128; // Ex: 128 registradores físicos GPR
  parameter int NUM_PHYS_FPRS = 128; // Ex: 128 registradores físicos FPR
  
  // Largura dos "Tags" (ponteiros)
  parameter int ROB_IDX_WIDTH = $clog2(ROB_ENTRIES);
  parameter int RS_IDX_WIDTH = $clog2(RS_ENTRIES);
  parameter int GPR_TAG_WIDTH = $clog2(NUM_PHYS_GPRS);
  parameter int FPR_TAG_WIDTH = $clog2(NUM_PHYS_FPRS);
  parameter int ARCH_REG_WIDTH = 5; // 2^5 = 32


  // --- Estruturas de Dados OoO ---

  /**
   * @brief Entrada da Reservation Station (RS)
   * Armazena uma instrução aguardando seus operandos.
   */
  typedef struct packed {
    logic busy; // 1 = Esta entrada RS está em uso

    stu_pkg::instr_t  instr; // A instrução
    stu_pkg::addr_t   pc;
    
    // Ponteiro para o ROB
    logic [ROB_IDX_WIDTH-1:0] rob_idx; 
    
    // Operando Fonte 1 (Src1)
    logic             src1_ready;
    logic [GPR_TAG_WIDTH-1:0] src1_tag;
    stu_pkg::reg_width_t    src1_data;
    
    // Operando Fonte 2 (Src2)
    logic             src2_ready;
    logic [GPR_TAG_WIDTH-1:0] src2_tag;
    stu_pkg::reg_width_t    src2_data;
    
    // Destino (Físico)
    logic [GPR_TAG_WIDTH-1:0] rd_phys_tag;
    
    // (Adicionar campos para FPU, LSU, etc.)
    
  } rs_entry_t;


  /**
   * @brief Entrada do Reorder Buffer (ROB)
   * Rastreia uma instrução "em voo" para garantir o commit em ordem.
   */
  typedef struct packed {
    logic valid;     // 1 = Esta entrada ROB está em uso
    logic done;      // 1 = Execução concluída
    
    stu_pkg::addr_t   pc;
    stu_pkg::instr_t  instr;
    
    // Registrador de Destino Arquitetural (o "nome" x0-x31)
    logic [ARCH_REG_WIDTH-1:0] rd_arch;
    
    // Registrador Físico Anterior (para restaurar no flush)
    logic [GPR_TAG_WIDTH-1:0] rd_phys_old; 
    
    // Resultado da Execução (ou valor do Store)
    stu_pkg::reg_width_t result_data; 
    
    // Estado de Exceção
    logic             has_exception;
    stu_pkg::addr_t   trap_cause;
    
    // Informações de Controle
    logic             is_branch;
    logic             is_store;
    logic             is_load;
    
  } rob_entry_t;

endpackage : supernova_pkg

`endif
