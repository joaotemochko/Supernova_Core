`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_fetch
 * @brief Estágio de Fetch (Busca) e Predição de Desvio do Supernova.
 *
 * @details Este módulo é o front-end do núcleo OoO. Ele é responsável
 * por buscar um bloco largo de instruções (FETCH_WIDTH) e
 * prever o próximo PC usando um BTB (Branch Target Buffer).
 *
 * Ele é projetado para lidar com sinais de 'flush' (descarga) de alta
 * prioridade vindos do back-end (mispredict) ou da STU (SQUASH / L2_START).
 */
module supernova_fetch #(
    parameter int HART_ID = 0,
    parameter int BTB_ENTRIES = 64, // 64 entradas no Branch Target Buffer
    parameter int BTB_ADDR_BITS = 6   // $clog2(64)
) (
    input  wire clk,
    input  wire rst_n,

    // --- Interface com o Pipeline (Estágio de Decode) ---
    output logic [supernova_pkg::FETCH_WIDTH-1:0][stu_pkg::INSTR_WIDTH-1:0] fetch_instr_out,
    output logic [stu_pkg::ADDR_WIDTH-1:0]       fetch_pc_out,
    output logic                                 fetch_valid_out,
    input  wire                                  fetch_ready_in, // 1 = Decode está pronto

    // --- Interface com a Memória de Instrução (iTLB/ICache) ---
    output logic [stu_pkg::ADDR_WIDTH-1:0] imem_addr_va_out, // VA para o iTLB
    output logic                           imem_req_out,
    input  wire [supernova_pkg::FETCH_WIDTH*stu_pkg::INSTR_WIDTH-1:0] imem_rdata_in,
    input  wire                            imem_ack_in,
    input  wire                            imem_error_in, // Ex: iTLB Page Fault

    // --- Interface de Redirecionamento (Flush) ---
    // (Vindo do Commit (mispredict) ou da STU (SQUASH/L2_START))
    input  wire                            redirect_valid_in,
    input  wire [stu_pkg::ADDR_WIDTH-1:0]  redirect_pc_in
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Registrador do Program Counter (PC) ---
    addr_t pc;
    addr_t pc_next;
    logic  pc_stall;
    logic  is_flushed; // 1 = Um flush está ocorrendo

    // --- Lógica de Predição de Desvio (BTB Simples) ---
    // 
    // (Uma implementação real teria tags de PC para evitar colisões)
    addr_t btb_table [0:BTB_ENTRIES-1]; // Armazena o *alvo* do pulo
    logic  btb_valid [0:BTB_ENTRIES-1];
    
    addr_t btb_index;
    addr_t btb_predicted_target;
    logic  btb_hit;

    assign btb_index = pc[BTB_ADDR_BITS+1:2];
    assign btb_predicted_target = btb_table[btb_index];
    assign btb_hit = btb_valid[btb_index] && !pc_stall;

    // --- Lógica de Controle do PC ---
    always_comb begin
        // Stall se o Decode não estiver pronto (backpressure)
        // ou se estivermos esperando a memória
        pc_stall = !fetch_ready_in || (imem_req_out && !imem_ack_in);
        
        // Padrão: Próxima linha de cache
        pc_next = pc + (FETCH_WIDTH * 4); 

        // 1. Predição de Desvio (BTB Hit)
        if (btb_hit) begin
            pc_next = btb_predicted_target;
        end
        
        // 2. Redirecionamento (STU ou Mispredict) - Prioridade Máxima
        if (redirect_valid_in) begin
            pc_next = redirect_pc_in;
        end
    end

    // --- Lógica de Fetch (Sequencial) ---
    logic [FETCH_WIDTH-1:0][ILEN-1:0] fetch_buffer;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= PC_RESET_VEC; // Endereço de Reset
            fetch_valid_out <= 1'b0;
            imem_req_out <= 1'b0;
            // Limpar BTB
            for (int i = 0; i < BTB_ENTRIES; i++) begin
                btb_valid[i] <= 1'b0;
            end
        end else begin
            
            // Verifica se um flush ocorreu
            is_flushed = redirect_valid_in;
            
            // --- Lógica de Flush (STU ou Mispredict) ---
            if (is_flushed) 
            begin
                fetch_valid_out <= 1'b0; // Invalida o buffer
                imem_req_out <= 1'b0;    // Cancela requisição
                pc <= pc_next;           // PC é atualizado para o alvo do redirect
            end
            // --- Lógica de Stall ---
            else if (pc_stall) begin
                // Mantém os sinais (imem_req_out, fetch_valid_out, pc)
                // O PC não avança.
            end
            // --- Lógica de Fetch Normal ---
            else begin
                pc <= pc_next; // Avança o PC (para o próximo bloco ou alvo do BTB)
                
                // Se o ACK da memória chegou, capture os dados
                if (imem_ack_in) begin
                    fetch_valid_out <= 1'b1;
                    // Extrai 4 instruções de 32 bits do barramento de 128 bits
                    for (int i = 0; i < FETCH_WIDTH; i++) begin
                        fetch_buffer[i] <= imem_rdata_in[ (i*ILEN) +: ILEN ];
                    end
                end else begin
                    // Se não houver ack, mas também não houver stall,
                    // estamos iniciando uma nova requisição
                    fetch_valid_out <= 1'b0;
                end
                
                // Faça a próxima requisição (ou mantenha a atual se stall)
                imem_req_out <= 1'b1;
                imem_addr_va_out <= pc; // Envia o PC atual para o iTLB
            end
        end
    end
    
    // Conecta o buffer de saída
    assign fetch_instr_out = fetch_buffer;
    assign fetch_pc_out = pc; // Envia o PC do bloco atual

    // TODO: Lógica de Atualização do BTB
    // (Precisa de uma interface de 'write' vinda do estágio de Commit
    // para atualizar a tabela quando um branch é resolvido)

endmodule
