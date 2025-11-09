`include "stu_pkg.sv"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module ptw_sv39_full
 * @brief Page Table Walker (PTW) para Sv39.
 * @details Implementa o 'page walk' de 3 níveis para Sv39
 * para preencher o TLB em caso de 'miss'.
 */
module ptw_sv39_full #(
    parameter XLEN = 64,
    parameter VPN_WIDTH = 39,
    parameter PADDR_WIDTH = 56, 
    parameter PPN_WIDTH = 44    
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // --- Interface de Requisição (do TLB) ---
    input  wire                 ptw_req_valid,
    input  wire [VPN_WIDTH-1:0] ptw_req_vpn,
    input  wire [15:0]          ptw_req_asid,
    output reg                  ptw_req_ready,

    // --- Entrada de CSRs (do Núcleo) ---
    input  wire [XLEN-1:0]      satp, 

    // --- Interface Mestre de Memória (para o D-Cache/Mem) ---
    output reg                  ptw_mem_req,
    output reg [PADDR_WIDTH-1:0] ptw_mem_addr,
    input  wire                 ptw_mem_resp_valid, // (Nosso dmem_ack_in)
    input  wire [63:0]          ptw_mem_resp_data,  // (Nosso dmem_rdata)
    input  wire                 ptw_mem_resp_err,   // (Nosso dmem_error_in)

    // --- Interface de Resposta (para o TLB) ---
    output reg                  ptw_resp_valid,
    output reg [PPN_WIDTH-1:0]  ptw_resp_ppn,
    output reg                  ptw_resp_page_fault,
    // (Saídas para preencher o TLB)
    output logic [VPN_WIDTH-1:0] ptw_resp_vpn,
    output logic [15:0]          ptw_resp_asid
);

    localparam LEVELS = 3;
    localparam PAGE_OFFSET_BITS = 12;
    localparam PTE_SIZE_BITS = 3; // 2^3 = 8 bytes

    // (Funções helper do seu original [cite: 432-441])
    function automatic [PPN_WIDTH-1:0] satp_ppn(input [XLEN-1:0] s); 
        return s[PPN_WIDTH-1:0]; 
    endfunction
    
    function automatic [PADDR_WIDTH-1:0] make_pte_addr(input [PPN_WIDTH-1:0] ppn, input [8:0] idx);
        return ({{(PADDR_WIDTH-PPN_WIDTH){1'b0}}, ppn} << PAGE_OFFSET_BITS) + (idx << PTE_SIZE_BITS);
    endfunction

    function automatic pte_v(input [63:0] p) ; pte_v = p[0]; endfunction
    function automatic pte_r(input [63:0] p) ; pte_r = p[1]; endfunction
    function automatic pte_w(input [63:0] p) ; pte_w = p[2]; endfunction
    function automatic pte_x(input [63:0] p) ; pte_x = p[3]; endfunction
    function automatic [PPN_WIDTH-1:0] pte_ppn_extract(input [63:0] p);
        return p[53:10]; // Sv39 PPN [cite: 441]
    endfunction

    // --- FSM (Máquina de Estados Finitos) ---
    typedef enum logic [2:0] { S_IDLE=0, S_READ, S_WAIT, S_CHECK, S_RESP } state_t;
    state_t state, next_state;

    // Registradores de Contexto do Walk
    reg [VPN_WIDTH-1:0] vpn_reg;
    reg [15:0]          asid_reg;
    reg [1:0]           level_reg; // 2, 1, 0
    reg [PPN_WIDTH-1:0]   pte_ppn;
    reg [63:0]            pte_data_reg;
    reg                   walk_page_fault;

    // --- FSM Sequencial ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            ptw_req_ready <= 1'b1;
            ptw_mem_req <= 1'b0;
            ptw_mem_addr <= '0;
            ptw_resp_valid <= 1'b0;
            vpn_reg <= '0;
            asid_reg <= '0;
            level_reg <= 2;
        end else begin
            state <= next_state;
            
            // Lógica de Requisição (pulso de 1 ciclo)
            if (next_state == S_READ) begin
                ptw_mem_req <= 1'b1;
            end else begin
                ptw_mem_req <= 1'b0;
            end
            
            // Lógica de Resposta (pulso de 1 ciclo)
            ptw_resp_valid <= (next_state == S_RESP);

            // Captura de dados
            if (next_state == S_IDLE) begin
                ptw_req_ready <= 1'b1;
                if (ptw_req_valid) begin // Captura nova requisição
                    vpn_reg <= ptw_req_vpn;
                    asid_reg <= ptw_req_asid;
                    level_reg <= 2; // Inicia no topo
                    ptw_req_ready <= 1'b0;
                end
            end
            
            if (state == S_READ && ptw_mem_resp_valid) begin
                pte_data_reg <= ptw_mem_resp_data; // Captura PTE
            end
            
            // Atualiza o endereço da memória
            if (next_state == S_READ) begin
                 wire [8:0] vpn_idx = (level_reg == 2) ? vpn_reg[38:30] :
                                    (level_reg == 1) ? vpn_reg[29:21] :
                                                      vpn_reg[20:12];
                wire [PPN_WIDTH-1:0] base_ppn = (level_reg == 2) ? satp_ppn(satp) : pte_ppn;
                
                ptw_mem_addr <= make_pte_addr(base_ppn, vpn_idx);
            end
        end
    end
    
    // --- FSM Combinacional ---
    always_comb begin
        next_state = state;
        
        ptw_resp_ppn = '0;
        ptw_resp_page_fault = 1'b0;
        ptw_resp_vpn = vpn_reg;
        ptw_resp_asid = asid_reg;

        case (state)
            S_IDLE: begin
                if (ptw_req_valid && ptw_req_ready) begin
                    next_state = S_READ;
                end
            end
            
            S_READ: begin
                if (ptw_mem_resp_valid) begin
                    next_state = S_CHECK;
                end else if (ptw_mem_resp_err) begin
                    walk_page_fault = 1'b1;
                    next_state = S_RESP;
                end
            end
            
            S_CHECK: begin
                // Checa o PTE lido (pte_data_reg)
                if (!pte_v(pte_data_reg) || (!pte_r(pte_data_reg) && pte_w(pte_data_reg))) begin
                    walk_page_fault = 1'b1;
                    next_state = S_RESP; // Page Fault
                end 
                // É um ponteiro? (R, W, X = 0)
                else if (!pte_r(pte_data_reg) && !pte_x(pte_data_reg)) begin
                    if (level_reg == 0) begin
                        walk_page_fault = 1'b1; // Ponteiro no último nível
                        next_state = S_RESP;
                    end else begin
                        // Desça um nível
                        level_reg <= level_reg - 1;
                        pte_ppn <= pte_ppn_extract(pte_data_reg);
                        next_state = S_READ; // Próxima leitura
                    end
                end 
                // É uma folha (Leaf)? (R=1 ou X=1)
                else begin
                    // TODO: Verificar violações de superpage
                    walk_page_fault = 1'b0;
                    ptw_resp_ppn = pte_ppn_extract(pte_data_reg);
                    next_state = S_RESP; // Sucesso
                end
            end
            
            S_RESP: begin
                ptw_resp_page_fault = walk_page_fault;
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end

endmodule
