`include "stu_pkg.sv"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module tlb_associative
 * @brief TLB (Translation Lookaside Buffer) simples, totalmente associativo.
 *
 * @details Armazena em cache as traduções de Endereço Virtual (VA) para
 * Endereço Físico (PA) para acelerar a MMU.
 */
module tlb_associative #(
    parameter VPN_WIDTH = 39,
    parameter PPN_WIDTH = 44,
    parameter TLB_ENTRIES = 64
)(
    input  wire clk,
    input  wire rst_n,
    
    // --- Interface de Consulta (Lookup) ---
    input  wire                 lookup_valid,
    input  wire [VPN_WIDTH-1:0] lookup_vpn,
    input  wire [15:0]          lookup_asid,
    output logic                  lookup_hit,
    output logic [PPN_WIDTH-1:0]  lookup_ppn,
    output logic                  lookup_page_fault, // (Permissões, etc.)

    // --- Interface de Inserção (Preenchimento) ---
    input  wire                 insert_valid,
    input  wire [VPN_WIDTH-1:0] insert_vpn,
    input  wire [PPN_WIDTH-1:0] insert_ppn,
    input  wire [15:0]          insert_asid,
    
    // --- Interface de Invalidação (Flush) ---
    input  wire                 invalidate_all,
    input  wire                 invalidate_by_asid,
    input  wire [15:0]          invalidate_asid
);

    typedef struct packed {
        logic                   valid;
        logic [VPN_WIDTH-1:0]   vpn;
        logic [PPN_WIDTH-1:0]   ppn;
        logic [15:0]            asid;
        // (Adicionar bits de permissão: R, W, X, U)
    } tlb_entry_t;

    tlb_entry_t entries [0:TLB_ENTRIES-1];
    integer i;
    reg [$clog2(TLB_ENTRIES)-1:0] rr_ptr; // Ponteiro Round-Robin

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<TLB_ENTRIES; i=i+1) begin
                entries[i].valid <= 1'b0;
            end
            rr_ptr <= '0;
        end else begin
            // --- Lógica de Invalidação (Flush) ---
            if (invalidate_all) begin
                for (i=0; i<TLB_ENTRIES; i=i+1) entries[i].valid <= 1'b0;
            end else if (invalidate_by_asid) begin
                for (i=0; i<TLB_ENTRIES; i=i+1) begin
                    if (entries[i].valid && entries[i].asid == invalidate_asid)
                        entries[i].valid <= 1'b0;
                end
            end

            // --- Lógica de Inserção (Preenchimento) ---
            if (insert_valid) begin
                entries[rr_ptr].valid <= 1'b1;
                entries[rr_ptr].vpn   <= insert_vpn;
                entries[rr_ptr].ppn   <= insert_ppn;
                entries[rr_ptr].asid  <= insert_asid;
                rr_ptr <= rr_ptr + 1; // (Wrap-around implícito)
            end
        end
    end
    
    // --- Lógica de Consulta (Combinacional) ---
    always_comb begin
        lookup_hit = 1'b0;
        lookup_ppn = '0;
        lookup_page_fault = 1'b0; // (Lógica de permissão TBD)

        if (lookup_valid) begin
            for (i=0; i<TLB_ENTRIES; i=i+1) begin
                if (entries[i].valid && 
                    entries[i].vpn == lookup_vpn && 
                    (entries[i].asid == lookup_asid)) // (TODO: Lógica 'Global')
                begin
                    lookup_hit = 1'b1;
                    lookup_ppn = entries[i].ppn;
                    // (TODO: Verificar bits de permissão R/W/X
                    //  e definir lookup_page_fault se violado)
                    break;
                end
            end
        end
    end

endmodule
