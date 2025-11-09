`include "stu_pkg.sv"
`include "supernova_pkg.svh"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module supernova_alu_unit
 * @brief Unidade de Execução (ALU) completa (RV64I) para o Supernova.
 *
 * @details Este módulo recebe uma 'rs_entry' do scheduler (issue),
 * a executa (1 ciclo de latência) e retorna o resultado.
 * A lógica de execução é portada do Nebula Core.
 */
module supernova_alu_unit (
    input  wire clk,
    input  wire rst_n,

    // --- Interface de Emissão (Issue) ---
    input  wire                       req_valid_in,
    input  supernova_pkg::rs_entry_t  req_entry_in, // A instrução e seus dados

    // --- Interface de Saída (Writeback) ---
    output logic                      wb_valid_out,
    output logic [stu_pkg::XLEN-1:0]  wb_data_out,
    output logic [supernova_pkg::GPR_TAG_WIDTH-1:0] wb_gpr_tag_out,
    output logic [supernova_pkg::ROB_IDX_WIDTH-1:0] wb_rob_idx_out,
    output logic                      wb_exception_out,
    output logic [stu_pkg::ADDR_WIDTH-1:0] wb_trap_cause_out
);

    import stu_pkg::*;
    import supernova_pkg::*;

    // --- Registradores de Pipeline (1 ciclo de latência) ---
    logic                      ex_valid_reg;
    stu_pkg::reg_width_t       ex_result_reg;
    supernova_pkg::GPR_TAG_WIDTH_t ex_gpr_tag_reg;
    supernova_pkg::ROB_IDX_WIDTH_t ex_rob_idx_reg;
    logic                      ex_exception_reg;
    stu_pkg::addr_t            ex_trap_cause_reg;

    // --- Estruturas de Decode/Execute (Portadas do Nebula) ---
    // (Necessário para a função 'execute_alu_function')
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
    } decoded_instr_t;

    typedef struct packed {
        logic [XLEN-1:0] alu_result;
        logic [XLEN-1:0] mem_addr;
        logic [XLEN-1:0] store_data;
        logic [4:0] rd;
        logic mem_we;
        logic [7:0] mem_wstrb;
        logic reg_we;
        logic freg_we;
        logic branch_taken;
        logic [XLEN-1:0] branch_target;
        logic [XLEN-1:0] pc;
        logic is_amo;
        logic [4:0] amo_funct5;
        logic trap;
        logic [XLEN-1:0] trap_cause;
        logic [XLEN-1:0] trap_value;
        logic [4:0] wb_rd;
        logic         wb_reg_we;
        logic [XLEN-1:0] wb_data;
    } execute_result_t;
    
    // --- Lógica de Execução (Combinacional) ---
    
    execute_result_t alu_result;
    decoded_instr_t decoded_instr;

    // (Função de decode interna, necessária para obter funct3/funct7/imm)
    assign decoded_instr = decode_instr_function(req_entry_in.instr); 

    // (Chama a função de execução completa)
    assign alu_result = execute_alu_function(
        decoded_instr, 
        req_entry_in.src1_data, 
        req_entry_in.src2_data, 
        req_entry_in.pc
    );

    // --- Lógica Sequencial (Estágio de Pipeline) ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_valid_reg <= 1'b0;
        end else begin
            ex_valid_reg <= req_valid_in;
            if (req_valid_in) begin
                ex_result_reg <= alu_result.alu_result;
                ex_gpr_tag_reg <= req_entry_in.rd_phys_tag;
                ex_rob_idx_reg <= req_entry_in.rob_idx;
                ex_exception_reg <= alu_result.trap;
                ex_trap_cause_reg <= alu_result.trap_cause;
            end
        end
    end

    // --- Saídas (Registradas) ---
    assign wb_valid_out = ex_valid_reg;
    assign wb_data_out = ex_result_reg;
    assign wb_gpr_tag_out = ex_gpr_tag_reg;
    assign wb_rob_idx_out = ex_rob_idx_reg;
    assign wb_exception_out = ex_exception_reg;
    assign wb_trap_cause_out = ex_trap_cause_reg;

    // --- Funções de Pipeline (Portadas do Nebula [cite: 721-807]) ---

    // (Nota: 'decode_instr_function' é uma cópia simplificada,
    //  pois o 'decode_rename' [cite: 813-954] já fez o trabalho pesado)
    function automatic decoded_instr_t decode_instr_function(input [ILEN-1:0] instr);
        decoded_instr_t result;
        result.opcode = instr[6:0];
        result.rd = instr[11:7];
        result.funct3 = instr[14:12];
        result.rs1 = instr[19:15];
        result.rs2 = instr[24:20];
        result.funct7 = instr[31:25];
        result.is_alu = 1'b1; // (Assume que só recebe ALU ops)
        result.is_mdu = (result.opcode == 7'b0110011 && result.funct7 == 7'b0000001);
        
        // Geração de Imediato (necessário para ALU-I)
        case (result.opcode)
            7'b0010011: result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
            7'b0110111, 7'b0010111: result.imm = {{XLEN-32{instr[31]}}, instr[31:12], 12'b0};
            default: result.imm = '0;
        endcase
        return result;
    endfunction

    function automatic execute_result_t execute_alu_function(
        input decoded_instr_t instr, 
        input [XLEN-1:0] rs1_data, 
        input [XLEN-1:0] rs2_data, 
        input [XLEN-1:0] pc_val
    );
        execute_result_t result;
        // (Padrões)
        result.alu_result = '0;
        result.reg_we = 1'b0;
        result.trap = 1'b0;
        result.trap_cause = '0;

        if (instr.is_alu) begin
            result.reg_we = 1'b1;
            case (instr.opcode)
                7'b0110111: result.alu_result = instr.imm; // LUI
                7'b0010111: result.alu_result = pc_val + instr.imm; // AUIPC
                
                7'b0010011: begin // ALU-I
                    case (instr.funct3)
                        3'b000: result.alu_result = rs1_data + instr.imm; // ADDI
                        3'b010: result.alu_result = ($signed(rs1_data) < $signed(instr.imm)) ? 1 : 0; // SLTI
                        3'b011: result.alu_result = (rs1_data < instr.imm) ? 1 : 0; // SLTIU
                        3'b100: result.alu_result = rs1_data ^ instr.imm; // XORI
                        3'b110: result.alu_result = rs1_data | instr.imm; // ORI
                        3'b111: result.alu_result = rs1_data & instr.imm; // ANDI
                        3'b001: result.alu_result = rs1_data << instr.imm[5:0]; // SLLI
                        3'b101: begin // SRLI / SRAI
                            if (instr.funct7[5] == 0)
                                result.alu_result = rs1_data >> instr.imm[5:0]; // SRLI
                            else
                                result.alu_result = ($signed(rs1_data)) >>> instr.imm[5:0]; // SRAI
                        end
                        default: begin result.trap = 1'b1; result.trap_cause = 2; end // Illegal Instr
                    endcase
                end
                
                7'b0110011: begin // ALU-R (I-Ext)
                    case ({instr.funct7, instr.funct3})
                        {7'b0000000, 3'b000}: result.alu_result = rs1_data + rs2_data; // ADD
                        {7'b0100000, 3'b000}: result.alu_result = rs1_data - rs2_data; // SUB
                        {7'b0000000, 3'b001}: result.alu_result = rs1_data << rs2_data[5:0]; // SLL
                        {7'b0000000, 3'b010}: result.alu_result = ($signed(rs1_data) < $signed(rs2_data)) ? 1 : 0; // SLT
                        {7'b0000000, 3'b011}: result.alu_result = (rs1_data < rs2_data) ? 1 : 0; // SLTU
                        {7'b0000000, 3'b100}: result.alu_result = rs1_data ^ rs2_data; // XOR
                        {7'b0000000, 3'b101}: result.alu_result = rs1_data >> rs2_data[5:0]; // SRL
                        {7'b0100000, 3'b101}: result.alu_result = ($signed(rs1_data)) >>> rs2_data[5:0]; // SRA
                        {7'b0000000, 3'b110}: result.alu_result = rs1_data | rs2_data; // OR
                        {7'b0000000, 3'b111}: result.alu_result = rs1_data & rs2_data; // AND
                        default: begin result.trap = 1'b1; result.trap_cause = 2; end // Illegal Instr
                    endcase
                end
            endcase
        end
        else if (instr.is_mdu) begin
             // STUB: Requer MDU multi-ciclo
             result.reg_we = 1'b1;
             result.alu_result = rs1_data * rs2_data; // (Placeholder MUL)
        end
        
        return result;
    endfunction
    
    // (Função de decode interna, necessária para obter funct3/funct7/imm)
    function automatic decoded_instr_t decode_instr_function(input [ILEN-1:0] instr);
        decoded_instr_t result;
        result.opcode = instr[6:0];
        result.rd = instr[11:7];
        result.funct3 = instr[14:12];
        result.rs1 = instr[19:15];
        result.rs2 = instr[24:20];
        result.funct7 = instr[31:25];
        result.is_alu = 1'b1; // (Assume que só recebe ALU ops)
        result.is_mdu = (result.opcode == 7'b0110011 && result.funct7 == 7'b0000001);
        
        // Geração de Imediato (necessário para ALU-I)
        case (result.opcode)
            7'b0010011: result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
            7'b0110111, 7'b0010111: result.imm = {{XLEN-32{instr[31]}}, instr[31:12], 12'b0};
            default: result.imm = '0;
        endcase
        return result;
    endfunction
    
endmodule
