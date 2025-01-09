`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Dovydas Vabalas
// 
// Create Date: 11/03/2024 11:54:26 AM
// Design Name: 
// Module Name: OTTER_MCU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_MCU(        
    input RST,    
    input intr,       
    input clk,         
    input [31:0] iobus_in,   
    output logic [31:0] iobus_out, 
    output logic [31:0] iobus_addr, 
    output logic iobus_wr   
    );
    
    // Instantiate wire connections
    logic [31:0] pc_out, pc_in, u_type, i_type, s_type, jal, branch, jalr, j_type, b_type, ir, mem_dout2, ALU_srcA, ALU_srcB, ALU_out, w_data, rs1, rs2, mtvec, mepc, csr_RD;
    logic [3:0] ALU_FUN;
    logic [2:0] pc_sel, srcB_sel;
    logic [1:0] rf_sel, srcA_sel;
    logic rf_we, memWE2, memRDEN1, memRDEN2, reset, br_eq, br_lt, br_ltu, int_taken, mret_exec, csr_WE, mstatus;
    
    
    // Control Unit FSM
    CU_FSM CU_FSM (
        .intr       (intr && mstatus),
        .clk        (clk),
        .RST        (RST),
        .opcode     (ir[6:0]),
        .func3      (ir[14:12]),
        .PC_WE      (pc_we),
        .RF_WE      (rf_we),
        .memWE2     (memWE2),
        .memRDEN1   (memRDEN1),
        .memRDEN2   (memRDEN2),
        .reset      (reset),
        .csr_WE     (csr_WE),
        .int_taken  (int_taken),
        .mret_exec  (mret_exec)
        );
        
        
    // Control Unit Decoder
    CU_DCDR CU_DCDR (
        .br_eq      (br_eq),
        .br_lt      (br_lt),
        .br_ltu     (br_ltu),
        .int_taken  (int_taken),
        .opcode     (ir[6:0]),
        .func7      (ir[30]),
        .func3      (ir[14:12]),
        .ALU_FUN    (ALU_FUN),
        .PC_SEL     (pc_sel),
        .srcA_SEL   (srcA_sel),
        .srcB_SEL   (srcB_sel),
        .RF_SEL     (rf_sel)
        );
        
        
    // IMMED GEN - Immediate Value Generator
        assign u_type = {ir[31:12], {12{1'b0}}};   
        assign i_type = {{21{ir[31]}}, ir[30:25], ir[24:20]};
        assign s_type = {{21{ir[31]}}, ir[30:25], ir[11:7]};
        assign b_type = {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0};
        assign j_type = {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};
        
        
    // BAG - Branch Address Generator
        assign branch = pc_out + b_type;
        assign jal = pc_out + j_type;
        assign jalr = rs1 + i_type;
        
        
    // BRANCH_COND_GEN
        assign br_eq = rs1 == rs2 ? 1 : 0;
        assign br_lt = $signed(rs1) < $signed(rs2) ? 1 : 0;
        assign br_ltu = rs1 < rs2 ? 1 : 0;
        
        
    // CSR - Control and Status Register
    CSR CSR (
        .CLK                (clk),
        .RST                (reset),
        .MRET_EXEC          (mret_exec),
        .INT_TAKEN          (int_taken),
        .ADDR               (ir[31:20]),
        .PC                 (pc_out),
        .WD                 (ALU_out),
        .WR_EN              (csr_WE),
        .RD                 (csr_RD),
        .CSR_MEPC           (mepc),
        .CSR_MTVEC          (mtvec),
        .CSR_MSTATUS_MIE    (mstatus)
        );    
        
    // Register File and Mux
    RegFile RegFile (
        .w_data     (w_data),
        .clk        (clk),
        .en         (rf_we),
        .adr1       (ir[19:15]),
        .adr2       (ir[24:20]),
        .w_adr      (ir[11:7]),
        .rs1        (rs1),
        .rs2        (rs2)
        );
        
        // Register File MUX
        mux_4t1_nb #(.n(32)) REG_MUX (
                .SEL        (rf_sel),
                .D0         (pc_out + 4),
                .D1         (csr_RD), 
                .D2         (mem_dout2),
                .D3         (ALU_out), 
                .D_OUT      (w_data)         
                );    
        
        
    // Arithmetic Logic Unit and Source MUXES
    ALU ALU (
        .srcA (ALU_srcA),
        .srcB (ALU_srcB),
        .alu_fun (ALU_FUN),
        .result (ALU_out)
        );
        
        // Source A MUX
        mux_4t1_nb #(.n(32)) SRCA_MUX (
                    .SEL        (srcA_sel),
                    .D0         (rs1),
                    .D1         (u_type),
                    .D2         (~rs1),
                    .D3         (0),
                    .D_OUT      (ALU_srcA)
                    );
            
        // Source B MUX
        mux_8t1_nb #(.n(32)) SRCB_MUX (
                    .SEL        (srcB_sel),
                    .D0         (rs2),
                    .D1         (i_type),
                    .D2         (s_type),
                    .D3         (pc_out), 
                    .D4         (csr_RD),
                    .D5         (0),
                    .D6         (0),
                    .D7         (0),
                    .D_OUT      (ALU_srcB)         
                    );
                
                
    // Program Counter and MUX         
    reg_nb #(.n(32)) PC (
        .data_in    (pc_in),
        .ld         (pc_we),
        .clr        (reset),
        .clk        (clk),
        .data_out   (pc_out) 
        );
        
        // Program Counter MUX
        mux_8t1_nb #(.n(32)) PC_MUX (
            .SEL        (pc_sel),
            .D0         (pc_out + 4),
            .D1         (jalr),
            .D2         (branch),
            .D3         (jal), 
            .D4         (mtvec),
            .D5         (mepc),
            .D6         (0),
            .D7         (0),
            .D_OUT      (pc_in)         
            );
        
    // Memory
    Memory OTTER_MEMORY (
        .MEM_CLK    (clk),
        .MEM_RDEN1  (memRDEN1),
        .MEM_RDEN2  (memRDEN2),
        .MEM_WE2    (memWE2),
        .MEM_ADDR1  (pc_out[15:2]), // 14-bit signal
        .MEM_ADDR2  (ALU_out),
        .MEM_DIN2   (rs2),
        .MEM_SIZE   (ir[13:12]),
        .MEM_SIGN   (ir[14]),
        .IO_IN      (iobus_in),
        .IO_WR      (iobus_wr), 
        .MEM_DOUT1  (ir), 
        .MEM_DOUT2  (mem_dout2) 
        );
        
   // Output Declarations
   assign iobus_out = rs2;
   assign iobus_addr = ALU_out;
    
    
endmodule
