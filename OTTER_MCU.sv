`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/17/2024 02:49:05 PM
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
    input [31:0] CPU_IOBUS_IN,
    input CPU_RST,
    input CPU_INTR,  
    input CPU_CLK,
    output [31:0] CPU_IOBUS_OUT,
    output [31:0] CPU_IOBUS_ADDR,
    output CPU_IOBUS_WR
    );                                                                                                                                                                                 
    logic [31:0] PC, PC_DIN, DOUT2, DOUT1, rs1, rs2, U_Type, I_Type, S_Type, B_Type, J_Type, jal, branch, jalr, srcA, srcB, result,  PCPlusFour, wd, mstatus, mepc, mtvec, csr_RD;//will be added with interrupts
//output of:  PC^ PCMUX^ memory^    Reg_File^ IMM_GEN^                          Branch_ADDR_Gen^         aluMuxA^ aluMuxB^ ALU^  PC+4^    regMux^  CSR^             

    logic br_eq, br_lt, br_ltu, PCWrite, regWrite, memWE2, memRDEN1, memRDEN2, reset, csr_WE, int_taken, mret_exec;
//out of:   ^BranchCondGen^   FSM^                                                                            

//out of DCR:
    logic[3:0] alu_fun;
    logic[2:0] alu_srcB, pcSource;
    logic [1:0] alu_srcA, rf_wr_sel;
    
    assign CPU_IOBUS_OUT=rs2;
    assign CPU_IOBUS_ADDR=result;
    assign PCPlusFour=PC+4;
       
    //PC connections
    PC pc(
        .PC_DIN(PC_DIN), 
        .PC_RST(reset), 
        .PC_WE(PCWrite), 
        .CLK(CPU_CLK), 
        .PC_COUNT(PC));
        
    PC_DIN_MUX PCMUX(
        .SEL(pcSource), 
        .JALR(jalr), 
        .BRANCH(branch), 
        .JAL(jal), 
        .MTVEC(mtvec), 
        .MEPC(mepc), 
        .PLUS_FOUR(PCPlusFour), 
        .PC_DIN(PC_DIN));
        
    BRANCH_ADDR_GEN bad(
        .J_TYPE_IMM(J_Type), 
        .B_TYPE_IMM(B_Type), 
        .I_TYPE_IMM(I_Type), 
        .rs1(rs1), 
        .PC(PC), 
        .branch(branch), 
        .jal(jal), 
        .jalr(jalr));

    //core of computer: memory, reg, immediate creation connections
    Memory mem(
        .MEM_CLK(CPU_CLK), 
        .MEM_RDEN1(memRDEN1), 
        .MEM_RDEN2(memRDEN2), 
        .MEM_WE2(memWE2), 
        .MEM_ADDR1(PC[15:2]), 
        .MEM_ADDR2(result), 
        .MEM_DIN2(rs2), 
        .MEM_SIZE(DOUT1[13:12]), 
        .MEM_SIGN(DOUT1[14]), 
        .IO_IN(CPU_IOBUS_IN), 
        .IO_WR(CPU_IOBUS_WR), 
        .MEM_DOUT1(DOUT1), 
        .MEM_DOUT2(DOUT2));
        
    RegFile regFile(
        .clk(CPU_CLK), 
        .en(regWrite), 
        .adr1(DOUT1[19:15]), 
        .adr2(DOUT1[24:20]), 
        .w_adr(DOUT1[11:7]), 
        .w_data(wd), 
        .rs1(rs1),
        .rs2(rs2));
        
    reg_mux REGMUX(
        .rf_wr_sel(rf_wr_sel), 
        .PC_Plus_Four(PCPlusFour), 
        .csr_RD(csr_RD), 
        .DOUT2(DOUT2), 
        .alu_res(result), 
        .wd(wd));
        
    IMMED_GEN immy(
        .Instruction(DOUT1[31:7]), 
        .U_TYPE(U_Type), 
        .I_TYPE(I_Type), 
        .S_TYPE(S_Type), 
        .J_TYPE(J_Type), 
        .B_TYPE(B_Type));
    
    //ALU connections
    srcA_mux muxA(
        .alu_srcA(alu_srcA), 
        .rs1(rs1), 
        .U_Type(U_Type), 
        .srcA(srcA));
        
    srcB_mux muxB(
        .alu_srcB(alu_srcB), 
        .rs2(rs2), 
        .I_Type(I_Type), 
        .S_Type(S_Type), 
        .PC(PC), 
        .srcB(srcB));
        
    ALU MathYay(
        .srcA(srcA), 
        .srcB(srcB), 
        .alu_fun(alu_fun), 
        .alu_result(result));
    
    //Control Unit connections
    logic fsm_INTR;
    assign fsm_INTR = CPU_INTR && mstatus[3];
    
    BRANCH_COND_GEN bcd(
        .rs1(rs1), 
        .rs2(rs2), 
        .ir12(DOUT1[14:12]), 
        .ir30(DOUT1[30]), 
        .pcSource(pcSource));
        
    CU_DCDR decoder(
        .int_taken(int_taken), 
        .ir0(DOUT1[6:0]), 
        .ir12(DOUT1[14:12]), 
        .ir30(DOUT1[30]), 
        .br_eq(br_eq), 
        .br_lt(br_lt), 
        .br_ltu(br_ltu), 
        .alu_fun(alu_fun), 
        .alu_srcA(alu_srcA), 
        .alu_srcB(alu_srcB), 
        .pcSource(pcSource), 
        .rf_wr_sel(rf_wr_sel));
        
    //CU_FSM fsm(.clk(CPU_CLK), .RST(CPU_RST), .opcode(DOUT1[6:0]), .INTR(fsm_INTR), .PCWrite(PCWrite), .regWrite(regWrite), .memWE2(memWE2), .memRDEN1(memRDEN1), .memRDEN2(memRDEN2), .reset(reset), .func3(DOUT1[14:12]), .csr_WE(csr_WE), .int_taken(int_taken), .mret_exec(mret_exec));
    
    //CSR
    /*
    CSR interupts(
        .CLK(CPU_CLK), 
        .RST(reset), 
        .mret_exec(mret_exec), 
        .INT_TAKEN(int_taken), 
        .ADDR(DOUT1[31:20]), 
        .WR_EN(csr_WE), 
        .PC(PC), 
        .WD(result), 
        .mepc(mepc), 
        .mtvec(mtvec), 
        .mstatus(mstatus), 
        .RD(csr_RD));
        */
endmodule
