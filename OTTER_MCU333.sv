`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/02/2025 05:14:39 PM
// Design Name: 
// Module Name: OTTER_MCU333
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
  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic [2:0] fun3;
    logic [6:0] ir0;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic [31:0] alu_srcA;
    logic [31:0] alu_srcB;
    logic [31:0] rs2;
    logic [31:0] alu_res;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pcPlus4;
} instr_t;

module OTTER_MCU333(
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

    logic br_eq, br_lt, br_ltu, PCWrite, regWrite, memWE2, memRDEN1, memRDEN2, reset; //, csr_WE, int_taken, mret_exec;
    logic[3:0] alu_fun;
    logic[2:0] alu_srcB, pcSource;
    logic [1:0] alu_srcA, rf_wr_sel;
    logic [31:0] decIR;

    

    //Instruction Fetch
    logic [31:0] decPC4;
    assign PCPlusFour=PC+4;
    PC_DIN_MUX PCMUX(.SEL(pcSource), .JALR(jalr), .BRANCH(branch), .JAL(jal), .PLUS_FOUR(PCPlusFour), .PC_DIN(PC_DIN));
    PC pc(.PC_DIN(PC_DIN), .PC_RST(reset), .PC_WE(PCWrite), .CLK(CPU_CLK), .PC_COUNT(PC));
    assign PCWrite=1'b1;
    assign reset = 1'b0;
    always_ff @(posedge CPU_CLK) begin
        decIR <= DOUT1;
        decPC4<=PCPlusFour;
     end
  
    

    //pipeline registers
    instr_t de_reg, ex_reg, mem_reg, wb_reg;

    //Instruction Decode
    assign de_reg.rs1_addr=decIR[19:15];
    assign de_reg.rs2_addr=decIR[24:20];
    assign de_reg.rd_addr=decIR[11:7];
    assign de_reg.fun3=decIR[14:12];
    assign de_reg.ir0=decIR[6:0];
    assign de_reg.pcPlus4=decPC4;
    assign de_reg.rs2=rs2;
    opcode_t OPCODE;
    assign OPCODE_t = opcode_t'(decIR[6:0]);
    assign de_reg.opcode=OPCODE;
   
    assign de_reg.rs1_used=    de_reg.rs1_addr != 0
                                && de_reg.opcode != LUI
                                && de_reg.opcode != AUIPC
                                && de_reg.opcode != JAL;
    assign de_reg.rs2_used=   de_reg.rs2!=0 && (de_reg.opcode==OP || de_reg.opcode==STORE || de_reg.opcode==BRANCH);
    
    CU_DCDR decoder(.ir0(decIR[6:0]), .ir12(decIR[14:12]), .ir30(decIR[30]), .alu_fun(de_reg.alu_fun), .alu_srcA(alu_srcA), .alu_srcB(alu_srcB), .rf_wr_sel(de_reg.rf_wr_sel), .memWrite(de_reg.memWrite), .memRead2(de_reg.memRead2));

    
    RegFile regFile(.clk(CPU_CLK), .en(regWrite), .adr1(de_reg.rs1_addr), .adr2(de_reg.rs2_addr), .w_adr(de_reg.rd_addr), .w_data(wd), .rs1(rs1),.rs2(rs2));
    IMMED_GEN immy(.Instruction(DOUT1[31:7]), .U_TYPE(U_Type), .I_TYPE(I_Type), .S_TYPE(S_Type), .J_TYPE(J_Type), .B_TYPE(B_Type));
    
    srcA_mux muxA(.alu_srcA(alu_srcA), .rs1(rs1), .U_Type(U_Type), .srcA(ex_reg.alu_srcA));
    srcB_mux muxB(.alu_srcB(alu_srcB), .rs2(rs2), .I_Type(I_Type), .S_Type(S_Type), .PC(PC), .csr_RD(csr_RD), .srcB(de_reg.alu_srcB));
    
    
    always_ff@(posedge CPU_CLK) begin
        ex_reg <= de_reg;
    end
    
   
    //Instruction Execute
    
    BRANCH_COND_GEN bcd(.rs1(ex_reg.alu_srcA), .rs2(ex_reg.alu_srcB), .pcSource(pcSource), .ir12(ex_reg.fun3), .ir0(ex_reg.ir0));
    BRANCH_ADDR_GEN bad(.J_TYPE_IMM(J_Type), .B_TYPE_IMM(B_Type), .I_TYPE_IMM(I_Type), .rs1(rs1), .PC(PC), .branch(branch), .jal(jal), .jalr(jalr));
    ALU MathYay(.srcA(ex_reg.alu_srcA), .srcB(ex_reg.alu_srcB), .alu_fun(ex_reg.fun3), .alu_result(ex_reg.alu_res));

    always_ff@(posedge CPU_CLK) begin
        mem_reg <= ex_reg;
    end
    //Instruction Memory:
    assign memRDEN1=1;   //for now, assuming no stalls
    Memory mem(.MEM_CLK(CPU_CLK), .MEM_RDEN1(memRDEN1), .MEM_RDEN2(mem_reg.memRead2), .MEM_WE2(mem_reg.memWrite), .MEM_ADDR1(PC[15:2]), .MEM_ADDR2(mem_reg.alu_res), .MEM_DIN2(mem_reg.rs2), .MEM_SIZE(mem_reg.mem_type[1:0]), .MEM_SIGN(mem_reg.mem_type[2:2]), .IO_IN(CPU_IOBUS_IN), .IO_WR(CPU_IOBUS_WR), .MEM_DOUT1(DOUT1), .MEM_DOUT2(DOUT2));

    always_ff@(posedge CPU_CLK) begin
        wb_reg <= mem_reg;
    end
    //instruction WB:
    reg_mux REGMUX(.rf_wr_sel(wb_reg.rf_wr_sel), .PC_Plus_Four(wb_reg.pcPlus4), .DOUT2(DOUT2), .alu_res(wb_reg.alu_res), .wd(wd));

    
endmodule


