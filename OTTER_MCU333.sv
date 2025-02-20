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

module OTTER_MCU333(
    input [31:0] CPU_IOBUS_IN,
    input CPU_RST,
    //input CPU_INTR,  //used for interrupts. when the time comes for re-enabling interrupts, do so in the otter wrapper, and also uncomment the lines in constraints assining BTNL
    input CPU_CLK,
    output [31:0] CPU_IOBUS_OUT,
    output [31:0] CPU_IOBUS_ADDR,
    output CPU_IOBUS_WR
    ); 
    
    logic notStall; //assigned in code for forwarding Unit, controls PCWrite, and pipeline register progression
    logic branchTaken;  //will make the instruction that got pulled from Mem on the cc interpreted as a no-op
    logic branchTakenPropogated;    //propogate branch taken to the next loaded instruction so that it can be interpreted as a no-op
    always_ff @(posedge CPU_CLK) begin
        branchTakenPropogated <= branchTaken;
    end
    //Instruction Fetch
    //local fetch logic:
    logic [31:0] DOUT1, PC; //wires for registers
    logic[31:0] PC_DIN, PCPlusFour; //fully internal
    
    //feedback fetch logic: (values used here that are output elsewere
    logic [31:0] jalr, branch, jal;  //set by branch address generator
    logic [2:0] pcSource;       //set by branch cond gen
    logic PCWrite;  //will be set by hazard control, for now set always true
    

    assign PCWrite=notStall;
    assign PCPlusFour=PC+4;
    PC_DIN_MUX PCMUX(.SEL(pcSource), .JALR(jalr), .BRANCH(branch), .JAL(jal), .PLUS_FOUR(PCPlusFour), .PC_DIN(PC_DIN));
    PC pc(.PC_DIN(PC_DIN), .PC_RST(CPU_RST), .PC_WE(PCWrite), .CLK(CPU_CLK), .PC_COUNT(PC));
    
   //pipeline registers. these are the registers the decode part of computer needs to read on the next cycle
    logic [31:0] deReg_IR, deReg_PC;
    //
//    assign deReg_IR = DOUT1;   //written with assign since DOUT1 is computed on posedge (in not combinational)
    
    always_comb begin
        if(branchTaken || branchTakenPropogated) begin   //perform nop if branch taken
            deReg_IR=8'h00000013;   //nop instruction
        end else begin
            deReg_IR=DOUT1;
        end
    end
    always_ff @(posedge CPU_CLK) begin
        //deReg_IR <= DOUT1;  //set by mem file
        if  (notStall && !branchTaken) begin        //check that IR is also being delayed!
            deReg_PC<=PC;
        end
     end


    //DECODE    
    //decode logic local
    logic rs1_used, rs2_used, rs1Selected, rs2Selected;
    logic [6:0] opcode;
    //decoder outputs:
    logic regWrite, memWrite, memRead2;
    logic [3:0] alu_fun;
    logic [1:0] muxASel;    //fully internal
    logic [2:0] muxBSel;    //fully internal
    logic [1:0] rf_wr_sel;
    //mux Outputs
    logic [31:0] srcA, srcB;
    //regfile
    logic [31:0] rs1, rs2; //used by memory module for writing data outputs
    logic [31:0] wd;         //inputs, used in WB phase, (output of the regMux)
    //imm gen outputs
    logic [31:0] U_Type, I_Type, S_Type, B_Type, J_Type;
    
    logic wbReg_regWrite;   //from wb state
    logic [4:0] wbReg_wa;
    

    //Instruction Decode
    //assign deReg_rs1Addr=ifReg_IR[19:15];
    //assign deReg_rs2Addr=ifReg_IR[24:20];
    //assign deReg_rdAddr=ifReg_IR[11:7];
    
    //assign deReg_fun3=ifReg_IR[14:12];
    //assign deReg_pcPlus4=ifReg_PC4;
    
    assign opcode = opcode_t'(deReg_IR[6:0]);
   
   // rs1 and rs2 used are not yet needed since no forwarding, will be used in future
    assign rs1_used=    deReg_IR[19:15] != 0
                                && opcode != LUI
                                && opcode != AUIPC
                                && opcode != JAL;
    assign rs2_used=   deReg_IR[24:20] !=0 && (opcode==OP || opcode==STORE || opcode==BRANCH);
    assign rs1Selected=muxASel==2'b00;
    assign rs2Selected=muxBSel==3'b000;
    CU_DCDR decoder(.ir0(deReg_IR[6:0]), .ir12(deReg_IR[14:12]), .ir30(deReg_IR[30]), .alu_fun(alu_fun), .alu_srcA(muxASel), .alu_srcB(muxBSel), .rf_wr_sel(rf_wr_sel), .regWrite(regWrite), .memWrite(memWrite), .memRead2(memRead2));

    
    RegFile regFile(.clk(CPU_CLK), .en(wbReg_regWrite), .adr1(deReg_IR[19:15]), .adr2(deReg_IR[24:20]), .w_adr(wbReg_wa), .w_data(wd), .rs1(rs1),.rs2(rs2));
    IMMED_GEN immy(.Instruction(deReg_IR[31:7]), .U_TYPE(U_Type), .I_TYPE(I_Type), .S_TYPE(S_Type), .J_TYPE(J_Type), .B_TYPE(B_Type));
    
    srcA_mux muxA(.alu_srcA(muxASel), .rs1(rs1), .U_Type(U_Type), .srcA(srcA));
    srcB_mux muxB(.alu_srcB(muxBSel), .rs2(rs2), .I_Type(I_Type), .S_Type(S_Type), .PC(deReg_PC), .srcB(srcB));
    
    //ex pipeline registers 
    logic [31:0] exReg_PC; //previous pipeline values
    logic [6:0] exReg_opcode;   //trimming IR
    logic [4:0] exReg_wa;
    logic [2:0] exReg_fun3;
    logic [4:0] exReg_rs1Addr;  //needed for hazard detection
    logic [4:0] exReg_rs2Addr;
    //logic [31:0] exReg_rs1, exReg_rs2;  //new ones
    logic [31:0] exReg_ALUinpA, exReg_ALUinpB;
    logic exReg_regWrite, exReg_memWrite, exReg_memRead2;
    logic [3:0] exReg_aluFun;
    logic [1:0] exReg_rf_wr_sel;
    logic [31:0] exReg_J_Type, exReg_B_Type, exReg_I_Type;
    logic exReg_rs1_used, exReg_rs2_used;
    logic exReg_rs1Selected;
    logic exReg_rs2Selected;
    //
    
    always_ff @(posedge CPU_CLK) begin
//        if (branchTaken) begin
//            exReg_regWrite <=0; //make this instruction a no-op instruction
//            exReg_memWrite <=0;
//            exReg_opcode <=7'b0010011;  
//            exReg_wa=0; //set to write to 0 so no accidental forwarding
//            exReg_rs1Addr=0;
//            exReg_rs2Addr=0;
//        end else
         if (notStall) begin
            exReg_opcode <= deReg_IR[6:0];  //transfer previous pipeline values, trimming IR
            exReg_wa <= deReg_IR[11:7];
            exReg_fun3 <= deReg_IR[14:12];
            exReg_rs1Addr <=deReg_IR[19:15];    //needed for forwarding
            exReg_rs2Addr <=deReg_IR[24:20];
            exReg_rs1Selected <=rs1Selected;
            exReg_rs2Selected <= rs2Selected;
            exReg_rs1_used <=rs1_used;
            exReg_rs2_used <=rs2_used;
            exReg_rf_wr_sel <= rf_wr_sel;
            exReg_PC <= deReg_PC;
            //new pipeline
//            exReg_rs1 <= rs1;   //register values
//            exReg_rs2 <= rs2;
            exReg_ALUinpA <= srcA;  //ALU inputs
            exReg_ALUinpB <= srcB;
            exReg_aluFun <= alu_fun;
            exReg_J_Type <= J_Type; //generated immediates
            exReg_B_Type <= B_Type;
            exReg_I_Type <= I_Type;
            exReg_regWrite <= regWrite; //other control decoder outputs
            exReg_memWrite <= memWrite;
//            exReg_regWrite <= 1'b0; 
//            exReg_memWrite <= 1'b0;
            exReg_memRead2 <= memRead2;
        end
        

    end
    
   
    //Instruction Execute
    logic [31:0] aluRes;
    //F_ is a value forwarded from dataForwarding Unit
    logic [31:0] F_rs1, F_rs2ALU, F_rs2Mem; 
//    BRANCH_COND_GEN bcd(.rs1(exReg_rs1), .rs2(exReg_rs2), .pcSource(pcSource), .branchTaken(branchTaken), .ir12(exReg_fun3), .ir0(exReg_opcode));    //PCSource =SEL for pcMux above
//    BRANCH_ADDR_GEN bad(.J_TYPE_IMM(exReg_J_Type), .B_TYPE_IMM(exReg_B_Type), .I_TYPE_IMM(exReg_I_Type), .rs1(exReg_rs1), .PC(exReg_PC), .branch(branch), .jal(jal), .jalr(jalr));  //these outputs used in PC Mux above
    BRANCH_COND_GEN bcd(.rs1(F_rs1), .rs2(F_rs2ALU), .pcSource(pcSource), .branchTaken(branchTaken), .ir12(exReg_fun3), .ir0(exReg_opcode));    //PCSource =SEL for pcMux above
    BRANCH_ADDR_GEN bad(.J_TYPE_IMM(exReg_J_Type), .B_TYPE_IMM(exReg_B_Type), .I_TYPE_IMM(exReg_I_Type), .rs1(F_rs1), .PC(exReg_PC), .branch(branch), .jal(jal), .jalr(jalr));  //these outputs used in PC Mux above
    

    ALU MathYay(.srcA(F_rs1), .srcB(F_rs2ALU), .alu_fun(exReg_aluFun), .alu_result(aluRes));

    //pipeline registers:
    logic [31:0] memReg_aluRes; //new pipeline values
    logic [31:0] memReg_PC; //previous pipeline values
    logic [31:0] memReg_rs2; 
    logic memReg_regWrite, memReg_memWrite, memReg_memRead2;    
    logic [1:0] memReg_rf_wr_sel;
    logic [2:0] memReg_fun3; //trimming IR_Reg, no longer need the entire instruction
    logic [4:0] memReg_wa;
    
    always_ff@(posedge CPU_CLK) begin
            memReg_aluRes <= aluRes;    //new value
//            memReg_opcode <=exReg_opcode;
            memReg_PC <= exReg_PC;  //previous values
            memReg_rs2 <= F_rs2Mem;    //forwarded value of rs2
            if (notStall) begin
                memReg_regWrite<= exReg_regWrite;
                memReg_memWrite <= exReg_memWrite;
            end else begin
                memReg_regWrite <= 1'b0;
                memReg_memWrite <= 1'b0;
            end
            memReg_memRead2 <= exReg_memRead2;
            memReg_rf_wr_sel <= exReg_rf_wr_sel;
            memReg_fun3 <= exReg_fun3; //only continueing to use part of IR
            memReg_wa <= exReg_wa;
    end
    
    
    //Instruction Memory:
    //not currently assigning or using memBusy1 or memBusy2 (which are present on diagram, and will be used for read after load hazards) 
    assign CPU_IOBUS_ADDR = memReg_aluRes;  //IO outputs
    assign CPU_IOBUS_OUT = F_rs2Mem;    //i think?
    
    logic [31:0] DOUT2;
    logic memRDEN1;
    assign memRDEN1=1;   //for now, assuming no hazards
    
    logic [2:0] wbReg_fun3; //needed by mem for combinational logic resolving output Dout2 (since this effectively happens in WB stage
    logic [31:0] wbReg_aluRes;
    logic wbReg_memRead2;
    //computing DOUT2 is combinational, so needs a pipeline register as a buffer before entering regMux
    //computing DOUT1 is always_ff, so needs to be directly mapped into decoder, regfile, and immediate gen
                                                        //stack of questionable changes: mapped MEMRDEN2 on mem instead of wb phase. in mem module reading memory[memAddr2Parse] instead of memory[memAddr2]
                                                         //actual reading happens on wb, testing on mem                               //danger below (changed to wb write to mem), effectively doing all mem actions on same cycle 
    Memory mem(.MEM_CLK(CPU_CLK), .MEM_RDEN1(notStall), .MEM_RDEN2(memReg_memRead2), .MEM_WE2(memReg_memWrite), .MEM_ADDR1(PC[15:2]), .MEM_ADDR2(memReg_aluRes), .MEM_ADDR2Parse(wbReg_aluRes), .MEM_DIN2(memReg_rs2), .MEM_SIZE(memReg_fun3[1:0]), .MEM_SIZEParse(wbReg_fun3[1:0]), .MEM_SIGN(memReg_fun3[2:2]), .MEM_SIGNParse(wbReg_fun3[2:2]), .IO_IN(CPU_IOBUS_IN), .IO_WR(CPU_IOBUS_WR), .MEM_DOUT1(DOUT1), .MEM_DOUT2(DOUT2));

    //pipeline registers
    //logic [31:0] wbReg_DOUT2;   //new pipeline value
    //logic [31:0] wbReg_aluRes; //previous pipeline values
    logic [31:0] wbReg_PC; 
    logic [1:0] wbReg_rf_wr_sel;


    
    //wbReg_regWrite, and wbReg_wa declared in decode since they both enter regFile
    always_ff@(posedge CPU_CLK) begin
        //wbReg_DOUT2 <= DOUT2;
        //this continues during a stall
//        wbReg_opcode <=memReg_opcode;
        //wbReg_aluRes <= memReg_aluRes;
        wbReg_PC <= memReg_PC;
        wbReg_regWrite <= memReg_regWrite;
        wbReg_rf_wr_sel <= memReg_rf_wr_sel;
        wbReg_wa <= memReg_wa;
        wbReg_aluRes <=memReg_aluRes;
        wbReg_fun3 <= memReg_fun3;
        wbReg_memRead2 <=memReg_memRead2;
    end
    //instruction WB:
    //local
    logic [31:0] wbPCPlusFour;
    
    assign wbPCPlusFour = wbReg_PC + 4;                             //removed, DOUT2 alr on an ff
    reg_mux REGMUX(.rf_wr_sel(wbReg_rf_wr_sel), .PC_Plus_Four(wbPCPlusFour), .DOUT2(DOUT2), .alu_res(wbReg_aluRes), .wd(wd));
    //regFile declared in decode state above
    
    //forwarding Unit
    logic [1:0] F_Sel1;
    logic [1:0] F_Sel2;
    logic [1:0] F_Sel2ALU;
    logic MEMloadInstr, WBloadInstr;    //internal, for dataForwarder
    assign MEMloadInstr= memReg_rf_wr_sel[1:0] == 2'b10;    //indicates if this instr is a load, if so may need to stall
    assign WBloadInstr= wbReg_rf_wr_sel[1:0] == 2'b10;  //indicates if this instr is a load, if so forward from DOUT2
    
    always_comb begin
        if (exReg_rs2Selected) begin    //check if forwarding value is valid for ALU
            F_Sel2ALU=F_Sel2;
        end
        else begin
            F_Sel2ALU=2'b00;
        end
    end
    dataForwardingUnit forwarder(.Wb_rdAddr(wbReg_wa), .Mem_rdAddr(memReg_wa), .Ex_rs1Addr(exReg_rs1Addr), .Ex_rs2Addr(exReg_rs2Addr), .Wb_regWrite(wbReg_regWrite), .Mem_regWrite(memReg_regWrite), .Ex_rs1_used(exReg_rs1_used), .Ex_rs2_used(exReg_rs2_used), .rs1Selected(exReg_rs1Selected), .MEMloadInstr(MEMloadInstr), .WBloadInstr(WBloadInstr), .rs1SEL(F_Sel1), .rs2SEL(F_Sel2), .notStall(notStall));
    
    rs1Mux forwardMux1(.Ex_rs1(exReg_ALUinpA), .Mem_rs1(memReg_aluRes), .Wb_rs1(wbReg_aluRes), .DOUT2(DOUT2), .rs1Sel(F_Sel1), .rs1(F_rs1));
    rs2Mux forwardMux2ALU(.Ex_rs2(exReg_ALUinpB), .Mem_rs2(memReg_aluRes), .Wb_rs2(wbReg_aluRes), .DOUT2(DOUT2), .rs2Sel(F_Sel2ALU), .rs2(F_rs2ALU));
    rs2Mux forwardMux2Mem(.Ex_rs2(exReg_ALUinpB), .Mem_rs2(memReg_aluRes), .Wb_rs2(wbReg_aluRes), .DOUT2(DOUT2), .rs2Sel(F_Sel2), .rs2(F_rs2Mem));

    //rs2MemMux forwardMem2
    

    
    //what want:
    //if rs2 availabe: (
    //  memReg_rs2 = exReg_rs2
    //if rs2 in processing:
    //  memReg_rs2= ?
    
endmodule


