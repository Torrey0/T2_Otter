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
    logic notStall;
    logic notStallRAL; //assigned in code for forwarding Unit, controls PCWrite, and pipeline register progression
    logic cacheMissStall;    //cacheMissStall causes stall makes us hold PC to the same value until we correclty read it and get DOUT1. Similar to notStall, stalls for instruction memory rather than data memory
    logic dataMemStall; //notStall's twin
    assign notStall=notStallRAL && (~dataMemStall); //0,- =>notStall=0. -,1 =>notStall=0. 1,0 =>notStall=1
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

    assign PCWrite=(notStall && (!cacheMissStall || (branchTaken) ) );    //dont update the PC while we are stalling for new DOUT1
                                                    //^^^^ if cacheMissStall= 1 and branchtaken=1. we want to take the branch anyway, unless of course, notStall=0
    assign PCPlusFour=PC+4;
    PC_DIN_MUX PCMUX(.SEL(pcSource), .JALR(jalr), .BRANCH(branch), .JAL(jal), .PLUS_FOUR(PCPlusFour), .PC_DIN(PC_DIN));
    PC pc(.PC_DIN(PC_DIN), .PC_RST(CPU_RST), .PC_WE(PCWrite), .CLK(CPU_CLK), .PC_COUNT(PC));
    
   //pipeline registers. these are the registers the decode part of computer needs to read on the next cycle
    logic [31:0] deReg_IR, deReg_PC;
    //
    always_comb begin
        if(branchTaken || branchTakenPropogated) begin   //perform nop if branch taken. Also just read no-ops until the correct DOUT1 is read for cache misses
            deReg_IR=8'h00000013;   //nop instruction
        end else begin
            deReg_IR=DOUT1;
        end
    end
    always_ff @(posedge CPU_CLK) begin
        if  (notStall) begin        //only need to prevent PC propogation on data hazard stall. on branchTaken we run a nop, which doesnt use PC anywar
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
    
    //need to know where and if rs1 and rs2 are used for forwarding and hazard detection
    assign opcode = opcode_t'(deReg_IR[6:0]);
    assign rs1_used=    deReg_IR[19:15] != 0        //if rs1 is used at all
                                && opcode != LUI
                                && opcode != AUIPC
                                && opcode != JAL;
    assign rs2_used=   deReg_IR[24:20] !=0 && (opcode==OP || opcode==STORE || opcode==BRANCH);  //if rs2 is used at all
    assign rs1Selected=muxASel==2'b00;  //if rs1 is used by the ALU
    assign rs2Selected=muxBSel==3'b000; //if rs2 is used by the ALU
    //
    
    CU_DCDR decoder(.ir0(deReg_IR[6:0]), .ir12(deReg_IR[14:12]), .ir30(deReg_IR[30]), .alu_fun(alu_fun), .alu_srcA(muxASel), .alu_srcB(muxBSel), .rf_wr_sel(rf_wr_sel), .regWrite(regWrite), .memWrite(memWrite), .memRead2(memRead2));
    IMMED_GEN immy(.Instruction(deReg_IR[31:7]), .U_TYPE(U_Type), .I_TYPE(I_Type), .S_TYPE(S_Type), .J_TYPE(J_Type), .B_TYPE(B_Type));
    
    RegFile regFile(.clk(CPU_CLK), .en(wbReg_regWrite), .adr1(deReg_IR[19:15]), .adr2(deReg_IR[24:20]), .w_adr(wbReg_wa), .w_data(wd), .rs1(rs1),.rs2(rs2));
    
    srcA_mux muxA(.alu_srcA(muxASel), .rs1(rs1), .U_Type(U_Type), .srcA(srcA));
    srcB_mux muxB(.alu_srcB(muxBSel), .rs2(rs2), .I_Type(I_Type), .S_Type(S_Type), .PC(deReg_PC), .srcB(srcB));
    
    //ex pipeline registers 
    logic [31:0] exReg_PC; //previous pipeline values
    logic [6:0] exReg_opcode;   //trimming IR
    logic [4:0] exReg_wa;
    logic [2:0] exReg_fun3;
    logic [4:0] exReg_rs1Addr;  //needed for hazard detection
    logic [4:0] exReg_rs2Addr;
    logic [31:0] exReg_ALUinpA, exReg_ALUinpB;
    logic [31:0] exReg_rs2;
    logic exReg_regWrite, exReg_memWrite, exReg_memRead2;
    logic [3:0] exReg_aluFun;
    logic [1:0] exReg_rf_wr_sel;
    logic [31:0] exReg_J_Type, exReg_B_Type, exReg_I_Type;
    logic exReg_rs1_used, exReg_rs2_used;
    logic exReg_rs1Selected;
    logic exReg_rs2Selected;
    //
    //testing if wiating for DOUT1 mem reads to finish fixes the issue, and allows running on hardware.
    always_ff @(posedge CPU_CLK) begin
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
            exReg_ALUinpA <= srcA;  //ALU inputs
            exReg_ALUinpB <= srcB;
            exReg_rs2 <= rs2;
            exReg_aluFun <= alu_fun;
            exReg_J_Type <= J_Type; //generated immediates
            exReg_B_Type <= B_Type;
            exReg_I_Type <= I_Type;
            exReg_regWrite <= regWrite; //other control decoder outputs
            exReg_memWrite <= memWrite;
            exReg_memRead2 <= memRead2;
        end        
    end    
   
    //Instruction Execute
    logic [31:0] aluRes;
    logic [31:0] F_rs1, F_rs2ALU, F_rs2Mem;     //F_ is a value forwarded from dataForwarding Unit
    //all combinational:
    BRANCH_COND_GEN bcd(.rs1(F_rs1), .rs2(F_rs2ALU), .pcSource(pcSource), .branchTaken(branchTaken), .ir12(exReg_fun3), .ir0(exReg_opcode));    //PCSource =SEL for pcMux above
    BRANCH_ADDR_GEN bad(.J_TYPE_IMM(exReg_J_Type), .B_TYPE_IMM(exReg_B_Type), .I_TYPE_IMM(exReg_I_Type), .rs1(F_rs1), .PC(exReg_PC), .branch(branch), .jal(jal), .jalr(jalr));  //these outputs used in PC Mux above
    
    ALU MathYay(.srcA(F_rs1), .srcB(F_rs2ALU), .alu_fun(exReg_aluFun), .alu_result(aluRes));

    //pipeline registers:
    logic [31:0] memReg_aluRes; //new pipeline values
    logic [31:0] memReg_rs2;    //previous pipeline values
    logic [31:0] memReg_PC; 
    logic memReg_regWrite, memReg_memWrite, memReg_memRead2;    
    logic [1:0] memReg_rf_wr_sel;
    logic [2:0] memReg_fun3; //trimming IR_Reg, no longer need the entire instruction
    logic [4:0] memReg_wa;
    
    always_ff@(posedge CPU_CLK) begin
            if (~dataMemStall) begin    //hold data values here for reading on data mem stall
                memReg_aluRes <= aluRes;    //new value
                
                memReg_PC <= exReg_PC;  //previous values
                memReg_rs2 <= exReg_rs2;    //forwarded value of rs2
                if (notStallRAL) begin  //on RAL stall let values continue, signaling stall is over
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
    end
        
    //Instruction Memory:
    //note: memory only supports writing to MMIO at the moment, since it is not necessary to read MMIO to test test_all, and doing so would require modyfing the wrapper, which we probably wont want to do until final lab if possible.
    //not currently assigning or using memBusy1 or memBusy2 (which are present on the 233 diagram, and may need to be used to induce a stall, or when a stall is necessary when writing or reading MMIO
    assign CPU_IOBUS_ADDR = memReg_aluRes;  //IO outputs
    assign CPU_IOBUS_OUT = F_rs2Mem;    //output of CPU is the input to memory cuz we treating cpu external like another memory
    
    logic [31:0] DOUT2; //output data of memory

    //the wbRegisters are used by the memory for parsing DOUT2. For example, the output of memory will need to be adjusted depending on signed/unsigned, lw, lh, lb.
    logic [2:0] wbReg_fun3; 
    logic [31:0] wbReg_aluRes;
    //computing DOUT1 and DOUT2 is always_ff, so needs to be directly mapped into wbReg,and co.
    Memory mem(.MEM_CLK(CPU_CLK), .MEM_RDEN1(notStall), .MEM_RDEN2(memReg_memRead2), .MEM_WE2(memReg_memWrite), .IO_IN(CPU_IOBUS_IN), .IO_WR(CPU_IOBUS_WR), //flags inducing memory actions
                .MEM_ADDR1(PC[15:2]), .MEM_ADDR2(memReg_aluRes), .MEM_DIN2(F_rs2Mem),   .MEM_DOUT1(DOUT1), .MEM_DOUT2(DOUT2),                   //key inputs (and addresses) and outputs
                .MEM_SIZE(memReg_fun3[1:0]), .MEM_SIZEParse(wbReg_fun3[1:0]), .MEM_SIGNParse(wbReg_fun3[2:2]),.MEM_ADDR2Parse(wbReg_aluRes),    //parsing information
                .cacheMissStall(cacheMissStall), .dataMemStall(dataMemStall), .branchTaken(branchTaken), .MEM_RST(CPU_RST)                      //cache related control flow input/outputs
    );     

    //other wb registers
    logic [31:0] wbReg_PC; 
    logic [1:0] wbReg_rf_wr_sel;
    
    //wbReg_regWrite, and wbReg_wa declared in decode since they both enter regFile
    always_ff@(posedge CPU_CLK) begin
        if(~dataMemStall) begin
            wbReg_PC <= memReg_PC;
            wbReg_regWrite <= memReg_regWrite;
            wbReg_rf_wr_sel <= memReg_rf_wr_sel;
            wbReg_wa <= memReg_wa;
            wbReg_aluRes <=memReg_aluRes;
            wbReg_fun3 <= memReg_fun3;
        end
    end
    
    //instruction WB:
    //local
    logic [31:0] wbPCPlusFour;
    
    assign wbPCPlusFour = wbReg_PC + 4;                            
    reg_mux REGMUX(.rf_wr_sel(wbReg_rf_wr_sel), .PC_Plus_Four(wbPCPlusFour), .DOUT2(DOUT2), .alu_res(wbReg_aluRes), .wd(wd));
    //regFile declared in decode state above
    
    //forwarding Unit
    logic [1:0] F_Sel1;
    logic [1:0] F_Sel2;
    logic [1:0] F_Sel2Delayed;  //buffer the forwarding selection one behind for memory (we want to forward on mem instead of ex for this
    logic [31:0] aluResWbDelayed;
    logic [1:0] F_Sel2ALU;
    logic MEMloadInstr, WBloadInstr;    //internal, for dataForwarder
    assign MEMloadInstr= memReg_rf_wr_sel[1:0] == 2'b10;    //indicates if this instr is a load, if so may need to stall
    assign WBloadInstr= wbReg_rf_wr_sel[1:0] == 2'b10;  //indicates if this instr is a load, if so forward from DOUT2
    
    always_comb begin
        if (exReg_rs2Selected) begin    //check if forwarding value is valid specifically for ALU
            F_Sel2ALU=F_Sel2;
        end
        else begin
            F_Sel2ALU=2'b00;
        end
    end
    always_ff@(posedge CPU_CLK) begin
        if((~dataMemStall)) begin
            F_Sel2Delayed <= F_Sel2;
            aluResWbDelayed <= wbReg_aluRes;    //we need to buffer the aluRes by 1 for forwarding 2 instr above to memory. This value has already been written back 1 cycle before now, but we didnt pull its value from regFile 2 cycles ago when we accessed rs2
        end
    end
    
    //forwarding Unit, and forwarding related muxes
    dataForwardingUnit forwarder(.Wb_rdAddr(wbReg_wa), .Mem_rdAddr(memReg_wa), .Ex_rs1Addr(exReg_rs1Addr), .Ex_rs2Addr(exReg_rs2Addr), .Wb_regWrite(wbReg_regWrite), .Mem_regWrite(memReg_regWrite), .Ex_rs1_used(exReg_rs1_used), .Ex_rs2_used(exReg_rs2_used), 
                                 .rs1Selected(exReg_rs1Selected), .MEMloadInstr(MEMloadInstr), .WBloadInstr(WBloadInstr), .rs1SEL(F_Sel1), .rs2SEL(F_Sel2), .notStall(notStallRAL));
    
    rs1Mux forwardMux1(.Ex_rs1(exReg_ALUinpA), .Mem_rs1(memReg_aluRes), .Wb_rs1(wbReg_aluRes), .DOUT2(DOUT2), .rs1Sel(F_Sel1), .rs1(F_rs1));
    rs2Mux forwardMux2ALU(.Ex_rs2(exReg_ALUinpB), .Mem_rs2(memReg_aluRes), .Wb_rs2(wbReg_aluRes), .DOUT2(DOUT2), .rs2Sel(F_Sel2ALU), .rs2(F_rs2ALU));
 
    rs2MemMux forwardMux2Mem(.Mem_rs2(memReg_rs2), .Wb_aluResDelayed(aluResWbDelayed), .Wb_aluRes(wbReg_aluRes), .DOUT2(DOUT2), .rs2Sel(F_Sel2Delayed), .rs2(F_rs2Mem));

endmodule