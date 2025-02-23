`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2025 10:14:29 PM
// Design Name: 
// Module Name: dataForwardingUnit
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


module dataForwardingUnit(
    input [4:0] Wb_rdAddr,
    input [4:0] Mem_rdAddr,
    input [4:0] Ex_rs1Addr,
    input [4:0] Ex_rs2Addr,
    input Wb_regWrite,
    input Mem_regWrite,
    input Ex_rs1_used, Ex_rs2_used,    //from ex register, the instruction to which we are forwarding for one above
    input rs1Selected, 
    input MEMloadInstr,    // from wb register, we only have the chance to meaningfully forward a loadInstr from wb
    input WBloadInstr,
    output logic [1:0] rs1SEL,  //0=nothing, 1=forward from ALU, 2=forward from Memory
    output logic [1:0] rs2SEL,
    output logic notStall
    );
    
    always_comb begin
        notStall=1;
        //forwarding rs1
        //if we get a load, should forward from memory, this relies on writes being stalled to be correct
        if (Mem_regWrite && (Ex_rs1_used) && rs1Selected&& (Mem_rdAddr==Ex_rs1Addr)) begin  //EX hazard
            rs1SEL=2'b10;   //forward ALU output (from Mem buffer)
            if(MEMloadInstr) begin //if we are trying to forward ALU out, but have a load instruction, we need to stall
                notStall=0;
            end
        end
        else if (Wb_regWrite && (Ex_rs1_used) && rs1Selected && (Wb_rdAddr==Ex_rs1Addr)) begin
            rs1SEL[0]=1'b1;   //forward from WB buffer
            rs1SEL[1]=WBloadInstr;    //instead, forward directly from mem output
        end
        else begin
            rs1SEL=2'b00;
        end
        
        //forwarding rs2
         if (Mem_regWrite && (Ex_rs2_used) && Ex_rs2Addr!=0 && (Mem_rdAddr==Ex_rs2Addr)) begin  //EX hazard
            rs2SEL=2'b10;   //forward ALU output (from Mem buffer)
            if(MEMloadInstr) begin //if we are trying to forward ALU out, but have a load instruction, we need to stall
                notStall=0;
            end
        end
        else if (Wb_regWrite && (Ex_rs2_used) && Ex_rs2Addr!=0 && (Wb_rdAddr==Ex_rs2Addr)) begin
            rs2SEL[0]=1'b1;   //forward from WB buffer
            rs2SEL[1]=WBloadInstr;    //instead, forward directly from mem output
        end
        else begin
            rs2SEL=2'b00;
        end
        
    end 
    
endmodule
