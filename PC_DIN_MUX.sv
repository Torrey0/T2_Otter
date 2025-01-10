`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/01/2024 09:46:20 AM
// Design Name: 
// Module Name: PC_DIN_MUX
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


module PC_DIN_MUX(
    input [31:0] PLUS_FOUR,
    input [31:0] JALR,
    input [31:0] BRANCH,
    input [31:0] JAL,
    input [31:0] MTVEC,
    input [31:0] MEPC,
    input [2:0] SEL,
    output logic [31:0] PC_DIN
    
    );
    
    always_comb begin
    case (SEL)
        3'b000: PC_DIN=PLUS_FOUR;   //increments to next line
        //other options for jumps, branches, or interrupts
        3'b001: PC_DIN=JALR;
        3'b010: PC_DIN=BRANCH;
        3'b011: PC_DIN=JAL;
        3'b100: PC_DIN=MTVEC;
        3'b101: PC_DIN=MEPC;
        //
        default: PC_DIN=PLUS_FOUR;  //incrementing to the next line is chosen as the defualt behavior in case SEL is not one of the options.
   endcase
   end
    
endmodule
