`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/12/2024 09:29:48 AM
// Design Name: 
// Module Name: IMMED_GEN
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


module IMMED_GEN(
    input [31:7] Instruction,
    output [31:0] U_TYPE, I_TYPE, S_TYPE, J_TYPE, B_TYPE
);

//Each of these outputs is created by taking apart, and concatenating different parts of the instruction input
assign U_TYPE = {Instruction[31:12],12'b0};
assign I_TYPE = {{21{Instruction[31]}}, Instruction[30:20]};
assign S_TYPE = {{21{Instruction[31]}},Instruction[30:25],Instruction[11:7]};
assign B_TYPE = {{20{Instruction[31]}},Instruction[7],Instruction[30:25],Instruction[11:8],1'b0};
assign J_TYPE = {{12{Instruction[31]}},Instruction[19:12],Instruction[20],Instruction[30:21],1'b0};


endmodule