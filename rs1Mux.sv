`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2025 11:04:03 PM
// Design Name: 
// Module Name: rs1Mux
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


module rs1Mux(
    input [31:0] Ex_rs1,
    input [31:0] Mem_rs1,
    input [31:0] Wb_rs1,
    input [31:0] DOUT2, //for load instructions
    input [1:0] rs1Sel,
    output logic [31:0] rs1
    );
    
    always_comb begin
        case(rs1Sel)
            2'b10:
                rs1=Mem_rs1;
            2'b01:
                rs1=Wb_rs1;
            2'b11:
                rs1=DOUT2;
            default:
                rs1=Ex_rs1;
            endcase
    end
endmodule
