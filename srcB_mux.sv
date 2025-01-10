`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/17/2024 02:49:05 PM
// Design Name: 
// Module Name: srcB_mux
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


module srcB_mux(
    input [2:0] alu_srcB,
    input [31:0] rs2,
    input [31:0] I_Type,
    input [31:0] S_Type,
    input [31:0] PC,
    input [31:0] csr_RD,
    output logic [31:0] srcB
    );
    always_comb begin
        case(alu_srcB)
            3'b000:
                srcB=rs2;
            3'b001:
                srcB=I_Type;
            3'b010:
                srcB=S_Type;
            3'b011:
                srcB=PC;
            default:
                srcB=csr_RD;
        endcase
    end
endmodule
