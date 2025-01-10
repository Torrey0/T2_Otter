`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/17/2024 02:49:05 PM
// Design Name: 
// Module Name: srcA_mux
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


module srcA_mux(
    input [1:0] alu_srcA,
    input [31:0] rs1,
    input [31:0] U_Type,
    output logic [31:0] srcA
    );
    logic [31:0] rs1_Inverted;
    assign rs1_Inverted=~rs1;
    
    always_comb begin
        case(alu_srcA)
            2'b00:
                srcA=rs1;
            2'b01:
                srcA=U_Type;
            default:
                srcA=rs1_Inverted;
            endcase
    end
endmodule
