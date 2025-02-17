`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2025 11:04:03 PM
// Design Name: 
// Module Name: rs2Mux
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


module rs2Mux(
    input [31:0] Ex_rs2,
    input [31:0] Mem_rs2,
    input [31:0] Wb_rs2,
    input [31:0] DOUT2,
    input [1:0] rs2Sel,
    output logic [31:0] rs2
    );
    
        always_comb begin
        case(rs2Sel)
            2'b10:
                rs2=Mem_rs2;
            2'b01:
                rs2=Wb_rs2;
            2'b11:
                rs2=DOUT2;
            default:
                rs2=Ex_rs2;
            endcase
    end
endmodule
