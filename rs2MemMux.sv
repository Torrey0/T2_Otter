`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/22/2025 12:33:49 PM
// Design Name: 
// Module Name: rs2MemMux
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


module rs2MemMux(
    input [31:0] Mem_rs2,
    input [31:0] Wb_aluResDelayed,
    input [31:0] Wb_aluRes,
    input [31:0] DOUT2,
    input [1:0] rs2Sel,
    output logic [31:0] rs2
    );
    
        always_comb begin
        case(rs2Sel)
            2'b10:
                rs2=Wb_aluRes; //wb_rs2
            2'b01:
                rs2=Wb_aluResDelayed;  //mem_rs2
            2'b11:
                rs2=DOUT2;
            default:
                rs2=Mem_rs2;    //dont forward from ex, not necessary
            endcase
    end
endmodule

