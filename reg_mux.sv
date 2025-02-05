`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/17/2024 02:49:05 PM
// Design Name: 
// Module Name: reg_mux
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


module reg_mux(
    input [1:0] rf_wr_sel,
    input [31:0] PC_Plus_Four,
    //input [31:0] csr_RD,
    input [31:0] DOUT2,
    input [31:0] alu_res,
    output logic [31:0] wd
    );
    always_comb begin
        case(rf_wr_sel)
            2'b00:
                wd=PC_Plus_Four;
            //2'b01:
            //    wd=csr_RD;
            2'b10:
                wd=DOUT2;
            default:
                wd=alu_res;
        endcase
    end
endmodule
