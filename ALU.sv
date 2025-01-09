`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/19/2024 05:17:52 PM
// Design Name: 
// Module Name: alu
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


 module ALU(

    input [31:0] srcA,
    input [31:0] srcB,
    input [3:0] alu_fun, 
    output reg [31:0] result
    );
    
    always @ (*)
    begin
      case(alu_fun)
            4'b0000: //Add
                result = $signed(srcA) + $signed(srcB); 
            4'b1000: //Sub 
                result = $signed(srcA) - $signed(srcB); 
            4'b0110: //Or
                result = srcA | srcB;
            4'b0111: //And 
                result = srcA & srcB;
            4'b0100: //Xor 
                result = srcA^srcB;
            4'b0101: //Srl : Shift Right Logical
                result = srcA >> srcB[4:0];
            4'b0001: //Sll: Shift Left Logical
                result = srcA << srcB[4:0];
            4'b1101: //Sra: Shift Right Arithmetic
                result = $signed(srcA) >>> srcB[4:0]; 
            4'b0010: //Slt 
                result = $signed(srcA) < $signed(srcB) ? 1 : 0;
                
            4'b0011: //Sltu 
                result = srcA < srcB ? 1: 0;
            4'b1001: //Lui
                result = srcA;         
                
    default: 
        result = 32'hdead_beef;
    endcase
    end
endmodule
