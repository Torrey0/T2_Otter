`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/15/2024 11:41:22 AM
// Design Name: 
// Module Name: Branch_COND_GEN
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


module BRANCH_COND_GEN(
    input [31:0] rs1,
    input [31:0] rs2,
    input [2:0] ir12,
    input [6:0] ir0,
    output logic [2:0] pcSource
    /*
    output br_eq,
    output br_lt,
    output br_ltu
    */
    );
        assign br_eq = rs1==rs2;
        assign br_lt = $signed(rs1)<$signed(rs2);
        assign br_ltu = rs1 < rs2;
        
        always_comb begin
        pcSource = 0;
        case (ir0)
            7'b1100111: begin   //third (of 3) opcodes for I-Type, specifically identifies jalr
                pcSource = 1; //select jalr  
            end
            
            7'b1100011: begin   //B-Type instructions
                //branch types dont use alu
                pcSource=2; //select branch only if the condition is true
                case(ir12[14:13])
                    //3rd bit of ir12
                    2'b00:   //br_eq and bne  
                        pcSource[1]=br_eq^ir12[12];
                    2'b10:  //br_lt and bge
                        pcSource[1]=br_lt^ir12[12];
                    default:    //fpr bltu and bgeu: 11.
                        pcSource[1]=br_ltu^ir12[12];
                 endcase
             end
            
            7'b1101111: begin//not using alu
                pcSource = 3; //select jal
            end
            
            default: begin
                pcSource = 0; //default to pc+4
            end 
            
        endcase
    end
           
endmodule
