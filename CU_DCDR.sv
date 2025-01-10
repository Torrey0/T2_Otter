`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/16/2024 01:07:59 PM
// Design Name: 
// Module Name: CU_DCDR
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


module CU_DCDR(
    //segments of the instruction for deciding what it is
    input [6:0] ir0,
    input [14:12] ir12,
    input ir30,
    
    input int_taken,  //will be used in future for interupts. not anymore...
    input br_eq,
    input br_lt,
    input br_ltu,
    output logic [3:0] alu_fun,     //{bit30, func code}= alu operation for most (maybe all?) instructions that use ALU
    output logic [1:0] alu_srcA,    //0=rs1, 1=U-Type, 2=~rs1
    output logic [2:0] alu_srcB,    //0=rs2, 1=I_type. 2=S_Type, 3=PC, 4=interrupt stuff
    output logic [2:0] pcSource,    //0=PC+4, 1=jalr, 2=branch, 3=jal, 4^5=interrupt stuff (in future)
    output logic [1:0] rf_wr_sel    //writeback value from alu =3. loads=2. interrupts=1. linking current location=0
    );
    
    always_comb begin
        alu_fun=0;  //default values are for those used by R-Type instructions, and adding
        alu_srcA=0;
        alu_srcB=0;
        pcSource=0;
        rf_wr_sel=3;
        case (ir0)
            7'b0110011: begin   //R-Type Instruction                    
                 //defualt values for R-Type same as deafault values above
                 alu_fun={ir30,ir12};//this is the correct code for add, and, or, sll, slt, sltu, sra, srl, sub, xor           
                end
                
            7'b0010011: begin   //first (of 3) op codes for I type
                alu_srcB=1; // defualt values for I type use I-type immediate
                case(ir12)
                    3'b101: 
                        alu_fun={ir30,ir12}; //this is correct for srai and srli
                    default:
                        alu_fun={1'b0,ir12};  //this is correct for addi, andi, ori, slli, sltiu, xor
                endcase
            end
            
            7'b0000011: begin   //second (of 3) opcodes for I-type. load instructions
                //differentiation for sign, byte, halfword, and word taken off instruction, assigned directly into memory already.
                //alu defaults to add, and getting used
                alu_srcB=1;     //use I type immediate
                rf_wr_sel=2;    //select output from memory to go into register file
            end
            7'b1100111: begin   //third (of 3) opcodes for I-Type, specifically identifies jalr
                //alu_fun defaults to add. and taking rs1
                alu_srcB=3'b001; //use i type immediate
                pcSource=1; //select jalr  
                rf_wr_sel=0; //save the location of instruction after jalr (links)             
            end
            
            7'b0100011: begin   //S-Type instructions. storing instructions
                //differentiation for sign, byte, halfword, and word taken off instruction, assigned directly into memory already.                
                //alu defaults to adding rs1
                alu_srcB=3'b010;    //use S type immediate
            
            end
            
            7'b1100011: begin   //B-Type instructions
                //branch types dont use alu
                pcSource=2; //select branch only if the condition is true
                //rf_wr_sel=3;
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
            7'b1110011: begin // interrupt related instructions
                 alu_srcB=4;    //default to using csr_RD
                 rf_wr_sel=1; //write csr RD value into reg file
                case (ir12[14:12])
                    3'b001: begin    //csrrw 
                        //defaults to using rs1
                        alu_fun=4'b1001;    //lui copy
                    end
                    3'b011: begin   //csrrc
                        alu_srcA=2;         //use ~rs1
                        alu_fun=4'b0111;        //and
                    end
                    3'b010: begin   //csrrs
                        alu_fun=4'b0110;         //or
                    end
                    default: begin  //mret=000. note: mret is specified using all 32 bits. its possible that other instructions that may want to be implemented in the future overlapp with this, in which case the case setup needs to be changed                           
                        pcSource=5; //set pc to mepc
                    end              
                endcase   
            end
            //U and J type instructions start here, these guys aren't very "groupable" (since there are few of them)
            7'b0110111: begin   //lui
                alu_fun=4'b1001;   //lui-copy
                alu_srcA=2'b01;    //select U type immediate
                //other default values are sufficient
            end
                
            7'b0010111: begin   //auipc
                //alu defaults to add, and getting used for reg file.
                alu_srcA=2'b01;
                alu_srcB=3'b011;
                end
            default: begin  //jal: 1101111
                //not using alu
                pcSource=3; //select jal
                rf_wr_sel=0;    //link, saving the value to reg file
            end
        endcase
        if(int_taken)   //override whatever we have set for pcSource if we are taking an interrupt (jump to mtvec
            pcSource=4;
    end
           
endmodule
