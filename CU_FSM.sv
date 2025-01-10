`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/16/2024 01:07:59 PM
// Design Name: 
// Module Name: CU_FSM
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



module CU_FSM(
    input clk,
    input RST,
    input INTR,   
    input [6:0] opcode,
    input [14:12] func3,    //not yet being used? ask when will I need this monday
    output logic PCWrite,   //change the PC
    output logic regWrite,  //write to a regiser
    output logic memWE2,    //write to memory
    output logic memRDEN1,  //read an instruction
    output logic memRDEN2,  //read data
    output logic reset,      //start back at PC=0
    output logic mret_exec,   //will be used in future for interrupts
    output logic int_taken,   //will be used in future for interrupts
    output logic csr_WE       //will be used in future for interrupts
    );
    typedef enum {
        INIT,
        FETCH,
        EXECUTE,
        WRITEBACK,
        INTERRUPT
        } State;
        
        State ps,ns;
        
        always_ff @(posedge clk) begin
            if(RST)
                ps<=INIT;
            else
                ps<= ns;
        end
        
        always_comb begin
            PCWrite =1'b0;  //by default, everything off            
            reset = 1'b0;
            regWrite = 1'b0;
            memWE2 = 1'b0;
            memRDEN1 = 1'b0;
            memRDEN2 = 1'b0;
            csr_WE= 1'b0;
            int_taken = 1'b0;
            mret_exec = 1'b0;
            
            case (ps)
                INIT: begin 
                    reset = 1'b1;   //restarts the program
                    ns = FETCH;
                end
                FETCH: begin
                    ns = EXECUTE;
                    memRDEN1=1'b1;  //always high here, we need to aquire the instruction.                           
                end
                EXECUTE: begin
                    if (opcode==7'b0000011) //we need to load
                        ns=WRITEBACK; 
                    else if (INTR)  //we are being interrupted
                        ns=INTERRUPT;                       
                    else
                        ns=FETCH;   //continue to fetch otherwise
                                                              
                    PCWrite=1'b1;   //always wright to PC here, except for loads
                    case (opcode)
                         
                        7'b0000011: begin   //I-Type Load instructions
                            //ns=WRITEBACK;
                            PCWrite=1'b0;
                            memRDEN2=1'b1;  //if we are doing a memoery load, we need to have read enabled, otherwise no. only ever high here
                        end
                        7'b0100011: begin   //S-Types, for storing
                            memWE2=1'b1;
                        end
                        7'b1100011:   //B-Types. we only want to modify the PC in event of a branch, nothing else
                            PCWrite=1'b1;   //can delete?
                        7'b1110011: begin // interrupt related instructions
                            //alu defaults to being written in reg file
                            case (func3)
                                3'b000: begin  //mret. note: mret is specified using all 32 bits. its possible that other instructions that may want to be implemented in the future overlapp with this, in which case the case setup needs to be changed                           
                                mret_exec=1;
                                end
                                default: begin //for csrrc, csrrs and csrrw
                                    regWrite=1; //write to the CSR_RD to register
                                    csr_WE=1;    //write alu result to CSR
                                end              
                            endcase                  
                        end 
                        default: begin  //works for all "computation" instructions: R-Type, non-storing I types, including JALR, U-type instructions (lui and auipc), and J-Type (jal)
                            regWrite=1'b1; //write computation back to reg file
                        end
                        
                    endcase
                end
                WRITEBACK: begin  //WRITEBACK
                    if (INTR)
                        ns=INTERRUPT;
                    else
                        ns=FETCH;
                    PCWrite=1'b1;
                    regWrite=1'b1;                                      
                end
                default: begin  //INTERRUPT
                    PCWrite=1;
                    int_taken=1;
                    ns=FETCH;                
                end
            endcase
        end
endmodule
