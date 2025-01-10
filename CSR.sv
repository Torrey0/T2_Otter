`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/26/2024 11:37:38 AM
// Design Name: 
// Module Name: CSR
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


module CSR(
    input CLK,
    input RST,
    input mret_exec,
    input INT_TAKEN,
    input [31:20] ADDR,
    input WR_EN,
    input [31:0] PC,
    input [31:0] WD,
    //registers "stored" at CSR...
    output logic [31:0] mstatus,    //0x300
    output logic [31:0] mepc,       //0x341
    output logic [31:0] mtvec,      //0x305
    
    output logic [31:0] RD  //selected register that is being output
    );
      
    
    always_ff @(posedge CLK) begin
        if(RST) begin    //Reset registers to 0
            mepc <= 0;
            mtvec <= 0;
            mstatus <= 0;
        end else if(WR_EN) begin    //write alu result = WD to RD
            //write to CSR register specified by ADDR
            case (ADDR[31:20])
                12'h300: begin   //mstatus
                    mstatus <= WD;                    
                end
                12'h341: begin  //mepc
                    mepc <= WD;                    
                end
                default: begin    //mtvec at 0x305
                    mtvec <= WD;                    
                end
            endcase
            
        end else if (mret_exec) begin   //restore interrupt status
            mstatus[3] <= mstatus[7];
            mstatus[7] <= 0;
        end
        else if (INT_TAKEN) begin   //copy interrupt status, temporarily disable interupts for duration of this interrupt
            mepc <= PC;
            mstatus[7] <= mstatus[3];        
            mstatus[3] <= 0;    
        end
        end
         
     always_comb begin
        case (ADDR[31:20])
                12'h300:    //mstatus
                    RD = mstatus;
                12'h341:    //mepc
                    RD = mepc;
                default:    //mtvec at 0x305
                    RD = mtvec;
            endcase
    end
        
endmodule
