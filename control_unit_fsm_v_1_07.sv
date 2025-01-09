`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Ratner Surf Designs
// Engineer: James Ratner
// 
// Create Date: 01/07/2020 09:12:54 PM
// Design Name: 
// Module Name: top_level
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Control Unit Template/Starter File for RISC-V OTTER
//
//     //- instantiation template 
//     CU_FSM my_fsm(
//        .intr     (xxxx),
//        .clk      (xxxx),
//        .RST      (xxxx),
//        .opcode   (xxxx),   // ir[6:0]
//        .PC_WE    (xxxx),
//        .RF_WE    (xxxx),
//        .memWE2   (xxxx),
//        .memRDEN1 (xxxx),
//        .memRDEN2 (xxxx),
//        .reset    (xxxx)   );
//   
// Dependencies: 
// 
// Revision  History:
// Revision 1.00 - File Created - 02-01-2020 (from other people's files)
//          1.01 - (02-08-2020) switched states to enum type
//          1.02 - (02-25-2020) made PS assignment blocking
//                              made rst output asynchronous
//          1.03 - (04-24-2020) added "init" state to FSM
//                              changed rst to reset
//          1.04 - (04-29-2020) removed typos to allow synthesis
//          1.05 - (10-14-2020) fixed instantiation comment (thanks AF)
//          1.06 - (12-10-2020) cleared most outputs, added commentes
//          1.07 - (12-27-2023) changed signal names 
// 
//////////////////////////////////////////////////////////////////////////////////

module CU_FSM(
    input intr,
    input clk,
    input RST,
    input [6:0] opcode,     // ir[6:0]
    input [2:0] func3,      // ir[14:12]
    output logic PC_WE,
    output logic RF_WE,
    output logic memWE2,
    output logic memRDEN1,
    output logic memRDEN2,
    output logic reset,
    output logic csr_WE,
    output logic int_taken,
    output logic mret_exec
  );
    
    typedef  enum logic [2:0] {
       st_INIT,
	   st_FET,
       st_EX,
       st_WB,
       interrupt
    }  state_type; 
    state_type  NS,PS; 
      
    //- datatypes for RISC-V opcode types
    typedef enum logic [6:0] {
        LUI    = 7'b0110111,
        AUIPC  = 7'b0010111,
        JAL    = 7'b1101111,
        JALR   = 7'b1100111,
        BRANCH = 7'b1100011,
        LOAD   = 7'b0000011,
        STORE  = 7'b0100011,
        OP_IMM = 7'b0010011,
        OP_RG3 = 7'b0110011,
        INTR   = 7'b1110011
    } opcode_t;
    
	opcode_t OPCODE;    //- symbolic names for instruction opcodes
     
	assign OPCODE = opcode_t'(opcode); //- Cast input as enum 
	 

	//- state registers (PS)
	always @ (posedge clk)  
        if (RST == 1)
            PS <= st_INIT;
        else
            PS <= NS;

    always_comb
    begin              
        //- schedule all outputs to avoid latch
        PC_WE = 1'b0;    RF_WE = 1'b0;    reset = 1'b0;  
		memWE2 = 1'b0;     memRDEN1 = 1'b0;    memRDEN2 = 1'b0;
                   
        case (PS)

            st_INIT: //waiting state  
            begin
                reset = 1'b1;                    
                NS = st_FET; 
            end

            st_FET: //waiting state  
            begin
                memRDEN1 = 1'b1;                    
                NS = st_EX; 
                int_taken = 1'b0;
            end
              
            st_EX: //decode + execute
            begin
                PC_WE = 1'b1;
                memRDEN1 = 1'b0; // No need to read instruction after fetching
                csr_WE = 1'b0;
                int_taken = 1'b0;
                mret_exec = 1'b0;
				case (OPCODE)
				    LUI: 
					   begin
                          RF_WE = 1'b1;					      
					      NS = st_FET;
					   end
					  
					AUIPC:
					   begin
					   RF_WE = 1'b1;
					   NS = st_FET;
					   end
					   
					JAL: 
					   begin
					      RF_WE = 1'b1; // link
					      NS = st_FET;
					   end
					   
					JALR:
					   begin
					      RF_WE = 1'b1; // linking register
					      NS = st_FET;
					   end
					   
					BRANCH:
					   begin
					      NS = st_FET;
					   end   
					
				    LOAD: 
                       begin
                          PC_WE =1'b0; // Need to handle WB
                          RF_WE = 1'b0; // Don't write
                          memRDEN2 = 1'b1; // Read
                          NS = st_WB;
                       end
                    
					STORE: 
                       begin
                          RF_WE = 1'b0; // Storing in memory
                          memWE2 = 1'b1; // Write memory
                          
                          NS = st_FET;
                       end
                       
					OP_IMM:  // Immediate math operations 
					   begin 
					      RF_WE = 1'b1;	
					      NS = st_FET;
					   end
					   
					OP_RG3: // R-type instructions
					   begin
					       RF_WE = 1'b1;
					       NS = st_FET;
					   end
					   
					INTR: // interrupt instructions
					   begin
					       case (func3)
					           3'b000: // mret
                               begin
                                    csr_WE = 0;
                                    mret_exec = 1;
                                    PC_WE = 1;
                               end
                               
                               default: // csrrw, csrrc, csrrs
                               begin
                                    csr_WE = 1;
                                    mret_exec = 0;
                                    RF_WE = 1;
                               end
                           endcase
					       NS = st_FET;
					   end
					 
                    default:  
					   begin 
					      NS = st_FET;
					   end
					
                endcase
                
                if (intr && NS != st_WB) 
                    begin
                        NS = interrupt;
                    end
            end
               
            st_WB:
            begin
               PC_WE = 1'b1; // Increment PC
               RF_WE = 1'b1; // Load data
               memRDEN2 = 1'b0;
               
               NS = st_FET;
               if (intr) 
                    begin
                        NS = interrupt;
                    end
            end
            
            interrupt:
            begin
                csr_WE = 0;
                int_taken = 1;
                PC_WE = 1;
                NS = st_FET;
            end
 
            default: NS = st_FET;
           
        endcase //- case statement for FSM states
    end
           
endmodule
