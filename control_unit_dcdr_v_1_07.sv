`timescale 1ns / 1ps
///////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer: James Ratner
// 
// Create Date: 01/29/2019 04:56:13 PM
// Design Name: 
// Module Name: CU_DCDR
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies:
// 
// Instantiation Template:
//
// CU_DCDR my_cu_dcdr(
//   .br_eq     (xxxx), 
//   .br_lt     (xxxx), 
//   .br_ltu    (xxxx),
//   .opcode    (xxxx),    
//   .func7     (xxxx),    
//   .func3     (xxxx),    
//   .ALU_FUN   (xxxx),
//   .PC_SEL    (xxxx),
//   .srcA_SEL  (xxxx),
//   .srcB_SEL  (xxxx), 
//   .RF_SEL    (xxxx)   );
//
// 
// Revision:
// Revision 1.00 - Created (02-01-2020) - from Paul, Joseph, & Celina
//          1.01 - (02-08-2020) - removed  else's; fixed assignments
//          1.02 - (02-25-2020) - made all assignments blocking
//          1.03 - (05-12-2020) - reduced func7 to one bit
//          1.04 - (05-31-2020) - removed misleading code
//          1.05 - (12-10-2020) - added comments
//          1.06 - (02-11-2021) - fixed formatting issues
//          1.07 - (12-26-2023) - changed signal names
//
// Additional Comments:
// 
///////////////////////////////////////////////////////////////////////////

module CU_DCDR(
   input br_eq, 
   input br_lt, 
   input br_ltu,
   input int_taken,
   input [6:0] opcode,   //-  ir[6:0]
   input func7,          //-  ir[30]
   input [2:0] func3,    //-  ir[14:12] 
   output logic [3:0] ALU_FUN,
   output logic [2:0] PC_SEL,
   output logic [1:0] srcA_SEL,
   output logic [2:0] srcB_SEL, 
   output logic [1:0] RF_SEL   
   );
    
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
   opcode_t OPCODE; //- define variable of new opcode type
    
   assign OPCODE = opcode_t'(opcode); //- Cast input enum 

   //- datatype for func3Symbols tied to values
   typedef enum logic [2:0] {
        //BRANCH labels
        BEQ = 3'b000,
        BNE = 3'b001,
        BLT = 3'b100,
        BGE = 3'b101,
        BLTU = 3'b110,
        BGEU = 3'b111
   } func3_t;    
   func3_t FUNC3; //- define variable of new opcode type
    
   assign FUNC3 = func3_t'(func3); //- Cast input enum 
       
   always_comb
   begin 
      //- schedule all values to avoid latch
        PC_SEL = 3'b000;  srcB_SEL = 3'b00;     RF_SEL = 2'b00; 
      srcA_SEL = 2'b0;   ALU_FUN  = 4'b0000;
		
      case(OPCODE)
         LUI:
         begin  // LUI instruction
            ALU_FUN = 4'b1001; // LUI
            srcA_SEL = 2'b01; // u-type
            srcB_SEL = 3'b00; // rs2
            RF_SEL = 2'b11; // ALU output
         end
         
         AUIPC:
         begin
            ALU_FUN = 4'b0000; // Adding offset
            srcA_SEL = 2'b01; // u-type
            srcB_SEL = 3'b011; // PC
            RF_SEL = 2'b11; // ALU output
         end
            
			
         JAL:
         begin
            RF_SEL = 2'b00; // PC + 4
			PC_SEL = 3'b011; // jal
		 end
		 
		 JALR:
		 begin
		    RF_SEL = 2'b00; // PC + 4
		    PC_SEL = 3'b001; // jalr
		 end
		 
		 BRANCH:
		 begin
		    case (FUNC3)
		       BEQ:
		          PC_SEL = (br_eq == 1 ? 3'b010 : 3'b000);
		       BNE:
		          PC_SEL = (br_eq != 1 ? 3'b010 : 3'b000);
		       BLT:
		          PC_SEL = (br_lt == 1 ? 3'b010 : 3'b000);
		       BGE:
		          PC_SEL = (br_lt != 1 ? 3'b010 : 3'b000);
		       BLTU:
		          PC_SEL = (br_ltu == 1 ? 3'b010 : 3'b000);
		       BGEU:
		          PC_SEL = (br_ltu != 1 ? 3'b010 : 3'b000);
		       default:
		          PC_SEL = 3'b000;
		          
		    endcase
		 end
			
         LOAD: 
         begin  //
            ALU_FUN = 4'b0000; // Adding offset
            srcA_SEL = 2'b0; // rs1
            srcB_SEL = 3'b001; // i-type
            RF_SEL = 2'b10;  // load data from memory
         end
			
         STORE:
         begin
            ALU_FUN = 4'b0000; // Adding offset
            srcA_SEL = 2'b0; // rs1
            srcB_SEL = 3'b010; // S-type imm
            //RF_WE is 0 because storing
         end
			
         OP_IMM:
         begin
            srcA_SEL = 2'b0; // rs1
            srcB_SEL = 3'b001; // i-type
            RF_SEL = 2'b11; // ALU output
            
            case(FUNC3)
               3'b000: // instr: ADDI
                  ALU_FUN = 4'b0000; // add
                  
               3'b010:
                  ALU_FUN = 4'b0010; // slt
                  
               3'b011:
                  ALU_FUN = 4'b0011; // sltu
               
               3'b110:
                  ALU_FUN = 4'b0110; // or
                  
               3'b100:
                  ALU_FUN = 4'b0100; // xor
                  
               3'b111:
                  ALU_FUN = 4'b0111; // and
                  
               3'b001:
                  ALU_FUN = 4'b0001; // sll
                  
               3'b101:
                   case (func7)
                      1'b0:  
                        ALU_FUN = 4'b0101; // srl
                      1'b1:
                        ALU_FUN = 4'b1101; // sra
                      default:
                        ALU_FUN = 4'b0101;
                   endcase
               
               default:
                  ALU_FUN = 4'b0000;
             
            endcase
         end
         
         OP_RG3:
         begin
            srcA_SEL = 2'b0; // rs1
            srcB_SEL = 3'b00; // rs2
            RF_SEL = 2'b11; // ALU output
            
            case(FUNC3)
                3'b000: // ADD instruction
                    case(func7)
                        1'b0:
                            ALU_FUN = 4'b0000; // add
                        1'b1:
                            ALU_FUN = 4'b1000; // sub
                        default:
                            ALU_FUN = 4'b0000;
                    endcase
                3'b001:
                    ALU_FUN = 4'b0001; // sll
                3'b010:
                    ALU_FUN = 4'b0010; // slt
                3'b011:
                    ALU_FUN = 4'b0011; // sltu
                3'b100:
                    ALU_FUN = 4'b0100; // xor
                3'b101:
                    case (func7)
                        1'b0:
                            ALU_FUN = 4'b0101; // srl
                        1'b1:
                            ALU_FUN = 4'b1101; // sra
                        default:
                            ALU_FUN = 4'b0101;
                    endcase
                3'b110:
                    ALU_FUN = 4'b0110; // or
                3'b111:
                    ALU_FUN = 4'b0111; // and
                default:
                    ALU_FUN = 4'b0000;
            endcase
            
         end
            
         INTR:
             case (FUNC3)
                 3'b000:    // mret
                    PC_SEL = 3'b101;
                    
                 3'b001:    // csrrw
                 begin
                    RF_SEL = 1;
                    srcA_SEL = 0;
                    // Don't care about srcB
                    ALU_FUN = 4'b1001;  // copy
                 end
                 
                 3'b011:    // csrrc
                 begin
                    RF_SEL = 1;
                    srcA_SEL = 2;   // ~rs1
                    srcB_SEL = 4;   // CSR
                    ALU_FUN = 4'b0111;  // and         
                 end
                 
                 3'b010:
                 begin
                    RF_SEL = 1;
                    srcA_SEL = 0;   // rs1
                    srcB_SEL = 4;   // CSR
                    ALU_FUN = 4'b0110;  // or
                 end
                 
                 default:
                    RF_SEL = 1;
                 endcase
         
            
         default:
         begin
             PC_SEL = 3'b000; 
             srcB_SEL = 3'b000; 
             RF_SEL = 2'b00; 
             srcA_SEL = 2'b0; 
             ALU_FUN = 4'b0000;
         end
      endcase
      
      if (int_taken) PC_SEL = 4;
      
   end

endmodule