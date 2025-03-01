`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: J. Callenes, P. Hummel
//
// Create Date: 01/27/2019 08:37:11 AM
// Module Name: OTTER_mem
// Project Name: Memory for OTTER RV32I RISC-V
// Tool Versions: Xilinx Vivado 2019.2
// Description: 64k Memory, dual access read single access write. Designed to
//              purposely utilize BRAM which requires synchronous reads and write
//              ADDR1 used for Program Memory Instruction. Word addressable so it
//              must be adapted from byte addresses in connection from PC
//              ADDR2 used for data access, both internal and external memory
//              mapped IO. ADDR2 is byte addressable.
//              RDEN1 and EDEN2 are read enables for ADDR1 and ADDR2. These are
//              needed due to synchronous reading
//              MEM_SIZE used to specify reads as byte (0), half (1), or word (2)
//              MEM_SIGN used to specify unsigned (1) vs signed (0) extension
//              IO_IN is data from external IO and synchronously buffered
//
// Memory OTTER_MEMORY (
//    .MEM_CLK   (),
//    .MEM_RDEN1 (),
//    .MEM_RDEN2 (),
//    .MEM_WE2   (),
//    .MEM_ADDR1 (),
//    .MEM_ADDR2 (),
//    .MEM_DIN2  (),
//    .MEM_SIZE  (),
//    .MEM_SIGN  (),
//    .IO_IN     (),
//    .IO_WR     (),
//    .MEM_DOUT1 (),
//    .MEM_DOUT2 ()  );
//
// Revision:
// Revision 0.01 - Original by J. Callenes
// Revision 1.02 - Rewrite to simplify logic by P. Hummel
// Revision 1.03 - changed signal names, added instantiation template
// Revision 1.04 - added defualt to write case statement
// Revision 1.05 - changed MEM_WD to MEM_DIN2, changed default to save nothing
// Revision 1.06 - removed typo in instantiation template
// Revision 1.07 - remove unused wordAddr1 signal
//
//////////////////////////////////////////////////////////////////////////////////
                                                                                                                             
  module Memory (
    input MEM_CLK,
    input MEM_RDEN1,        // read enable Instruction
    input MEM_RDEN2,        // read enable data
    input MEM_WE2,          // write enable.
    input MEM_RST,
    
    input [13:0] MEM_ADDR1, // Instruction Memory word Addr (Connect to PC[15:2])
//    input logic [31:0] a,   //from imem module, for cache use
    
    input [31:0] MEM_ADDR2, // Data Memory Addr
    input [31:0] MEM_ADDR2Parse,
    input [31:0] MEM_DIN2,  // Data to save
    input [1:0] MEM_SIZE,   // 0-Byte, 1-Half, 2-Word
    input [1:0] MEM_SIZEParse,
    input MEM_SIGNParse,
    input [31:0] IO_IN,     // Data from IO
    //output ERR,           // only used for testing
    output logic IO_WR,     // IO 1-write 0-read
    
    output logic [31:0] MEM_DOUT1,  // Instruction
//    //instead, read 8 at a time
//    output logic [31:0] w0,
//    output logic [31:0] w1,
//    output logic [31:0] w2,
//    output logic [31:0] w3,
//    output logic [31:0] w4,
//    output logic [31:0] w5,
//    output logic [31:0] w6,
//    output logic [31:0] w7,
    output logic cacheMissStall,
    output logic [31:0] MEM_DOUT2 // Data
    );
    logic [13:0] wordAddr2Parse;
    logic [13:0] wordAddr2;
    logic [31:0] memReadWord, ioBuffer, memReadSized;
    logic [1:0] byteOffsetParse;
    logic [1:0] byteOffset;
    logic weAddrValid;      // active when saving (WE) to valid memory address
       
    (* rom_style="{distributed | block}" *)
    (* ram_decomp = "power" *) logic [31:0] memory [0:16383];
    
    initial begin
//        $readmemh("matMult10b10.mem", memory, 0, 16383);
        $readmemh("Test_All.mem", memory, 0, 16383);
    end

    
    assign wordAddr2 = MEM_ADDR2[15:2];
    assign byteOffset = MEM_ADDR2[1:0];     // byte offset of memory address
    assign wordAddr2Parse = MEM_ADDR2Parse[15:2];
    assign byteOffsetParse = MEM_ADDR2Parse[1:0];
    // NOT USED IN OTTER
    //Check for misalligned or out of bounds memory accesses
    //assign ERR = ((MEM_ADDR1 >= 2**ACTUAL_WIDTH)|| (MEM_ADDR2 >= 2**ACTUAL_WIDTH)
    //                || MEM_ADDR1[1:0] != 2'b0 || MEM_ADDR2[1:0] !=2'b0)? 1 : 0;
            
    // buffer the IO input for reading
    always_ff @(posedge MEM_CLK) begin
      if(MEM_RDEN2)
        ioBuffer <= IO_IN;
    end
    
 //all cache stuff for reading DOUT1:
    logic cache_hit, cache_miss, cache_update;
    logic [31:0] w0;
    logic [31:0] w1;
    logic [31:0] w2;
    logic [31:0] w3;
    logic [31:0] w4;
    logic [31:0] w5;
    logic [31:0] w6;
    logic [31:0] w7;
    //assign w's in groups of 8 words by doing this allignment
    // 0x-0000  want start
    // 0x-0001
    //0x- 0010
    // 0x-0011  <-current   incorrect start
    // 0x-0100
    // 0x-0101
    // 0x-0110
    // 0x-0111  want end
    // 0x-1000
    // 0x-1001
    // 0x-1010              incorrect end
    
    //need to fix bottom 3 here since 3= log2 (8) = log2 (blockSize)
    //in future, make sure this is consistant with cache when modifying it
    logic [13:0] MEM_ADDR1Offset8;
    assign MEM_ADDR1Offset8[13:3]= MEM_ADDR1[13:3];
    assign MEM_ADDR1Offset8[2:0]=3'b000;
//    assign w0 = memory[MEM_ADDR1];
//    assign w1 = memory[MEM_ADDR1+1];
//    assign w2 = memory[MEM_ADDR1+2];
//    assign w3 = memory[MEM_ADDR1+3];
//    assign w4 = memory[MEM_ADDR1+4];
//    assign w5 = memory[MEM_ADDR1+5];
//    assign w6 = memory[MEM_ADDR1+6];
//    assign w7 = memory[MEM_ADDR1+7];
    assign w0 = memory[MEM_ADDR1Offset8];
    assign w1 = memory[MEM_ADDR1Offset8+1];
    assign w2 = memory[MEM_ADDR1Offset8+2];
    assign w3 = memory[MEM_ADDR1Offset8+3];
    assign w4 = memory[MEM_ADDR1Offset8+4];
    assign w5 = memory[MEM_ADDR1Offset8+5];
    assign w6 = memory[MEM_ADDR1Offset8+6];
    assign w7 = memory[MEM_ADDR1Offset8+7];

    logic [31:0] CacheDOUT1;
    Cache Cache(.PC(MEM_ADDR1), .CLK(MEM_CLK), .update(cache_update), .w0(w0), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .w5(w5), .w6(w6), .w7(w7), .rd(CacheDOUT1), .hit(cache_hit), .miss(cache_miss));
    //WHERE DOES RST GO?
        logic cacheMissStallPreInstr;

    CacheFSM CacheFSM(.hit(cache_hit), .miss(cache_miss), .CLK(MEM_CLK), .RST(MEM_RST), .update(cache_update), .pc_stall(cacheMissStall));
//     always_ff @(posedge MEM_CLK) begin
//        cacheMissStall <= cacheMissStallPreInstr;
//     end
    
    // BRAM requires all reads and writes to occur synchronously
    always_ff @(posedge MEM_CLK) begin
    
      if (weAddrValid == 1) begin     // write enable and valid address space
        case({MEM_SIZE,byteOffset})
            //making wordAddr2 (on mem phase)?
            4'b0000: memory[wordAddr2][7:0]   <= MEM_DIN2[7:0];     // sb at byte offsets
            4'b0001: memory[wordAddr2][15:8]  <= MEM_DIN2[7:0];
            4'b0010: memory[wordAddr2][23:16] <= MEM_DIN2[7:0];
            4'b0011: memory[wordAddr2][31:24] <= MEM_DIN2[7:0];
            4'b0100: memory[wordAddr2][15:0]  <= MEM_DIN2[15:0];    // sh at byte offsets
            4'b0101: memory[wordAddr2][23:8]  <= MEM_DIN2[15:0];
            4'b0110: memory[wordAddr2][31:16] <= MEM_DIN2[15:0];
            4'b1000: memory[wordAddr2]        <= MEM_DIN2;          // sw
			      //default: memory[wordAddr2]      <= 32'b0   // unsupported size, byte offset
			      // removed to avoid mistakes causing memory to be zeroed.
        endcase
      end

 
      // read all data synchronously required for BRAM
      if (MEM_RDEN1)                       // need EN for extra load cycle to not change instruction
        //MEM_DOUT1 <= memory[MEM_ADDR1];
        MEM_DOUT1 <=CacheDOUT1;
        //we now read 8 words instead of 1.
      if (MEM_RDEN2)                       // Read word from memory
        memReadWord <= memory[wordAddr2];  //hmm
    end
       
    // Change the data word into sized bytes and sign extend
    always_comb begin
      case({MEM_SIGNParse,MEM_SIZEParse,byteOffsetParse})
        5'b00011: memReadSized = {{24{memReadWord[31]}},memReadWord[31:24]};  // signed byte
        5'b00010: memReadSized = {{24{memReadWord[23]}},memReadWord[23:16]};
        5'b00001: memReadSized = {{24{memReadWord[15]}},memReadWord[15:8]};
        5'b00000: memReadSized = {{24{memReadWord[7]}},memReadWord[7:0]};
                                    
        5'b00110: memReadSized = {{16{memReadWord[31]}},memReadWord[31:16]};  // signed half
        5'b00101: memReadSized = {{16{memReadWord[23]}},memReadWord[23:8]};
        5'b00100: memReadSized = {{16{memReadWord[15]}},memReadWord[15:0]};
            
        5'b01000: memReadSized = memReadWord;                   // word
               
        5'b10011: memReadSized = {24'd0,memReadWord[31:24]};    // unsigned byte
        5'b10010: memReadSized = {24'd0,memReadWord[23:16]};
        5'b10001: memReadSized = {24'd0,memReadWord[15:8]};
        5'b10000: memReadSized = {24'd0,memReadWord[7:0]};
               
        5'b10110: memReadSized = {16'd0,memReadWord[31:16]};    // unsigned half
        5'b10101: memReadSized = {16'd0,memReadWord[23:8]};
        5'b10100: memReadSized = {16'd0,memReadWord[15:0]};
            
        default:  memReadSized = 32'b0;     // unsupported size, byte offset combination
      endcase
    end
    
     // Memory Mapped IO
    always_comb begin
        //combinatoinal logic to trigger accessing of MMIO
      if(MEM_ADDR2 >= 32'h00010000) begin  // external address range
        IO_WR = MEM_WE2;                 // IO Write
        MEM_DOUT2 = ioBuffer;            // IO read from buffer
        weAddrValid = 0;                 // address beyond memory range
      end
      else begin
        IO_WR = 0;                  // not MMIO
        MEM_DOUT2 = memReadSized;   // output sized and sign extended data
        weAddrValid = MEM_WE2;      // address in valid memory range
      end
      
      //Parsing MMIO,
      if(MEM_ADDR2Parse >= 32'h00010000) begin  // MEM_ADDR2Parse is buffered by one from MEM_ADDR
        MEM_DOUT2 = ioBuffer;            // IO read from buffer
      end
      else begin
        MEM_DOUT2 = memReadSized;   // output sized and sign extended data
      end
    end
        
 endmodule