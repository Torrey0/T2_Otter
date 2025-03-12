`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: J. Callenes, P. Hummel, Torrey Zaches
//
// Create Date: 01/27/2019 08:37:11 AM
// Module Name: OTTER_mem
// Project Name: Memory for OTTER RV32I RISC-V
// Tool Versions: Xilinx Vivado 2019.2
// Description: 64k Memory, with direct mapped instruction cache and 4-way
//              set associative data cache. Designed to purposely
//              utilize BRAM which requires synchronous reads and write
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
// Revision 1.08 - embedded direct mapped and set associative cache, replacing dual port read/write with "one block at a time" reads and writes
//
//////////////////////////////////////////////////////////////////////////////////
                                                                                                                             
  module Memory (
    input MEM_CLK,
    input MEM_RDEN1,        // read enable Instruction
    input MEM_RDEN2,        // read enable data
    input MEM_WE2,          // write enable.
    input MEM_RST,
    
    input [13:0] MEM_ADDR1, // Instruction Memory word Addr (Connect to PC[15:2])
    input branchTaken,      //when we take a branch, we need to reset stage of instruction cache, whatever it was loading before is irrelevant
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
    output logic cacheMissStall,
    output logic [31:0] MEM_DOUT2, // Data
    output logic dataMemStall         //memory module's notStall, triggered on miss reading from data Memory
    );

    logic IOADDR;   //indicates if we are reading/writing to IO
    logic IOADDRParse;
    assign IOADDR=(MEM_ADDR2 >= 32'h00010000);
    assign IOADDRParse= (MEM_ADDR2Parse >= 32'h00010000);
    logic cacheRead2;
    assign cacheRead2= (~IOADDR) & MEM_RDEN2;
      
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
    assign byteOffsetParse = MEM_ADDR2Parse[1:0];
    
    // buffer the IO input for reading
    always_ff @(posedge MEM_CLK) begin
      if(MEM_RDEN2)
        ioBuffer <= IO_IN;
    end
    //
    
//Direct Mapped Instruction Cache- 16 blocks, 8 words per block
    logic instr_cache_hit, instr_cache_miss, instr_cache_loadMem;
    logic [31:0] instr_cache_w0;

    logic [3:0] instr_loadMemState;   //tracks which word in a given block is currently being loaded to memory from cache
    logic [3:0] instr_loadCacheState; //propogation of above value
    
    logic [13:0] MEM_ADDR1_OFFSET;
    assign MEM_ADDR1_OFFSET[13:3]= MEM_ADDR1[13:3];   //assign reading address given block of memory
    assign MEM_ADDR1_OFFSET[2:0] = instr_loadMemState-1;

    logic [31:0] CacheDOUT1;
    Cache Cache(.PC(MEM_ADDR1), .CLK(MEM_CLK), .w0(instr_cache_w0), .rd(CacheDOUT1), .hit(instr_cache_hit), .loadCacheState(instr_loadCacheState), .miss(instr_cache_miss));
    CacheFSM CacheFSM(.hit(instr_cache_hit), .miss(instr_cache_miss), .CLK(MEM_CLK), .RST(MEM_RST), .branchTaken(branchTaken), .loadMem(instr_cache_loadMem), .loadMemState(instr_loadMemState), .loadCacheState(instr_loadCacheState), .pc_stall(cacheMissStall));
//
    
//Set Associative Data Cache- 64 word, 4-way, 4 words per block
    logic [31:0] dataw0;    //data transfer from memory into cache
    logic [31:0] memOut0;   //data transfer from cache into memory module
    logic [31:0] CacheDOUT2;    //output of dataCache fed into pipeline

    //tracks state of data transfer
    logic [2:0] dataLoadMemState;
    logic [2:0] dataLoadCacheState; //propogation of above value
    
    logic [2:0] dataStoreCacheState;
    logic [2:0] dataStoreMemState;  //propogation of above value
    //
    
    //parse which word within block to load
    logic [13:0] MEM_ADDR2Offset8_1;    
    assign MEM_ADDR2Offset8_1[13:2]= wordAddr2[13:2];   //assign reading address to given block of memory
    assign MEM_ADDR2Offset8_1[1:0] = dataLoadMemState-1;    //assign specific word within that memory
    
    //parse which word within block to store
    logic [13:0] storeMEM_ADDR2Offset8_1;
    assign storeMEM_ADDR2Offset8_1[13:2]= wordAddr2[13:2];   //assign reading address given block of memory
    assign storeMEM_ADDR2Offset8_1[1:0] = dataStoreMemState[1]-1;

    //logic shared between dataCache and cacheFSM
    logic tryWrite;
    logic tryRead;
    logic dirtyTarget;
    logic overwrite;
    dataCache dataCache(.overwrite(overwrite), .dirtyTarget(dirtyTarget), .MEM_SIZE(MEM_SIZE), .byteOffset(byteOffset), .Addr(wordAddr2), .CLK(MEM_CLK), .w0(dataw0), .dataOut(CacheDOUT2), .loadMemState(dataLoadCacheState), .storeMemState(dataStoreCacheState), .tryWrite(tryWrite), .tryRead(tryRead), .storeInput(MEM_DIN2), .memOut0(memOut0));
    dataCacheFSM dataCacheFSM(.overwrite(overwrite), .dirtyTarget(dirtyTarget), .readEnable(cacheRead2), .writeEnable(weAddrValid), .CLK(MEM_CLK), .RST(MEM_RST), .loadMemState(dataLoadMemState), .loadCacheState(dataLoadCacheState), .storeMemState(dataStoreMemState) , .storeCacheState(dataStoreCacheState), .tryWrite(tryWrite), .tryRead(tryRead), .pc_stall(dataMemStall));
//
    
//single port BRAM access. BRAM requires all reads/writes to be synchronous
    always_ff @(posedge MEM_CLK) begin
    
        if(dataLoadMemState!=0) begin   //Data cache is requesting a load, load 4 words from memory to cache
            dataw0 <= memory[MEM_ADDR2Offset8_1];
            
        end  if(dataStoreMemState!=0) begin   //Data cache is requesting a store, store 4 words from cache to memory
            memory[storeMEM_ADDR2Offset8_1]<=memOut0;
            
        end   if(instr_cache_loadMem) begin             //Instruction cache is requesting a load, load 8 words from memory to cache
            instr_cache_w0 <=memory[MEM_ADDR1_OFFSET];
        end
    end
    
    //output from the caches module
    always_ff @(posedge MEM_CLK) begin
      if (MEM_RDEN1)                       
        MEM_DOUT1 <=CacheDOUT1;     //output of instruction cache, is directly used as the output from memory module
      if (cacheRead2)                      
        memReadWord <= CacheDOUT2;  //output of data cache is parsed below based on lw, lh, lb, etc, before being outputed.
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
        //used when writing
      if(IOADDR) begin  // external address range
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
      //used when reading
      if(IOADDRParse) begin  // MEM_ADDR2Parse is buffered by one from MEM_ADDR
        MEM_DOUT2 = ioBuffer;            // IO read from buffer
      end
      else begin
        MEM_DOUT2 = memReadSized;   // output sized and sign extended data
      end
    end
        
 endmodule