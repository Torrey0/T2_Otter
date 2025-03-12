`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/26/2025 11:44:24 PM
// Design Name: 
// Module Name: imem
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


module dataCache(
    input CLK,
    input [13:0] Addr,    //This is the address used for both storing and writing
    //to parse the input to be stored need to differentiate between sw, sh, sb, etc.
    input [1:0] MEM_SIZE,
    input [1:0] byteOffset,
    // the FSM tells the cache when to try writing or reading
    input tryWrite,
    input tryRead,
    //updating to/from memory
    input logic [2:0] loadMemState,
    input logic [2:0] storeMemState,

    input [31:0] storeInput,    //write into cache from register
    input logic [31:0] w0,  //write into cache from memory

    output logic [31:0] dataOut,    //value read from cache, output to instruction    
    output logic [31:0] memOut0,    //for transfering data from cache to memory
    output logic overwrite, //indicate to FSM if we need to overwrite a block
    output logic dirtyTarget    //indicate if the block we need to overwrite is dirty
    );

    parameter NUM_SETS = 4;
    parameter NUM_BLOCKS_PER_SET = 4;   //Number of blocks / Num_sets
    parameter BLOCK_SIZE =4;    //4 words per block
    parameter INDEX_SIZE = 2;   // =log2(NUM_SETS)
    parameter SET_SIZE = 2;     // =log2(NUM_BLOCKS_PER_SET)
    parameter BLOCK_OFFSET_SIZE = 2;    // =log2(blockSize)
    
    parameter BYTE_OFFSET = 0;  //byte offset already removed from input address, and is accounted for in MEM_SIZE and Byte offset, and ha
    parameter INSTRUCTION_SIZE = 14;    //our memory only holds locations up to 2^14 (it is 16kb)
    parameter TAG_SIZE = INSTRUCTION_SIZE - BLOCK_OFFSET_SIZE - SET_SIZE - BYTE_OFFSET;    //currently: 14-2-2-0 = 8. So want Addr[13:4] stored in here
    
    //the space for the actual cache
       //space                //each set     each block           //each word
    logic [31:0]         data[NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0][BLOCK_SIZE-1:0];
    logic [TAG_SIZE-1:0] tags[NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    logic valid_bits         [NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    logic dirty_bits         [NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    logic [INDEX_SIZE-1:0] blockRecency [NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    //

    //used for parsing the input address
    logic [INDEX_SIZE-1:0] index;               //the index of the set
    logic [TAG_SIZE-1:0] reqAddr_tag;           //the tag of the input
    logic [BLOCK_OFFSET_SIZE-1:0] block_offset; //the word within the block the input is targetting
        //parsing input address
    assign block_offset= Addr[BLOCK_OFFSET_SIZE-1 : 0];
    assign index = Addr[INDEX_SIZE + BLOCK_OFFSET_SIZE - 1 : BLOCK_OFFSET_SIZE];
    assign reqAddr_tag = Addr[13 : INDEX_SIZE + BLOCK_OFFSET_SIZE];
    
    
    //initialize cache to 0
    initial begin
    int i, j, p;
    for (p = 0; p < NUM_SETS; p = p + 1) begin
        for (i = 0; i < NUM_BLOCKS_PER_SET; i = i + 1) begin // Need begin-end here
            for (j = 0; j < BLOCK_SIZE; j = j + 1) begin
                data[p][i][j] = 32'b0;
            end
            tags[p][i] = 32'b0;
            valid_bits[p][i] = 1'b0;
            dirty_bits[p][i] = 1'b0;
            blockRecency[p][i] = '0;
        end
    end
end
    
    //gather necessary information about the set we are targeting
    logic readHit;
    logic [NUM_BLOCKS_PER_SET-1:0] readHits;        //which block are "hits"
    logic [NUM_BLOCKS_PER_SET-1:0] validBlocks;     //which blocks are valid
    logic [NUM_BLOCKS_PER_SET-1:0] matchingTags;    //which blocks have tags that match the input address
    logic [NUM_BLOCKS_PER_SET-1:0] matchingIndex;   //save index of the block whose tag we match
    
    always_comb begin
        dataOut = 32'h00000000; // Default value
        readHits = '{default: 0}; // Reset all hits to 0
        validBlocks = '{default: 0};
        matchingTags = '{default: 0};
        matchingIndex= '{default: 0};

        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            if (valid_bits[index][i])begin
                validBlocks[i]=1;
            end
            if (tags[index][i] == reqAddr_tag) begin
                matchingTags[i]=1;
                matchingIndex=i;
            end
        end
        
    //constantly output data on readHit regardless of whether it is actually being requested to avoid delays
        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            readHits[i] = ((validBlocks[i]) & matchingTags[i]);
            if (readHits[i]) begin             
                dataOut = data[index][i][block_offset];
            end
        end
    end
    assign readHit = |readHits; // |x notation ors all elements in x array. Tracks whether we got a hit anywhere

    
    //This cache implements a least recently used block priority. A block priority of 0=just used, a block priority of 3=least recently used
    
    //record information about hit Recency for our chosen set anytime we get a hit
    logic [SET_SIZE-1:0] hitBlockRecency;
    logic [SET_SIZE-1:0] hitBlock;
    logic hitFlag;  //give priority to the first found hit. We need some way of picking a block when multiple could be chosen, and synthesizer infers this type of logic well
    always_comb begin
        hitFlag=0;
        hitBlockRecency=0;
        hitBlock=0;
        for (int i=0;i<NUM_BLOCKS_PER_SET; i++) begin
            if(!hitFlag) begin
                if(((tryRead|tryWrite) & readHits[i])) begin //| (tryWrite & writeHit & writeHits[i])) begin
                    hitBlockRecency=blockRecency[index][i];
                    hitBlock=i;
                    hitFlag=1;
                end
            end
        end
    end
    
    //update block recency  based on the recorded information above
    always_ff @(posedge CLK) begin   
        if ((readHit&(tryRead|tryWrite))) begin 
                for(int i=0; i< NUM_BLOCKS_PER_SET; i++) begin
                    if(i==hitBlock) begin   //check if we are reading or writing something.
                        blockRecency[index][i]<=0;  //mark the hit block as most recent                        
                    end else if(blockRecency[index][i]<=hitBlockRecency)begin
                            blockRecency[index][i]<=blockRecency[index][i]+1;   //increment this blocks recency, indicating it is more outdated
                    end else begin
                        blockRecency[index][i]<=blockRecency[index][i];
                    end
                end
        end
    end
    
    //logic used to process when to (or not to) store or load to memory    
    logic readMiss;
    logic tagMatch;

    assign tagMatch= |matchingTags;
    assign readMiss = ~readHit;
    assign overwrite= (tryWrite |tryRead) & (readMiss);    //indicate if we need to overwrite a block


    //find which block we need to target
    logic [INDEX_SIZE-1:0] oldestBlockIndex;                //the index of the lest recently used block
    logic [NUM_BLOCKS_PER_SET-1:0] targetBlockIndex;        //the target block is the block with matching index whenver there is one, otherwise, tis the least recently used one
    assign dirtyTarget=dirty_bits[index][targetBlockIndex]; //indicate if our target block is dirty
    
    logic oldestAssignedFlag;   //flag used to create priority for first oldest block. No particular priority needed, but we have to pick one when there are multiple (at the start of the program)
    always_comb begin   //finds the oldest block
        oldestAssignedFlag=0;
        oldestBlockIndex = 2'b01; // Default to 0  //causes a 0 bits of block Recency to go unused. not a real issue, is ok since is good for code clarity
        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            if(!oldestAssignedFlag) begin
                if (valid_bits[index][i]=='0 || blockRecency[index][i] == 2'b11)
                    oldestAssignedFlag=1;
                    oldestBlockIndex = i;
            end
        end
    end
    
    //sets our target block to iether oldest block or block with matching tag
    always_comb begin
        if(tagMatch) begin  //this is the target block index
            targetBlockIndex=matchingIndex;
        end else begin
            targetBlockIndex=(oldestBlockIndex);
        end
    end
    //

    //logic for handling data moving in and out of cache:

    always_ff @(posedge CLK) begin  
    //handle loading to cache from Mem, needed on miss for 
        if(loadMemState!=3'b000) begin  
            data[index][targetBlockIndex][loadMemState-1] <= w0;
            if(loadMemState==3'b100) begin  //at end of loading sequence, update information about the block
                tags[index][targetBlockIndex] <= reqAddr_tag;   //update the tag
                valid_bits[index][targetBlockIndex] <= 1'b1;    //location is valid after being written to
                dirty_bits[index][targetBlockIndex] <= 1'b0;    //writing from memory, this location is no longer dirty
            end
        end 

    //handle storing to cache directly from user
        if(tryWrite & readHit) begin    
            case({MEM_SIZE,byteOffset})
                4'b0000: data[index][targetBlockIndex][block_offset][7:0]   <= storeInput[7:0];     // sb at byte offsets
                4'b0001: data[index][targetBlockIndex][block_offset][15:8]  <= storeInput[7:0];
                4'b0010: data[index][targetBlockIndex][block_offset][23:16] <= storeInput[7:0];
                4'b0011: data[index][targetBlockIndex][block_offset][31:24] <= storeInput[7:0];
                4'b0100: data[index][targetBlockIndex][block_offset][15:0]  <= storeInput[15:0];    // sh at byte offsets
                4'b0101: data[index][targetBlockIndex][block_offset][23:8]  <= storeInput[15:0];
                4'b0110: data[index][targetBlockIndex][block_offset][31:16] <= storeInput[15:0];
                4'b1000: data[index][targetBlockIndex][block_offset]        <= storeInput;          // sw
           endcase
                dirty_bits[index][targetBlockIndex]<=1'b1;  //indicate this data is now dirty
                tags[index][targetBlockIndex]<= reqAddr_tag;
                valid_bits[index][targetBlockIndex]<=1'b1;  //I suppose this is valid now?? This is super sussy
        end 
            
            
   //handle storing to mem from cache
        if(storeMemState!=3'b000) begin  //handle storing from cache to MEM (when cache full)
            memOut0 <= data[index][targetBlockIndex][0];
            if(storeMemState==3'b100) begin     //at end of storing sequence, update information about the block
                valid_bits[index][targetBlockIndex] <= 1'b1;    
                dirty_bits[index][targetBlockIndex] <= 1'b0;    //writing to memory, this location is no longer dirty!
            end
        end
    end

    
endmodule