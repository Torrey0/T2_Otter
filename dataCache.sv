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
    input [13:0] Addr,    //this is our new memAddr1. This is used for both storing and writing
    input CLK,
    
    // storing stuff
    input tryWrite,
    input tryRead,
    input [31:0] storeInput,
    input [1:0] MEM_SIZE,
    input [1:0] byteOffset,
//    output logic missUpdate,
//    output logic storeMiss,
    //
    
    //updating from memory
    input logic [2:0] loadMemState,
    input logic [2:0] storeMemState,

    input logic [31:0] w0,  //write into cache
//    input logic [31:0] w1,
    //

    output logic [31:0] dataOut,    //value read from cache
    
    output logic [31:0] memOut0,    //for transfering data from cache to memory
//    output logic [31:0] memOut1,
//    output logic readHit,   //readHit is true when a block with matching tag is found and is valid 
    //overwrite is equivelant to miss
    output logic overwrite, //storeFirst is true any time we need to replace a block, and its marked as dirty. This can occur on both a store or a load
    output logic dirtyTarget
    //a "loadFirst" is inferred whenever readHit is false.

    );
    //things that may need to be outputs
//    logic writeHit;
    logic readMiss;
//    logic writeMiss;
    logic blockFull;
//    logic dirtyTarget;
//    typedef struct {
//  		logic valid;
//  		logic dirty;
//  		logic tag;
//  		logic data[3:0][31:0];
  		
//  		int    count;
//  		byte  expiry;
//	} Block;

    parameter NUM_SETS = 4;
    parameter NUM_BLOCKS_PER_SET = 4;   //NUmber of blocks / Num_sets
    parameter BLOCK_SIZE =4;    //4 words per block
    parameter INDEX_SIZE = 2;   // =log2(NUM_SETS)
    parameter SET_SIZE = 2;     // =log2(NUM_BLOCKS_PER_SET)
    parameter BLOCK_OFFSET_SIZE = 2;    // =log2(blockSize)
    
    parameter BYTE_OFFSET = 0;  //byte offset already accounted for
//    parameter TAG_SIZE = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET;
    parameter INSTRUCTION_SIZE = 14;
    parameter TAG_SIZE = INSTRUCTION_SIZE - BLOCK_OFFSET_SIZE - SET_SIZE - BYTE_OFFSET;    //currently: 14-2-2-0 = 8. So want Addr[13:4] stored in here
    
    //the space for the actual cache
       //space                //each set     each block      //each word
    logic [31:0]         data[NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0][BLOCK_SIZE-1:0];
    logic [TAG_SIZE-1:0] tags[NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    logic valid_bits         [NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    logic dirty_bits         [NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    logic [INDEX_SIZE-1:0] blockRecency [NUM_SETS-1:0][NUM_BLOCKS_PER_SET-1:0];
    //

    //used for parsing the address
    logic [INDEX_SIZE-1:0] index;
    logic [TAG_SIZE-1:0] reqAddr_tag;
//    logic [TAG_SIZE-1:0] cache_tags [SET_SIZE-1:0];
    logic [BLOCK_OFFSET_SIZE-1:0] block_offset;
    

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
    

    //parsing input address
    assign block_offset= Addr[BLOCK_OFFSET_SIZE-1 : 0];
    assign index = Addr[INDEX_SIZE + BLOCK_OFFSET_SIZE - 1 : BLOCK_OFFSET_SIZE];
    assign reqAddr_tag = Addr[13 : INDEX_SIZE + BLOCK_OFFSET_SIZE];

    //
    //checking for validity
//    assign cache_tag = tags[index];
    //SET_SIZE or NUM_BLOCKS_PER_SET?
    logic tagMatch;
    logic [NUM_BLOCKS_PER_SET-1:0] readHits;
//    logic [NUM_BLOCKS_PER_SET-1:0] writeHits;
    logic [NUM_BLOCKS_PER_SET-1:0] validBlocks;
    logic [NUM_BLOCKS_PER_SET-1:0] matchingTags;
    logic [NUM_BLOCKS_PER_SET-1:0] matchingIndex;
//    always_comb begin
//        dataOut = 32'h0000000; //assign default value
//            for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin  //handle loading from cache
//                blockHit[i] = valid_bits[index][i] && (tags[index][i] == reqAddr_tag);
//                if(blockHit[i]) dataOut = data[index][i][block_offset];
//            end
//    end
//    logic writeFlag;
//    logic [NUM_BLOCKS_PER_SET-1:0] writeMatch;
    //decodes which blocks are hits
    always_comb begin
        dataOut = 32'h00000000; // Default value
        readHits = '{default: 0}; // Reset all hits to 0
//        writeHits= '{default: 0};
        validBlocks = '{default: 0};
        matchingTags = '{default: 0};
        matchingIndex= '{default: 0};
//        writeFlag=0;
        //we rlly should j make these sperate loops
        
        //obtain information from the correct set (so must be matching in "index"
        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            if (valid_bits[index][i])begin
                validBlocks[i]=1;
            end
            if (tags[index][i] == reqAddr_tag) begin
                matchingTags[i]=1;
                matchingIndex=i;
            end
        end
        
    //output data effectively very always comb
        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            readHits[i] = ((validBlocks[i]) & matchingTags[i]);
            if (readHits[i]) begin             
                dataOut = data[index][i][block_offset];
//                dataOut = data[index][i][matchingIndex];

                //want to use some sort of target Address. Is this what we alr have?
                // want to read one with matching tag. Period. no need to refer to one with oldest block, this is useless on a read
               // break;
            end

        end
    end
    
    //update block recency whenever we get a hit
    logic [SET_SIZE-1:0] hitBlockRecency;
    logic [SET_SIZE-1:0] hitBlock;
    logic hitFlag;
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
    always_ff @(posedge CLK) begin //update block recency    
        if ((readHit&(tryRead|tryWrite))) begin //| (writeHit&tryWrite)) begin
//            if(readEnable | tryWrite) begin    //the block at
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
//        if((readEnable && hit) || (tryWrite &&hit)) begin
//        end
    end
    //
    
    
    assign blockFull= &validBlocks;
    assign readHit = |readHits; // |x notation ors all elements in x array
//    assign writeHit= |writeHits;
    assign tagMatch= |matchingTags;
    assign readMiss = ~readHit;
//    assign writeMiss= ~writeHit;
//    assign addr_tag = Addr[13:7];
//    assign miss = !hit;
    logic [INDEX_SIZE-1:0] oldestBlockIndex;
    logic [NUM_BLOCKS_PER_SET-1:0] targetBlockIndex;

    assign dirtyTarget=dirty_bits[index][targetBlockIndex];
//    logic overWriteBlock;
    assign overwrite= (tryWrite |tryRead) & (readMiss);    //indicate if we need to overwrite a block
//    assign storeFirst=overWriteBlock & dirtyTarget;    //if we are replacing a block, and its dirty, we nede to store it to mem first
    //for a given set, determine oldest block. Used for replacements
    
    
    //determine which block we want to target
//    logic [NUM_BLOCKS_PER_SET-1:0] blockSet;   
    logic oldestAssignedFlag;
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


    always_ff @(posedge CLK) begin  //handle loading to cache from Mem, needed on miss for 
        if(loadMemState!=3'b000) begin
            data[index][targetBlockIndex][loadMemState-1] <= w0;
            if(loadMemState==3'b100) begin
                tags[index][targetBlockIndex] <= reqAddr_tag;  //highly experimental? is this the correct timing? why didnt she have this here?
                valid_bits[index][targetBlockIndex] <= 1'b1;
                dirty_bits[index][targetBlockIndex] <= 1'b0;    //writing from memory, this location is indeed not dirty!
            end
        end 

        //handle storing to cache directly from user
//            //determine if user is missing or hitting on a store           
        if(tryWrite & readHit) begin    //handle storing to cache from user. if the target 
//            if((~dirtyTarget)) begin   //if the block we need to store to is already present
                //insert parsing mechanism from original memory module:
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
                //memOut1 <= data[index][oldestBlockIndex][1];
                if(storeMemState==3'b100) begin
                    //tags[index][oldestBlockIndex] <= reqAddr_tag;  //highly experimental? is this the correct timing? why didnt she have this here?
                    valid_bits[index][targetBlockIndex] <= 1'b1;
                    dirty_bits[index][targetBlockIndex] <= 1'b0;    //writing to memory, this location is no longer dirty!
                end
            end
//            end else if(storeMemState==2'b10) begin
//                memOut0 <= data[index][oldestBlockIndex][2];
//                memOut1 <= data[index][oldestBlockIndex][3];
//                //tags[index][oldestBlockIndex] <= reqAddr_tag;  //highly experimental? is this the correct timing? why didnt she have this here?
//                valid_bits[index][oldestBlockIndex] <= 1'b1;
//                dirty_bits[index][oldestBlockIndex] <= 1'b0;    //writing to memory, this location is no longer dirty!
//            end
    end

    
endmodule