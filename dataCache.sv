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
    input readEnable,
    input [31:0] storeInput,
    input [1:0] MEM_SIZE,
    input [1:0] byteOffset,
//    output logic missUpdate,
//    output logic storeMiss,
    //
    
    //updating from memory
    input logic [2:0] loadMemState,
    input logic [31:0] w0,
//    input logic [31:0] w1,
    //

    output logic [31:0] dataOut,
    
    input logic [2:0] storeMemState,
    output logic [31:0] memOut0,    //for transfering data from cache to memory
//    output logic [31:0] memOut1,
    output logic readHit,
    output logic storeFirst

    );
    //things that may need to be outputs
    logic writeHit;
    logic readMiss;
    logic writeMiss;
    logic blockFull;
    logic dirtyTarget;
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
    parameter TAG_SIZE = INSTRUCTION_SIZE - INDEX_SIZE - SET_SIZE - BYTE_OFFSET;    //currently: 14-2-2-0 = 10. So want Addr[13:4] stored in here?
    
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
    logic [TAG_SIZE-1:0] cache_tags [SET_SIZE-1:0];
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
    logic [NUM_BLOCKS_PER_SET-1:0] writeHits;
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
    
    always_comb begin
        dataOut = 32'h00000000; // Default value
        readHits = '{default: 0}; // Reset all hits to 0
        writeHits= '{default: 0};
        validBlocks = '{default: 0};
        matchingTags = '{default: 0};
        matchingIndex= '{default: 0};
        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            if (valid_bits[index][i])begin
                validBlocks[i]=1;
            end

            //write hit is when theres an invalid tag or matching tag, just write over it always
            //read hit is when matching tag and it is a valid tag
            //on write miss, we check dirty bit. if dirty bit true, we update to mem first
            //if dirty bit false we can j overwrite the data
            writeHits[i] = (~validBlocks[i]) | matchingTags[i];
            if (tags[index][i] == reqAddr_tag) begin
                matchingTags[i]=1;
                matchingIndex=i;
            end
            readHits[i] = (validBlocks[i]) & matchingTags[i];
            if (readHits[i]) begin             
                dataOut = data[index][i][block_offset];
                break;
            end

        end
    end
    always_ff @(posedge CLK) begin //update block recency
        if ((readHit&readEnable) | (writeHit&tryWrite)) begin
//            if(readEnable | tryWrite) begin    //the block at
                for(int i=0; i< NUM_BLOCKS_PER_SET; i++) begin
                    if((readEnable & readHit & readHits[i]) | (tryWrite & writeHit & writeHits[i])) begin   //check if we are reading or writing something.
                        blockRecency[index][i]<=0;  //mark the hit block as most recent
                    end else begin
                        blockRecency[index][i]<=blockRecency[index][i]+1;   //increment this blocks recency, indicating it is more outdated
                    end
//                end
            end
        end
//        if((readEnable && hit) || (tryWrite &&hit)) begin
//        end
    end
    assign blockFull= &validBlocks;
    assign readHit = |readHits; // |x notation ors all elements in x array
    assign writeHit= |writeHits;
    assign tagMatch= |matchingTags;
    assign readMiss = ~readHit;
    assign writeMiss= ~writeHit;
//    assign addr_tag = Addr[13:7];
//    assign miss = !hit;
    logic [INDEX_SIZE-1:0] oldestBlockIndex;
    assign dirtyTarget=dirty_bits[index][oldestBlockIndex];
    logic overWriteBlock;
    assign overWriteBlock= (tryWrite & writeMiss) | (readEnable & readMiss);    //indicate if we need to overwrite a block
    assign storeFirst=overWriteBlock & dirtyTarget;    //if 
    //for a given set, determine oldest block. Used for replacements
    logic targetIndex;
//    logic [NUM_BLOCKS_PER_SET-1:0] blockSet;   
    always_comb begin
        oldestBlockIndex = 2'b01; // Default to 0  //causes a 0 bits of block Recency to go unused. not a real issue, is ok since is good for code clarity
        for (int i = 0; i < NUM_BLOCKS_PER_SET; i++) begin
            if (valid_bits[index][i]=='0 || blockRecency[index][i] == 2'b11)
                oldestBlockIndex = i;
        end
    end
    always_comb begin
        if(tagMatch) begin
            targetIndex=matchingIndex;
        end else begin
            targetIndex=oldestBlockIndex;
        end
    end
    //

    always_ff @(posedge CLK) begin  //handle loading to cache from Mem, needed on miss for 

        if(loadMemState!=3'b000) begin
            data[index][targetIndex][loadMemState-1] <= w0;
            if(loadMemState==3'b100) begin
                tags[index][targetIndex] <= reqAddr_tag;  //highly experimental? is this the correct timing? why didnt she have this here?
                valid_bits[index][targetIndex] <= 1'b1;
                dirty_bits[index][targetIndex] <= 1'b0;    //writing from memory, this location is indeed not dirty!
            end
        end else

//            //determine if user is missing or hitting on a store           
        if(tryWrite & writeHit & (~dirtyTarget)) begin    //handle storing to cache from user. if the target 
            if((~dirtyTarget)) begin   //if the block we need to store to is already present
                //insert parsing mechanism from original memory module:
            case({MEM_SIZE,byteOffset})
                4'b0000: data[index][targetIndex][block_offset][7:0]   <= storeInput[7:0];     // sb at byte offsets
                4'b0001: data[index][targetIndex][block_offset][15:8]  <= storeInput[7:0];
                4'b0010: data[index][targetIndex][block_offset][23:16] <= storeInput[7:0];
                4'b0011: data[index][targetIndex][block_offset][31:24] <= storeInput[7:0];
                4'b0100: data[index][targetIndex][block_offset][15:0]  <= storeInput[15:0];    // sh at byte offsets
                4'b0101: data[index][targetIndex][block_offset][23:8]  <= storeInput[15:0];
                4'b0110: data[index][targetIndex][block_offset][31:16] <= storeInput[15:0];
                4'b1000: data[index][targetIndex][block_offset]        <= storeInput;          // sw
           endcase
//                data[index][oldestBlockIndex][block_offset]<=storeInput;  //replced this with proper parsing above
                
                //
                dirty_bits[index][targetIndex]<=1'b1;  //indicate this data is now dirty
                tags[index][targetIndex]<= reqAddr_tag;
                valid_bits[index][targetIndex]<=1'b1;  //I suppose this is valid now?? This is super sussy
                
            end else if(storeMemState!=3'b000) begin  //handle storing from cache to MEM (when cache full)
                memOut0 <= data[index][targetIndex][0];
                //memOut1 <= data[index][oldestBlockIndex][1];
                if(storeMemState==3'b100) begin
                    //tags[index][oldestBlockIndex] <= reqAddr_tag;  //highly experimental? is this the correct timing? why didnt she have this here?
                    valid_bits[index][targetIndex] <= 1'b1;
                    dirty_bits[index][targetIndex] <= 1'b0;    //writing to memory, this location is no longer dirty!
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
    end

    
endmodule