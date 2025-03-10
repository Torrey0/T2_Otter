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


module dataCacheFSM(
    input hit, 
    input miss, 
    input CLK, 
    input RST, 
    input writeEnable,
    input readEnable,
    input dirtyTarget,
//    input branchTaken,
    //input tryRead,    //cache is effectively always trying to read
    output logic storeMem,
    output logic tryWrite,
    output logic loadMem,
    output logic [1:0] loadMemState,    //loading the memory takes three cycles since we are loading 8 bytes, but can only load at most 3 without exceeding BRAM usage
    
    output logic [1:0] loadCacheState,    //loading the memory takes three cycles since we are loading 8 bytes, but can only load at most 3 without exceeding BRAM usage
    output logic [1:0] storeMemState,    //loading the memory takes two cycles since we are loading 4 bytes, but can only load at most 2 without exceeding BRAM usage
    output logic [1:0] storeCacheState,
//    output logic updateCache, 
    output logic pc_stall
    );

    typedef enum{
        //load
        ST_READ_CACHE,
        ST_READ_MEM,
        ST_UPDATE_CACHE,
        
        //store
        ST_WRITE_CACHE,
        ST_WRITE_MEM
        //ST_REPLACE_CACHE
    } state_type;
    
    state_type PS, NS;
    logic incrementStoreState;
    logic resetStoreState;
    always_ff @(posedge CLK) begin
        if(RST == 1) begin
            PS <= ST_READ_CACHE;
            loadMemState<=0; 
            loadCacheState<=0;
            storeMemState<=0;
            storeCacheState<=0;
        end
//        if(branchTaken) begin
//            PS <= ST_READ_CACHE;
//            loadMemState<=0; 
//            loadCacheState<=0;
//        end
        else begin
            PS <= NS;
            if(NS==ST_UPDATE_CACHE)
                loadMemState<=0; 
            else if(PS==ST_READ_MEM) begin
                //add one to the current loading State, supports adding up to 4
                loadMemState[0] <= ~loadMemState[0]; 
                loadMemState[1] <= loadMemState[1] ^ loadMemState[0];
            end
            loadCacheState<=loadMemState;
  
            if(incrementStoreState)begin
                storeCacheState[0] <= ~storeCacheState[0];
                storeCacheState[1] <= storeCacheState[1] ^ storeCacheState[0];
            end else if (resetStoreState) begin
                storeCacheState<=0; 
            end
            storeMemState<=storeCacheState;
        end
    end
    
    always_comb begin
//        updateCache =1'b0;
        loadMem = 1'b0;
        storeMem= 1'b0;
        tryWrite= 1'b0;
        pc_stall = 0;
        resetStoreState=1'b0;
        incrementStoreState=1'b0;
        case (PS)
            ST_READ_CACHE: begin
//                updateCache = 1'b0;
                if(readEnable) begin
                    if(hit) begin
                        NS = ST_READ_CACHE;
                    end else if(miss) begin
                        pc_stall = 1'b1;  //removing this since we are using pc_stall with comb logic, we still want to read the last guy
                        NS = ST_READ_MEM;
                        //loadMemState <=0;
                    end else NS = ST_READ_CACHE;
                    
                end else if(writeEnable) begin
                    NS=ST_WRITE_CACHE;
                end else begin
                    NS=ST_READ_CACHE;
                end
            end
            
            ST_READ_MEM: begin                  
                loadMem =1'b1;
                pc_stall = 1'b1;     
                if(loadMemState==3'b100) begin                  
                    NS = ST_UPDATE_CACHE;
                end
                else begin
                    NS=ST_READ_MEM; //continue loading the memory
                    //increment loadMemState by 1
                end

            end
            ST_UPDATE_CACHE: begin  //effectively stalling to make sure cache finishes it's update part
//                    updateCache =1'b1;
                pc_stall = 1'b1;
                NS=ST_READ_CACHE;
            end
            ST_WRITE_CACHE: begin
                tryWrite=1'b1;
                if(miss || dirtyTarget) begin
                    NS=ST_WRITE_MEM;
                    pc_stall=1'b1;
                    incrementStoreState=1'b1;
                end else begin
                    NS=ST_READ_CACHE;   //return to original state
                    resetStoreState=1'b1;
                end
            end
            ST_WRITE_MEM: begin
                pc_stall=1'b1;
                storeMem=1'b1;
                if(storeMemState==2'b10) begin
                    NS=ST_WRITE_CACHE;
                end else begin
                    NS=ST_WRITE_MEM; //stay here
                    incrementStoreState=1'b1;
                end              
                
            end
        default: NS = ST_READ_CACHE;
        endcase
    end
endmodule