`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly SLO
// Engineer: Torrey Zaches
// 
// Create Date: 02/26/2025 11:44:24 PM
// Design Name: 
// Module Name: dataCacheFSM
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
//    input readHit, 

    input CLK, 
    input RST, 
    input writeEnable,
    input readEnable,
    input dirtyTarget,
    input overwrite,

    output logic tryRead,
    output logic tryWrite,

    //keeps track of where in the store and load process we are. Since blocks are 4 words, and BRAM limits to one read and one write at at time,
    //it takes 4 cycles to load/store a block. These states track which word we are currently loading, and when we are done 
    output logic [2:0] loadMemState,    
    output logic [2:0] loadCacheState,    
    output logic [2:0] storeMemState,    
    output logic [2:0] storeCacheState,

    output logic pc_stall   //stall the PC when loading/storing to mem
    );

    typedef enum{
    //default store and load to cache
        ST_USE_CACHE,   
        
        //load from mem
        ST_READ_MEM,
        ST_FINISH_READ_MEM,
        //ST_UPDATE_CACHE,
        
        //store to mem
        ST_WRITE_MEM,
        ST_FINISH_WRITE_MEM    
    } state_type;
    
    state_type PS, NS;
    
    logic incrementStoreState;
    logic resetStoreState;
    logic incrementLoadState;
    logic resetLoadState;
    
    always_ff @(posedge CLK) begin
        if(RST == 1) begin  //reset cache to starting state, although, no need to clear it.
            PS <= ST_USE_CACHE;
            loadMemState<=0; 
            loadCacheState<=0;
            storeMemState<=0;
            storeCacheState<=0;
        end
        else begin
            PS <= NS;
            if(incrementStoreState) begin
                storeCacheState<=storeCacheState +1;
            end else if (resetStoreState) begin
                storeCacheState<=0;
            end
            if(incrementLoadState) begin
                loadMemState<= loadMemState+1;
            end else if (resetLoadState) begin
                loadMemState<=0;
            end
            
            loadCacheState<=loadMemState;
            storeMemState<=storeCacheState;
        end
    end
    
    //simple outputs that depend on the state alone are assigned here:
    
    //only attempt to read or write when an instruction triggers us to do so, and we have returned to the default "use cache" state
    assign tryWrite=writeEnable && (PS==ST_USE_CACHE);
    assign tryRead=readEnable && (PS==ST_USE_CACHE);
    assign pc_stall=(NS!=ST_USE_CACHE) || (PS!=ST_USE_CACHE);   //whenever we are not in standard "use cache" mode, we are loading/storing to mem, and need to stall the PC
        
    always_comb begin
        incrementStoreState = 1'b0;
        resetStoreState = 1'b0;
        incrementLoadState = 1'b0;
        resetLoadState = 1'b0;

        case (PS)
            ST_USE_CACHE: begin
                if(overwrite & (~dirtyTarget)) begin
                    NS=ST_READ_MEM;
                end else if(overwrite) begin
                    NS=ST_WRITE_MEM;
                end else begin
                    NS=ST_USE_CACHE;
                end
            end
            
            ST_READ_MEM: begin                  
                incrementLoadState=1'b1;
                if(loadMemState==3'b100) begin                  
                    NS = ST_FINISH_READ_MEM;
                end
                else begin
                    NS=ST_READ_MEM; //continue loading the memory
                end
            end
            
            ST_FINISH_READ_MEM: begin  //stalling to make sure cache finishes it's update part, while also reseting the memory's loading state
                resetLoadState=1'b1;
                NS=ST_USE_CACHE;
            end
            
            ST_WRITE_MEM: begin                
                incrementStoreState=1'b1;
                if(storeMemState==3'b100) begin                  
                    NS = ST_FINISH_WRITE_MEM;
                end
                else begin
                    NS=ST_WRITE_MEM; //continue writing to  memory
                end
            end
            
            ST_FINISH_WRITE_MEM: begin    //stalling to make sure memory finishes it's update part, while also reseting the cache's storing state
                resetStoreState=1'b1;
                NS=ST_READ_MEM;                
            end
            
        default: NS = ST_USE_CACHE;
        endcase
    end
endmodule