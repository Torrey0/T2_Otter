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


module CacheFSM(
    input hit, 
    input miss, 
    input CLK, 
    input RST, 
    input branchTaken,
    output logic loadMem,
    output logic [3:0] loadMemState,    //loading the memory takes three cycles since we are loading 8 bytes, but can only load at most 3 without exceeding BRAM usage
    output logic [3:0] loadCacheState,    //loading the memory takes three cycles since we are loading 8 bytes, but can only load at most 3 without exceeding BRAM usage
//    output logic updateCache, 
    output logic pc_stall
    );

    typedef enum{
        ST_READ_CACHE,
        ST_READ_MEM,
        ST_STORE_CACHE
    } state_type;
    
    state_type PS, NS;
    always_ff @(posedge CLK) begin
        if(RST == 1) begin
            PS <= ST_READ_MEM;
            loadMemState<=0; 
            loadCacheState<=0;
        end
        if(branchTaken) begin
            PS <= ST_READ_CACHE;
            loadMemState<=0; 
            loadCacheState<=0;
        end
        else begin
            PS <= NS;
            if(NS==ST_STORE_CACHE)
                loadMemState<=0; 
            else if(PS==ST_READ_MEM) begin
                //add one to the current loading State, supports adding up to 4
                loadMemState <= loadMemState +1;
//                loadMemState[0] <= ~loadMemState[0]; 
//                loadMemState[1] <= loadMemState[1] ^ loadMemState[0];
//                loadMemState[2] <= loadMemState[1] & loadMemState[0];
            end
            
            loadCacheState<=loadMemState;
        end
    end
    
    always_comb begin
//        updateCache =1'b0;
        loadMem = 1'b0;
        pc_stall = 0;
        if (branchTaken)
            NS = ST_READ_CACHE;
        else begin
            case (PS)
                ST_READ_CACHE: begin
    //                updateCache = 1'b0;
                    
                    if(hit) begin
                        NS = ST_READ_CACHE;
                    end
                    
                    else if(miss) begin
                        pc_stall = 1'b1;  //removing this since we are using pc_stall with comb logic, we still want to read the last guy
                        NS = ST_READ_MEM;
                        //loadMemState <=0;
                    end
                    
                    else NS = ST_READ_CACHE;
                end
                
                ST_READ_MEM: begin                  
                    loadMem =1'b1;
                    pc_stall = 1'b1;     
                    if(loadMemState==4'b1000) begin                  
                        NS = ST_STORE_CACHE;
                    end
                    else begin
                        NS=ST_READ_MEM; //continue loading the memory
                        //increment loadMemState by 1
                    end

                end
                ST_STORE_CACHE: begin
//                    updateCache =1'b1;
                    pc_stall = 1'b1;
                    NS=ST_READ_CACHE;
                end
                    
            default: NS = ST_READ_CACHE;
            endcase
        end
    end
endmodule