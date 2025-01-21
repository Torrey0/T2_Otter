`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/22/2024 11:18:36 AM
// Design Name: 
// Module Name: Test_All_TB
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


module Test_All_TB();
    logic TB_CLCK, TB_BTNC;
    logic [15:0] TB_LEDS, TB_SWITCHES;
    logic [7:0] TB_CATHODES;
    logic [3:0] TB_ANODES;

    
    OTTER_Wrapper UUT (.CLK(TB_CLCK), .BTNC(TB_BTNC), .LEDS(TB_LEDS), .SWITCHES(TB_SWITCHES), .CATHODES(TB_CATHODES), .ANODES(TB_ANODES));
    
    initial
    begin
    TB_CLCK=0;
        forever #5 TB_CLCK = ~TB_CLCK;
    end
    //btn 011101
    //no: 011000
    always begin
    TB_BTNC=0;
    TB_SWITCHES=0;
    #10000
    //TB_SWITCHES=3;  //wait until test is complete, so some seperation
    #20
    TB_SWITCHES=0;  //always want a flag if a test fails
        
    end
    endmodule
