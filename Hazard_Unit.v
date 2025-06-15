module Hazard_Unit(
    input clk,
    //inputs from controller
    input RegwriteM_hazard,
    input RegwriteW_hazard,
    input Memtoreg_hazard,
    input BranchtakenE_hazard,
    input PCSrcD_hazard,
    input PCSrcE_hazard,
    input PCSrcM_hazard,
    input PCSrcW_hazard,
    //input to controller
    output FlushE,
    //inputs from datapath
    input [3:0] RA1D_out,
    input [3:0] RA2D_out,
    input [3:0] RA1E_out,
    input [3:0] RA2E_out,
    input [3:0] WA3E_out,
    input [3:0] WA3M_out,
    input [3:0] WA3W_out,
    //outputs to datapath
    output [1:0] ForwardAE, ForwardBE,
    output reg StallF,
    output StallD,
    output FlushD
);

    // Match detection
    wire Match_1E_M = (RA1E_out == WA3M_out);
    wire Match_1E_W = (RA1E_out == WA3W_out);
    wire Match_2E_M = (RA2E_out == WA3M_out);
    wire Match_2E_W = (RA2E_out == WA3W_out);

    wire Match_1D_E = (RA1D_out == WA3E_out);
    wire Match_2D_E = (RA2D_out == WA3E_out);
    wire Match_12D_E = Match_1D_E | Match_2D_E;

    wire LDRstall = Match_12D_E & Memtoreg_hazard;
    wire PCWrPendingF = PCSrcE_hazard;  // PCSrcD_hazard | | PCSrcM_hazard

    // Forwarding logic
    assign ForwardAE = (Match_1E_M && RegwriteM_hazard) ? 2'b10 :
                       (Match_1E_W && RegwriteW_hazard) ? 2'b01 :
                       2'b00;

    assign ForwardBE = (Match_2E_M && RegwriteM_hazard) ? 2'b10 :
                       (Match_2E_W && RegwriteW_hazard) ? 2'b01 :
                       2'b00;

    // Stalling and Flushing logic
   // always @(posedge clk) begin
     //   StallF = LDRstall | PCWrPendingF;
    //end
    assign StallF = LDRstall | PCWrPendingF;
    assign StallD = LDRstall;
    assign FlushD = PCWrPendingF | PCSrcW_hazard | BranchtakenE_hazard;
    assign FlushE = LDRstall | BranchtakenE_hazard;

endmodule
