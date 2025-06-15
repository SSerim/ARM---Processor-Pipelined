module pipelined_computer(
input clk,
input  reset,
input [3:0] debug_reg_select,
output [31:0]debug_reg_out,
output [31:0] fetchPC
);
wire PC_select;
wire PC_select_W;
wire RA1_select;
wire RA2_select;
wire Register_file_write_enable;
wire [1:0] extender_select;
wire R14_select;
wire shifter_input_select;
wire shifter_amount_select;
wire shifter_type_select;
wire ALUsrcE;
wire MemtoregW;
wire [3:0] ALU_control;
wire data_memory_write_enable;

wire [1:0] OP;
wire [5:0] FUNCT;
wire [3:0] COND;
wire [3:0] Rd;
wire [3:0] Flags;

wire [3:0] RA1D_out;
wire [3:0] RA2D_out;
wire [3:0] RA1E_out;
wire [3:0] RA2E_out;
wire [3:0] WA3E_out;
wire [3:0] WA3M_out;
wire [3:0] WA3W_out;

wire [1:0] ForwardAE;
wire [1:0] ForwardBE;
wire StallF;
wire FlushE;
wire StallD;
wire FlushD;

wire RegwriteM_hazard;
wire RegwriteW_hazard;
wire Memtoreg_hazard;
wire BranchtakenE_hazard;
wire PCSrcD_hazard;
wire PCSrcE_hazard;
wire PCSrcM_hazard;
wire PCSrcW_hazard;
Pipelined_Datapath my_datapath(
    .fetch_PC(fetchPC),
    .Debug_in(debug_reg_select), 
    .Debug_out(debug_reg_out),
    .clk(clk),
    .reset(reset),
//  Control .signals
    .PC_select(PC_select),
    .PC_select_W(PC_select_W),
    .RA1_select( RA1_select),
    .RA2_select( RA2_select),
    .Register_file_write_enable( Register_file_write_enable),
    .extender_select( extender_select),
    .R14_select( R14_select),
    .shifter_input_select(  shifter_input_select),
    .shifter_amount_select( shifter_amount_select),
    .shifter_type_select( shifter_type_select),
    .ALUsrcE( ALUsrcE),
    .MemtoregW( MemtoregW),
    .ALU_control( ALU_control),
    .data_memory_write_enable( data_memory_write_enable),

// Hazard Unit .signals
    .ForwardAE( ForwardAE), 
    .ForwardBE( ForwardBE),
    .StallF( StallF),
    .FlushE( FlushE),
    .StallD( StallD),
    .FlushD( FlushD),

// Hazard Unit needs those signals
    .RA1D_out( RA1D_out),
    .RA2D_out( RA2D_out),
    .RA1E_out( RA1E_out),
    .RA2E_out( RA2E_out),
    .WA3E_out( WA3E_out),
    .WA3M_out( WA3M_out),
    .WA3W_out( WA3W_out),
// Instruction Decoded signals
    .OP(  OP  ),
    .FUNCT( FUNCT),
    .COND( COND),
    .Rd( Rd),
    .Flags( Flags)
);

Pipelined_Controller my_controller(
    .clk(clk),
    .reset(rst),

    // Controller Inputs from Instruction and ALU Flags
    .OP( OP),
    .FUNCT( FUNCT),
    .COND( COND),
    .Rd( Rd),
    .Flags( Flags),
    // Controller Outputs to Datapath
    .PC_select( PC_select), 
    .PC_select_W( PC_select_W),
    .RA1_select( RA1_select),
    .RA2_select( RA2_select),
    .Register_file_write_enable( Register_file_write_enable),
    .extender_select( extender_select),
    .R14_select( R14_select),
    .shifter_input_select(  shifter_input_select),
    .shifter_amount_select( shifter_amount_select),
    .shifter_type_select( shifter_type_select),
    .ALUsrcE( ALUsrcE),
    .MemtoregW( MemtoregW),
    .ALU_control( ALU_control),
    .data_memory_write_enable( data_memory_write_enable),
    // Controller Outputs to Hazard Unit
    .RegwriteM_hazard( RegwriteM_hazard),
    .RegwriteW_hazard( RegwriteW_hazard),
    .Memtoreg_hazard( Memtoreg_hazard),
    .BranchtakenE_hazard( BranchtakenE_hazard),
    .PCSrcD_hazard( PCSrcD_hazard),
    .PCSrcE_hazard(     PCSrcE_hazard),
    .PCSrcM_hazard( PCSrcM_hazard),
    .PCSrcW_hazard( PCSrcW_hazard),
    // .from hazard unit
    .FlushE( FlushE)
);

Hazard_Unit my_hazard_unit(
    .clk(clk),
    //inputs from controller
    .RegwriteM_hazard( RegwriteM_hazard),
    .RegwriteW_hazard( RegwriteW_hazard),
    .Memtoreg_hazard( Memtoreg_hazard),
    .BranchtakenE_hazard( BranchtakenE_hazard),
    .PCSrcD_hazard( PCSrcD_hazard),
    .PCSrcE_hazard( PCSrcE_hazard),
    .PCSrcM_hazard(     PCSrcM_hazard),
    .PCSrcW_hazard( PCSrcW_hazard),
    //.to controller
    .FlushE( FlushE),
    //inputs from datapath
    .RA1D_out( RA1D_out),
    .RA2D_out( RA2D_out),
    .RA1E_out( RA1E_out),
    .RA2E_out( RA2E_out),
    .WA3E_out( WA3E_out),
    .WA3M_out( WA3M_out),
    .WA3W_out( WA3W_out),
    //outputs to datapath
    .ForwardAE( ForwardAE), 
    .ForwardBE( ForwardBE),
    .StallF( StallF),
    .StallD( StallD),
    .FlushD( FlushD)
);
endmodule