module Pipelined_Datapath(
    input clk,
    input reset,

//  Control input signals
    input PC_select, PC_select_W,
    input RA1_select,
    input RA2_select,
    input Register_file_write_enable,
    input [1:0] extender_select,
    input R14_select,
    input shifter_input_select,
    input shifter_amount_select,
    input shifter_type_select,
    input ALUsrcE,
    input MemtoregW,
    input [3:0] ALU_control,
    input data_memory_write_enable,

// Hazard Unit input signals
    input [1:0] ForwardAE, ForwardBE,
    input StallF,
    input FlushE,
    input StallD,
    input FlushD,

// Hazard Unit needs those signals
    output [3:0] RA1D_out,
    output [3:0] RA2D_out,
    output [3:0] RA1E_out,
    output [3:0] RA2E_out,
    output [3:0] WA3E_out,
    output [3:0] WA3M_out,
    output [3:0] WA3W_out,
// Instruction Decoded signals
    output [1:0] OP,
    output [5:0] FUNCT,
    output [3:0] COND,
    output [3:0] Rd,
    output [3:0] Flags,

    output [31:0] fetch_PC,
    input [3:0] Debug_in, 
    output [31:0] Debug_out
);
assign fetch_PC = PCF;

assign Flags = {CO, OVF, N, Z};
assign RA1D_out = RA1D;
assign RA2D_out = RA2D;
assign RA1E_out = RA1E;
assign RA2E_out = RA2E;
assign WA3E_out = WA3E;
assign WA3M_out = WA3M;
assign WA3W_out = WA3W;
//wire declerations
wire [31:0] PC_plus_4F;
wire [31:0] Resultante;
wire [31:0] PC_ , PC__;
wire [31:0] PCF;
wire [31:0] InstructionF;
wire [31:0] InstructionD;
assign OP = InstructionD[27:26];
assign FUNCT = InstructionD[25:20];
assign COND = InstructionD[31:28];
assign Rd = InstructionD[15:12];
wire [3:0] RA1D;
wire [3:0] RA2D;
wire [3:0] WA3W;
wire [3:0] WA3W_final;



wire [31:0] RD1;
wire [31:0] RD2;

wire [31:0] Extended_data;
wire [31:0] shifter_input;

wire [4:0] shifter_amount;
wire [1:0] shifter_type;

wire [31:0] shifted_out;



// Mux of PC instantatiation
Mux_2to1 #(32) Mux_of_PC(
    .select(PC_select),
    .input_0(PC_plus_4F),
    .input_1(Resultante),
    .output_value(PC_)
);

Mux_2to1 #(32) Mux_of_PC_2(
    .select(PC_select_W),
    .input_0(PC_),
    .input_1(ALU_resultE),
    .output_value(PC__)
);
// PC register
Register_rsten #(32) PC_register(
    .clk(clk),
    .reset(reset),
    .we(~StallF),
    .DATA(PC__),
    .OUT(PCF)
);

// PC plus 4 adders
Adder #(32) PC_plus_4_adder(
    .DATA_A(PCF),
    .DATA_B(32'd4),
    .OUT(PC_plus_4F)
);

// Instruction memory instantiation
Instruction_memory #(4, 32) Instruction_memo(
    .ADDR(PCF),
    .RD(InstructionF)
);

// Instruction Register
Register_rsten #(32) Instruction_register(
    .clk(clk),
    .reset(FlushD || reset),
    .we(~StallD),
    .DATA(InstructionF),
    .OUT(InstructionD)
);

// MUX of RA1 instantiation
Mux_2to1 #(4) Mux_of_RA1D(
    .select(RA1_select),
    .input_0(InstructionD[19:16]),
    .input_1(4'b1111),
    .output_value(RA1D)
);

// MUX of RA2 instantiation
Mux_2to1 #(4) Mux_of_RA2D(
    .select(RA2_select),
    .input_0(InstructionD[3:0]),
    .input_1(InstructionD[15:12]),
    .output_value(RA2D)
);

//MUX OF WA3 instantiation
Mux_2to1 #(4) Mux_of_R14(
    .select(R14_select),
    .input_0(WA3W),
    .input_1(4'b1110),
    .output_value(WA3W_final)
);
wire [31:0] R14_data;
Mux_2to1 #(32) Mux_of_R14_data(
    .select(R14_select),
    .input_0(Resultante),
    .input_1(PCF),
    .output_value(R14_data)
);

// Register file instantiation
Register_file #(32) Register_file_(
    .clk(clk),
    .write_enable(Register_file_write_enable),
    .reset(reset),
    .Source_select_0(RA1D),
    .Source_select_1(RA2D),
    .Debug_Source_select(Debug_in),
    .Destination_select(WA3W_final),
    .DATA(R14_data),
    .Reg_15(PC_plus_4F),
    .out_0(RD1),
    .out_1(RD2),
    .Debug_out(Debug_out)
);

// Extender instantiation
Extender Extender_baba(
    .Extended_data(Extended_data),
    .DATA(InstructionD[23:0]),
    .select(extender_select)
);

//Shifter input select
Mux_2to1 #(32) Mux_of_shifter_input(
    .select(shifter_input_select),
    .input_0(RD2),
    .input_1(Extended_data),
    .output_value(shifter_input)
);

//Shifter Amount select
Mux_2to1 #(5) Mux_of_shifter_amount(
    .select(shifter_amount_select),
    .input_0(InstructionD[11:7]),
    .input_1(5'b00001),
    .output_value(shifter_amount)
);

//Shifter type select
Mux_2to1 #(2) Mux_of_shifter_type(
    .select(shifter_type_select),
    .input_0(InstructionD[6:5]),
    .input_1(2'b11),
    .output_value(shifter_type)
);

//Shifter baba
shifter #(32) shifter_baba(
    .control(shifter_type),
    .shamt(shifter_amount),
    .DATA(shifter_input),
    .OUT(shifted_out)
);

//Execute Registers
wire [31:0] RD1E, RD2E, shifted_outE, ImmediateE;
wire [3:0] WA3E, RA1E, RA2E;
Register_rsten #(32) Execute_register_RD1(
    .clk(clk),
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(RD1),
    .OUT(RD1E)
);

Register_rsten #(32) Execute_register_RD2(
    .clk(clk),
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(RD2),
    .OUT(RD2E)
);

Register_rsten #(32) Execute_register_RD2_shifted(      //mifght be unnecessary
    .clk(clk),  
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(shifted_out),
    .OUT(shifted_outE)
);

Register_rsten #(32) Execute_register_Immediate(
    .clk(clk),
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(Extended_data),
    .OUT(ImmediateE)
);

Register_rsten #(4) Execute_register_WA3(
    .clk(clk),
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(InstructionD[15:12]),
    .OUT(WA3E)
);

Register_rsten #(4) Execute_register_RA1_E(
    .clk(clk),
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(RA1D),
    .OUT(RA1E)
);

Register_rsten #(4) Execute_register_RA2_E(
    .clk(clk),
    .reset(FlushE || reset),
    .we(1'b1),
    .DATA(RA2D),
    .OUT(RA2E)
);
//MUXES OF ALU
wire [31:0] ALU_resultM , SRCAE, SRCBE_1, SRCBE_2;

Mux_4to1 #(32) Mux_of_A(
    .select(ForwardAE),
    .input_0(RD1E),
    .input_1(Resultante),
    .input_2(ALU_resultM),
    .input_3(),
    .output_value(SRCAE)
);

Mux_4to1 #(32) Mux_of_B(
    .select(ForwardBE),
    .input_0(shifted_outE),
    .input_1(Resultante),
    .input_2(ALU_resultM),
    .input_3(RD2E),
    .output_value(SRCBE_1)
);

Mux_2to1 #(32) Mux_of_B_2(
    .select(ALUsrcE),
    .input_0(SRCBE_1),
    .input_1(ImmediateE),
    .output_value(SRCBE_2)
);


//ALU instantiation
wire [31:0] ALU_resultE;
wire CO, OVF, N, Z;
ALU #(32) ALU_unit(
    .control(ALU_control),
    .CI(1'b0),
    .DATA_A(SRCAE),
    .DATA_B(SRCBE_2),
    .OUT(ALU_resultE),
    .CO(CO),
    .OVF(OVF),
    .N(N),
    .Z(Z)
);

//Memory Register
wire [31:0] write_data_m;
wire [3:0] WA3M;
Register_rsten #(32) Memory_register_ALU_result(
    .clk(clk),
    .reset(FlushD),
    .we(1'b1),
    .DATA(ALU_resultE),
    .OUT(ALU_resultM)
);

Register_rsten #(32) Memory_register_WriteData(      //mifght be unnecessary
    .clk(clk),  
    .reset(1'b0),
    .we(1'b1),
    .DATA(SRCBE_1),
    .OUT(write_data_m)  
);

Register_rsten #(4) Memory_register_WA3(
    .clk(clk),
    .reset(1'b0),
    .we(1'b1),
    .DATA(WA3E),
    .OUT(WA3M)
);

//Data Memory
wire [31:0] Read_data_m;
Memory #(4, 32) Data_memory(
    .clk(clk),
    .WE(data_memory_write_enable),
    .ADDR(ALU_resultM),
    .WD(write_data_m),
    .RD(Read_data_m)
);

//Write Back Register
wire [31:0] Read_data_w, ALU_result_w;
Register_rsten #(32) WB_register_Read_dataW(
    .clk(clk),
    .reset(1'b0),
    .we(1'b1),
    .DATA(Read_data_m),
    .OUT(Read_data_w)
);

Register_rsten #(32) WB_register_aluoutW(      //mifght be unnecessary
    .clk(clk),  
    .reset(1'b0),
    .we(1'b1),
    .DATA(ALU_resultM),
    .OUT(ALU_result_w)  
);

Register_rsten #(4) WB_register_WA3W(
    .clk(clk),
    .reset(1'b0),
    .we(1'b1),
    .DATA(WA3M),
    .OUT(WA3W)
);

//result mux
Mux_2to1 #(32) Mux_of_result(
    .select(MemtoregW),
    .input_1(Read_data_w),
    .input_0(ALU_result_w),
    .output_value(Resultante)
);
endmodule