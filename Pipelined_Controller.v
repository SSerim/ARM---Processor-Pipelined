module Pipelined_Controller (
    input clk,
    input reset,

    // Controller Inputs from Instruction and ALU Flags
    input [1:0] OP,
    input [5:0] FUNCT,
    input [3:0] COND,
    input [3:0] Rd,
    input [3:0] Flags,
    // Controller Outputs to Datapath
    output reg PC_select, PC_select_W,
    output reg RA1_select,
    output reg RA2_select,
    output reg Register_file_write_enable,
    output reg [1:0] extender_select,
    output reg R14_select,
    output reg shifter_input_select,
    output reg shifter_amount_select,
    output reg shifter_type_select,
    output reg ALUsrcE,
    output reg MemtoregW,
    output reg [3:0] ALU_control,
    output reg data_memory_write_enable,
    // Controller Outputs to Hazard Unit
    output reg RegwriteM_hazard,
    output reg RegwriteW_hazard,
    output reg Memtoreg_hazard,
    output reg BranchtakenE_hazard,
    output reg PCSrcD_hazard,
    output reg PCSrcE_hazard,
    output reg PCSrcM_hazard,
    output reg PCSrcW_hazard,
    // input from hazard unit
    input FlushE
);
reg zero, cond_met_output;

    //zero register
    Register_rsten #(1) Flags3_2 (
        .clk(clk),
        .reset(reset),
        .we((FUNCT[0] == 1'b1) && (OP == 2'b00)),
        .DATA(Flags[0]),
        .OUT(zero)
    );
    //Condition met register
    Register_simple #(.WIDTH(1)) Cond_ex(
        .clk(clk),
        .DATA(condition_met),
        .OUT(cond_met_output)
    );
reg condition_met;
    //Condition Decoder
    always @(*) begin
        condition_met = 0;
        case (COND)
            4'b0000: condition_met = zero;               // EQ: equal
            4'b0001: condition_met = ~zero;              // NE: not equal
            4'b1110: condition_met = 1;               // AL: always
            default: condition_met = 1;               // always
        endcase
    end

parameter [2:0]
    FETCH = 3'd0,
    DECODE = 3'd1,
    EXECUTE = 3'd2,
    MEMORY = 3'd3,
    WRITEBACK = 3'd4;
    reg [2:0] state, next_state;


reg PC_selectD, PC_select_W_D;
reg Register_file_write_enableD,R14_selectD,ALUsrcD,MemtoregD,data_memory_write_enableD;
reg [3:0] ALU_controlD;

always @(*) begin       
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0100; 
            data_memory_write_enableD = 1'b0;    
    case (OP)
    2'b00: begin    // Data Processing Instructions
        case (FUNCT)
        6'b010000: begin    // ADD Operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0100; 
            data_memory_write_enableD = 1'b0;
        end
        6'b000100: begin    // SUB Operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0010; 
            data_memory_write_enableD = 1'b0;
        end
        6'b000000: begin    // AND Operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0000; 
            data_memory_write_enableD = 1'b0;
        end
        6'b011000: begin    // ORR Operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b1100; 
            data_memory_write_enableD = 1'b0;
        end
        6'b011010: begin    // MOV Operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b1101; 
            data_memory_write_enableD = 1'b0;
        end
        6'b111010: begin    // MOV with Immediate Operation
            PC_selectD  = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b1;    // will be sent to datapath directly
            shifter_type_select = 1'b1;   // will be sent to datapath directly
            shifter_amount_select = 1'b1;   // will be sent to datapath directly
            ALUsrcD = 1'b1;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b1101; 
            data_memory_write_enableD = 1'b0;
        end
        6'b010101: begin    // Compare Operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b0;
            extender_select = 2'b00;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0110; 
            data_memory_write_enableD = 1'b0;
        end
        endcase
    
    end
    2'b01: begin
        case(FUNCT)
        6'b011000: begin    // STR operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b1;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b0;
            extender_select = 2'b01;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b1;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0100; 
            data_memory_write_enableD = 1'b1;
        end
        6'b011001: begin    // LDR operation
            PC_selectD = 1'b0;
            PC_select_W_D  = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b1;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b01;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b1;
            MemtoregD = 1'b1;
            ALU_controlD = 4'b0100; 
            data_memory_write_enableD = 1'b0;
        end
        endcase
    end
    2'b10: begin
        case(FUNCT)
        6'b100000: begin        // Branch Operation
            PC_selectD = 1'b1;
            PC_select_W_D = 1'b1;  
            RA1_select = 1'b1;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b0;
            extender_select = 2'b10;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b1;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0100; 
            data_memory_write_enableD = 1'b0;
        end
        6'b11????: begin        // Branch and Link Operation
            PC_selectD = 1'b1;
            PC_select_W_D = 1'b0;  
            RA1_select = 1'b1;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b1;
            extender_select = 2'b10;    // will be sent to datapath directly
            R14_selectD = 1'b1;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b1;
            MemtoregD = 1'b0;
            ALU_controlD = 4'b0100; 
            data_memory_write_enableD = 1'b0;
        end
        6'b010010: begin        // Branch and exchange Operation
            PC_selectD = 1'b1;
            PC_select_W_D = 1'b0;  
            RA1_select = 1'b0;      // will be sent to datapath directly
            RA2_select = 1'b0;          // will be sent to datapath directly
            Register_file_write_enableD = 1'b0;
            extender_select = 2'b10;    // will be sent to datapath directly
            R14_selectD = 1'b0;
            shifter_input_select = 1'b0;    // will be sent to datapath directly
            shifter_type_select = 1'b0;   // will be sent to datapath directly
            shifter_amount_select = 1'b0;   // will be sent to datapath directly
            ALUsrcD = 1'b0;
            MemtoregD = 1'b0;   
            ALU_controlD = 4'b1101; 
            data_memory_write_enableD = 1'b0;
        end
        endcase
    end
    endcase
end

//execute registers

wire PC_selectE, PC_select_W_E;
wire Register_file_write_enableE,R14_selectE,ALUsrc_E,MemtoregE,data_memory_write_enableE;
wire [3:0] ALU_controlE;

Register_rsten #(1) pc_sel_execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(PC_selectD),
        .OUT(PC_selectE)
    );

Register_rsten #(1) pc_sel_W_execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(PC_select_W_D),
        .OUT(PC_select_W_E)
    );
Register_rsten #(1) Reg_file_WE_execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(Register_file_write_enableD),
        .OUT(Register_file_write_enableE)
    );
Register_rsten #(1) R14_Execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(R14_selectD),
        .OUT(R14_selectE)
    );
Register_rsten #(1) ALUSRC_Execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(ALUsrcD),
        .OUT(ALUsrc_E)
    );
Register_rsten #(1) Mem_to_reg_execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(MemtoregD),
        .OUT(MemtoregE)
    );
Register_rsten #(1) data_memory_execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(data_memory_write_enableD),
        .OUT(data_memory_write_enableE)
    );
Register_rsten #(4) ALU_control_execute (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(ALU_controlD),
        .OUT(ALU_controlE)
    );

assign ALUsrcE = ALUsrc_E;
assign ALU_control = ALU_controlE;
assign PC_select_W = PC_select_W_E && cond_met_output;

// Memory Registers
wire PC_selectM, Register_file_write_enableM,R14_selectM,MemtoregM,data_memory_write_enableM;
Register_rsten #(1) pc_sel_memo (
        .clk(clk),
        .reset(reset || FlushE),
        .we(1'b1),
        .DATA(PC_selectE && cond_met_output),
        .OUT(PC_selectM)
    );
Register_rsten #(1) Reg_file_WE_memo (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(Register_file_write_enableE && cond_met_output),
        .OUT(Register_file_write_enableM)
    );
Register_rsten #(1) R14_memo (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(R14_selectE && cond_met_output),
        .OUT(R14_selectM)
    );
Register_rsten #(1) Mem_to_reg_memo (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(MemtoregE && cond_met_output),
        .OUT(MemtoregM)
    );

Register_rsten #(1) data_memory_memo (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(data_memory_write_enableE && cond_met_output),
        .OUT(data_memory_write_enableM)
    );

assign data_memory_write_enable = data_memory_write_enableM;
// Write Back Registers
wire PC_selectWB, Register_file_write_enableWB,R14_selectWB,MemtoregWB;
Register_rsten #(1) pc_sel_WB (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(PC_selectM),
        .OUT(PC_selectWB)
    );
Register_rsten #(1) Reg_file_WE_WB (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(Register_file_write_enableM),
        .OUT(Register_file_write_enableWB)
    );
Register_rsten #(1) R14_WB (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(R14_selectM),
        .OUT(R14_selectWB)
    );
Register_rsten #(1) Mem_to_reg_WB (
        .clk(clk),
        .reset(reset),
        .we(1'b1),
        .DATA(MemtoregM),
        .OUT(MemtoregWB)
    );

assign PC_select = PC_selectWB;
assign Register_file_write_enable = Register_file_write_enableWB;
assign R14_select = R14_selectWB;
assign MemtoregW = MemtoregWB;


//Hazard Unit inputs
assign RegwriteM_hazard = Register_file_write_enableM;
assign RegwriteW_hazard = Register_file_write_enableWB;
assign Memtoreg_hazard = MemtoregE;
assign BranchtakenE_hazard = PC_select_W;
assign PCSrcD_hazard = PC_selectD;
assign PCSrcE_hazard = PC_selectE;
assign PCSrcM_hazard = PC_selectM;
assign PCSrcW_hazard = PC_selectWB;
endmodule