/*
    CS/ECE 552 FALL '22
    Homework #2, Problem 2
    
    A barrel shifter module.  It is designed to shift a number via rotate
    left, shift left, shift right arithmetic, or shift right logical based
    on the 'Oper' value that is passed in.  It uses these
    shifts to shift the value any number of bits.
 */
module shifter (In, ShAmt, Oper, Out);

    // declare constant for size of inputs, outputs, and # bits to shift
    parameter OPERAND_WIDTH = 16;
    parameter SHAMT_WIDTH   =  4;
    parameter NUM_OPERATIONS = 2;

    input  [OPERAND_WIDTH -1:0] In   ; // Input operand
    input  [SHAMT_WIDTH   -1:0] ShAmt; // Amount to shift/rotate
    input  [NUM_OPERATIONS-1:0] Oper ; // Operation type
    output [OPERAND_WIDTH -1:0] Out  ; // Result of shift/rotate

	// INTERMEDIATE WIRES

	wire   [OPERAND_WIDTH -1:0] inverter_out0;
	wire   [OPERAND_WIDTH -1:0] shift_8b_out;
	wire   [OPERAND_WIDTH -1:0] shift_4b_out;
	wire   [OPERAND_WIDTH -1:0] shift_2b_out;
	wire   [OPERAND_WIDTH -1:0] shift_1b_out;

	wire   [OPERAND_WIDTH -1:0] shift_8b_mIn;
	wire   [OPERAND_WIDTH -1:0] shift_4b_mIn;
	wire   [OPERAND_WIDTH -1:0] shift_2b_mIn;
	wire   [OPERAND_WIDTH -1:0] shift_1b_mIn;
	wire   [7:0]                shift_8b_op_out;
	wire   [3:0]                shift_4b_op_out;
	wire   [1:0]                shift_2b_op_out;
	wire                        shift_1b_op_out;
	wire   [7:0]                shift_8b_op_in;
	wire   [3:0]                shift_4b_op_in;
	wire   [1:0]                shift_2b_op_in;



    /* In -> STAGE 1 -> inverter_out0 */

	inverter16 inverter0(.in(In), .en(~Oper[0]), .out(inverter_out0));

    /* inverter_out0 -> STAGE 2 ->  shift_8b_out*/

	assign shift_8b_op_in[0] = inverter_out0[0];
	assign shift_8b_op_in[1] = inverter_out0[1];
	assign shift_8b_op_in[2] = inverter_out0[2];
	assign shift_8b_op_in[3] = inverter_out0[3];
	assign shift_8b_op_in[4] = inverter_out0[4];
	assign shift_8b_op_in[5] = inverter_out0[5];
	assign shift_8b_op_in[6] = inverter_out0[6];
	assign shift_8b_op_in[7] = inverter_out0[7];

	op op0[7:0](.msb( {8{inverter_out0[15]}} ), .in(shift_8b_op_in), .oper( {8{Oper}} ), .out(shift_8b_op_out));

	assign shift_8b_mIn[15] = shift_8b_op_out[7];
	assign shift_8b_mIn[14] = shift_8b_op_out[6];
	assign shift_8b_mIn[13] = shift_8b_op_out[5];
	assign shift_8b_mIn[12] = shift_8b_op_out[4];
	assign shift_8b_mIn[11] = shift_8b_op_out[3];
	assign shift_8b_mIn[10] = shift_8b_op_out[2];
	assign shift_8b_mIn[9] = shift_8b_op_out[1];
	assign shift_8b_mIn[8] = shift_8b_op_out[0];
	assign shift_8b_mIn[7] = inverter_out0[15];
	assign shift_8b_mIn[6] = inverter_out0[14];
	assign shift_8b_mIn[5] = inverter_out0[13];
	assign shift_8b_mIn[4] = inverter_out0[12];
	assign shift_8b_mIn[3] = inverter_out0[11];
	assign shift_8b_mIn[2] = inverter_out0[10];
	assign shift_8b_mIn[1] = inverter_out0[9];
	assign shift_8b_mIn[0] = inverter_out0[8];

	mux2_1 mux_shift_8b[15:0](.a(inverter_out0), .b(shift_8b_mIn), .s( {16{ShAmt[3]}} ), .out(shift_8b_out));
	
    /* shift_8b_out -> STAGE 3 ->  shift_4b_out*/

	assign shift_4b_op_in[0] = shift_8b_out[0];
	assign shift_4b_op_in[1] = shift_8b_out[1];
	assign shift_4b_op_in[2] = shift_8b_out[2];
	assign shift_4b_op_in[3] = shift_8b_out[3];
	
	op op1[3:0](.msb( {4{shift_8b_out[15]}} ), .in(shift_4b_op_in), .oper( {4{Oper}} ), .out(shift_4b_op_out));

	assign shift_4b_mIn[15] = shift_4b_op_out[3];
	assign shift_4b_mIn[14] = shift_4b_op_out[2];
	assign shift_4b_mIn[13] = shift_4b_op_out[1];
	assign shift_4b_mIn[12] = shift_4b_op_out[0];
	assign shift_4b_mIn[11] = shift_8b_out[15];
	assign shift_4b_mIn[10] = shift_8b_out[14];
	assign shift_4b_mIn[9] = shift_8b_out[13];
	assign shift_4b_mIn[8] = shift_8b_out[12];
	assign shift_4b_mIn[7] = shift_8b_out[11];
	assign shift_4b_mIn[6] = shift_8b_out[10];
	assign shift_4b_mIn[5] = shift_8b_out[9];
	assign shift_4b_mIn[4] = shift_8b_out[8];
	assign shift_4b_mIn[3] = shift_8b_out[7];
	assign shift_4b_mIn[2] = shift_8b_out[6];
	assign shift_4b_mIn[1] = shift_8b_out[5];
	assign shift_4b_mIn[0] = shift_8b_out[4];

	mux2_1 mux_shift_4b[15:0](.a(shift_8b_out), .b(shift_4b_mIn), .s( {16{ShAmt[2]}} ), .out(shift_4b_out));
	
    /* shift_4b_out -> STAGE 4 ->  shift_2b_out*/

	assign shift_2b_op_in[0] = shift_4b_out[0];
	assign shift_2b_op_in[1] = shift_4b_out[1];

	op op2[1:0](.msb( {2{shift_4b_out[15]}} ), .in(shift_2b_op_in), .oper( {2{Oper}} ), .out(shift_2b_op_out));

	assign shift_2b_mIn[15] = shift_2b_op_out[1];
	assign shift_2b_mIn[14] = shift_2b_op_out[0];
	assign shift_2b_mIn[13] = shift_4b_out[15];
	assign shift_2b_mIn[12] = shift_4b_out[14];
	assign shift_2b_mIn[11] = shift_4b_out[13];
	assign shift_2b_mIn[10] = shift_4b_out[12];
	assign shift_2b_mIn[9] = shift_4b_out[11];
	assign shift_2b_mIn[8] = shift_4b_out[10];
	assign shift_2b_mIn[7] = shift_4b_out[9];
	assign shift_2b_mIn[6] = shift_4b_out[8];
	assign shift_2b_mIn[5] = shift_4b_out[7];
	assign shift_2b_mIn[4] = shift_4b_out[6];
	assign shift_2b_mIn[3] = shift_4b_out[5];
	assign shift_2b_mIn[2] = shift_4b_out[4];
	assign shift_2b_mIn[1] = shift_4b_out[3];
	assign shift_2b_mIn[0] = shift_4b_out[2];

	mux2_1 mux_shift_2b[15:0](.a(shift_4b_out), .b(shift_2b_mIn), .s( {16{ShAmt[1]}} ), .out(shift_2b_out));

    /* shift_2b_out -> STAGE 5 ->  shift_1b_out*/

	op op3(.msb(shift_2b_out[15]), .in(shift_2b_out[0]), .oper(Oper), .out(shift_1b_op_out));

	assign shift_1b_mIn[15] = shift_1b_op_out;
	assign shift_1b_mIn[14] = shift_2b_out[15];
	assign shift_1b_mIn[13] = shift_2b_out[14];
	assign shift_1b_mIn[12] = shift_2b_out[13];
	assign shift_1b_mIn[11] = shift_2b_out[12];
	assign shift_1b_mIn[10] = shift_2b_out[11];
	assign shift_1b_mIn[9] = shift_2b_out[10];
	assign shift_1b_mIn[8] = shift_2b_out[9];
	assign shift_1b_mIn[7] = shift_2b_out[8];
	assign shift_1b_mIn[6] = shift_2b_out[7];
	assign shift_1b_mIn[5] = shift_2b_out[6];
	assign shift_1b_mIn[4] = shift_2b_out[5];
	assign shift_1b_mIn[3] = shift_2b_out[4];
	assign shift_1b_mIn[2] = shift_2b_out[3];
	assign shift_1b_mIn[1] = shift_2b_out[2];
	assign shift_1b_mIn[0] = shift_2b_out[1];

	mux2_1 mux_shift_1b[15:0](.a(shift_2b_out), .b(shift_1b_mIn), .s( {16{ShAmt[0]}} ), .out(shift_1b_out));

    /* shift_1b_out -> STAGE 6 ->  Out*/ 

	inverter16 inverter1(.in(shift_1b_out), .en(~Oper[0]), .out(Out));

endmodule
