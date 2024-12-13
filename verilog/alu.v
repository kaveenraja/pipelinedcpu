/*
    CS/ECE 552 FALL '22
    Homework #2, Problem 3

*/
module alu (InA, InB, Cin, Oper, invA, invB, sign, Out, Cfl, Sfl, Ofl, Zfl);

    parameter OPERAND_WIDTH = 16;    
       
    input  [OPERAND_WIDTH -1:0] InA ; // Input operand A
    input  [OPERAND_WIDTH -1:0] InB ; // Input operand B
    input                       Cin ; // Carry in
    input  [3:0] 				Oper; // Operation type
    input                       invA; // Signal to invert A
    input                       invB; // Signal to invert B
    input                       sign; // Signal for signed operation
    output [OPERAND_WIDTH -1:0] Out ; // Result of computation
	output 						Cfl; // Carry flag
	output						Sfl; // Sign Flag
    output                      Ofl; // Signal if overflow occured
    output                      Zfl; // Signal if Out is 0

    /* WIRE */
	wire  [OPERAND_WIDTH -1:0] invA_in;
	wire  [OPERAND_WIDTH -1:0] invB_in;
	wire  [OPERAND_WIDTH -1:0] invA_out;
	wire  [OPERAND_WIDTH -1:0] invB_out;

	wire  [OPERAND_WIDTH -1:0] logic_in_a;
	wire  [OPERAND_WIDTH -1:0] logic_in_b_pre;
	wire  [OPERAND_WIDTH -1:0] logic_in_b;

	wire  [OPERAND_WIDTH -1:0] shifter_in_a;
	wire  [OPERAND_WIDTH -1:0] shifter_in_b;

	wire  [OPERAND_WIDTH -1:0] shifter_out_1;
	wire  [OPERAND_WIDTH -1:0] shifter_out_0;
	wire  [OPERAND_WIDTH -1:0] shifter_out;

	wire  [OPERAND_WIDTH -1:0] xor_out;
	wire  [OPERAND_WIDTH -1:0] and_out;
	wire  [OPERAND_WIDTH -1:0] cla_sum;
	wire                       cla_cout;

	wire  [OPERAND_WIDTH -1:0] logic_out_pre;
	wire  [OPERAND_WIDTH -1:0] logic_out;

	wire  [OPERAND_WIDTH -1:0] flag_in;
	wire  [OPERAND_WIDTH -1:0] flag_out;
	

	/* Inverter */
	assign invA_in = ~InA;
	assign invB_in = ~InB;

	mux2_1 mux0[15:0](.a(InA), .b(invA_in), .s( {16{invA}} ), .out(invA_out));
	mux2_1 mux1[15:0](.a(InB), .b(invB_in), .s( {16{invB}} ), .out(invB_out));

	demux1_2 demux0[15:0](.Inp(invA_out), .S( {16{Oper[2]}} ), .OutA(shifter_in_a), .OutB(logic_in_a));
	demux1_2 demux1[15:0](.Inp(invB_out), .S( {16{Oper[2]}} ), .OutA(shifter_in_b), .OutB(logic_in_b_pre));

	assign logic_in_b = (Oper === 4'b0110) ? 16'h0000 : logic_in_b_pre;
    

	/* Shifter */

	inverter16 inv0(.in(shifter_in_a), .en(1'b1), .out(shifter_out_1));
	shifter    shf0(.In(shifter_in_a), .ShAmt(shifter_in_b[3:0]), .Oper(Oper[1:0]), .Out(shifter_out_0));
	
	mux2_1 mux2[15:0](.a(shifter_out_0), .b(shifter_out_1), .s( {16{Oper[3]}} ), .out(shifter_out));

	/* ALU */
	assign xor_out = logic_in_a ^ logic_in_b;
	assign and_out = logic_in_a & logic_in_b;

	cla_16b cla0(.a(logic_in_a), .b(logic_in_b), .c_in(Cin), .sum(cla_sum), .c_out(cla_cout));
	overflow of0(.a(logic_in_a[15]), .b(logic_in_b[15]), .s(cla_sum[15]), .carry(cla_cout), .sign(sign), .out(Ofl));
	assign Cfl = cla_cout;

	ecmux4_1 ecmux0[15:0](.a(cla_sum), .b(and_out), .c(cla_sum), .d(xor_out), .s( {16{Oper[1:0]}} ), .out(logic_out_pre));
	mux2_1 mux3[15:0](.a(logic_out_pre), .b(cla_sum), .s( {16{Oper[3]}} ), .out(logic_out));
	

	/* FLAGS */

	assign flag_in = logic_out | shifter_out;

	assign Zfl = flag_in === 16'b0000000000000000;
	assign Sfl = flag_in[15];

	set set0(.Of(Ofl), .Cf(Cfl), .Zf(Zfl), .Sf(Sfl), .Oper(Oper[1:0]), .out(flag_out));


	/* OUT */
	
	assign Out = (Oper[3] & Oper[2]) ? flag_out : flag_in;

endmodule
