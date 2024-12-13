/*
    CS/ECE 552 FALL '22
    Homework #2, Problem 1
    
    a 16-bit CLA module
*/
module cla_16b(sum, c_out, a, b, c_in);

    // declare constant for size of inputs, outputs (N)
    parameter   N = 16;

    output [N-1:0] sum;
    output         c_out;
    input [N-1: 0] a, b;
    input          c_in;

    // YOUR CODE HERE
	wire [4:0] carries;
	assign carries[0] = c_in;

	cla_4b cla[3:0](.a(a), .b(b), .c_in(carries[3:0]), .c_out(carries[4:1]), .sum(sum));

	assign c_out = carries[4];

endmodule

/*
    CS/ECE 552 FALL'22
    Homework #2, Problem 1
    
    a 4-bit CLA module
*/
module cla_4b(sum, c_out, a, b, c_in);

    // declare constant for size of inputs, outputs (N)
    parameter   N = 4;

    output [N-1:0] sum;
    output         c_out;
    input [N-1: 0] a, b;
    input          c_in;

	wire [N-1: 0]   null;
    wire [N: 0]   carries;
    wire [N-1: 0] g;
    wire [N-1: 0] p;


    assign g = a & b;
    assign p = a | b;

	assign carries[0] = c_in;
	assign carries[1] = g[0] | (p[0] & carries[0]);
	assign carries[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & carries[0]);
	assign carries[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & carries[0]);
	assign carries[4] = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]) | (p[3] & p[2] & p[1] & p[0] & carries[0]);

	fullAdder_1b adder[N-1: 0] (.a(a), .b(b), .c_in(carries[N-1:0]), .c_out(null), .s(sum));

	assign c_out = carries[4];

	

endmodule

/*
    CS/ECE 552 FALL '22
    Homework #2, Problem 1
    
    a 1-bit full adder
*/
module fullAdder_1b(s, c_out, a, b, c_in);
    output s;
    output c_out;
    input  a, b;
    input  c_in;

    wire x1, x2, x3, x4, x5, x6;

    // Sum
    xor2  xo1(.in1(a), .in2(b), .out(x1));
    xor2  xo2(.in1(x1), .in2(c_in), .out(s));

    //Cout
    nand2 na1(.in1(x1), .in2(c_in), .out(x2));
    not1  n1(.in1(x2), .out(x3));

    nand2 na2(.in1(a), .in2(b), .out(x4));
    not1  n2(.in1(x4), .out(x5));

    nor2 na3(.in1(x3), .in2(x5), .out(x6));
    not1  n3(.in1(x6), .out(c_out));

endmodule
