module fulladder16 (A, B, S, Cout);
input [15:0] A;
input [15:0] B;
output [15:0] S;
output Cout;

wire [2:0] carry;

fulladder4 fa0(.A(A[3:0]),  .B(B[3:0]),  .Cin(1'b0),     .S(S[3:0]),  .Cout(carry[0]));
fulladder4 fa1(.A(A[7:4]),  .B(B[7:4]),  .Cin(carry[0]), .S(S[7:4]),  .Cout(carry[1]));
fulladder4 fa2(.A(A[11:8]), .B(B[11:8]), .Cin(carry[1]), .S(S[11:8]), .Cout(carry[2]));
fulladder4 fa3(.A(A[15:12]),.B(B[15:12]),.Cin(carry[2]), .S(S[15:12]),.Cout(Cout));


endmodule

module fulladder4(A, B, Cin, S, Cout);
input [3:0] A;
input [3:0] B;
input Cin;
output [3:0] S;
output Cout;

wire [2:0] carry;

fulladder1 fa0(.A(A[0]), .B(B[0]), .Cin(Cin),      .S(S[0]), .Cout(carry[0]));
fulladder1 fa1(.A(A[1]), .B(B[1]), .Cin(carry[0]), .S(S[1]), .Cout(carry[1]));
fulladder1 fa2(.A(A[2]), .B(B[2]), .Cin(carry[1]), .S(S[2]), .Cout(carry[2]));
fulladder1 fa3(.A(A[3]), .B(B[3]), .Cin(carry[2]), .S(S[3]), .Cout(Cout));

endmodule

module fulladder1 (A, B, Cin, S, Cout);
input A;
input B;
input Cin;
output S;
output Cout;

wire x1, x2, x3, x4, x5, x6;

//not1  n1(.in1(), .out());
//xor2  xo1(.in1(), .in2(), .out());
//nand2 na1(.in1(), .in2(), .out());
//nor2  no1(.in1(), .in2(), .out());

// Sum
xor2  xo1(.in1(A), .in2(B), .out(x1));
xor2  xo2(.in1(x1), .in2(Cin), .out(S));

//Cout
nand2 na1(.in1(x1), .in2(Cin), .out(x2));
not1  n1(.in1(x2), .out(x3));

nand2 na2(.in1(A), .in2(B), .out(x4));
not1  n2(.in1(x4), .out(x5));

nor2 na3(.in1(x3), .in2(x5), .out(x6));
not1  n3(.in1(x6), .out(Cout));

endmodule
