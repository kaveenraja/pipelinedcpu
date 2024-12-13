module mux4_1(inA, inB, inC, inD, s, out);

	parameter WIDTH = 1;
	input  [WIDTH-1 :0]	inA, inB, inC, inD;
	input  [3:0]	s;
	output [WIDTH-1 :0]	out;

assign out = ({WIDTH{s[0]}} & inA) | ({WIDTH{s[1]}} & inB) | ({WIDTH{s[2]}} & inC) | ({WIDTH{s[3]}} & inD);
endmodule
