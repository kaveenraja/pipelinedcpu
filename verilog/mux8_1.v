module mux8_1(inA, inB, inC, inD, inE, inF, inG, inH, s, out);

	parameter WIDTH = 1;
	input  [WIDTH-1 :0]	inA, inB, inC, inD, inE, inF, inG, inH;
	input  [7:0]	s;
	output [WIDTH-1 :0]	out;

assign out = ({WIDTH{s[0]}} & inA) | ({WIDTH{s[1]}} & inB) | ({WIDTH{s[2]}} & inC) | ({WIDTH{s[3]}} & inD) | ({WIDTH{s[4]}} & inE) | ({WIDTH{s[5]}} & inF) | ({WIDTH{s[6]}} & inG) | ({WIDTH{s[7]}} & inH);
endmodule
