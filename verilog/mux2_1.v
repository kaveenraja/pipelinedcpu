module mux2_1(a, b, s, out);
	input a, b, s;
	output out;

	assign out = (a & ~s) | (b & s);

endmodule
