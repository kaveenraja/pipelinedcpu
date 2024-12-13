module overflow(a, b, s, carry, sign, out);
	input a, b, s, carry, sign;
	output out;
	
	wire comparemsb;

	assign comparemsb =  (a & b & ~s) | (~a & ~b & s);
	
	mux2_1 mux(.a(carry), .b(comparemsb), .s(sign), .out(out));


endmodule