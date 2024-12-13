module op(msb, in, oper, out);

	input [1:0] oper;
	input 		msb;
	input       in;

	output      out;

	mux2_1 mux0(.a(in), .b(1'b0), .s(oper[1]), .out(out));

endmodule
