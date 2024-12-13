module ecmux4_1(a, b, c, d, s, out);
	input a, b, c, d;
	input [1:0] s;
	output out;

	reg o;

	always @(s or a or b or c or d)
	begin
		case(s)
			2'b00: o = a;
			2'b01: o = b;
			2'b10: o = c;
			2'b11: o = d;
		endcase
	end

	assign out = o;

endmodule
