
module inverter16(in, en, out);
	input  [15:0] in;
	input         en;
	output [15:0] out;

	wire [15:0]   inverted;

	assign inverted[0] = in[15];
	assign inverted[1] = in[14];
	assign inverted[2] = in[13];
	assign inverted[3] = in[12];
	assign inverted[4] = in[11];
	assign inverted[5] = in[10];
	assign inverted[6] = in[9];
	assign inverted[7] = in[8];
	assign inverted[8] = in[7];
	assign inverted[9] = in[6];
	assign inverted[10] = in[5];
	assign inverted[11] = in[4];
	assign inverted[12] = in[3];
	assign inverted[13] = in[2];
	assign inverted[14] = in[1];
	assign inverted[15] = in[0];

	mux2_1 selector[15:0](.a(in), .b(inverted), .s({16{en}}), .out(out));

endmodule