module register_bypass(in, out, wr, rst, clk);

	parameter WIDTH = 1;

	input [WIDTH-1 :0]  in;
	input               wr;
	input               clk;
	input               rst;

	output [WIDTH-1 :0] out;

	wire [WIDTH-1: 0]	reg_out;

	register #(WIDTH) reg0(.in(in), .out(reg_out), .wr(wr), .rst(rst), .clk(clk));

	mux2_1 mux0[WIDTH-1 :0](.a(reg_out), .b(in), .s( {WIDTH{wr}} ), .out(out));

endmodule