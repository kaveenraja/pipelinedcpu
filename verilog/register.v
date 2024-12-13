module register(in, out, wr, rst, clk);

	parameter WIDTH = 1;

	input [WIDTH-1 :0]  in;
	input               wr;
	input               clk;
	input               rst;

	output [WIDTH-1 :0] out;
	
	wire [WIDTH-1 :0]	dff_in;
	wire [WIDTH-1 :0]	dff_out;

	mux2_1 mux0[WIDTH-1 :0](.a(dff_out), .b(in), .s( {WIDTH{wr}} ), .out(dff_in));

	dff dff0[WIDTH-1 :0](.d(dff_in), .q(dff_out), .clk( {WIDTH{clk}} ), .rst( {WIDTH{rst}} ));

	assign out = dff_out;

endmodule
