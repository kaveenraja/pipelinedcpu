module sext16(in, out);

parameter WIDTH = 5;
parameter space = 16-WIDTH;

input [WIDTH-1:0] in;
output[15:0] out;

wire top = out[WIDTH-1];

assign out = {{space{top}}, in};

endmodule
