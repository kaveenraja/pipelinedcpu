module zext16(in, out);

parameter WIDTH = 5;
parameter space = 16-WIDTH;

input  [WIDTH-1:0] in;
output [15:0] 	   out;

assign out = {{space{1'b0}}, in};

endmodule