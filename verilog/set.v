module set(Of, Cf, Zf, Sf, Oper, out);
	input Of, Cf, Zf, Sf;
	input [1:0] Oper;
	output reg [15:0] out;

	always @(Of, Cf, Zf, Sf, Oper)
	begin
		case(Oper)
			2'b00: out = Zf ? 16'h0001 : 16'h0000;
			2'b01: out = ~(Sf == Of) ? 16'h0001 : 16'h0000;
			2'b10: out = ~(Sf == Of) | Zf ? 16'h0001 : 16'h0000;
			2'b11: out = Cf ? 16'h0001 : 16'h0000;
			
		endcase
	end

endmodule
