
module ALU_OP (// Outputs
   ALU, InvA, InvB, cin,
   // Inputs
   instr, func);

	input [4:0] 	 instr;
	input [1:0] 	 func;
	
	output reg [3:0] ALU;
	output reg 		 InvA;
	output reg 		 InvB;
	output reg  	 cin;


	always @(instr or func)
	begin
		case(instr)
			5'b01000:
			begin
				ALU  = 4'b0100;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b01001:
			begin
				ALU  = 4'b0100;
				InvA = 1'b1;
				InvB = 1'b0;
				cin  = 1'b1;
			end

			5'b01010:
			begin
				ALU  = 4'b0111;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b01011:
			begin
				ALU  = 4'b0101;
				InvA = 1'b0;
				InvB = 1'b1;
				cin  = 1'b0;
			end

			5'b10100:
			begin
				ALU  = 4'b0000;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10101:
			begin
				ALU  = 4'b0010;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10101:
			begin
				ALU  = 4'b0010;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10110:
			begin
				ALU  = 4'b0001;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10111:
			begin
				ALU  = 4'b0011;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10000:
			begin
				ALU  = 4'b0100;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10001:
			begin
				ALU  = 4'b0100;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10011:
			begin
				ALU  = 4'b0100;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b11001:
			begin
				ALU  = 4'b1011;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b11011:
			begin
				case(func)
					2'b00:
						begin
							ALU  = 4'b0100;
							InvA = 1'b0;
							InvB = 1'b0;
							cin  = 1'b0;
						end
					2'b01:
						begin
							ALU  = 4'b0100;
							InvA = 1'b1;
							InvB = 1'b0;
							cin  = 1'b1;
						end
					2'b10:
						begin
							ALU  = 4'b0111;
							InvA = 1'b0;
							InvB = 1'b0;
							cin  = 1'b0;
						end
					2'b11:
						begin
							ALU  = 4'b0101;
							InvA = 1'b0;
							InvB = 1'b1;
							cin  = 1'b0;
						end
						

				endcase
			end

			5'b11010:
			begin
				case(func)
					2'b00:
						begin
							ALU  = 4'b0000;
							InvA = 1'b0;
							InvB = 1'b0;
							cin  = 1'b0;
						end
					2'b01:
						begin
							ALU  = 4'b0010;
							InvA = 1'b0;
							InvB = 1'b0;
							cin  = 1'b0;
						end
					2'b10:
						begin
							ALU  = 4'b0001;
							InvA = 1'b0;
							InvB = 1'b0;
							cin  = 1'b0;
						end
					2'b11:
						begin
							ALU  = 4'b0011;
							InvA = 1'b0;
							InvB = 1'b0;
							cin  = 1'b0;
						end
						

				endcase
			end

			5'b11100:
			begin
				ALU  = 4'b1100;
				InvA = 1'b0;
				InvB = 1'b1;
				cin  = 1'b1;
			end

			5'b11101:
			begin
				ALU  = 4'b1101;
				InvA = 1'b0;
				InvB = 1'b1;
				cin  = 1'b1;
			end

			5'b11110:
			begin
				ALU  = 4'b1110;
				InvA = 1'b0;
				InvB = 1'b1;
				cin  = 1'b1;
			end

			5'b11111:
			begin
				ALU  = 4'b1111;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b01100:
			begin
				ALU  = 4'b0110;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b01101:
			begin
				ALU  = 4'b0110;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b01110:
			begin
				ALU  = 4'b0110;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b01111:
			begin
				ALU  = 4'b0110;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b11000:
			begin
				ALU  = 4'b0101;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b10010:
			begin
				ALU  = 4'b0010;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b00101:
			begin
				ALU  = 4'b0100;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

			5'b00111:
			begin
				ALU  = 4'b0100;
				InvA = 1'b0;
				InvB = 1'b0;
				cin  = 1'b0;
			end

		endcase
	end




endmodule