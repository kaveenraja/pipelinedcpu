/*
   CS/ECE 552 Spring '22
  
   Filename        : instr_decoder.v
   Description     : This module is used to decode the top 6 bits of the instruction into several control signals
*/

module instr_decoder (// Outputs
   RegDst, RegSrc, to_ALUOP, _0ext, RegWrt, Bsrc, brin, MemWrt, ALUJmp, ImmSrc,
   // Inputs
   instr);

	// IN/OUT
	input  [4:0] instr;

	output reg  [1:0]  RegDst;
	output reg 	[1:0]  RegSrc;
	output  	[4:0]  to_ALUOP;
	output reg		   _0ext;
	output reg		   RegWrt;
	output reg	[1:0]  Bsrc;
	output reg	[4:0]  brin;
	output reg		   MemWrt;
	output reg		   ALUJmp;
	output reg		   ImmSrc;

	

	// MAIN
	assign to_ALUOP = instr;

	always @(instr)
	begin
		case(instr)

			5'b01000:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01001:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01010:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01011:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end
			
			5'b10100:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10101:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10110:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10111:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b10;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10000:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b1;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10001:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b01;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10011:
				begin
					RegDst = 2'b01;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b01;
					brin   = 5'b00000;
					MemWrt = 1'b1;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11001:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11011:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11010:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11100:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11101:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11110:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11111:
				begin
					RegDst = 2'b10;
					RegSrc = 2'b10;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01100:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b00;
					brin   = 5'b00001;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01101:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b00;
					brin   = 5'b00010;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01110:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b00;
					brin   = 5'b00100;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b01111:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b00;
					brin   = 5'b01000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b11000:
				begin
					RegDst = 2'b01;
					RegSrc = 2'b11;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b10;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b10010:
				begin
					RegDst = 2'b01;
					RegSrc = 2'b11;
					_0ext  = 1'b1;
					RegWrt = 1'b1;
					Bsrc   = 2'b11;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end

			5'b00100:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b00;
					brin   = 5'b10000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b1;
				end

			5'b00101:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b10;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b1;
					ImmSrc = 1'b0;
				end

			5'b00110:
				begin
					RegDst = 2'b11;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b00;
					brin   = 5'b10000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b1;
				end

			5'b00111:
				begin
					RegDst = 2'b11;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b1;
					Bsrc   = 2'b10;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b1;
					ImmSrc = 1'b0;
				end

			default:
				begin
					RegDst = 2'b00;
					RegSrc = 2'b00;
					_0ext  = 1'b0;
					RegWrt = 1'b0;
					Bsrc   = 2'b00;
					brin   = 5'b00000;
					MemWrt = 1'b0;
					ALUJmp = 1'b0;
					ImmSrc = 1'b0;
				end
		endcase
	end



   
endmodule

