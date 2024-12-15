/*
   CS/ECE 552, Fall '22
   Homework #3, Problem #2
  
   This module creates a wrapper around the 8x16b register file, to do
   do the bypassing logic for RF bypassing.
*/
module regFile_bypass (
                       // Outputs
                       read1Data, read2Data, err,
                       // Inputs
                       clk, rst, read1RegSel, read2RegSel, writeRegSel, writeData, writeEn
                       );
	parameter 	WIDTH = 16;

	input               	clk, rst;
	input [2:0]         	read1RegSel;
	input [2:0]         	read2RegSel;
	input [2:0]         	writeRegSel;
	input [WIDTH-1 :0] 		writeData;
	input              		writeEn;

	output [WIDTH-1 :0] 	read1Data;
	output [WIDTH-1 :0] 	read2Data;
	output              	err;

	wire [WIDTH-1 :0] 		reg_out_0;
	wire [WIDTH-1 :0] 		reg_out_1;
	wire [WIDTH-1 :0] 		reg_out_2;
	wire [WIDTH-1 :0] 		reg_out_3;
	wire [WIDTH-1 :0] 		reg_out_4;
	wire [WIDTH-1 :0] 		reg_out_5;
	wire [WIDTH-1 :0] 		reg_out_6;
	wire [WIDTH-1 :0] 		reg_out_7;

	wire [7:0]				reg_write_select_raw;
	wire [7:0]				reg_write_select;
	wire [7:0]				reg_read1_select;
	wire [7:0]				reg_read2_select;

	/* Error check */

	assign err = (^writeData === 1'bX) | (^writeEn === 1'bX);

   /* Register */

	decoder3_8 decoder0(.in(writeRegSel), .out(reg_write_select_raw));
	decoder3_8 decoder1(.in(read1RegSel), .out(reg_read1_select));
	decoder3_8 decoder2(.in(read2RegSel), .out(reg_read2_select));

	assign reg_write_select = reg_write_select_raw & {8{writeEn}};

	
	register_bypass #(WIDTH) reg0[7:0](.in({8{writeData}}), .out( {reg_out_7, reg_out_6, reg_out_5, reg_out_4, reg_out_3, reg_out_2, reg_out_1, reg_out_0} ), .wr(reg_write_select), .rst({8{rst}}), .clk({8{clk}}));
	
	mux8_1 #(WIDTH) mux0(.inA(reg_out_0), .inB(reg_out_1), .inC(reg_out_2), .inD(reg_out_3), .inE(reg_out_4), .inF(reg_out_5), .inG(reg_out_6), .inH(reg_out_7), .s(reg_read1_select), .out(read1Data));
	mux8_1 #(WIDTH) mux1(.inA(reg_out_0), .inB(reg_out_1), .inC(reg_out_2), .inD(reg_out_3), .inE(reg_out_4), .inF(reg_out_5), .inG(reg_out_6), .inH(reg_out_7), .s(reg_read2_select), .out(read2Data));
	


endmodule
