/*
   CS/ECE 552 Spring '22
  
   Filename        : wb.v
   Description     : This is the module for the overall Write Back stage of the processor.
*/

module wb (// Outputs
   	wbdata,
   	// Inputs
   	In0, In1, In2, In3, RegSrc);

   	// IN/OUT
	input  [15:0] In0;
	input  [15:0] In1;
	input  [15:0] In2;
	input  [15:0] In3;
	input  [1:0]  RegSrc;

	output [15:0] wbdata;

	// MAIN

	ecmux4_1 mux1[15:0](.a(In0), .b(In1), .c(In2), .d(In3), .s( {16{RegSrc}} ), .out(wbdata));
   
endmodule

