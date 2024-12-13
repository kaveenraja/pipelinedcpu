/*
   CS/ECE 552 Spring '22
  
   Filename        : memory.v
   Description     : This module contains all components in the Memory stage of the 
                     processor.
*/

module memory (// Outputs
   	Out1, Out2,
   	// Inputs
   	ALUout, wrdata, MemWrt, clk, rst);

   	// IN/OUT
	input  [15:0] ALUout;
	input  [15:0] wrdata;
	input 		  MemWrt;
	input		  clk;
	input		  rst;

	output [15:0] Out1;
	output [15:0] Out2;

	// WIRE

	// MAIN

	memory2c memcell(.data_out(Out1), .data_in(wrdata), .addr(ALUout), .enable(1'b1), .wr(MemWrt), .createdump(1'b1), .clk(clk), .rst(rst));
	assign Out2 = ALUout;


   

   
endmodule

