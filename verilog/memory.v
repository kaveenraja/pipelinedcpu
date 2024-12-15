/*
   CS/ECE 552 Spring '22
  
   Filename        : memory.v
   Description     : This module contains all components in the Memory stage of the 
                     processor.
*/

module memory (// Outputs
   	Out1, Out2, err,
   	// Inputs
   	ALUout, wrdata, MemRd, MemWrt, clk, rst);

   	// IN/OUT
	input  [15:0] ALUout;
	input  [15:0] wrdata;
	input		  MemRd;
	input 		  MemWrt;
	input		  clk;
	input		  rst;

	output [15:0] Out1;
	output [15:0] Out2;
	output 		  err;

	// WIRE

	// MAIN

	memory2c_align memcell(.data_out(Out1), .data_in(wrdata), .addr(ALUout), .enable(MemRd | MemWrt), .wr(MemWrt), .createdump(1'b0), .clk(clk), .rst(rst), .err(err));
	assign Out2 = ALUout;


   

   
endmodule

