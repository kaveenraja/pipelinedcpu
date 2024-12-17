/*
   CS/ECE 552 Spring '22
  
   Filename        : memory.v
   Description     : This module contains all components in the Memory stage of the 
                     processor.
*/

module memory (// Outputs
   	Out1, Out2, Stall, Done, err,
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
	output 		  Stall;
	output 		  Done;
	output 		  err;

	// WIRE
	wire [15:0] Out1Int;
	wire		StallInt;
	wire		CacheHit;


	//MAIN

	mem_system datamem(.DataOut(Out1Int), .Done(Done), .Stall(StallInt), .CacheHit(CacheHit), .err(err), .Addr(ALUout), .DataIn(wrdata), .Rd(MemRd), .Wr(MemWrt), .createdump(1'b0), .clk(clk), .rst(rst));
	register_bypass #(16) datamemout(.in(Out1Int), .out(Out1), .wr(Done), .clk(clk), .rst(rst));

	assign Stall = StallInt & ~Done;
	assign Out2 = ALUout;


   

   
endmodule

