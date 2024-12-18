/*
   CS/ECE 552 Spring '22
  
   Filename        : fetch.v
   Description     : This is the module for the overall fetch stage of the processor.
*/

module fetch (// Outputs
   pcOUT, instruction, err,
   // Inputs
   pcIN, pcwren, pcselect, clk, rst);

	// IN/OUT
	input  [15:0] pcIN;
	input  		  pcwren;
	input 		  pcselect;
	input 		  clk;
	input 		  rst;

	output [15:0] pcOUT;
	output [15:0] instruction;
	output 		  err;

	// WIRES
	wire [15:0]   pc_reg_in;
	wire [15:0]   pc_reg_out;

	wire [15:0]	  instr_int;
	wire [15:0]	  instr_pre_in;
	wire [15:0]	  instr_pre_out;

	wire		  adder_cout;
	wire 		  halt;

	wire		  CacheHit;
	wire		  Done;
	wire		  Stall;

	wire		  Read;

	

	// MAIN
	assign halt = (instruction === 16'h0000);

	mux2_1 mux0[15:0](.a(pcOUT), .b(pcIN), .s({16{pcselect}}), .out(pc_reg_in));
	register #(16) pc_reg(.in(pc_reg_in), .out(pc_reg_out), .wr( (pcwren & ~halt & Done) | pcselect), .clk(clk), .rst(rst));
	fulladder16 FA1 (.A(pc_reg_out), .B(16'h0002), .S(pcOUT), .Cout(adder_cout));

	stallmem datamem(.DataOut(instr_int), .Done(Done), .Stall(Stall), .CacheHit(CacheHit), .err(err), .Addr(pc_reg_out), .DataIn(16'b0), .Rd(1'b1), .Wr(1'b0), .createdump(1'b0), .clk(clk), .rst(rst));
	register_bypass #(16) datamemout(.in(instr_int), .out(instr_pre_in), .wr(Done), .clk(clk), .rst(rst));
	mux2_1 instrmux[15:0](.a(instr_pre_in), .b(16'h0800), .s({16{ ~(Done | halt) }}), .out(instr_pre_out));


	assign instruction = instr_pre_out & ~({16{err}});


   
endmodule

