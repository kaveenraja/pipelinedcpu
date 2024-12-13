/*
   CS/ECE 552 Spring '22
  
   Filename        : fetch.v
   Description     : This is the module for the overall fetch stage of the processor.
*/

module fetch (// Outputs
   pcOUT, instruction,
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

	// WIRES
	wire [15:0]   pc_reg_in;
	wire [15:0]   pc_reg_out;
	wire		  adder_cout;
	wire 		  halt;
	

	// MAIN
	assign halt = (instruction === 16'h0000);

	mux2_1 mux0[15:0](.a(pcOUT), .b(pcIN), .s({16{pcselect}}), .out(pc_reg_in));

	register #(16) pc_reg(.in(pc_reg_in), .out(pc_reg_out), .wr( (pcwren & ~halt) | pcselect), .clk(clk), .rst(rst));

	fulladder16 FA1 (.A(pc_reg_out), .B(16'h0002), .S(pcOUT), .Cout(adder_cout));

	memory2c memcell(.data_out(instruction), .data_in(), .addr(pc_reg_out), .enable(1'b1), .wr(1'b0), .createdump(1'b1), .clk(clk), .rst(rst));

	


   
endmodule

