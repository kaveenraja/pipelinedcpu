/*
   CS/ECE 552 Spring '22
  
   Filename        : decode.v
   Description     : This is the module for the overall decode stage of the processor.
*/

module decode (
   	// Outputs
  	Imm5, Imm8, Imm11, Reg1, Reg2, RegSrc, to_ALUOP, func, Bsrc, brin, RegWrt_out, RegDst_out, MemWrt, ALUJmp, ImmSrc, RegDst_addr, err,
   	// Inputs
   	Instruction, wbdata, RegWrt_in, RegDst_in, RegDst_addr_in, clk, rst
   	);

   	// IN/OUT
	input   [15:0] Instruction;
	input 	[15:0] wbdata;
	input		   RegWrt_in;
	input [1:0]    RegDst_in;
	input [2:0]    RegDst_addr_in;
	input		   clk;
	input		   rst;

	output  [15:0] Imm5;
	output  [15:0] Imm8;
	output  [15:0] Imm11;
	output  [15:0] Reg1;
	output  [15:0] Reg2;
	output  [1:0]  RegSrc;
	output  [4:0]  to_ALUOP;
	output 	[1:0]  func;
	output 	[1:0]  Bsrc;
	output 	[4:0]  brin;
	output		   RegWrt_out;
	output	[1:0]  RegDst_out;
	output 		   MemWrt;
	output 		   ALUJmp;
	output 		   ImmSrc;
	output 		   err;
	output  [2:0]  RegDst_addr;

	// WIRE
	wire           _0ext;

	wire [15:0]	   imm5_zero;
	wire [15:0]	   imm5_sign;

	wire [15:0]	   imm8_zero;
	wire [15:0]	   imm8_sign;

	wire [15:0]	   imm11_zero;
	wire [15:0]	   imm11_sign;


	wire [2:0]	   wrReg_out;

	// INSTRUCTION DECODER
	instr_decoder ID1(.instr(Instruction[15:11]), .RegDst(RegDst_out), .RegSrc(RegSrc), .to_ALUOP(to_ALUOP), ._0ext(_0ext), .RegWrt(RegWrt_out), .Bsrc(Bsrc), .brin(brin), .MemWrt(MemWrt), .ALUJmp(ALUJmp), .ImmSrc(ImmSrc));
	assign func = Instruction[1:0];

	// IMMEDIATE VALUES
	zext16 #(5) se0(.in(Instruction[4:0]), .out(imm5_zero));
	sext16 #(5) ze0(.in(Instruction[4:0]), .out(imm5_sign));
	mux2_1 mux0[15:0](.a(imm5_sign), .b(imm5_zero), .s({16{_0ext}}), .out(Imm5));

	zext16 #(8) se1(.in(Instruction[7:0]), .out(imm8_zero));
	sext16 #(8) ze1(.in(Instruction[7:0]), .out(imm8_sign));
	mux2_1 mux1[15:0](.a(imm8_sign), .b(imm8_zero), .s({16{_0ext}}), .out(Imm8));

	zext16 #(11) se2(.in(Instruction[10:0]), .out(imm11_zero));
	sext16 #(11) ze2(.in(Instruction[10:0]), .out(imm11_sign));
	mux2_1 mux2[15:0](.a(imm11_sign), .b(imm11_zero), .s({16{_0ext}}), .out(Imm11));

	// REG
	regFile_bypass #(16) regFile0(.read1RegSel(Instruction[10:8]), .read2RegSel(Instruction[7:5]), .writeRegSel(RegDst_addr_in), .writeData(wbdata), .writeEn(RegWrt_in), .clk(clk), .rst(rst), .read1Data(Reg1), .read2Data(Reg2), .err(err));
    
	ecmux4_1 mux4[2:0](.a(Instruction[7:5]), .b(Instruction[10:8]), .c(Instruction[4:2]), .d({3{1'b1}}), .s({3{RegDst_out}}), .out(wrReg_out));
	assign RegDst_addr = wrReg_out;

endmodule

