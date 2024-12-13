/*
   CS/ECE 552 Spring '22
  
   Filename        : execute.v
   Description     : This is the overall module for the execute stage of the processor.
*/

module execute (// Outputs
	Out0, Out3, ALUOut, PCwb, pcselect,
 	// Inputs
  	instr, PC, to_ALUOP, func, Reg1, Reg2, Imm5, Imm8, Imm11, BSrc, brin, ALUJmp, ImmSrc);

  	// IN/OUT
	input [15:0] 	instr;
	input [15:0]	PC;
	input [4:0]		to_ALUOP;
	input [1:0] 	func;
	input [15:0] 	Reg1;
	input [15:0]	Reg2;
	input [15:0]	Imm5;
	input [15:0]	Imm8;
	input [15:0]	Imm11;
	input [1:0]		BSrc;
	input [4:0]		brin;
	input 			ALUJmp;
	input			ImmSrc;

	output [15:0]	Out0;
	output [15:0]	Out3;
	output [15:0]	ALUOut;
	output [15:0]   PCwb;
	output 			pcselect;
	
	// WIRE
	
	wire 			internal_InvA;
	wire 			internal_InvB;
	wire 			internal_cin;
	wire [3:0]		internal_aluop;
	wire [15:0]		internal_InB;

	wire			internal_Cf;
	wire			internal_Sf;
	wire			internal_Of;
	wire			internal_Zf;

	wire			internal_BR;


	wire [15:0]  	internal_immjmp;
	wire [15:0]  	internal_pcimpjmp;
	wire [15:0]  	internal_pc_preALUJmp;


	// ALU LOGIC

	ALU_OP aluop0(.ALU(internal_aluop), .InvA(internal_InvA), .InvB(internal_InvB), .cin(internal_cin), .instr(to_ALUOP), .func(func));


	ecmux4_1 ecmux0[15:0](.a(Reg2), .b(Imm5), .c(Imm8), .d(16'h0008), .s( {16{BSrc}} ), .out(internal_InB));
	alu alu0(.InA(Reg1), .InB(internal_InB), .Cin(internal_cin), .Oper(internal_aluop), .invA(internal_InvA), .invB(internal_InvB), .sign(1'b1), .Out(ALUOut), .Cfl(internal_Cf), .Sfl(internal_Sf), .Ofl(internal_Of), .Zfl(internal_Zf));
   
	assign Out3 = ALUOut | Imm8;

	// JUMP/BRANCH LOGIC
	brncnd brcd0(.brin(brin), .SF(internal_Sf), .ZF(internal_Zf), .OF(internal_Of), .CF(internal_Cf), .BR(internal_BR));

	mux2_1 mux0[15:0](.a(Imm8), .b(Imm11), .s({16{ImmSrc}}), .out(internal_immjmp));

	fulladder16 fa0(.A(PC), .B(internal_immjmp), .S(internal_pcimpjmp), .Cout());

	mux2_1 mux1[15:0](.a(PC), .b(internal_pcimpjmp), .s({16{internal_BR}}), .out(internal_pc_preALUJmp));
	mux2_1 mux2[15:0](.a(internal_pc_preALUJmp), .b(ALUOut), .s({16{ALUJmp}}), .out(PCwb));

	//assign pcselect = (instr[15:11] === 5'b01100) | (instr[15:11] === 5'b01101) | (instr[15:11] === 5'b01110) | (instr[15:11] === 5'b01111) | (instr[15:11] === 5'b00100) | (instr[15:11] === 5'b00101) | (instr[15:11] === 5'b00110) | (instr[15:11] === 5'b00111);
	assign pcselect = internal_BR | ALUJmp;
	
	assign Out0 = PC;

endmodule

