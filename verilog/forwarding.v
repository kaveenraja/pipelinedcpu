module forwarding(
   	// Outputs
   	ForwardA, ForwardB,
   	// Inputs
	RegA, RegB, EX_MEM_Reg, MEM_WB_Reg, EX_MEM_RegWr, MEM_WB_RegWr
   );

	output [1:0] 	ForwardA;
	output [1:0]	ForwardB;

	input [2:0] 	RegA;
	input [2:0] 	RegB;
	input [2:0]		EX_MEM_Reg;
	input [2:0]		MEM_WB_Reg;
	input 			EX_MEM_RegWr;
	input			MEM_WB_RegWr;

	assign ForwardA[1] = EX_MEM_RegWr & (EX_MEM_Reg == RegA);
	assign ForwardB[1] = EX_MEM_RegWr & (EX_MEM_Reg == RegB);

	assign ForwardA[0] = MEM_WB_RegWr & ~(EX_MEM_RegWr & (EX_MEM_Reg == RegA)) & (MEM_WB_Reg == RegA);
	assign ForwardB[0] = MEM_WB_RegWr & ~(EX_MEM_RegWr & (EX_MEM_Reg == RegB)) & (MEM_WB_Reg == RegB);


endmodule