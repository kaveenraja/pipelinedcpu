/* $Author: sinclair $ */
/* $LastChangedDate: 2020-02-09 17:03:45 -0600 (Sun, 09 Feb 2020) $ */
/* $Rev: 46 $ */

// I don't know why I have to do this but whatever
`define register register

module proc (/*AUTOARG*/
   // Outputs
   err, 
   // Inputs
   clk, rst
   );

   input wire clk;
   input wire rst;

   output reg err;


	// WIRES
	wire 		   	dec_err;
	wire [15:0] 	wb_data;
	wire 			IF_pcwren;

	wire [15:0] 	IF_ID_pc_in;
	wire [15:0] 	IF_ID_pc_out;
	wire [15:0] 	IF_ID_instr_in;
	wire [15:0] 	IF_ID_instr_out;

	// ---------------------------


	wire [15:0] 	ID_EX_PC_in;
	wire [15:0] 	ID_EX_PC_out;

	wire [15:0] 	ID_EX_Reg1_in;
	wire [15:0] 	ID_EX_Reg1_out;

	wire [15:0] 	ID_EX_Reg2_in;
	wire [15:0] 	ID_EX_Reg2_out;

	wire [15:0] 	ID_EX_Imm5_in;
	wire [15:0] 	ID_EX_Imm5_out;

	wire [15:0] 	ID_EX_Imm8_in;
	wire [15:0] 	ID_EX_Imm8_out;

	wire [15:0] 	ID_EX_Imm11_in;
	wire [15:0] 	ID_EX_Imm11_out;

	wire [1:0]  	ID_EX_RegSrc_in;
	wire [1:0]  	ID_EX_RegSrc_out;

	wire [4:0]  	ID_EX_ALUOP_in;
	wire [4:0]  	ID_EX_ALUOP_out;

	wire [1:0]  	ID_EX_func_in;
	wire [1:0]  	ID_EX_func_out;

	wire [1:0]  	ID_EX_BSrc_in;
	wire [1:0]  	ID_EX_BSrc_out;

	wire [4:0]  	ID_EX_brin_in;
	wire [4:0]  	ID_EX_brin_out;

	wire 		  	ID_EX_MemWrt_in;
	wire 		  	ID_EX_MemWrt_out;

	wire 		  	ID_EX_ALUJmp_in;
	wire 		  	ID_EX_ALUJmp_out;

	wire 		  	ID_EX_ImmSrc_in;
	wire 		  	ID_EX_ImmSrc_out;

	wire 		  	ID_EX_RegWrt_in;
	wire 		  	ID_EX_RegWrt_out;

	wire [1:0]	  	ID_EX_RegDst_in;
	wire [1:0]	  	ID_EX_RegDst_out;

	// ---------------------------

	wire [15:0] 	EX_MEM_Out0_in;
	wire [15:0] 	EX_MEM_Out0_out;

	wire [15:0] 	EX_MEM_Out3_in;
	wire [15:0] 	EX_MEM_Out3_out;

	wire [15:0] 	EX_MEM_ALUOut_in;
	wire [15:0] 	EX_MEM_ALUOut_out;

	wire [15:0] 	EX_MEM_PC_in;
	wire [15:0] 	EX_MEM_PC_out;

	wire [15:0] 	EX_MEM_wrdata_in;
	wire [15:0] 	EX_MEM_wrdata_out;

	wire [1:0]		EX_MEM_RegDst_in;
	wire [1:0]		EX_MEM_RegDst_out;

	wire [1:0]		EX_MEM_RegSrc_in;
	wire [1:0]		EX_MEM_RegSrc_out;

	wire 			EX_MEM_RegWrt_in;
	wire 			EX_MEM_RegWrt_out;

	wire 			EX_MEM_MemWrt_in;
	wire 			EX_MEM_MemWrt_out;

	// ----------------------------

	
	wire [15:0]		MEM_WB_Out0_in;
	wire [15:0]		MEM_WB_Out0_out;

	wire [15:0]		MEM_WB_Out1_in;
	wire [15:0]		MEM_WB_Out1_out;

	wire [15:0]		MEM_WB_Out2_in;
	wire [15:0]		MEM_WB_Out2_out;

	wire [15:0]		MEM_WB_Out3_in;
	wire [15:0]		MEM_WB_Out3_out;

	wire [1:0] 		MEM_WB_RegSrc_in;
	wire [1:0] 		MEM_WB_RegSrc_out;

	wire [1:0] 		MEM_WB_RegDst_in;
	wire [1:0] 		MEM_WB_RegDst_out;

	wire			MEM_WB_RegWrt_in;
	wire			MEM_WB_RegWrt_out;



	wire			IF_ID_latchwr;
	wire			nop;
	wire			Flush;
	
	wire [2:0]		ID_EX_wraddr_in;
	wire [2:0]		ID_EX_wraddr_out;

	wire [2:0]		EX_MEM_wraddr_in;
	wire [2:0]		EX_MEM_wraddr_out;

	wire [2:0]		MEM_WB_wraddr_in;
	wire [2:0]		MEM_WB_wraddr_out;

	wire EX_MEM_PCSelect_In;
	wire EX_MEM_PCSelect_Out;


	wire [15:0] 	ID_EX_instr_in;
	wire [15:0] 	ID_EX_instr_in_pre;
	wire [15:0]		ID_EX_instr_out;


	wire [15:0] EX_MEM_instr_in;
	wire [15:0] EX_MEM_instr_out;

	wire IF_ID_instrerr_in;
	wire IF_ID_instrerr_out;

	wire ID_EX_instrerr_in;
	wire ID_EX_instrerr_out;

	wire EX_MEM_instrerr_in;
	wire EX_MEM_instrerr_out;

	wire MEM_WB_instrerr_in;
	wire MEM_WB_instrerr_out;

	wire MEM_WB_dataerr_in;
	wire MEM_WB_dataerr_out;

	wire data_stall;
	wire gated_clk;

	assign gated_clk = clk | data_stall;

   
  	/* MAIN */

	fetch   fetch0(.pcIN(EX_MEM_PC_out), .pcwren(IF_pcwren), .pcselect(EX_MEM_PCSelect_Out), .clk(gated_clk), .rst(rst), .pcOUT(IF_ID_pc_in), .instruction(IF_ID_instr_in), .err(IF_ID_instrerr_in));

`
	/* --------- IF -> ID LATCH START ---------- */

	register #(16) ifidpc   (.in(IF_ID_pc_in),    .out(IF_ID_pc_out),    .wr(IF_ID_latchwr), .rst(rst | Flush), .clk(gated_clk));
	register #(16) ifidinstr(.in(IF_ID_instr_in), .out(IF_ID_instr_out), .wr(IF_ID_latchwr), .rst(rst | Flush), .clk(gated_clk));
	register IF_ID_instrerror(.in(IF_ID_instrerr_in), .out(IF_ID_instrerr_out), .wr(1'b1), .rst(rst), .clk(gated_clk));

	/* --------- IF -> ID LATCH END   ---------- */

	decode decode0(.RegDst_addr(ID_EX_wraddr_in), .Imm5(ID_EX_Imm5_in), .Imm8(ID_EX_Imm8_in), .Imm11(ID_EX_Imm11_in), .Reg1(ID_EX_Reg1_in), .Reg2(ID_EX_Reg2_in), .RegSrc(ID_EX_RegSrc_in), .to_ALUOP(ID_EX_ALUOP_in), .func(ID_EX_func_in), .Bsrc(ID_EX_BSrc_in), .brin(ID_EX_brin_in), .MemWrt(ID_EX_MemWrt_in), .ALUJmp(ID_EX_ALUJmp_in), .RegWrt_out(ID_EX_RegWrt_in), .RegDst_out(ID_EX_RegDst_in), .ImmSrc(ID_EX_ImmSrc_in), .err(dec_err), .Instruction(IF_ID_instr_out), .wbdata(wb_data), .RegWrt_in(MEM_WB_RegWrt_out), .RegDst_in(MEM_WB_RegDst_out), .RegDst_addr_in(MEM_WB_wraddr_out), .clk(clk), .rst(rst));
	hazard h0(.PCWrite(IF_pcwren), .IF_ID_Write(IF_ID_latchwr), .nop(nop), .Flush(Flush), .ReadReg1(IF_ID_instr_out[10:8]), .ReadReg2(IF_ID_instr_out[7:5]), .EXWriteReg(ID_EX_wraddr_out), .MEMWriteReg(EX_MEM_wraddr_out), .WBWriteReg(MEM_WB_wraddr_out), .IDinstr(IF_ID_instr_out), .EXinstr(ID_EX_instr_out), .MEMinstr(EX_MEM_instr_out), .EXWren(ID_EX_RegWrt_out), .MemWren(EX_MEM_RegWrt_out), .WBWren(MEM_WB_RegWrt_out), .branchtaken(EX_MEM_PCSelect_Out));


	/* --------- ID -> EX LATCH START ---------- */
	assign ID_EX_PC_in = IF_ID_pc_out;

		// Control Latches: RegSrc [1:0], to_ALUOP [4:0], func [1:0], BSrc [1:0], brin [4:0], MemWrt, ALUJmp, ImmSrc, RegWrt, RegDst
	wire [21:0] ID_EX_CNTRLSIG_IN_pre;
	wire [21:0] ID_EX_CNTRLSIG_IN;
	wire [21:0] ID_EX_CNTRLSIG_OUT;

	assign ID_EX_CNTRLSIG_IN_pre  = {ID_EX_RegSrc_in,ID_EX_ALUOP_in,ID_EX_func_in,ID_EX_BSrc_in,ID_EX_brin_in,ID_EX_MemWrt_in,ID_EX_ALUJmp_in,ID_EX_ImmSrc_in,ID_EX_RegWrt_in,ID_EX_RegDst_in};
    mux2_1 mux0[21:0](.a(ID_EX_CNTRLSIG_IN_pre), .b(22'b0), .s({22{nop}}), .out(ID_EX_CNTRLSIG_IN));
	register #(22) idexcntrl(.in(ID_EX_CNTRLSIG_IN), .out(ID_EX_CNTRLSIG_OUT), .wr(1'b1), .rst(rst|Flush), .clk(gated_clk));

	assign ID_EX_RegSrc_out   = ID_EX_CNTRLSIG_OUT[21:20];
	assign ID_EX_ALUOP_out    = ID_EX_CNTRLSIG_OUT[19:15];
	assign ID_EX_func_out     = ID_EX_CNTRLSIG_OUT[14:13];
	assign ID_EX_BSrc_out     = ID_EX_CNTRLSIG_OUT[12:11];
	assign ID_EX_brin_out     = ID_EX_CNTRLSIG_OUT[10:6];
	assign ID_EX_MemWrt_out   = ID_EX_CNTRLSIG_OUT[5];
	assign ID_EX_ALUJmp_out   = ID_EX_CNTRLSIG_OUT[4];
	assign ID_EX_ImmSrc_out   = ID_EX_CNTRLSIG_OUT[3];
	assign ID_EX_RegWrt_out   = ID_EX_CNTRLSIG_OUT[2];
	assign ID_EX_RegDst_out   = ID_EX_CNTRLSIG_OUT[1:0];


		// Data Latches: PC, Reg1, Reg2, Imm5, Imm8, Imm11, wraddr
	register #(16) idexpc   (.in(ID_EX_PC_in),    .out(ID_EX_PC_out),    .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) idexreg1 (.in(ID_EX_Reg1_in),  .out(ID_EX_Reg1_out),  .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) idexreg2 (.in(ID_EX_Reg2_in),  .out(ID_EX_Reg2_out),  .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) ideximm5 (.in(ID_EX_Imm5_in),  .out(ID_EX_Imm5_out),  .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) ideximm8 (.in(ID_EX_Imm8_in),  .out(ID_EX_Imm8_out),  .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) ideximm11(.in(ID_EX_Imm11_in), .out(ID_EX_Imm11_out), .wr(1'b1), .rst(rst), .clk(gated_clk));

	register #(3)  idexwraddr(.in(ID_EX_wraddr_in), .out(ID_EX_wraddr_out), .wr(1'b1), .rst(rst), .clk(gated_clk));


	assign ID_EX_instr_in_pre = IF_ID_instr_out; 

	mux2_1 idexinstrmux[15:0](.a(ID_EX_instr_in_pre), .b(16'h0800), .s({16{nop}}), .out(ID_EX_instr_in));

	register #(16) idexinstr (.in(ID_EX_instr_in), .out(ID_EX_instr_out), .wr(1'b1), .rst(rst | Flush), .clk(gated_clk));
	
	assign ID_EX_instrerr_in = IF_ID_instrerr_out;
	register ID_EX_instrerror(.in(ID_EX_instrerr_in), .out(ID_EX_instrerr_out), .wr(1'b1), .rst(rst), .clk(gated_clk));
	/* --------- ID -> EX LATCH END   ---------- */

	wire [1:0] 	ForwardA;
	wire [1:0] 	ForwardB;
	wire [3:0] 	forwardAs;
	wire [3:0] 	forwardBs;
	wire [15:0] ForwardAOut;
	wire [15:0] ForwardBOut;

	wire [15:0] InCData;

	forwarding forward(.ForwardA(ForwardA), .ForwardB(ForwardB), .RegA(ID_EX_instr_out[10:8]), .RegB(ID_EX_instr_out[7:5]), .EX_MEM_Reg(EX_MEM_wraddr_out), .MEM_WB_Reg(MEM_WB_wraddr_out), .EX_MEM_RegWr(EX_MEM_RegWrt_out), .MEM_WB_RegWr(MEM_WB_RegWrt_out));
	
	assign forwardAs[0] = ~ForwardA[1] & ~ForwardA[0];
	assign forwardAs[1] = ~ForwardA[1] &  ForwardA[0];
	assign forwardAs[2] =  ForwardA[1] & ~ForwardA[0];
	assign forwardAs[3] =  0;

	assign forwardBs[0] = ~ForwardB[1] & ~ForwardB[0];
	assign forwardBs[1] = ~ForwardB[1] &  ForwardB[0];
	assign forwardBs[2] =  ForwardB[1] & ~ForwardB[0];
	assign forwardBs[3] =  0;

	assign InCData = (EX_MEM_RegSrc_out == 2'b11) ? EX_MEM_Out3_out : EX_MEM_ALUOut_out;


	mux4_1 #(16) forA(.inA(ID_EX_Reg1_out), .inB(wb_data), .inC(InCData), .inD(16'b0), .s(forwardAs), .out(ForwardAOut));
	mux4_1 #(16) forB(.inA(ID_EX_Reg2_out), .inB(wb_data), .inC(InCData), .inD(16'b0), .s(forwardBs), .out(ForwardBOut));

	execute execu0(.Out0(EX_MEM_Out0_in), .Out3(EX_MEM_Out3_in), .ALUOut(EX_MEM_ALUOut_in), .PCwb(EX_MEM_PC_in), .pcselect(EX_MEM_PCSelect_In), .instr(ID_EX_instr_out), .PC(ID_EX_PC_out), .to_ALUOP(ID_EX_ALUOP_out), .func(ID_EX_func_out), .Reg1(ForwardAOut), .Reg2(ForwardBOut), .Imm5(ID_EX_Imm5_out), .Imm8(ID_EX_Imm8_out), .Imm11(ID_EX_Imm11_out), .BSrc(ID_EX_BSrc_out), .brin(ID_EX_brin_out), .ALUJmp(ID_EX_ALUJmp_out), .ImmSrc(ID_EX_ImmSrc_out));


	/* --------- EX -> MEM LATCH START ---------- */

	assign EX_MEM_wrdata_in = ForwardBOut;
	assign EX_MEM_RegDst_in = ID_EX_RegDst_out;
	assign EX_MEM_RegSrc_in = ID_EX_RegSrc_out;
	assign EX_MEM_RegWrt_in = ID_EX_RegWrt_out;
	assign EX_MEM_MemWrt_in = ID_EX_MemWrt_out;


		// Control Latches RegDst, RegSrc, RegWrt, MemWrt
	wire [5:0] EX_MEM_CNTRLSIG_IN;
	wire [5:0] EX_MEM_CNTRLSIG_OUT;

	assign EX_MEM_CNTRLSIG_IN = {EX_MEM_RegDst_in, EX_MEM_RegSrc_in, EX_MEM_RegWrt_in, EX_MEM_MemWrt_in};
	register #(6) exmemcntrl(.in(EX_MEM_CNTRLSIG_IN), .out(EX_MEM_CNTRLSIG_OUT), .wr(1'b1), .rst(rst | Flush), .clk(gated_clk));

	assign EX_MEM_RegDst_out = EX_MEM_CNTRLSIG_OUT[5:4];
	assign EX_MEM_RegSrc_out = EX_MEM_CNTRLSIG_OUT[3:2];
	assign EX_MEM_RegWrt_out = EX_MEM_CNTRLSIG_OUT[1];
	assign EX_MEM_MemWrt_out = EX_MEM_CNTRLSIG_OUT[0];
	


		// Data Latches Out0, Out3, ALUOut, PCwb, Reg2
	register #(16) exmemout0(.in(EX_MEM_Out0_in),   .out(EX_MEM_Out0_out),   .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) exmemout3(.in(EX_MEM_Out3_in),   .out(EX_MEM_Out3_out),   .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) exmemalu (.in(EX_MEM_ALUOut_in), .out(EX_MEM_ALUOut_out), .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) exmempc  (.in(EX_MEM_PC_in),     .out(EX_MEM_PC_out),     .wr(1'b1), .rst(rst), .clk(gated_clk));
	register #(16) exmemreg2(.in(EX_MEM_wrdata_in), .out(EX_MEM_wrdata_out), .wr(1'b1), .rst(rst), .clk(gated_clk));

	assign EX_MEM_wraddr_in = ID_EX_wraddr_out;
	register #(3)  exmemwraddr(.in(EX_MEM_wraddr_in), .out(EX_MEM_wraddr_out), .wr(1'b1), .rst(rst), .clk(gated_clk));

	// Pcwrite register
	
	register #(1) exmempcwrite(.in(EX_MEM_PCSelect_In), .out(EX_MEM_PCSelect_Out), .wr(1'b1), .rst(rst | Flush), .clk(gated_clk));

	// Instr register


	assign EX_MEM_instr_in = ID_EX_instr_out;
	register #(16) exmeminstr (.in(EX_MEM_instr_in), .out(EX_MEM_instr_out), .wr(1'b1), .rst(rst | Flush), .clk(gated_clk));

	assign EX_MEM_instrerr_in = ID_EX_instrerr_out;
	register EX_MEM_instrerror(.in(EX_MEM_instrerr_in), .out(EX_MEM_instrerr_out), .wr(1'b1), .rst(rst), .clk(gated_clk));
	/* --------- EX -> MEM LATCH END ---------- */
	wire memdone;

	memory memory0(.Out1(MEM_WB_Out1_in), .Out2(MEM_WB_Out2_in), .ALUout(EX_MEM_ALUOut_out), .wrdata(EX_MEM_wrdata_out), .MemRd(EX_MEM_instr_out[15:11] == 5'b10001), .MemWrt(EX_MEM_MemWrt_out), .clk(clk), .rst(rst), .err(MEM_WB_dataerr_in), .Stall(data_stall), .Done(memdone));


	/* --------- MEM -> WB LATCH START ---------- */

	assign MEM_WB_Out0_in = EX_MEM_Out0_out;
	assign MEM_WB_Out3_in = EX_MEM_Out3_out;

	assign MEM_WB_RegSrc_in = EX_MEM_RegSrc_out;
	assign MEM_WB_RegDst_in = EX_MEM_RegDst_out;
	assign MEM_WB_RegWrt_in = EX_MEM_RegWrt_out;

		//Control Latches: RegSrc, RegDst, RegWrt
	wire [4:0] MEM_WB_CNTRLSIG_in;
	wire [4:0] MEM_WB_CNTRLSIG_out;

	assign 	MEM_WB_CNTRLSIG_in = {MEM_WB_RegSrc_in, MEM_WB_RegDst_in, MEM_WB_RegWrt_in};
	register #(5) memwbcntrl(.in(MEM_WB_CNTRLSIG_in), .out(MEM_WB_CNTRLSIG_out), .wr(1'b1), .rst(rst | data_stall), .clk(clk));

	assign MEM_WB_RegSrc_out = MEM_WB_CNTRLSIG_out[4:3];
	assign MEM_WB_RegDst_out = MEM_WB_CNTRLSIG_out[2:1];
	assign MEM_WB_RegWrt_out = MEM_WB_CNTRLSIG_out[0];


		// Data: Out1, Out2, Out3, Out4
	register #(16) memwbout0(.in(MEM_WB_Out0_in), .out(MEM_WB_Out0_out), .wr(1'b1), .rst(rst), .clk(clk));
	register #(16) memwbout1(.in(MEM_WB_Out1_in), .out(MEM_WB_Out1_out), .wr(1'b1), .rst(rst), .clk(clk));
	register #(16) memwbout2(.in(MEM_WB_Out2_in), .out(MEM_WB_Out2_out), .wr(1'b1), .rst(rst), .clk(clk));
	register #(16) memwbout3(.in(MEM_WB_Out3_in), .out(MEM_WB_Out3_out), .wr(1'b1), .rst(rst), .clk(clk));

	assign MEM_WB_wraddr_in = EX_MEM_wraddr_out;
	register #(3)  memwbwraddr(.in(MEM_WB_wraddr_in), .out(MEM_WB_wraddr_out), .wr(1'b1), .rst(rst), .clk(clk));
	

	// Instr register

	wire [15:0] MEM_WB_instr_in;
	wire [15:0] MEM_WB_instr_out;

	assign MEM_WB_instr_in = EX_MEM_instr_out;
	register #(16) memWBinstr (.in(MEM_WB_instr_in), .out(MEM_WB_instr_out), .wr(1'b1), .rst(rst | data_stall), .clk(clk));

	assign MEM_WB_instrerr_in = EX_MEM_instrerr_out;
	register MEM_WB_instrerror(.in(MEM_WB_instrerr_in), .out(MEM_WB_instrerr_out), .wr(1'b1), .rst(rst), .clk(clk));
	register MEM_WB_dataerror (.in(MEM_WB_dataerr_in),  .out(MEM_WB_dataerr_out),  .wr(1'b1), .rst(rst), .clk(clk));
	/* --------- MEM -> WB LATCH END   ---------- */


	wb wb0(.wbdata(wb_data), .In0(MEM_WB_Out0_out), .In1(MEM_WB_Out1_out), .In2(MEM_WB_Out2_out), .In3(MEM_WB_Out3_out), .RegSrc(MEM_WB_RegSrc_out));
	
	always @(*)
	begin
		case(dec_err)
			default: err <= dec_err;
		endcase
	end



endmodule // proc

// DUMMY LINE FOR REV CONTROL :0:

