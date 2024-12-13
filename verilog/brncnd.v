module brncnd(brin, SF, ZF, OF, CF, BR);
	input 		SF;
	input 		ZF;
	input 		OF;
	input 		CF;
	input [4:0] brin;

	output  	BR;

	assign BR = (brin[4] & 1) | (brin[3] & ~SF) | (brin[2] & SF) | (brin[1] & ~ZF) | (brin[0] & ZF);

endmodule
