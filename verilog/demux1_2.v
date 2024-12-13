module demux1_2 (Inp, S, OutA, OutB);
input Inp;
input S;
output OutA;
output OutB;

assign OutA = S ? 1'b0 : Inp;
assign OutB = S ? Inp : 1'b0;

endmodule