//3.33 fixed point signed multiply.
module trig_mult (out, trig, b);
	output	[26:0]	out;
	input 	signed	[26:0] 	trig;//3.33 37bit
	input 	signed	[26:0] 	b;
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	assign out = {mult_out[53], mult_out[47:22]}; //1+39
endmodule