module trig_mult (clk, out, trig, b);
	output	[17:0]	abs_out;
	output	[17:0]   ph_out;
	input 	signed	[17:0] 	r;//3.33 37bit
	input 	signed	[17:0] 	i;
	reg 	signed	[35:0]	mult_out;
	reg [8:0] sqrt;
	reg [2:0] state;

	
	sqrt s(.clk(clk),.radical(mult_out[34:18]),.q(sqrt),.remainder())

	always@(posedge clock)
	begin
		case(state)
			0: begin
				mult_out <= r * r + i* i;
				state<=1;
			end
			1: begin
				
			end
		endcase
	end
	
endmodule











//3.33 fixed point signed multiply.
module trig_mult (out, trig, b);
	output	[26:0]	out;
	input 	signed	[26:0] 	trig;//3.33 37bit
	input 	signed	[26:0] 	b;
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	assign out = {mult_out[53], mult_out[47:22]}; //1+39
endmodule