module disassemble (clk, abs_out, ph_out, r, i);
	output	signed [17:0]	abs_out;
	output	signed [17:0]   ph_out;
	input 	signed	[17:0] 	r;//3.33 37bit
	input 	signed	[17:0] 	i;
	reg 	signed	[35:0]	mult_out;
	reg signed [17:0] cos;
	reg signed [17:0] sqrt;
	reg signed [17:0] ph_out;
	reg [2:0] state;

	
	sqrt s(.clk(clk),.radical(mult_out[34:18]),.q(sqrt[16:8]),.remainder())
	arccos_rom a(.clock(clk), cos(cos), theta(ph_out[16:0]));
	always@(posedge clock)
	begin
		case(state)
			0: begin
				mult_out <= r * r + i* i;
				state    <= 1;
			end
			1: begin
				abs_out   <= sqrt;
				cos       <= r/sqrt;
				ph_out[17]<= i[17];
				state     <= 0;
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