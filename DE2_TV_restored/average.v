//#############################################################################
// MODIFIED BY ANNIE (WEI) DAI FOR AVERAGING 5-6BITS AT VGA_CLK
// SPRING 2013
// ALL INPUTS ARE ASSUMED TO BE POSITIVE IN THIS MODULE
//#############################################################################
/////////////////////////////////////////////////////
//// Time weighted average amplitude          ///////
/////////////////////////////////////////////////////
module average (out, in, dk_const, clk);

	output wire signed [9:0] out ;
	
	input wire signed [31:0] in ;
	input wire [4:0] dk_const ;
	input wire clk;
	reg signed [31:0] fullout;
	wire signed  [31:0] new_out ;
	//first order lowpass of absolute value of input
	assign new_out = fullout - (fullout>>(dk_const)) + (in>>dk_const) ;
	
	assign out=fullout[30:21];
	always @(posedge clk)
	begin
		 fullout <= new_out ;
	end
endmodule
  

  