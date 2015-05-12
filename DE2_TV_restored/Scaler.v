`timescale 1ns / 1ps

// Scaler for block floating point FFT
module Scaler (
				clk, 
				reset_n, 
				sink_valid,
				sink_sop,
				sink_eop,
				sink_real, 
				sink_imag,
				sink_exp,
				sink_error,
				source_ready,
				 
				sink_ready,
				source_valid,
				source_sop,
				source_eop,
				source_real,
				source_imag,
				source_error
				);

	input clk;
	input reset_n;
	input sink_valid;
	input sink_sop;
	input sink_eop;
	input[15:0] sink_real;
	input[15:0] sink_imag;
	input[5:0] sink_exp;
	input[1:0] sink_error;
	input source_ready; 		  
	
	output sink_ready;
	output source_valid;
	output source_sop;
	output source_eop;
	output[16:0] source_real;
	output[16:0] source_imag;
	output[1:0] source_error; 	
	reg[16:0] source_real;
	reg[16:0] source_imag;
	

always @ (posedge clk)begin
	if (!reset_n)begin 
		source_real <= 0;
		source_imag <= 0;
	end
	else begin
		case (sink_exp)
			6'b110010 : //-14 Set date equal to MSBs	
			begin  
				source_real <= {sink_real[15],sink_real[1:0],14'b0};
				source_imag <= {sink_imag[15],sink_imag[1:0],14'b0};
			end
			6'b110011 : //-13 Set date equal to MSBs	
			begin  
				source_real <= {sink_real[15],sink_real[2:0],13'b0};
				source_imag <= {sink_imag[15],sink_imag[2:0],13'b0};
			end
			6'b110100 : //-12 Set date equal to MSBs	
			begin  
				source_real <= {sink_real[15],sink_real[3:0],12'b0};
				source_imag <= {sink_imag[15],sink_imag[3:0],12'b0};
			end
			6'b110101 : //-11 Set date equal to MSBs	
			begin  
				source_real <= {sink_real[15],sink_real[4:0],11'b0};
				source_imag <= {sink_imag[15],sink_imag[4:0],11'b0};
			end
			6'b110110 : //-10 Equals shift by 1 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[5:0],10'b0};
				source_imag <= {sink_imag[15],sink_imag[5:0],10'b0};
			end
			6'b110111 : //-9 Equals shift by 2 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[6:0],9'b0};
				source_imag <= {sink_imag[15],sink_imag[6:0],9'b0};
			end
			6'b111000 : //-8 Equals shift by 3 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[7:0],8'b0};
				source_imag <= {sink_imag[15],sink_imag[7:0],8'b0};
			end
			6'b111001 : //-7 Equals shift by 4 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[8:0],7'b0};
				source_imag <= {sink_imag[15],sink_imag[8:0],7'b0};
			end
			6'b111010 : //-6 Equals shift by 6 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[9:0],6'b0};
				source_imag <= {sink_imag[15],sink_imag[9:0],6'b0};
			end
			6'b111011 : //-5 Equals shift by 6 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[10:0],5'b0};
				source_imag <= {sink_imag[15],sink_imag[10:0],5'b0};
			end
			6'b111100 : //-4 Equals shift by 7 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[11:0],4'b0};
				source_imag <= {sink_imag[15],sink_imag[11:0],4'b0};
			end
			6'b111101 : //-3 Equals shift by 8 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[12:0],3'b0};
				source_imag <= {sink_imag[15],sink_imag[12:0],3'b0};
			end
			6'b111101 : //-2 Equals shift by 4 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[13:0],2'b0};
				source_imag <= {sink_imag[15],sink_imag[13:0],2'b0};
			end
			6'b111111 : //-1 Equals shift by 2 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[14:0],1'b0};
				source_imag <= {sink_imag[15],sink_imag[14:0],1'b0};
			end
			6'b000000 : //0 Equals shift by 0 with sign extension	
			begin  
				source_real <= sink_real[15:0];
				source_imag <= sink_imag[15:0];
			end
			6'b000001 : //1 Equals shift by -2 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[15:1]};
				source_imag <= {sink_imag[15],sink_imag[15:1]};
			end
			6'b000010 : //2 Equals shift by -4 with sign extension	
			begin  
				source_real <= {sink_real[15],sink_real[15],sink_real[15:2]};
				source_imag <= {sink_imag[15],sink_imag[15],sink_imag[15:2]};
			end
			default : //Default case = Set date equal to MSBs	
			begin  
				source_real <= 17'b0;
				source_imag <= 17'b0;
			end
		endcase
	end
end

reg source_valid;
reg source_sop;
reg source_eop;
reg [1:0] source_error;
always@(posedge clk) begin
	source_valid <= sink_valid;
	source_sop <= sink_sop;
	source_eop <= sink_eop;
	source_error <= sink_error;
end

wire sink_ready = source_ready;

endmodule

