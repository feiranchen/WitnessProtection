  //#############################################################################
  // WRITTEN BY ANNIE (WEI) DAI TO STORE DOWN SAMPLED SKIN MAP
  // SPRING 2013
  // ALLOWS ASYNCHRONOUS READS AND SYNCHRONOUS WRITES :D
  //#############################################################################
  /////////////////////////////////////////////////////
  //// DOWN SAMPLED SKIN MAP AS M4K MODULE      ///////
  /////////////////////////////////////////////////////
  module ram_infer(data_out,read_addr,write_addr,data_in,we,clk);
  output [15:0] data_out;
  input [10:0] read_addr, write_addr;//6bit x, 5bit y
  input [15:0] data_in;
  input clk,we;
  
  reg [15:0] mem[2047:0];//30 rows=2^5, 40 cells=2^6 of 8 bit (16*16=256=2^8 but we will overflow so 9bit)
 
  assign data_out = mem[read_addr];
  always @(posedge clk)
  begin
    if(we) mem[write_addr] <= data_in;
  end
  endmodule
  