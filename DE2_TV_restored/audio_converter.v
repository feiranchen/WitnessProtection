module audio_converter (
    // Audio side
    input AUD_BCK,    // Audio bit clock
    input AUD_LRCK,   // left-right clock
    input AUD_ADCDAT,
    output AUD_DATA,
    // Controller side
    input iRST_N,  // reset
    input [15:0] AUD_outL,
    input [15:0] AUD_outR,
    output reg[15:0] AUD_inL,
    output reg[15:0] AUD_inR
);


//    16 Bits - MSB First
// Clocks in the ADC input
// and sets up the output bit selector

reg [3:0] SEL_Cont;
always@(negedge AUD_BCK or negedge iRST_N)
begin
    if(!iRST_N) SEL_Cont <= 4'h0;
    else
    begin
       SEL_Cont <= SEL_Cont+1'b1; //4 bit counter, so it wraps at 16
       if (AUD_LRCK) AUD_inL[~(SEL_Cont)] <= AUD_ADCDAT;
       else AUD_inR[~(SEL_Cont)] <= AUD_ADCDAT;
    end
end

// output the DAC bit-stream
assign AUD_DATA = (AUD_LRCK)? AUD_outL[~SEL_Cont]: AUD_outR[~SEL_Cont] ;

endmodule