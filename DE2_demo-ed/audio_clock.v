module audio_clock (    
//    Audio Side
   output reg oAUD_BCK,
   output oAUD_LRCK,
//    Control Signals
   input iCLK_18_4,
   input iRST_N
);                

parameter    REF_CLK        =    18432000;    // 18.432 MHz
parameter    SAMPLE_RATE    =    48000;        // 48 KHz
parameter    DATA_WIDTH    =    16;        // 16 Bits
parameter    CHANNEL_NUM    =    2;        // Dual Channel

//    Internal Registers and Wires
reg [3:0] BCK_DIV;
reg [8:0] LRCK_1X_DIV;
reg [7:0] LRCK_2X_DIV;
reg [6:0] LRCK_4X_DIV;
reg LRCK_1X;
reg LRCK_2X;
reg LRCK_4X;

//  AUD_BCK Generator
always@(posedge iCLK_18_4 or negedge iRST_N)
begin
    if(!iRST_N)
    begin
      BCK_DIV <= 4'h0;
      oAUD_BCK <= 1'b0;
    end
    else
    begin
      // REF_CLK/SAMPLE_RATE = 384, 384/(DATA_WIDTH*CHANNEL_NUM) = 12
      //  12/2 - 1 = 5
      if (BCK_DIV >= REF_CLK/(SAMPLE_RATE*DATA_WIDTH*CHANNEL_NUM*2)-1 )
      begin
        BCK_DIV     <= 4'h0;
        oAUD_BCK <= ~oAUD_BCK;
      end
      else BCK_DIV <= BCK_DIV+1'b1;
    end
end
//
//  AUD_LRCK Generator
//    oAUD_LRCK is high for left and low for right channel
//
always@(posedge iCLK_18_4 or negedge iRST_N)
begin
    if(!iRST_N)
    begin
        LRCK_1X_DIV    <=    0;
        LRCK_2X_DIV    <=    0;
        LRCK_4X_DIV    <=    0;
        LRCK_1X        <=    0;
        LRCK_2X        <=    0;
        LRCK_4X        <=    0;
    end
    else
    begin
        //LRCK 1X
        if(LRCK_1X_DIV >= REF_CLK/(SAMPLE_RATE*2)-1 )
        begin
            LRCK_1X_DIV <=    0;
            LRCK_1X    <= ~LRCK_1X;
        end
        else LRCK_1X_DIV <= LRCK_1X_DIV+1'b1;
        // LRCK 2X
        if(LRCK_2X_DIV >= REF_CLK/(SAMPLE_RATE*4)-1 )
        begin
            LRCK_2X_DIV <= 0;
            LRCK_2X    <= ~LRCK_2X;
        end
        else LRCK_2X_DIV <= LRCK_2X_DIV+1'b1;        
        // LRCK 4X
        if(LRCK_4X_DIV >= REF_CLK/(SAMPLE_RATE*8)-1 )
        begin
            LRCK_4X_DIV <= 0;
            LRCK_4X    <= ~LRCK_4X;
        end
        else LRCK_4X_DIV <= LRCK_4X_DIV+1'b1;        
    end
end
assign    oAUD_LRCK = LRCK_1X;

endmodule
            