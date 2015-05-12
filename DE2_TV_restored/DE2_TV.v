// --------------------------------------------------------------------
// Copyright (c) 2005 by Terasic Technologies Inc. 
// --------------------------------------------------------------------
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// --------------------------------------------------------------------
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
// Major Functions:  DE2 TV Box
// Modified by Annie (Wei) Dai Spring 2013 for ECE 5760 Final Project
// Upper Body Motion Tracker
// --------------------------------------------------------------------

module DE2_TV
(
  // Clock Input      
  input         OSC_27,    //  27 MHz
  input         OSC_50,    //  50 MHz
  input         EXT_CLOCK,   //  External Clock
  // Push Button   
  input   [3:0] KEY,         //  Button[3:0]
  // DPDT DPDT_SWitch   
  input  [17:0] DPDT_SW,          //  DPDT DPDT_SWitch[17:0]
  // 7-SEG Dispaly 
  output  [6:0] HEX0,        //  Seven Segment Digital 0
  output  [6:0] HEX1,        //  Seven Segment Digital 1
  output  [6:0] HEX2,        //  Seven Segment Digital 2
  output  [6:0] HEX3,        //  Seven Segment Digital 3
  output  [6:0] HEX4,        //  Seven Segment Digital 4
  output  [6:0] HEX5,        //  Seven Segment Digital 5
  output  [6:0] HEX6,        //  Seven Segment Digital 6
  output  [6:0] HEX7,        //  Seven Segment Digital 7
  // LED  
  output  [8:0] LED_GREEN,   //  LED Green[8:0]
  output [17:0] LED_RED,     //  LED Red[17:0]
  // UART 
  output        UART_TXD,    //  UART Transmitter
  input         UART_RXD,    //  UART Receiver
  // IRDA
  output        IRDA_TXD,    //  IRDA Transmitter
  input         IRDA_RXD,    //  IRDA Receiver
  // SDRAM Interface  
  inout  [15:0] DRAM_DQ,     //  SDRAM Data bus 16 Bits
  output [11:0] DRAM_ADDR,   //  SDRAM Address bus 12 Bits
  output        DRAM_LDQM,   //  SDRAM Low-byte Data Mask 
  output        DRAM_UDQM,   //  SDRAM High-byte Data Mask
  output        DRAM_WE_N,   //  SDRAM Write Enable
  output        DRAM_CAS_N,  //  SDRAM Column Address Strobe
  output        DRAM_RAS_N,  //  SDRAM Row Address Strobe
  output        DRAM_CS_N,   //  SDRAM Chip Select
  output        DRAM_BA_0,   //  SDRAM Bank Address 0
  output        DRAM_BA_1,   //  SDRAM Bank Address 0
  output        DRAM_CLK,    //  SDRAM Clock
  output        DRAM_CKE,    //  SDRAM Clock Enable
  // Flash Interface  
  inout   [7:0] FL_DQ,       //  FLASH Data bus 8 Bits
  output [21:0] FL_ADDR,     //  FLASH Address bus 22 Bits
  output        FL_WE_N,     //  FLASH Write Enable
  output        FL_RST_N,    //  FLASH Reset
  output        FL_OE_N,     //  FLASH Output Enable
  output        FL_CE_N,     //  FLASH Chip Enable
  // SRAM Interface  
  inout  [15:0] SRAM_DQ,     //  SRAM Data bus 16 Bits
  output [17:0] SRAM_ADDR,   //  SRAM Adress bus 18 Bits
  output        SRAM_UB_N,   //  SRAM High-byte Data Mask 
  output        SRAM_LB_N,   //  SRAM Low-byte Data Mask 
  output        SRAM_WE_N,   //  SRAM Write Enable
  output        SRAM_CE_N,   //  SRAM Chip Enable
  output        SRAM_OE_N,   //  SRAM Output Enable
  // ISP1362 Interface 
  inout  [15:0] OTG_DATA,    //  ISP1362 Data bus 16 Bits
  output  [1:0] OTG_ADDR,    //  ISP1362 Address 2 Bits
  output        OTG_CS_N,    //  ISP1362 Chip Select
  output        OTG_RD_N,    //  ISP1362 Write
  output        OTG_WR_N,    //  ISP1362 Read
  output        OTG_RST_N,   //  ISP1362 Reset
  output        OTG_FSPEED,  //  USB Full Speed,  0 = Enable, Z = Disable
  output        OTG_LSPEED,  //  USB Low Speed,   0 = Enable, Z = Disable
  input         OTG_INT0,    //  ISP1362 Interrupt 0
  input         OTG_INT1,    //  ISP1362 Interrupt 1
  input         OTG_DREQ0,   //  ISP1362 DMA Request 0
  input         OTG_DREQ1,   //  ISP1362 DMA Request 1
  output        OTG_DACK0_N, //  ISP1362 DMA Acknowledge 0
  output        OTG_DACK1_N, //  ISP1362 DMA Acknowledge 1
  // LCD Module 16X2   
  output        LCD_ON,      //  LCD Power ON/OFF
  output        LCD_BLON,    //  LCD Back Light ON/OFF
  output        LCD_RW,      //  LCD Read/Write Select, 0 = Write, 1 = Read
  output        LCD_EN,      //  LCD Enable
  output        LCD_RS,      //  LCD Command/Data Select, 0 = Command, 1 = Data
  inout   [7:0] LCD_DATA,    //  LCD Data bus 8 bits
  // SD_Card Interface 
  inout         SD_DAT,      //  SD Card Data
  inout         SD_DAT3,     //  SD Card Data 3
  inout         SD_CMD,      //  SD Card Command Signal
  output        SD_CLK,      //  SD Card Clock
  // USB JTAG link  
  input         TDI,         // CPLD -> FPGA (data in)
  input         TCK,         // CPLD -> FPGA (clk)
  input         TCS,         // CPLD -> FPGA (CS)
  output        TDO,         // FPGA -> CPLD (data out)
  // I2C    
  inout         I2C_SDAT,    //  I2C Data
  output        I2C_SCLK,    //  I2C Clock
  // PS2   
  input         PS2_DAT,     //  PS2 Data
  input         PS2_CLK,     //  PS2 Clock
  // VGA   
  output        VGA_CLK,     //  VGA Clock
  output        VGA_HS,      //  VGA H_SYNC
  output        VGA_VS,      //  VGA V_SYNC
  output        VGA_BLANK,   //  VGA BLANK
  output        VGA_SYNC,    //  VGA SYNC
  output  [9:0] VGA_R,       //  VGA Red[9:0]
  output  [9:0] VGA_G,       //  VGA Green[9:0]
  output  [9:0] VGA_B,       //  VGA Blue[9:0]
  // Ethernet Interface 
  inout  [15:0] ENET_DATA,   //  DM9000A DATA bus 16Bits
  output        ENET_CMD,    //  DM9000A Command/Data Select, 0 = Command, 1 = Data
  output        ENET_CS_N,   //  DM9000A Chip Select
  output        ENET_WR_N,   //  DM9000A Write
  output        ENET_RD_N,   //  DM9000A Read
  output        ENET_RST_N,  //  DM9000A Reset
  input         ENET_INT,    //  DM9000A Interrupt
  output        ENET_CLK,    //  DM9000A Clock 25 MHz
  // Audio CODEC 
  inout         AUD_ADCLRCK, //  Audio CODEC ADC LR Clock
  input         AUD_ADCDAT,  //  Audio CODEC ADC Data
  inout         AUD_DACLRCK, //  Audio CODEC DAC LR Clock
  output        AUD_DACDAT,  //  Audio CODEC DAC Data
  inout         AUD_BCLK,    //  Audio CODEC Bit-Stream Clock
  output        AUD_XCK,     //  Audio CODEC Chip Clock
  // TV Decoder  
  input   [7:0] TD_DATA,     //  TV Decoder Data bus 8 bits
  input         TD_HS,       //  TV Decoder H_SYNC
  input         TD_VS,       //  TV Decoder V_SYNC
  output        TD_RESET,    //  TV Decoder Reset
  input         TD_CLK,      //  TV Decoder Line Locked Clock
  // GPIO  
  inout  [35:0] GPIO_0,      //  GPIO Connection 0
  inout  [35:0] GPIO_1       //  GPIO Connection 1
);

  //  For Audio CODEC
  //wire  AUD_CTRL_CLK;  //  For Audio Controller
  //assign  AUD_XCK = AUD_CTRL_CLK;

  //  7 segment LUT
  SEG7_LUT_8 u0 
  (
    .oSEG0  (HEX0),
    .oSEG1  (HEX1),
    .oSEG2  (HEX2),
    .oSEG3  (HEX3),
    .oSEG4  (HEX4),
    .oSEG5  (HEX5),
    .oSEG6  (HEX6),
    .oSEG7  (HEX7),
    .iDIG   (DPDT_SW) 
  );

  // Audio CODEC and video decoder setting
  avconf u1  
  (  //  Host Side
    .CLOCK_50     (OSC_50),
    .reset   (KEY[0]),
    //  I2C Side
    .I2C_SCLK (I2C_SCLK),
    .I2C_SDAT (I2C_SDAT)  
  );

  //  TV Decoder Stable Check
  TD_Detect u2 
  (  
    .oTD_Stable (TD_Stable),
    .iTD_VS     (TD_VS),
    .iTD_HS     (TD_HS),
    .iRST_N     (KEY[0])  
  );

  //  Reset Delay Timer
  Reset_Delay u3 
  (  
    .iCLK   (OSC_50),
    .iRST   (TD_Stable),
    .oRST_0 (DLY0),
    .oRST_1 (DLY1),
    .oRST_2 (DLY2)
  );

  //  ITU-R 656 to YUV 4:2:2
  ITU_656_Decoder u4 
  (  //  TV Decoder Input
    .iTD_DATA   (TD_DATA),
    //  Position Output
    .oTV_X      (TV_X),
    //.oTV_Y(TV_Y),
    //  YUV 4:2:2 Output
    .oYCbCr     (YCbCr),
    .oDVAL      (TV_DVAL),
    //  Control Signals
    .iSwap_CbCr (Quotient[0]),
    .iSkip      (Remain==4'h0),
    .iRST_N     (DLY1),
    .iCLK_27    (TD_CLK)  
  );

  //  For Down Sample 720 to 640
  DIV u5  
  (  
    .aclr     (!DLY0), 
    .clock    (TD_CLK),
    .denom    (4'h9),
    .numer    (TV_X),
    .quotient (Quotient),
    .remain   (Remain)
  );

  //  SDRAM frame buffer
  Sdram_Control_4Port u6  
  (  //  HOST Side
    .REF_CLK      (OSC_27),
    .CLK_18       (),
    .RESET_N      (1'b1),
    //  FIFO Write Side 1
    .WR1_DATA     (YCbCr),
    .WR1          (TV_DVAL),
    .WR1_FULL     (WR1_FULL),
    .WR1_ADDR     (0),
    .WR1_MAX_ADDR (640*507),    //  525-18
    .WR1_LENGTH   (9'h80),
    .WR1_LOAD     (!DLY0),
    .WR1_CLK      (TD_CLK),
    //  FIFO Read Side 1
    .RD1_DATA     (m1YCbCr),
    .RD1          (m1VGA_Read),
    .RD1_ADDR     (640*13),      //  Read odd field and bypess blanking
    .RD1_MAX_ADDR (640*253),
    .RD1_LENGTH   (9'h80),
    .RD1_LOAD     (!DLY0),
    .RD1_CLK      (OSC_27),
    //  FIFO Read Side 2
    .RD2_DATA     (m2YCbCr),
    .RD2          (m2VGA_Read),
    .RD2_ADDR     (640*267),      //  Read even field and bypess blanking
    .RD2_MAX_ADDR (640*507),
    .RD2_LENGTH   (9'h80),
    .RD2_LOAD     (!DLY0),
    .RD2_CLK      (OSC_27),
    //  SDRAM Side
    .SA           (DRAM_ADDR),
    .BA           ({DRAM_BA_1,DRAM_BA_0}),
    .CS_N         (DRAM_CS_N),
    .CKE          (DRAM_CKE),
    .RAS_N        (DRAM_RAS_N),
    .CAS_N        (DRAM_CAS_N),
    .WE_N         (DRAM_WE_N),
    .DQ           (DRAM_DQ),
    .DQM          ({DRAM_UDQM,DRAM_LDQM}),
    .SDR_CLK      (DRAM_CLK)  
  );

  //  YUV 4:2:2 to YUV 4:4:4
  YUV422_to_444 u7 
  (  //  YUV 4:2:2 Input
    .iYCbCr   (mYCbCr),
    //  YUV  4:4:4 Output
    .oY       (mY),
    .oCb      (mCb),
    .oCr      (mCr),
    //  Control Signals
    .iX       (VGA_X),
    .iCLK     (OSC_27),
    .iRST_N   (DLY0)
  );

  //  YCbCr 8-bit to RGB-10 bit 
  YCbCr2RGB u8 
  (  //  Output Side
    .Red      (mRed),
    .Green    (mGreen),
    .Blue     (mBlue),
    .oDVAL    (mDVAL),
    //  Input Side
    .iY       (mY),
    .iCb      (mCb),
    .iCr      (mCr),
    .iDVAL    (VGA_Read),
    //  Control Signal
    .iRESET   (!DLY2),
    .iCLK     (OSC_27)
  );

  // Comment out this module if you don't want the mirror effect
  Mirror_Col u100  
  (  //  Input Side
    .iCCD_R       (mRed),
    .iCCD_G       (mGreen),
    .iCCD_B       (mBlue),
    .iCCD_DVAL    (mDVAL),
    .iCCD_PIXCLK  (VGA_CLK), //(TD_CLK),
    .iRST_N       (DLY2),
    //  Output Side
    .oCCD_R       (Red),
    .oCCD_G       (Green),
    .oCCD_B       (Blue)//,
    //.oCCD_DVAL(TV_DVAL));
  );

  //VGA Controller
  VGA_Ctrl u9 
  (  //  Host Side
    .iRed       (mVGA_R), 
    .iGreen     (mVGA_G),
    .iBlue      (mVGA_B), 
    .oCurrent_X (VGA_X),
    .oCurrent_Y (VGA_Y),
    .oRequest   (VGA_Read),
    //  VGA Side
    .oVGA_R     (VGA_R),
    .oVGA_G     (VGA_G),
    .oVGA_B     (VGA_B),
    .oVGA_HS    (VGA_HS),
    .oVGA_VS    (VGA_VS),
    .oVGA_SYNC  (VGA_SYNC),
    .oVGA_BLANK (VGA_BLANK),
    .oVGA_CLOCK (VGA_CLK),
    //  Control Signal
    .iCLK       (OSC_27), // 27 MHz clock
    .iRST_N     (DLY2)  
  );
    
	 
/*--------------------------- SRAM INITIALIZATIONS---------------------------*/
	// SRAM_control
	assign SRAM_ADDR = {addr_reg[18:10],addr_reg[8:0]};
	assign SRAM_DQ   = (weSRAM)? 16'hzzzz : data_reg ;
	assign SRAM_UB_N = ~addr_reg[9];// hi byte select enabled
	assign SRAM_LB_N = addr_reg[9]; // lo byte select enabled
	assign SRAM_CE_N = 0;			  // chip is enabled
	assign SRAM_WE_N = weSRAM;			  // write when ZERO
	assign SRAM_OE_N = 0;			  //output enable is overidden by WE
  
  //CONTROL REGISTERS
  reg [18:0] addr_reg;
  reg weSRAM;
  reg [15:0] data_reg;
  // IF SWITCH 16 IS ACTIVATED, SHOW SRAM, OTHERWISE SHOW DOWNSAMPLE SKIN MAP IF SWITCH 17 IS NOT ACTIVATED OR CAMERA STREAM IS ACTIVATED
  /*assign  mVGA_R = (DPDT_SW[16])?{(VGA_X[0]?SRAM_DQ[15:13]:SRAM_DQ[7:5]),7'b0}:(DPDT_SW[17]?Red:raw_r);
  assign  mVGA_G = (DPDT_SW[16])?{(VGA_X[0]?SRAM_DQ[12:10]:SRAM_DQ[4:2]),7'b0}:(DPDT_SW[17]?Green:raw_g);
  assign  mVGA_B = (DPDT_SW[16])?{(VGA_X[0]?SRAM_DQ[9:8]:SRAM_DQ[1:0]),8'b0}:(DPDT_SW[17]?Blue:raw_b);
 */
  assign  mVGA_R = DPDT_SW[17]?((blur)?red_dataout:Red):raw_r;
  assign  mVGA_G = DPDT_SW[17]?((blur)?green_dataout:Green):raw_g;
  assign  mVGA_B = DPDT_SW[17]?((blur)?blue_dataout:Blue):raw_b;
 
 //COLORS TO VGA
  wire [9:0]  mVGA_R;
  wire [9:0]  mVGA_G;
  wire [9:0]  mVGA_B;
  wire signed[9:0] Red, Green, Blue;
  reg signed [9:0] raw_r,raw_g,raw_b;
  /*--------------------------- SRAM INIT ENDS---------------------------*/
  
  /*--------------------- RAW VIDEO RGB CONVERTER------------------------*/
  //  For ITU-R 656 Decoder
  wire  [15:0] YCbCr;
  wire  [9:0]  TV_X;
  wire         TV_DVAL;

  //  For VGA Controller
  wire  [9:0]  mRed;
  wire  [9:0]  mGreen;
  wire  [9:0]  mBlue;
  wire  [10:0] VGA_X;
  wire  [10:0] VGA_Y;
  wire  VGA_Read;  //  VGA data request
  wire  m1VGA_Read;  //  Read odd field
  wire  m2VGA_Read;  //  Read even field

  //  For YUV 4:2:2 to YUV 4:4:4
  wire  [7:0]  mY;
  wire  [7:0]  mCb;
  wire  [7:0]  mCr;

  //  For field select
  wire  [15:0]  mYCbCr;
  wire  [15:0]  mYCbCr_d;
  wire  [15:0]  m1YCbCr;
  wire  [15:0]  m2YCbCr;
  wire  [15:0]  m3YCbCr;

  //  For Delay Timer
  wire      TD_Stable;
  wire      DLY0;
  wire      DLY1;
  wire      DLY2;

  //  For Down Sample
  wire  [3:0]  Remain;
  wire  [9:0]  Quotient;

  assign  m1VGA_Read =  VGA_Y[0]  ?  1'b0     :  VGA_Read;
  assign  m2VGA_Read =  VGA_Y[0]  ?  VGA_Read :  1'b0;
  assign  mYCbCr_d   =  !VGA_Y[0] ?  m1YCbCr  :  m2YCbCr;
  assign  mYCbCr     =  m5YCbCr;

  wire      mDVAL;

  //  Line buffer, delay one line
  Line_Buffer u10  
  (  
    .clken    (VGA_Read),
    .clock    (OSC_27),
    .shiftin  (mYCbCr_d),
    .shiftout (m3YCbCr)
  );

  Line_Buffer u11
  (  
    .clken    (VGA_Read),
    .clock    (OSC_27),
    .shiftin  (m3YCbCr),
    .shiftout (m4YCbCr)
  );

  wire  [15:0] m4YCbCr;
  wire  [15:0] m5YCbCr;
  wire  [8:0]  Tmp1,Tmp2;
  wire  [7:0]  Tmp3,Tmp4;

  assign  Tmp1    = m4YCbCr[7:0] + mYCbCr_d[7:0];
  assign  Tmp2    = m4YCbCr[15:8] + mYCbCr_d[15:8];
  assign  Tmp3    = Tmp1[8:2] + m3YCbCr[7:1];
  assign  Tmp4    = Tmp2[8:2] + m3YCbCr[15:9];
  assign  m5YCbCr = { Tmp4, Tmp3 };

  assign  TD_RESET = 1'b1;  //  Allow 27 MHz


  /*------------------- RAW VIDEO RGB CONVERTER ENDS----------------------*/
  /*######################################################################*/
  /*####################HUMAN TRACKER PROJECT BEGINS######################*/
  /*######################################################################*/
  
  /*----------- DEFINE PLACE TO STORE DOWNSAMPLED 40X30 SKIN MAP----------*/
  wire weds;
  wire [15:0] datain,dataout;
  wire [10:0] raddr,waddr;
  reg we_ds;
  reg [15:0] data_in,data_out;
  reg [15:0] data_temp[39:0];//THIS IS FOR SAVING 1 DOWNSAMPLED CELL ROW TO PLOT FOR THE NEXT 15 LINES IN VGA
  reg [9:0] x_coord; //OF SKIN PIXEL IN 640X480 SCREEN
  reg [8:0] y_coord;
  
  wire red_weds;
  wire [15:0] red_datain,red_dataout;
  wire [10:0] red_raddr,red_waddr;
  reg red_we_ds;
  reg [15:0] red_data_in,red_data_out;
  
  wire green_weds;
  wire [15:0] green_datain,green_dataout;
  wire [10:0] green_raddr,green_waddr;
  reg green_we_ds;
  reg [15:0] green_data_in,green_data_out;
  
  wire blue_weds;
  wire [15:0] blue_datain,blue_dataout;
  wire [10:0] blue_raddr,blue_waddr;
  reg blue_we_ds;
  reg [15:0] blue_data_in,blue_data_out;
  
  ram_infer downSampledView(//M4K MEMORY
  .data_out(dataout),
  .read_addr(raddr),
  .write_addr(waddr),
  .data_in(datain),
  .we(weds),
  .clk(OSC_27));
  
  ram_infer redPixels(//M4K MEMORY
  .data_out(red_dataout),
  .read_addr(red_raddr),
  .write_addr(red_waddr),
  .data_in(red_datain),
  .we(red_weds),
  .clk(OSC_27));
  
  ram_infer greenPixels(//M4K MEMORY
  .data_out(green_dataout),
  .read_addr(green_raddr),
  .write_addr(green_waddr),
  .data_in(green_datain),
  .we(green_weds),
  .clk(OSC_27));
  
    ram_infer bluePixels(//M4K MEMORY
  .data_out(blue_dataout),
  .read_addr(blue_raddr),
  .write_addr(blue_waddr),
  .data_in(blue_datain),
  .we(blue_weds),
  .clk(OSC_27));
  
  assign weds = we_ds;
  assign datain = data_in;
  assign raddr = {x_coord[9:4],y_coord[8:4]};
  assign waddr = {x_coord[9:4],y_coord[8:4]};
  
  assign red_weds = red_we_ds;
  assign red_datain = red_data_in;
  assign red_raddr = {x_coord[9:4],y_coord[8:4]};
  assign red_waddr = {x_coord[9:4],y_coord[8:4]};
  
  assign green_weds = green_we_ds;
  assign green_datain = green_data_in;
  assign green_raddr = {x_coord[9:4],y_coord[8:4]};
  assign green_waddr = {x_coord[9:4],y_coord[8:4]};
  
  assign blue_weds = blue_we_ds;
  assign blue_datain = blue_data_in;
  assign blue_raddr = {x_coord[9:4],y_coord[8:4]};
  assign blue_waddr = {x_coord[9:4],y_coord[8:4]};
  
  //FOR DEBUGGING
  assign LED_RED = hcxT;
  //assign LED_GREEN=cellCount;
  
  /*---------------- CENTROID AVERAGING BEGINS---------------------*/
  reg [31:0] hcxT,hcyT;//
  wire [5:0] mx,my;//where to draw pixel
  reg [5:0] hcx,hcy;
  wire [5:0] hcxbar,hcybar;
  wire [9:0] hcxavg,hcyavg;
  reg [10:0] cellCount;
  
  assign hcxbar = hcx;
  assign hcybar = hcy;
  
  average av1(.out(hcxavg),.in({hcxbar,25'b0}),.dk_const(5'd21),.clk(VGA_CLK));
  average av2(.out(hcyavg),.in({hcybar,25'b0}),.dk_const(5'd21),.clk(VGA_CLK));
  
  assign mx = hcxT/cellCount;//COMBINATIONAL DIVIDE
  assign my = hcyT/cellCount;//CENTROID AVERAGE FOR 1 FRAME

  /*---------------- CENTROID AVERAGING ENDS----------------------*/
  
  /*------------------ SKIN DETECTION BEGINS----------------------*/
  reg [9:0] color; 
  wire signed[9:0] Yr,Ur,Vr;
  assign Yr = (Red>>>2)+(Green>>>1)+(Blue>>>2); 
  assign Ur = Red-Green;//INTENSITY REMOVED
  assign Vr = Blue-Green;
  wire [15:0] skl = DPDT_SW[15:0]; //THRESHOLD OF TEMPORAL AND SPATIAL AVERAGING
  /*------------------ SKIN DETECTION ENDS-----------------------*/
  
  /*------------ SKIN PROCESSING STATE MACHINE BEGINS------------*/
  
  wire blur = data_out>skl || blurness[y_coord[8:4]][x_coord[9:4]-1] ||
  blurness[y_coord[8:4]-1][x_coord[9:4]] ||
  blurness[y_coord[8:4]][x_coord[9:4]+1] || 
  blurness[y_coord[8:4]+1][x_coord[9:4]];
  reg [39:0] blurness [29:0];
  always @(posedge OSC_27)
  begin
    x_coord <=VGA_X;
	 y_coord <=VGA_Y;
	 if(~KEY[1])
	 begin
		we_ds<=1'b0;// begin with EACH CENTROID
		hcx<=6'd19; //IN THE CENTER OF ITS OWN REGION
		hcy<=6'd7;
		color<=10'h3FF;
	 end
	 else
	 begin
	 //IF WE ARE AT A NEW FRAME,RESET ALL THE OLD CENTROID ACCUMULATIONS
	if(x_coord==10'b0 && y_coord==9'b0)
	begin
				cellCount<=11'b0;
				hcxT<=32'b0;
				hcyT<=32'b0;
	end
	//IF WE ARE IN ANY CENTROID CELL,FILL IT RED
	if(x_coord[9:4]==hcxavg[9:4] && y_coord[8:4]==hcyavg[8:4])
	begin
		raw_r<=10'h3FF;
		raw_g<=10'h0;
		raw_b<=10'h0;
	end
	else
	// OTHERWISE, START SKIN DETECTION
		if(x_coord[3:0]==4'b0 && y_coord[3:0]==4'b0)
		begin
			// STORE CURRENT SKIN MAP VALUES IN A LINE BUFFER FOR THE NEXT 15 LINES
			data_temp[x_coord[9:4]]<=(y_coord>9'b0 && x_coord>10'b0 )?dataout:16'b0;
			//IGNORE THE BOUNDARIES, VGA CONTROLLER SEEM TO HAVE SYNC ISSUES
			data_out<=(y_coord>9'b0 && x_coord>10'b0 )?dataout:16'b0;
			//PLOT THE SKIN PIXEL IF IT IS ABOVE THRESHOLD
			raw_r<=red_dataout;
			raw_g<=green_dataout;
			raw_b<=blue_dataout;
			
			//HEAD CENTROID ACCUMULATION
			if(data_out>skl && y_coord[8:4]>6'b0 && x_coord<10'd640 && x_coord>10'b0)
			begin
				cellCount<=cellCount+11'b1;
				hcxT<=hcxT+x_coord[9:4];
				hcyT<=hcyT+y_coord[8:4];
			end			
		end
		else
		begin
			//READ WHAT IS IN THE LINE BUFFER AND PLOT THE RESULTS
			data_out<=data_temp[x_coord[9:4]];
			raw_r<=red_dataout;
			raw_g<=green_dataout;
			raw_b<=blue_dataout;	
		end
		// IF WE ARE AT THE END OF A FRAME, CHECK TO SEE IF WE ARE
		// CONFIDENT THAT A NEW CENTROID VALUE CAN BE COMPUTED IN EACH CASE
		// IF SO, STORE THE DIVIDER VALUES
		if(x_coord==10'd639 && y_coord==9'd479 && cellCount>11'd3)//the head
		begin
			hcx<=mx;
			hcy<=my;
		end
		
		// IF 100<(R-G)<500 AND RED>BLUE, IT'S A SKIN PIXEL
		// (G-B) CHECK IS NOT DONE AS ITS REDUNDANT FOR LIVE CAMERA FEEDS
		if(Ur>10'd100 && Ur<10'd500 && Red>Blue)
		begin
				data_in<=dataout-(dataout>>8)+16'hFF;//average a 16x16 cell block
				we_ds<=1'b1;// always write
		end
		else
		begin
				data_in<=dataout-(dataout>>8);
				we_ds<=1'b1;// always write
		end
			
		if(x_coord[3:0]==4'b1111 && y_coord[3:0]==4'b1111)	begin		
			red_data_in   <= Red;
			green_data_in <= Green;
			blue_data_in  <= Blue;
			red_we_ds <= 1'b1;
			green_we_ds <= 1'b1;
			blue_we_ds <= 1'b1;
		end
		else
		begin
			red_we_ds <= 1'b0;
			green_we_ds <= 1'b0;
			blue_we_ds <= 1'b0;
		end
		if (data_out>skl)
			blurness[y_coord[8:4]][x_coord[9:4]]<=1'b1;
		else
			blurness[y_coord[8:4]][x_coord[9:4]]<=1'b0;
	 end
  end
  
  
  
  
  /* AUDIO STUFF */

wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= audio_out;
assign right_channel_audio_out	= audio_out;
//assign left_channel_audio_out = {right_channel_audio_in[31:14],14'b0};
//assign right_channel_audio_out = {right_channel_audio_in[31:14],14'b0};
//assign left_channel_audio_out = left_channel_audio_in;
//assign right_channel_audio_out = right_channel_audio_in;
assign write_audio_out			= audio_in_available & audio_out_allowed;

reg[17:0] sample_in_reg;
reg sample_wrreq_reg;
reg sample_rdreq_reg;
wire [17:0] sample_in = sample_in_reg;
wire [17:0] sample_out;
wire sample_wrreq = sample_wrreq_reg;
wire sample_rdreq = sample_rdreq_reg;
wire samples_empty;

reg[17:0] anon_in_reg;
reg anon_wrreq_reg;
reg anon_rdreq_reg;
wire signed [17:0] anon_in;
wire [17:0] anon_out;
wire anon_wrreq;
wire anon_rdreq = anon_rdreq_reg;
wire anon_empty;
wire anon_full;

reg [31:0] audio_out;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

 fifo input_samples(
	.data(sample_in),
	.rdclk(OSC_50),       //read at 50MHZ
	.rdreq(sample_rdreq),
	.wrclk(sclk),         //write at 48KHZ sample rate
	.wrreq(sample_wrreq),
	.q(sample_out),
	.rdempty(samples_empty),
	.wrfull());
	
 fifo output_samples(
	.data(anon_in),
	.rdclk(OSC_50),       //read at 50MHZ
	.rdreq(anon_rdreq),
	.wrclk(OSC_50),         //write at 50MHZ sample rate
	.wrreq(anon_wrreq),
	.q(anon_out),
	.rdempty(anon_empty),
	.wrfull(anon_full));
	
reg [11:0] cntr48;
reg sample_clk;
wire sclk = sample_clk;

always@(posedge OSC_50) begin
	if (cntr48 == 12'd1042) begin 
		cntr48 <= 0;
		sample_clk <= sample_clk ^ 1'b1;
	end
	else cntr48 <= cntr48 + 12'd1;
end

always@(posedge sclk) begin
	sample_wrreq_reg <= 1;
	sample_in_reg <= right_channel_audio_in[31:14];
end

reg [4:0] sample_state;

//read from the sample fifo
/*always@(posedge OSC_50) begin
	case(sample_state)
		0: begin
			if (samples_empty == 0) begin
				//start a read
				sample_rdreq_reg <= 1;
				sample_state <= 1;
				//anon_wrreq_reg <= 0;
			end
		end
		
		1: begin
			//waste a clock cycle for the read.
			sample_rdreq_reg <= 0;
			sample_state <= 2;
		end
		
		2: begin
			//anon_wrreq_reg <= 1;
			//anon_in_reg <= sample_out;
			audio_out <= {sample_out, 14'b0};
			sample_state <= 0;
		end
	endcase
end*/


reg [11:0] sample_counter;

always@(posedge OSC_50) begin
	case(sample_state)
		0: begin
			if (samples_empty == 0) begin
				//start a read
				sample_rdreq_reg <= 1;
				sample_state <= 1;
				
			end
			
			//always turn off sink_eop and sink_sop
			fft_sink_eop_reg <= 0;
			fft_sink_sop_reg <= 0;

			//turn off valid signal so we can wait for another sample
			fft_sink_valid_reg <= 0;
		end
		
		1: begin
			//waste a clock cycle for the read.
			sample_rdreq_reg <= 0;
			sample_state <= 2;
		end
		
		2: begin
			if (fft_sink_ready == 1) begin
				//write the sample to the FFT module
				fft_sink_valid_reg <= 1;
				fft_sink_real_reg <= sample_out;
				//audio_out<=sample_out;
				
				//go back to wait for another sample
				sample_state <= 0;
				
				//update the sample counter (used to index win)
				if (sample_counter < 255) sample_counter <= sample_counter + 1;
				else begin
					sample_counter <= 0;
					fft_sink_eop_reg <= 1;
				end
				
				//pulse sop on first sample
				if (sample_counter == 0) fft_sink_sop_reg <= 1;
			end
		end
	endcase
end


reg [3:0] ifft_state;
reg [11:0] ifft_sample_counter;

//Always ready to read from fft whenever there is room in ifft input buffer.
wire		ifft_sink_valid = fft_source_valid;
wire		ifft_sink_sop = fft_source_sop;
wire		ifft_sink_eop = fft_source_eop;
//wire signed [17:0]	ifft_sink_real = (fft_source_real >>> fft_source_exp);
//wire signed [17:0]	ifft_sink_imag = (fft_source_imag >>> fft_source_exp);
wire signed [17:0]	ifft_sink_real = fft_source_real;
wire signed [17:0]	ifft_sink_imag = fft_source_imag;
wire[1:0]	ifft_sink_error = fft_source_error;



/*FFT*/
reg fft_sink_valid_reg, fft_sink_eop_reg, fft_sink_sop_reg;
reg [17:0] fft_sink_real_reg;

wire fft_sink_sop = fft_sink_sop_reg;
wire fft_sink_eop = fft_sink_eop_reg;
wire fft_sink_valid = fft_sink_valid_reg;
wire fft_source_ready = ifft_sink_ready;
wire fft_source_sop, fft_source_eop, fft_source_valid, fft_sink_ready;
wire [1:0] fft_source_error;
wire signed [5:0] fft_source_exp;
wire signed [17:0] fft_sink_real = fft_sink_real_reg;
wire signed [17:0] fft_source_real, fft_source_imag;

theFFT fft1(
	.clk(OSC_50),
	.reset_n(KEY[0]),
	.inverse(1'b0),
	.sink_valid(fft_sink_valid),
	.sink_sop(fft_sink_sop),
	.sink_eop(fft_sink_eop),
	.sink_real(fft_sink_real),
	.sink_imag(18'b0), //never input any imaginary part
	.sink_error(2'b0), //no upstream module, so no error yet.
	.source_ready(fft_source_ready),
	.sink_ready(fft_sink_ready),
	.source_error(fft_source_error), //source error is connected to sink error of ifft
	.source_sop(fft_source_sop),
	.source_eop(fft_source_eop),
	.source_valid(fft_source_valid),
	.source_exp(fft_source_exp),
	.source_real(fft_source_real),
	.source_imag(fft_source_imag));

	
//always ready to take another anonymized output sample whenever the buffer is not full
wire ifft_source_ready = ~anon_full;
wire ifft_source_sop; //don't care about source sop
wire ifft_source_eop; //don't care about source eop either
wire ifft_source_valid; //wired to anon_wrreq to trigger a write whenever the output is valid
wire ifft_sink_ready;
wire [1:0] ifft_source_error;
wire signed [5:0] ifft_source_exp;
wire signed [17:0] ifft_source_real, ifft_source_imag;

assign anon_wrreq = ifft_source_ready & ifft_source_valid;
//assign anon_in = ((ifft_source_real >>> ifft_source_exp) >>> $signed(8));
assign anon_in = ifft_source_real;

theFFT ifft2(
	.clk(OSC_50),
	.reset_n(KEY[0]),
	.inverse(1'b1),
	.sink_valid(ifft_sink_valid),
	.sink_sop(ifft_sink_sop),
	.sink_eop(ifft_sink_eop),
	.sink_real(ifft_sink_real),
	.sink_imag(ifft_sink_imag),
	.sink_error(ifft_sink_error),
	.source_ready(ifft_source_ready),
	.sink_ready(ifft_sink_ready),
	.source_error(ifft_source_error),
	.source_sop(ifft_source_sop),
	.source_eop(ifft_source_eop),
	.source_valid(ifft_source_valid),
	.source_exp(ifft_source_exp),
	.source_real(ifft_source_real),
	.source_imag(ifft_source_imag));

assign LED_GREEN[1:0] = ifft_source_error; 
assign LED_GREEN[3:2] = fft_source_error;
assign LED_GREEN[8:4] = 0;

reg [4:0] anon_state;
always@(posedge OSC_50) begin
	case(anon_state)
		0: begin
			if (anon_empty == 0 && cntr48 == 0) begin
				//start a read
				anon_rdreq_reg <= 1;
				anon_state <= 1;
			end
		end
		
		1: begin
			//waste a clock cycle for the read.
			anon_rdreq_reg <= 0;
			anon_state <= 2;
		end
		
		2: begin
			audio_out <= {anon_out, 14'b0};
			anon_state <= 0;
		end
	endcase
end
//always@(posedge sclk) begin
//	audio_out <= right_channel_audio_in;
//end


 
Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(OSC_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);
  
  endmodule
