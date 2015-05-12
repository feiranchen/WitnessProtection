// megafunction wizard: %ALTSQRT%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: ALTSQRT 

// ============================================================
// File Name: sqrt.v
// Megafunction Name(s):
// 			ALTSQRT
//
// Simulation Library Files(s):
// 			altera_mf
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 11.0 Build 157 04/27/2011 SJ Full Version
// ************************************************************


//Copyright (C) 1991-2011 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Altera Program License 
//Subscription Agreement, Altera MegaCore Function License 
//Agreement, or other applicable license agreement, including, 
//without limitation, that your use is for the sole purpose of 
//programming logic devices manufactured by Altera and sold by 
//Altera or its authorized distributors.  Please refer to the 
//applicable agreement for further details.


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module sqrt (
	clk,
	radical,
	q,
	remainder);

	input	  clk;
	input	[16:0]  radical;
	output	[8:0]  q;
	output	[9:0]  remainder;

	wire [8:0] sub_wire0;
	wire [9:0] sub_wire1;
	wire [8:0] q = sub_wire0[8:0];
	wire [9:0] remainder = sub_wire1[9:0];

	altsqrt	ALTSQRT_component (
				.clk (clk),
				.radical (radical),
				.q (sub_wire0),
				.remainder (sub_wire1)
				// synopsys translate_off
				,
				.aclr (),
				.ena ()
				// synopsys translate_on
				);
	defparam
		ALTSQRT_component.pipeline = 1,
		ALTSQRT_component.q_port_width = 9,
		ALTSQRT_component.r_port_width = 10,
		ALTSQRT_component.width = 17;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone II"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: PIPELINE NUMERIC "1"
// Retrieval info: CONSTANT: Q_PORT_WIDTH NUMERIC "9"
// Retrieval info: CONSTANT: R_PORT_WIDTH NUMERIC "10"
// Retrieval info: CONSTANT: WIDTH NUMERIC "17"
// Retrieval info: USED_PORT: clk 0 0 0 0 INPUT NODEFVAL "clk"
// Retrieval info: USED_PORT: q 0 0 9 0 OUTPUT NODEFVAL "q[8..0]"
// Retrieval info: USED_PORT: radical 0 0 17 0 INPUT NODEFVAL "radical[16..0]"
// Retrieval info: USED_PORT: remainder 0 0 10 0 OUTPUT NODEFVAL "remainder[9..0]"
// Retrieval info: CONNECT: @clk 0 0 0 0 clk 0 0 0 0
// Retrieval info: CONNECT: @radical 0 0 17 0 radical 0 0 17 0
// Retrieval info: CONNECT: q 0 0 9 0 @q 0 0 9 0
// Retrieval info: CONNECT: remainder 0 0 10 0 @remainder 0 0 10 0
// Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL sqrt_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL sqrt_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
