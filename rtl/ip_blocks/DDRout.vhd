-- megafunction wizard: %ALTDDIO_OUT%
-- GENERATION: STANDARD
-- VERSION: WM1.0
-- MODULE: ALTDDIO_OUT 

-- ============================================================
-- File Name: DDRout.vhd
-- Megafunction Name(s):
-- 			ALTDDIO_OUT
--
-- Simulation Library Files(s):
-- 			altera_mf
-- ============================================================
-- ************************************************************
-- THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
--
-- 18.1.0 Build 625 09/12/2018 SJ Standard Edition
-- ************************************************************


--Copyright (C) 2018  Intel Corporation. All rights reserved.
--Your use of Intel Corporation's design tools, logic functions 
--and other software and tools, and its AMPP partner logic 
--functions, and any output files from any of the foregoing 
--(including device programming or simulation files), and any 
--associated documentation or information are expressly subject 
--to the terms and conditions of the Intel Program License 
--Subscription Agreement, the Intel Quartus Prime License Agreement,
--the Intel FPGA IP License Agreement, or other applicable license
--agreement, including, without limitation, that your use is for
--the sole purpose of programming logic devices manufactured by
--Intel and sold by Intel or its authorized distributors.  Please
--refer to the applicable agreement for further details.


LIBRARY ieee;
USE ieee.std_logic_1164.all;

LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;

ENTITY DDRout IS
	PORT
	(
		aclr		: IN STD_LOGIC ;
		datain_h		: IN STD_LOGIC_VECTOR (0 DOWNTO 0);
		datain_l		: IN STD_LOGIC_VECTOR (0 DOWNTO 0);
		outclock		: IN STD_LOGIC ;
		outclocken		: IN STD_LOGIC ;
		dataout		: OUT STD_LOGIC_VECTOR (0 DOWNTO 0)
	);
END DDRout;


ARCHITECTURE SYN OF ddrout IS

	SIGNAL sub_wire0	: STD_LOGIC_VECTOR (0 DOWNTO 0);

BEGIN
	dataout    <= sub_wire0(0 DOWNTO 0);

	ALTDDIO_OUT_component : ALTDDIO_OUT
	GENERIC MAP (
		extend_oe_disable => "OFF",
		intended_device_family => "Cyclone V",
		invert_output => "OFF",
		lpm_hint => "UNUSED",
		lpm_type => "altddio_out",
		oe_reg => "UNREGISTERED",
		power_up_high => "OFF",
		width => 1
	)
	PORT MAP (
		aclr => aclr,
		datain_h => datain_h,
		datain_l => datain_l,
		outclock => outclock,
		outclocken => outclocken,
		dataout => sub_wire0
	);



END SYN;

-- ============================================================
-- CNX file retrieval info
-- ============================================================
-- Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
-- Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
-- Retrieval info: CONSTANT: EXTEND_OE_DISABLE STRING "OFF"
-- Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone V"
-- Retrieval info: CONSTANT: INVERT_OUTPUT STRING "OFF"
-- Retrieval info: CONSTANT: LPM_HINT STRING "UNUSED"
-- Retrieval info: CONSTANT: LPM_TYPE STRING "altddio_out"
-- Retrieval info: CONSTANT: OE_REG STRING "UNREGISTERED"
-- Retrieval info: CONSTANT: POWER_UP_HIGH STRING "OFF"
-- Retrieval info: CONSTANT: WIDTH NUMERIC "1"
-- Retrieval info: USED_PORT: aclr 0 0 0 0 INPUT NODEFVAL "aclr"
-- Retrieval info: CONNECT: @aclr 0 0 0 0 aclr 0 0 0 0
-- Retrieval info: USED_PORT: datain_h 0 0 1 0 INPUT NODEFVAL "datain_h[0..0]"
-- Retrieval info: CONNECT: @datain_h 0 0 1 0 datain_h 0 0 1 0
-- Retrieval info: USED_PORT: datain_l 0 0 1 0 INPUT NODEFVAL "datain_l[0..0]"
-- Retrieval info: CONNECT: @datain_l 0 0 1 0 datain_l 0 0 1 0
-- Retrieval info: USED_PORT: dataout 0 0 1 0 OUTPUT NODEFVAL "dataout[0..0]"
-- Retrieval info: CONNECT: dataout 0 0 1 0 @dataout 0 0 1 0
-- Retrieval info: USED_PORT: outclock 0 0 0 0 INPUT_CLK_EXT NODEFVAL "outclock"
-- Retrieval info: CONNECT: @outclock 0 0 0 0 outclock 0 0 0 0
-- Retrieval info: USED_PORT: outclocken 0 0 0 0 INPUT NODEFVAL "outclocken"
-- Retrieval info: CONNECT: @outclocken 0 0 0 0 outclocken 0 0 0 0
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout.vhd TRUE FALSE
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout.qip TRUE FALSE
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout.bsf FALSE TRUE
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout_inst.vhd FALSE TRUE
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout.inc FALSE TRUE
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout.cmp FALSE TRUE
-- Retrieval info: GEN_FILE: TYPE_NORMAL DDRout.ppf TRUE FALSE
-- Retrieval info: LIB_FILE: altera_mf
