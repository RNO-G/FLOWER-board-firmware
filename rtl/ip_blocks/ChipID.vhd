-- megafunction wizard: %Unique Chip ID Intel FPGA IP v18.1%
-- GENERATION: XML
-- ChipID.vhd

-- Generated using ACDS version 18.1 625

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity ChipID is
	port (
		clkin      : in  std_logic                     := '0'; --  clkin.clk
		reset      : in  std_logic                     := '0'; --  reset.reset
		data_valid : out std_logic;                            -- output.valid
		chip_id    : out std_logic_vector(63 downto 0)         --       .data
	);
end entity ChipID;

architecture rtl of ChipID is
	component altchip_id is
		generic (
			DEVICE_FAMILY : string                        := "";
			ID_VALUE      : std_logic_vector(63 downto 0) := "1111111111111111111111111111111111111111111111111111111111111111"
		);
		port (
			clkin      : in  std_logic                     := 'X'; -- clk
			reset      : in  std_logic                     := 'X'; -- reset
			data_valid : out std_logic;                            -- valid
			chip_id    : out std_logic_vector(63 downto 0)         -- data
		);
	end component altchip_id;

begin

	chipid_inst : component altchip_id
		generic map (
			DEVICE_FAMILY => "Cyclone V",
			ID_VALUE      => "1111111111111111111111111111111111111111111111111111111111111111"
		)
		port map (
			clkin      => clkin,      --  clkin.clk
			reset      => reset,      --  reset.reset
			data_valid => data_valid, -- output.valid
			chip_id    => chip_id     --       .data
		);

end architecture rtl; -- of ChipID
-- Retrieval info: <?xml version="1.0"?>
--<!--
--	Generated by Altera MegaWizard Launcher Utility version 1.0
--	************************************************************
--	THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
--	************************************************************
--	Copyright (C) 1991-2021 Altera Corporation
--	Any megafunction design, and related net list (encrypted or decrypted),
--	support information, device programming or simulation file, and any other
--	associated documentation or information provided by Altera or a partner
--	under Altera's Megafunction Partnership Program may be used only to
--	program PLD devices (but not masked PLD devices) from Altera.  Any other
--	use of such megafunction design, net list, support information, device
--	programming or simulation file, or any other related documentation or
--	information is prohibited for any other purpose, including, but not
--	limited to modification, reverse engineering, de-compiling, or use with
--	any other silicon devices, unless such use is explicitly licensed under
--	a separate agreement with Altera or a megafunction partner.  Title to
--	the intellectual property, including patents, copyrights, trademarks,
--	trade secrets, or maskworks, embodied in any such megafunction design,
--	net list, support information, device programming or simulation file, or
--	any other related documentation or information provided by Altera or a
--	megafunction partner, remains with Altera, the megafunction partner, or
--	their respective licensors.  No other licenses, including any licenses
--	needed under any third party's intellectual property, are provided herein.
---->
-- Retrieval info: <instance entity-name="altchip_id" version="18.1" >
-- Retrieval info: 	<generic name="DEVICE_FAMILY" value="Cyclone V" />
-- Retrieval info: 	<generic name="ID_VALUE" value="18446744073709551615" />
-- Retrieval info: </instance>
-- IPFS_FILES : ChipID.vho
-- RELATED_FILES: ChipID.vhd, altchip_id.v
