---------------------------------------------------------------------------------
-- Univ. of Chicago  
--    --KICP--
--
-- PROJECT:      phased-array trigger board
-- FILE:         scalers_top.vhd
-- AUTHOR:       e.oberla
-- EMAIL         ejo@uchicago.edu
-- DATE:         7/2017
--
-- DESCRIPTION:  manage board scalers and readout of scalers 
--               
---------------------------------------------------------------------------------
library IEEE;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

use work.defs.all;

entity scalers_top is
	generic(
		scaler_width   : integer := 12);
	port(
		rst_i				:		in		std_logic;
		clk_i				:		in 	std_logic;
		gate_i			:		in		std_logic;
		reg_i				:		in		register_array_type;
		trig_bits_i : in std_logic_vector(2*(num_beams)+1 downto 0);
		pps_cycle_counter_i : in std_logic_vector(47 downto 0);
		
		scaler_to_read_o  :   out	std_logic_vector(23 downto 0));
end scalers_top;

architecture rtl of scalers_top is

constant num_scalers : integer := 6*(num_beams+1)+1;
type scaler_array_type is array(num_scalers-1 downto 0) of std_logic_vector(scaler_width-1 downto 0);

signal internal_scaler_array : scaler_array_type := (others=>(others=>'0'));
signal latched_scaler_array : scaler_array_type; --//assigned after refresh pulse
signal latched_pps_cycle_counter : std_logic_vector(47 downto 0);

--//need to create a single pulse every Hz with width of 10 MHz clock period
signal refresh_clk_counter_100Hz 	:	std_logic_vector(27 downto 0) := (others=>'0');
signal refresh_clk_counter_1Hz 	:	std_logic_vector(27 downto 0) := (others=>'0');
signal refresh_clk_counter_100mHz:	std_logic_vector(27 downto 0) := (others=>'0');
signal refresh_clk_100Hz				:	std_logic := '0';
signal refresh_clk_1Hz				:	std_logic := '0';
signal refresh_clk_100mHz			:	std_logic := '0';
--//for 10 MHz
constant REFRESH_CLK_MATCH_100Hz 		: 	std_logic_vector(27 downto 0) := x"00186A0";   
--constant REFRESH_CLK_MATCH_100Hz 		: 	std_logic_vector(27 downto 0) := x"00186A0";  
constant REFRESH_CLK_MATCH_1HZ 		: 	std_logic_vector(27 downto 0) 	:= x"0989680";  
constant REFRESH_CLK_MATCH_100mHz 	: 	std_logic_vector(27 downto 0) 	:= x"5F5E100";  	
component scaler
port(
	rst_i 		: in 	std_logic;
	clk_i			: in	std_logic;
	refresh_i	: in	std_logic;
	count_i		: in	std_logic;
	scaler_o		: out std_logic_vector(scaler_width-1 downto 0));
end component;
-------------------------------------------------------------------------------
begin
-------------------------------------------------------------------------------
--proc_assign_scalers_to_metadata : running_scalers_o <= internal_scaler_array(32) & internal_scaler_array(0);
-------------------------------------------------------------------------------
--//scaler 63 is the `scaler pps'
proc_scaler_pps : process(clk_i, refresh_clk_1Hz)
begin
	if rising_edge(clk_i) and refresh_clk_1Hz = '1' then
		internal_scaler_array(0) <= internal_scaler_array(0) + 1;
	end if;
end process;
--//scalers 0-11
CoincTrigScalers1Hz : for i in 1 to 2*(num_beams+1)-1 generate
	xCOINC1Hz : scaler
	port map(
		rst_i => rst_i,
		clk_i => clk_i,
		refresh_i => refresh_clk_1Hz,
		count_i => trig_bits_i(i),
		scaler_o => internal_scaler_array(i));
end generate;
--//scalers 12-23
CoincTrigScalers1HzGated : for i in 0 to 2*(num_beams+1)-1 generate
	xCOINC1Hz : scaler
	port map(
		rst_i => rst_i,
		clk_i => clk_i,
		refresh_i => refresh_clk_1Hz,
		count_i => trig_bits_i(i) and gate_i,
		scaler_o => internal_scaler_array(i+2*(num_beams+1)));
end generate;
--//scalers 24-35
CoincTrigScalers100Hz : for i in 0 to 2*(num_beams+1)-1 generate
	xCOINC100Hz : scaler
	port map(
		rst_i => rst_i,
		clk_i => clk_i,
		refresh_i => refresh_clk_100Hz,
		count_i => trig_bits_i(i),
		scaler_o => internal_scaler_array(i+4*(num_beams+1)));
end generate;
-------------------------------------		
proc_save_scalers : process(rst_i, clk_i, reg_i)
begin
	if rst_i = '1' then
		for i in 0 to num_scalers-1 loop
			latched_scaler_array(i) <= (others=>'0');
		end loop;
		latched_pps_cycle_counter <= (others=>'0');
		scaler_to_read_o <= (others=>'0');
	
	elsif rising_edge(clk_i) and reg_i(40)(0) = '1' then
		latched_scaler_array <= internal_scaler_array;
		latched_pps_cycle_counter <= pps_cycle_counter_i; -- latch the pps counter in same fashion as other scalers
		
	elsif rising_edge(clk_i) then
		if unsigned(reg_i(41)(7 downto 0))<6*(num_beams+1)+2 then
			scaler_to_read_o<=latched_scaler_array(to_integer(unsigned(reg_i(41)(7 downto 0)))+1)&latched_scaler_array(to_integer(unsigned(reg_i(41)(7 downto 0))));
		else
			scaler_to_read_o<=latched_scaler_array(1)&latched_scaler_array(0);
		end if;
	end if;
end process;

-------------------------------------------------------------------
--//make 1 Hz and 100mHz refresh pulses from the main iface clock (10 MHz)
proc_make_refresh_pulse : process(clk_i)
begin
	if rising_edge(clk_i) then
		
		if refresh_clk_1Hz = '1' then
			refresh_clk_counter_1Hz <= (others=>'0');
		else
			refresh_clk_counter_1Hz <= refresh_clk_counter_1Hz + 1;
		end if;
		--//pulse refresh when refresh_clk_counter = REFRESH_CLK_MATCH
		case refresh_clk_counter_1Hz is
			when REFRESH_CLK_MATCH_1HZ =>
				refresh_clk_1Hz <= '1';
			when others =>
				refresh_clk_1Hz <= '0';
		end case;
		
		--//////////////////////////////////////
		
		if refresh_clk_100mHz = '1' then
			refresh_clk_counter_100mHz <= (others=>'0');
		else
			refresh_clk_counter_100mHz <= refresh_clk_counter_100mHz + 1;
		end if;
		--//pulse refresh when refresh_clk_counter = REFRESH_CLK_MATCH
		case refresh_clk_counter_100mHz is
			when REFRESH_CLK_MATCH_100mHz =>
				refresh_clk_100mHz <= '1';
			when others =>
				refresh_clk_100mHz <= '0';
		end case;
		
		--//////////////////////////////////////
		
		if refresh_clk_100Hz = '1' then
			refresh_clk_counter_100Hz <= (others=>'0');
		else
			refresh_clk_counter_100Hz <= refresh_clk_counter_100Hz + 1;
		end if;
		--//pulse refresh when refresh_clk_counter = REFRESH_CLK_MATCH
		case refresh_clk_counter_100Hz is
			when REFRESH_CLK_MATCH_100Hz =>
				refresh_clk_100Hz <= '1';
			when others =>
				refresh_clk_100Hz <= '0';
		end case;
		
	end if;
end process;
end rtl;