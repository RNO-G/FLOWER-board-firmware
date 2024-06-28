---------------------------------------------------------------------------------
-- Penn State  
-- Dept. of Physics
--
-- PROJECT:      RNO-G lowthresh
-- FILE:         phased_trigger.vhd
-- AUTHOR:       Ryan Krebs
-- EMAIL         rjk5416@psu.edu
-- DATE:         6/27/2024
--
-- DESCRIPTION:  CIC upsampler with FIR compensator for better phasing
--
---------------------------------------------------------------------------------

library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.log2;
use work.defs.all;

entity fancy_interpolation is
port(
		rst_i			:	in		std_logic;
		clk_i			:	in		std_logic; --register clock 
		enable_i    :  in    std_logic;
	
		ch_data_i	: 	in		std_logic_vector(31 downto 0); --118 MHz data in
		ch_data_o	: 	out	std_logic_vector(127 downto 0) --4x interpolated out

		);
end fancy_interpolation;

architecture rtl of fancy_interpolation is

constant num_stages:integer:=4;

type coeffs_t is array(2 downto 0) of integer;
constant coeffs: coeffs_t := (-4,72,-4);
constant cic_gain: integer:=10;

type input_buffer_t is array(11 downto 0) of signed(7 downto 0);
type cic_temp_t is array(4 downto 0) of signed(19 downto 0);
type cic_up_temp_t is array(16 downto 0) of signed(19 downto 0);
type output_buffer_t is array(15 downto 0) of signed(7 downto 0);
--type comp_out is array(15 downto 0) of signed(7 downto 0);
type mid_comp_t is array (3 downto 0) of signed(15 downto 0);
type post_comp_t is array (7 downto 0) of signed(7 downto 0);


signal input_sig: input_buffer_t := (others=>x"00");
signal post_comp: post_comp_t:=(others=>x"00");
signal mid_comp:mid_comp_t:=(others=>x"0000");

signal comb_1:cic_temp_t:=(others=>x"00000");
signal comb_2:cic_temp_t:=(others=>x"00000");
signal comb_3:cic_temp_t:=(others=>x"00000");
signal comb_4:cic_temp_t:=(others=>x"00000");

signal pre_up:cic_temp_t:=(others=>x"00000");
signal post_up:cic_up_temp_t:=(others=>x"00000");

signal int_1:cic_up_temp_t:=(others=>x"00000");
signal int_2:cic_up_temp_t:=(others=>x"00000");
signal int_3:cic_up_temp_t:=(others=>x"00000");
signal int_4:cic_up_temp_t:=(others=>x"00000");

signal post_int:cic_up_temp_t:=(others=>x"00000");
signal up_output:output_buffer_t:=(others=>x"00");

begin

buff_input: process(clk_i, rst_i, enable_i)
begin
	if rst_i='1' or enable_i ='0' then
		input_sig<=(others=>x"00");
	
	elsif rising_edge(clk_i) then
	
		--buffer in samples
		for i in 1 to 4 loop
			input_sig(i-1)<=signed(ch_data_i(8*i-1 downto 8*(i-1)))-128;
		end loop;
		
		--move samples deeper
		input_sig(7 downto 4)<=input_sig(3 downto 0);
		--input_sig(11 downto 8)<=input_sig(7 downto 4);
		
	end if;
end process;

compensator: process(clk_i,rst_i,enable_i)
begin
	--assign bit shift to divide -- might be able to just divide on mid comp line since its pow of 2
		mid_comp(0)<=coeffs(2)*input_sig(4)+coeffs(1)*input_sig(3)+coeffs(0)*input_sig(2);
		mid_comp(1)<=coeffs(2)*input_sig(5)+coeffs(1)*input_sig(4)+coeffs(0)*input_sig(3);
		mid_comp(2)<=coeffs(2)*input_sig(6)+coeffs(1)*input_sig(5)+coeffs(0)*input_sig(4);
		mid_comp(3)<=coeffs(2)*input_sig(7)+coeffs(1)*input_sig(6)+coeffs(0)*input_sig(5);	
	
	if rst_i='1' then
		mid_comp<=(others=>x"0000");
		post_comp<=(others=>x"00");
		
	elsif rising_edge(clk_i) and enable_i ='1' then--there's going to be a bunch of resize things here
		for i in 0 to 4 loop
			post_comp(i)<=resize(mid_comp(i)(15 downto 6),8);
			post_comp(i+4)<=post_comp(i);
		end loop;
		
	end if;
end process;


cic: process(clk_i,rst_i,enable_i)
begin
	if rst_i='1' then

		post_up<=(others=>x"00000");
		post_int<=(others=>x"00000");
		
		comb_1<=(others=>x"00000");
		comb_2<=(others=>x"00000");
		comb_3<=(others=>x"00000");
		comb_4<=(others=>x"00000");
		
		int_1<=(others=>x"00000");
		int_2<=(others=>x"00000");
		int_3<=(others=>x"00000");
		int_4<=(others=>x"00000");
		
		ch_data_o<=(others=>'0');
		
	elsif enable_i ='1' then

		--do the upsampling
		--since this just pads zeros to the next bits of logic this can probably be unclocked
		for i in 1 to 16 loop
			if (i mod 4) =0 then
				post_up(i)<=comb_4(i/4);
			else 
				post_up(i)<=x"00000";
			end if;
		end loop;
		
	elsif rising_edge(clk_i) and enable_i='1' then
		
		--comb stage - 4 stage can process 4 samples streamed at once (if adds keep up)
		for i in 0 to 3 loop
			comb_1(i)<=resize(post_comp(i),20)-resize(post_comp(i+1),20);
			comb_2(i)<=comb_1(i)-comb_1(i+1);
			comb_3(i)<=comb_2(i)-comb_2(i+1);
			comb_4(i)<=comb_3(i)-comb_3(i+1);
		end loop;
		
		comb_1(4)<=comb_1(0);
		comb_2(4)<=comb_2(0);
		comb_3(4)<=comb_3(0);
		comb_4(4)<=comb_4(0);
		
		
		--int stage (complicated)
		for i in 0 to 16 loop
			int_1(i)<=post_up(i)+int_1(i+1);
			int_2(i)<=int_1(i)+int_2(i+1);
			int_3(i)<=int_2(i)+int_3(i+1);
			int_4(i)<=int_3(i)+int_4(i+1);
		end loop;
		
		int_1(4)<=int_1(0);
		int_2(4)<=int_2(0);
		int_3(4)<=int_3(0);
		int_4(4)<=int_4(0);
		
		--apply gain and send out
		for i in 1 to 16 loop
			post_int(i-1)<=int_4(i-1); --do other things like scale
			ch_data_o(i*8-1 downto i*8)<=std_logic_vector(resize(post_int(i-1),8));
		end loop;
		
	end if;
end process;
		
end rtl;



