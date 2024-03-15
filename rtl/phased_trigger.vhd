---------------------------------------------------------------------------------
-- Penn State  
-- Dept. of Physics
--
-- PROJECT:      RNO-G lowthresh
-- FILE:         phased_trigger.vhd
-- AUTHOR:       Ryan Krebs
-- EMAIL         rjk5416@psu.edu
-- DATE:         2/24
--
-- DESCRIPTION:  phased_trigger
--
---------------------------------------------------------------------------------
library IEEE;
use ieee.std_logic_1164.all;
--use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
use ieee.math_real.log2;

use work.defs.all;

entity phased_trigger is
generic(
		ENABLE_PHASED_TRIG : std_logic := '1';
		--//trigger setting register: coinc trig enable is bit [8]
		trigger_enable_reg_adr : std_logic_vector(7 downto 0) := x"3D";
		phased_trig_reg_base	: std_logic_vector(7 downto 0):= x"50";
		phased_trig_param_reg	: std_logic_vector(7 downto 0):= x"80";
		address_reg_pps_delay: std_logic_vector(7 downto 0) := x"5E"
		);

port(
		rst_i			:	in		std_logic;
		clk_i			:	in		std_logic; --register clock 
		clk_data_i	:	in		std_logic; --data clock
		registers_i	:	in		register_array_type;
		
		ch0_data_i	: 	in		std_logic_vector(31 downto 0);
		ch1_data_i	:	in		std_logic_vector(31 downto 0);
		ch2_data_i	:	in		std_logic_vector(31 downto 0);
		ch3_data_i	:	in		std_logic_vector(31 downto 0);
		
		trig_bits_o : 	out	std_logic_vector(2*(num_beams+1)-1 downto 0); --for scalers
		phased_trig_o: 	out	std_logic; --trigger
		phased_trig_metadata_o: out std_logic_vector(num_beams-1 downto 0)
		);
end phased_trigger;

architecture rtl of phased_trigger is

constant streaming_buffer_length: integer := 16;
constant interp_factor: integer := 4;
constant interp_data_length: integer := interp_factor*(streaming_buffer_length-1)+1;
constant window_length:integer := 8;
constant baseline: signed(7 downto 0) := x"80";
constant phased_sum_bits: integer := 10;
constant phased_sum_length: integer := 8;
constant phased_sum_power_bits: integer := 20;
constant num_power_bits: integer := 23;
constant power_sum_bits:	integer := 23; --actually 25 but this fits into the io regs
constant input_power_thesh_bits:	integer := 12;
constant power_length: integer := 12;
constant power_low_bit: integer := 0;
constant power_high_bit: integer := power_low_bit+power_length-1;

type antenna_delays is array (num_beams-1 downto 0,num_channels-1 downto 0) of integer;
--signal beam_delays : antenna_delays := ((12,11,10,9),(45,45,45,45)); --it will optimize away a lot of the streaming buffer if these numbers are small
signal beam_delays : antenna_delays := (others=>(others=>32));
--should be ~ 500MHz delay * interp_factor, tho calc should return the interpolated delays for accuracy

type interpolated_data_array is array(3 downto 0, interp_data_length-1 downto 0) of signed(7 downto 0);
signal interp_data: interpolated_data_array;

type interpolated_buffer is array(3 downto 0, interp_data_length-1 downto 0) of signed(15 downto 0);
signal interp_buffer: interpolated_buffer;

type thresh_input is array (num_beams-1 downto 0) of unsigned(input_power_thesh_bits-1 downto 0);
signal input_trig_thresh : thresh_input;
signal input_servo_thresh : thresh_input;

--type streaming_data_array is array(7 downto 0) of std_logic_vector((streaming_buffer_length*8-1) downto 0);
--signal streaming_data : streaming_data_array := (others=>(others=>'0')); --pipeline data

type streaming_data_array is array(3 downto 0, streaming_buffer_length-1 downto 0) of signed(7 downto 0);
signal streaming_data : streaming_data_array := (others=>(others=>(others=>'0'))); --pipeline data


type phased_arr is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(phased_sum_bits-1 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves: phased_arr;

type square_waveform is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of unsigned(phased_sum_power_bits-1 downto 0);-- range 0 to 2**phased_sum_power_bits-1;--std_logic_vector(phased_sum_power_bits-1 downto 0);
signal phased_power : square_waveform;

type power_array is array (num_beams-1 downto 0) of unsigned(num_power_bits-1 downto 0);-- range 0 to 2**num_power_bits-1;--std_logic_vector(num_power_bits-1 downto 0); --log2(6*(16*6)^2) max power possible
signal trig_beam_thresh : power_array:=(others=>(others=>'0')) ; --trigger thresholds for all beams
signal servo_beam_thresh : power_array:=(others=>(others=>'0')) ;--(others=>(others=>'0')) --servo thresholds for all beams
signal power_sum : power_array; --power levels for all beams
signal avg_power: power_array;

signal triggering_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');
signal servoing_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');

signal phased_trigger : std_logic;
signal phased_trigger_reg : std_logic_vector(1 downto 0);

type trigger_regs is array(num_beams-1 downto 0) of std_logic_vector(1 downto 0);
signal beam_trigger_reg : trigger_regs;
signal beam_servo_reg : trigger_regs;

signal phased_servo : std_logic;
signal phased_servo_reg : std_logic_vector(1 downto 0);

type trigger_counter is array (num_beams-1 downto 0) of unsigned(15 downto 0);

signal trig_clear				: std_logic_vector(num_beams-1 downto 0);
signal servo_clear			: std_logic_vector(num_beams-1 downto 0);
signal trig_counter			: trigger_counter:= (others=>(others=>'0'));
signal servo_counter			: trigger_counter:= (others=>(others=>'0'));

signal last_trig_bits_latched : std_logic_vector(num_beams-1 downto 0);

signal trig_array_for_scalers : std_logic_vector(2*(num_beams+1) downto 0); --//on clk_data_i

signal internal_phased_trig_en : std_logic := '0'; --enable this trigger block from sw
signal internal_trigger_channel_mask : std_logic_vector(7 downto 0);
signal internal_trigger_beam_mask : std_logic_vector(num_beams-1 downto 0);
signal bits_for_trigger : std_logic_vector(num_beams-1 downto 0);
signal trig_array_for_scalars : std_logic_vector (2*(num_beams+1)-1 downto 0);

constant num_div: integer := integer(log2(real(phased_sum_length)));
constant pad_zeros: std_logic_vector(num_div-1 downto 0):=(others=>'0');

signal coinc_window_int	: unsigned(7 downto 0) := x"02"; --//num of clk_data_i periods

signal is_there_a_trigger: std_logic_vector(num_beams-1 downto 0);
signal is_there_a_servo: std_logic_vector(num_beams-1 downto 0);

signal trig_bits_metadata: std_logic_vector(num_beams-1 downto 0);

--------------
component signal_sync is
port(
		clkA			: in	std_logic;
		clkB			: in	std_logic;
		SignalIn_clkA	: in	std_logic;
		SignalOut_clkB	: out	std_logic);
end component;
component flag_sync is
port(
	clkA			: in	std_logic;
   clkB			: in	std_logic;
   in_clkA		: in	std_logic;
   busy_clkA	: out	std_logic;
   out_clkB		: out	std_logic);
end component;
--------------

begin
------------------------------------------------

proc_pipeline_data: process(clk_data_i)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		--ch 0
		for i in 4 to streaming_buffer_length-1 loop
			streaming_data(0,i)<=streaming_data(0,i-4);
		end loop;
		
		for i in 0 to 3 loop
			streaming_data(0,i)<=signed(ch0_data_i(8*(i+1)-1 downto 8*(i)))-baseline;
		end loop;
		
		
		--ch 1
		for i in 4 to streaming_buffer_length-1 loop
			streaming_data(1,i)<=streaming_data(1,i-4);
		end loop;
		
		for i in 0 to 3 loop
			streaming_data(1,i)<=signed(ch1_data_i(8*(i+1)-1 downto 8*(i)))-baseline;
		end loop;
		
		--ch 2
		for i in 4 to streaming_buffer_length-1 loop
			streaming_data(2,i)<=streaming_data(2,i-4);
		end loop;
		
		for i in 0 to 3 loop
			streaming_data(2,i)<=signed(ch2_data_i(8*(i+1)-1 downto 8*(i)))-baseline;
		end loop;
		
		--ch 3
		for i in 4 to streaming_buffer_length-1 loop
			streaming_data(3,i)<=streaming_data(3,i-4);
		end loop;
		
		for i in 0 to 3 loop
		 streaming_data(3,i)<=signed(ch3_data_i(8*(i+1)-1 downto 8*(i)))-baseline;
		end loop;
	end if;
end process;

proc_interpolate: process(clk_data_i)
--okay so interpolating 4 channels at a window size of 16 is probably better than 
--interpolating over 8 beams at a window size of 8. or maybe it's the same.. 8 beams at a window size of 16
--makes it 2x worse then. ok
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		--first pull off the samples we need. 
		for i in 0 to 3 loop --loop over channels
			for j in 0 to interp_data_length-1 loop
				if (j mod interp_factor) = 0 then
					interp_buffer(i,j)<=X"00"&streaming_data(i,j / interp_factor);
					interp_data(i,j) <= interp_buffer(i,j)(7 downto 0);
				else
					interp_buffer(i,j)<=(streaming_data(i,j/4)+(streaming_data(i,j/4+1)-streaming_data(i,j/4))*(j mod interp_factor)/interp_factor);--would be nice if didn't have to buffer this
					interp_data(i,j)(6 downto 0)<=interp_buffer(i,j)(6 downto 0);
					interp_data(i,j)(7)<=interp_buffer(i,j)(7);
				end if;
			end loop;
		end loop;
	end if;
end process;

proc_phasing: process(clk_data_i)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
		for i in 0 to num_beams-1 loop --loop over beams
			for j in 0 to phased_sum_length-1 loop
					
				phased_beam_waves(i,j) <= B"00"&(interp_data(0,beam_delays(i,0)-(j-3))
					+interp_data(1,beam_delays(i,1)-(j-3))
					+interp_data(2,beam_delays(i,2)-(j-3))
					+interp_data(3,beam_delays(i,3)-(j-3)));

			end loop;
		end loop;
	end if;
end process;


proc_square_to_power : process(clk_data_i,rst_i)
begin

	if rst_i = '1' then
		phased_power<=(others=>(others=>(others=>'0')));
	
	elsif rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		for i in 0 to num_beams-1 loop
			for j in 0 to phased_sum_length-1 loop
				phased_power(i,j)<=unsigned(phased_beam_waves(i,j)*phased_beam_waves(i,j))(phased_sum_power_bits-1 downto 0);
			end loop;
		end loop;
	
	end if;
end process;
--------------

proc_avg_beam_power : process(clk_data_i,rst_i)
begin		

	if rst_i = '1' then
		power_sum<=(others=>(others=>'0'));
	elsif rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		for i in 0 to num_beams-1 loop
			--power_sum(i)<=resize(phased_power(i,0)+phased_power(i,1),power_sum_bits);
				
			power_sum(i)<=B"000"&(phased_power(i,0)+phased_power(i,1)
				+phased_power(i,2)+phased_power(i,3));
				
				
			avg_power(i)(power_sum_bits-1 downto power_sum_bits-num_div)<=unsigned(pad_zeros);
			avg_power(i)(power_sum_bits-1-num_div downto 0)<=power_sum(i)(power_sum_bits-1 downto num_div); --divide by window size
		end loop;
	end if;
end process;

proc_get_triggering_beams : process(clk_data_i,rst_i)
begin
	if rst_i = '1' then
		phased_trigger_reg <= "00";
		phased_trigger <= '0'; -- the trigger

		phased_servo_reg <= "00";
		phased_servo <= '0';  --the servo trigger

		triggering_beam<= (others=>'0');
		servoing_beam<= (others=>'0');
		
		trig_clear <= (others=>'0');
		trig_counter <= (others=>(others=>'0'));
		servo_clear <= (others=>'0');
		servo_counter <= (others=>(others=>'0'));

		
	elsif rising_edge(clk_data_i) then
		--loop over the beams and this is a big mess
		for i in 0 to num_beams-1 loop
			phased_trig_metadata_o<=triggering_beam;
			if trig_counter(i) = coinc_window_int then
				trig_clear(i) <= '1';
			else
				trig_clear(i) <= '0';
			end if;
				
			if beam_trigger_reg(i)(0) = '1'  then
				trig_counter(i) <= trig_counter(i) + 1;
			else
				trig_counter(i) <= (others=>'0');
			end if;
			------------------------------------
			--for servoing only (basically a separate thresholding)
			if servo_counter(i) = coinc_window_int then
				servo_clear(i) <= '1';
			else
				servo_clear(i) <= '0';
			end if;
				
			if beam_servo_reg(i)(0) = '1' then
				servo_counter(i) <= servo_counter(i) + 1;
			else
				servo_counter(i) <= (others=>'0');
			end if;
			------------------------------------
		
			if avg_power(i)>trig_beam_thresh(i) then
				triggering_beam(i)<='1';
				beam_trigger_reg(i)(0)<='1';
			else
				triggering_beam(i)<='0';
				beam_trigger_reg(i)(0)<='0';
			end if;
			if avg_power(i)>servo_beam_thresh(i) then
				servoing_beam(i)<='1';
				beam_servo_reg(i)(0)<='1';
			else
				servoing_beam(i)<='0';
				beam_servo_reg(i)(0)<='0';
			end if;

		
			--if triggering_beam(i) = internal_trigger_beam_mask(i) then
			--	phased_trigger_reg(0)<='1';
			--else 
			--	phased_trigger_reg(0)<='0';
			--end if;
			--f servoing_beam(i) = internal_trigger_beam_mask(i) then
			--	phased_servo_reg(0)<='1';
			--else 
			--	phased_servo_reg(0)<='0';
			--end if;
		end loop;
		
		--is_there_a_trigger<= triggering_beam AND internal_trigger_beam_mask;
		--is_there_a_servo<= servoing_beam AND internal_trigger_beam_mask;
		
		--if to_integer(unsigned(is_there_a_trigger))>0 then
		--	phased_trigger_reg(0)<='1';
		--else
		--	phased_trigger_reg(0)<='0';
		--end if;
		--if to_integer(unsigned(is_there_a_servo))>0 then
		--	phased_servo_reg(0)<='1';
		--else
		--	phased_servo_reg(0)<='0';
		--end if;
		
		if (to_integer(unsigned(triggering_beam AND internal_trigger_beam_mask))>0) and (internal_phased_trig_en='1') then
			phased_trigger_reg(0)<='1';
		else
			phased_trigger_reg(0)<='0';
		end if;
		if (to_integer(unsigned(servoing_beam AND internal_trigger_beam_mask))>0) and (internal_phased_trig_en='1') then
			phased_servo_reg(0)<='1';
		else
			phased_servo_reg(0)<='0';
		end if;
		
		phased_trigger_reg(1)<=phased_trigger_reg(0);
		phased_servo_reg(1)<=phased_servo_reg(0);
		
		if phased_trigger_reg="01" then
			phased_trigger<='1';
		else
			phased_trigger<='0';
		end if;
		
		if phased_servo_reg="01" then
			phased_servo<='1';
		else
			phased_servo<='0';
		end if;
	end if;
end process;

proc_threshold_set:process(clk_data_i)
begin
	for i in 0 to num_beams-1 loop
		trig_beam_thresh(i)(power_high_bit downto power_low_bit)<=input_trig_thresh(i);
		servo_beam_thresh(i)(power_high_bit downto power_low_bit)<=input_servo_thresh(i);
	end loop;
end process;

--//sync some software commands to the data clock
TRIG_THRESHOLDS : for j in 0 to num_beams-1 generate
	INDIV_TRIG_BITS : for i in 0 to input_power_thesh_bits-1 generate
		xTRIGTHRESHSYNC : signal_sync
		port map(
		clkA				=> clk_i,
		clkB				=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_param_reg))+j)(i), --threshold from software
		SignalOut_clkB	=> input_trig_thresh(j)(i));
	end generate;
end generate;

SERVO_THRESHOLDS : for j in 0 to num_beams-1 generate
	INDIV_SERVO_BITS : for i in 0 to input_power_thesh_bits-1 generate
		xSERVOTHRESHSYNC : signal_sync
		port map(
		clkA				=> clk_i,
		clkB				=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_param_reg))+j)(i+12), --threshold from software
		SignalOut_clkB	=> input_servo_thresh(j)(i));
	end generate;
end generate;


------------

TRIGBEAMMASK : for i in 0 to num_beams-1 generate --beam masks. 1 == on
	xTRIGBEAMMASKSYNC : signal_sync
	port map(
	clkA	=> clk_i,   clkB	=> clk_data_i,
	SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_reg_base)))(i), --trig channel mask
	SignalOut_clkB	=> internal_trigger_beam_mask(i));
end generate;

------------

trig_array_for_scalars(2*num_beams+1 downto num_beams +2)<=servo_clear(num_beams-1 downto 0);
trig_array_for_scalars(num_beams+1)<=phased_servo;
trig_array_for_scalars(num_beams downto 1)<=trig_clear(num_beams-1 downto 0);
trig_array_for_scalars(0)<=phased_trigger;

----TRIGGER OUT!!
phased_trig_o <= phased_trigger_reg(0); --phased trigger for 0->1 transition. phased_trigger_reg(0) for absolute trigger 
--------------

TrigToScalers	:	 for i in 0 to 2*(num_beams)+1 generate 
	xTRIGSYNC : flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> trig_array_for_scalers(i),
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(i));
end generate TrigToScalers;
--------------
xTRIGENABLESYNC : signal_sync --phased trig enable bit
	port map(
	clkA				=> clk_i,
	clkB				=> clk_data_i,
	SignalIn_clkA	=> registers_i(to_integer(unsigned(trigger_enable_reg_adr)))(9), --overall phased trig enable bit
	SignalOut_clkB	=> internal_phased_trig_en);
end rtl;