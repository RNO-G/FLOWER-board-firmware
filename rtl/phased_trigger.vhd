---------------------------------------------------------------------------------
-- Penn State  
-- Dept. of Physics
--
-- PROJECT:      RNO-G lowthresh
-- FILE:         phased_trigger.vhd
-- AUTHOR:       Ryan Krebs
-- EMAIL         rjk5416@psu.edu
-- DATE:         5/8/2024
--
-- DESCRIPTION:  phased_trigger
--
---------------------------------------------------------------------------------
library IEEE;
use ieee.std_logic_1164.all;
--use ieee.std_logic_unsigned.all; internet says to use numeric_std only
use ieee.numeric_std.all;
use ieee.math_real.log2;
use work.defs.all;

entity phased_trigger is
generic(
		ENABLE_PHASED_TRIG : std_logic := '1';
		trigger_enable_reg_adr : std_logic_vector(7 downto 0) := x"3D";
		phased_trig_reg_base	: std_logic_vector(7 downto 0):= x"50";
		address_reg_pps_delay: std_logic_vector(7 downto 0) := x"5E";
		phased_trig_param_reg	: std_logic_vector(7 downto 0):= x"80"
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
		phased_trig_metadata_o: out std_logic_vector(num_beams-1 downto 0); --for triggering beams
		power_o: out std_logic_vector(22 downto 0) --test avg power for debugging located in metadata
		);
end phased_trigger;

architecture rtl of phased_trigger is

--definitions + constants -- I realize I can now just use 'length too
constant streaming_buffer_length: integer := 8;
constant interp_factor: integer := 4;
constant interp_data_length: integer := interp_factor*(24)+1;--interp_factor*(streaming_buffer_length-1)+1;
constant window_length:integer := 16;
constant baseline: unsigned(7 downto 0) := x"80";
constant phased_sum_bits: integer := 7; --8. trying 7 bit lut
constant phased_sum_length: integer := 32; --8 real samples ... not sure if it should be 8 or 16. longer windows smooths things. shorter window gives higher peak
constant phased_sum_power_bits: integer := 14;--16 with calc. trying 7-> 14 lut
constant num_power_bits: integer := 20;
constant power_sum_bits:	integer := 20; --actually 25 but this fits into the io regs
constant input_power_thesh_bits:	integer := 12;
constant power_length: integer := 12;
constant num_div: integer := 5;--can be calculated using -> integer(log2(real(phased_sum_length)));
constant pad_zeros: std_logic_vector(num_div-1 downto 0):=(others=>'0');

type antenna_delays is array (num_beams-1 downto 0,num_channels-1 downto 0) of integer;
--9 beams. 8 expected + one with equal delays THIS ONE HAS THE CHANNELS IN THE WRONG ORDER... should be (ch3 ch2 ch1 ch0), instead thought (ch0,ch1,ch2,ch3)
--constant beam_delays : antenna_delays:=	((32,32,32,32),(15,33,53,73),(15,31,49,66),(15,28,43,57),(15,25,36,46),
--	(15,21,30,36),(15,18,23,25),(15,15,17,16),(19,18,18,15));

--beam zero points up at 60 deg. beam 7 points down at 60. beam 8 is flat inputs
--n=1.8 
constant beam_delays:antenna_delays:=	((32,32,32,32),(15,17,17,18),(18,18,15,15),(27,23,18,15),(36,30,22,15),(47,37,25,15),(57,43,29,15),(65,49,32,15),(72,53,34,15));
--n=1.75 
--constant beam_delays:antenna_delays:=	((32,32,32,32),(15,17,16,18),(19,18,16,15),(27,24,19,15),(37,30,22,15),(47,37,25,15),(56,43,28,15),(64,49,31,15),(71,53,33,15));

type interpolated_data_array is array(3 downto 0, interp_data_length-1 downto 0) of signed(7 downto 0);
signal interp_data: interpolated_data_array;

signal input_data: std_logic_vector(127 downto 0);
signal output_data: std_logic_vector(511 downto 0);

type temp_interp is array (3 downto 0) of std_logic_vector(127 downto 0);
signal temp_int: temp_interp;

type interpolated_buffer is array(3 downto 0, interp_data_length-1 downto 0) of signed(15 downto 0);
signal interp_buffer: interpolated_buffer;

type thresh_input is array (num_beams-1 downto 0) of unsigned(input_power_thesh_bits-1 downto 0);
signal input_trig_thresh : thresh_input;
signal input_servo_thresh : thresh_input;

--short streaming buffer for linear interp
type streaming_data_array is array(3 downto 0, streaming_buffer_length-1 downto 0) of signed(7 downto 0);
signal streaming_data : streaming_data_array := (others=>(others=>(others=>'0'))); --pipeline data

--temp buffer to calculate coherent sum waveforms to check for saturation
type phased_arr_buff is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(9 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves_buff: phased_arr_buff;

--7 bit limited coherent sum for power LUT
type phased_arr is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(phased_sum_bits-1 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves: phased_arr;

--coherent sum power
type square_waveform is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of unsigned(phased_sum_power_bits-1 downto 0);-- range 0 to 2**phased_sum_power_bits-1;--std_logic_vector(phased_sum_power_bits-1 downto 0);
signal phased_power : square_waveform;

--big arrays for thresholds/ power integrations
type power_array is array (num_beams-1 downto 0) of unsigned(num_power_bits-1 downto 0);-- range 0 to 2**num_power_bits-1;--std_logic_vector(num_power_bits-1 downto 0); --log2(6*(16*6)^2) max power possible
signal trig_beam_thresh : power_array:=(others=>(others=>'0')) ; --trigger thresholds for all beams
signal servo_beam_thresh : power_array:=(others=>(others=>'0')) ;--(others=>(others=>'0')) --servo thresholds for all beams
signal power_sum : power_array; --power integration using all 32 samples
signal power_sum_lower : power_array; --power integration using the lower 16 samples 
signal power_sum_upper : power_array; --power integration using the upper 16 samples
signal avg_power: power_array; --average power (power_sum shifted down by log2(32)=5 bits)
signal latched_power_out: power_array; 

--threshold offset in case thresholds saturate over 4095
signal threshold_offset: unsigned(11 downto 0):=x"000";

--mask of which beam triggers/servos
signal triggering_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');
signal servoing_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');
--signal bits_for_trigger : std_logic_vector(num_beams-1 downto 0);

--actual output from the phased trigger and servo
signal phased_trigger : std_logic;
signal phased_trigger_reg : std_logic_vector(1 downto 0);
signal phased_servo : std_logic;
signal phased_servo_reg : std_logic_vector(1 downto 0);

--mimic of simple trigger channels regs (probably not needed)
type trigger_regs is array(num_beams-1 downto 0) of std_logic_vector(1 downto 0);
signal beam_trigger_reg : trigger_regs;
signal beam_servo_reg : trigger_regs;

type trigger_counter is array (num_beams-1 downto 0) of unsigned(15 downto 0);
signal trig_clear				: std_logic_vector(num_beams-1 downto 0);
signal servo_clear			: std_logic_vector(num_beams-1 downto 0);
signal trig_counter			: trigger_counter:= (others=>(others=>'0'));
signal servo_counter			: trigger_counter:= (others=>(others=>'0'));

--previous trig beam bits
signal last_trig_bits_latched : std_logic_vector(num_beams-1 downto 0);

signal trig_array_for_scalers : std_logic_vector(2*(num_beams+1) downto 0); --//on clk_data_i

--enables+input masks
signal internal_phased_trig_en : std_logic := '0'; --enable this trigger block from sw
signal internal_trigger_channel_mask : std_logic_vector(7 downto 0);
signal internal_trigger_beam_mask : std_logic_vector(num_beams-1 downto 0);

--full regs for ouput to scalers
signal trig_array_for_scalars : std_logic_vector (2*(num_beams+1)-1 downto 0);

--output for triggering beams for metadata
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

component power_lut_7 is --7 bit lut for calculating power
port(
		clk_i    : in std_logic;
		a			: in	signed(6 downto 0);
		z			: out	unsigned(13 downto 0));
end component;

component fir_upsampling is
port(
		clk: std_logic;
		reset_n: in std_logic;
		ast_sink_data: in std_logic_vector(127 downto 0);
		ast_sink_valid:in std_logic;
		ast_sink_error:in std_logic_vector(1 downto 0);
		ast_source_data: out std_logic_vector(511 downto 0);
		ast_source_valid: out std_logic;
		ast_source_error: out std_logic_vector(1 downto 0)
		);
end component;

--------------

begin
------------------------------------------------

--buffer samples into the phased trigger module
proc_pipeline_data: process(clk_data_i,internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
	
		--pull new data in
		for i in 0 to 3 loop
			streaming_data(0,i)<=signed(unsigned(ch0_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			streaming_data(1,i)<=signed(unsigned(ch1_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			streaming_data(2,i)<=signed(unsigned(ch2_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			streaming_data(3,i)<=signed(unsigned(ch3_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
		end loop;
				
		--shift the data
		for i in 4 to streaming_buffer_length-1 loop
			streaming_data(0,i)<=streaming_data(0,i-4);
			streaming_data(1,i)<=streaming_data(1,i-4);
			streaming_data(2,i)<=streaming_data(2,i-4);
			streaming_data(3,i)<=streaming_data(3,i-4);
		end loop;
	end if;
end process;




--xinterp0 : entity work.fancy_interpolation
--port map(
--	rst_i			=> rst_i,
--	clk_i			=> clk_data_i,
--	enable_i		=> internal_phased_trig_en,
--	ch_data_i	=> ch0_data_i(31 downto 0),
--	ch_data_o	=> temp_int(0)
--);
--xinterp1 : entity work.fancy_interpolation
--port map(
--	rst_i			=> rst_i,
--	clk_i			=> clk_data_i,
--	enable_i		=> internal_phased_trig_en,
--	ch_data_i	=> ch1_data_i(31 downto 0),
--	ch_data_o	=> temp_int(1)
--);
--xinterp2 : entity work.fancy_interpolation
--port map(
--	rst_i			=> rst_i,
--	clk_i			=> clk_data_i,
--	enable_i		=> internal_phased_trig_en,
--	ch_data_i	=> ch2_data_i(31 downto 0),
--	ch_data_o	=> temp_int(2)
--);
--xinterp3 : entity work.fancy_interpolation
--port map(
--	rst_i			=> rst_i,
--	clk_i			=> clk_data_i,
--	enable_i		=> internal_phased_trig_en,
--	ch_data_i	=> ch3_data_i(31 downto 0),
--	ch_data_o	=> temp_int(3)
--);

--assign_interp: process(clk_data_i)
--begin
--	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
--	for i in 0 to 3 loop
--		for j in 0 to 15 loop
--			interp_data(i,j)<=signed(temp_int(i)(8*(j+1)-1 downto 8*j));
--		end loop;
--	end loop;
--	end if;
--end process;
			
--generate
xUpsampling:fir_upsampling
port map(
		clk=>clk_data_i,
		reset_n=>not rst_i,
		ast_sink_data=>input_data,
		ast_sink_valid=>internal_phased_trig_en,
		ast_sink_error=>b"00",
		ast_source_data=>output_data,
		ast_source_valid=>open,
		ast_source_error=>open
		);

--process inputs and outputs
proc_process_fir_upsampling: process(clk_data_i,rst_i,internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) then
		for ch in 0 to 3 loop
			for sample in 0 to 3 loop
				input_data(ch*32+sample*8+7 downto ch*32+sample*8)<=std_logic_vector(streaming_data(ch,sample));
			end loop;
			
			for up_sample in 0 to 15 loop
				interp_data(ch,up_sample)<=signed(output_data(ch*128+up_sample*8+7 downto ch*128+up_sample*8));
			end loop;
		end loop;
	end if;
end process;
			
--linear interpolation. Only interpolate between the 4(+1 from the last block) samples coming in
proc_interpolate: process(clk_data_i, internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then

		for i in 0 to 3 loop --loop over channels
			
			--for j in 0 to 4*interp_factor-1 loop
			--
			--	--linear interpolate the samples coming in and keep known samples
			--	if (j mod interp_factor) = 0 then
			--		interp_data(i,j)<=streaming_data(i,j / interp_factor);--known samples dont need interpolation
			--	else
			--		interp_data(i,j)<=resize((streaming_data(i,j/4)+(streaming_data(i,j/4+1)-streaming_data(i,j/4))*(j mod interp_factor)/interp_factor),8);--I hope it does the shift in the compiler (pow od 2.)
			--	end if;				
			--end loop;
			
			--shift the interpolated samples so we don't need to recalculate
			for j in 4*interp_factor to interp_data_length-1 loop
				interp_data(i,j)<=interp_data(i,j-4*interp_factor);
			end loop;
		
		end loop;
	end if;
end process;

--do phasing to calculate the coherently summed waveforms
proc_phasing: process(clk_data_i,internal_phased_trig_en)
begin
	--if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
		--for i in 0 to num_beams-1 loop --loop over beams
			--for j in 0 to phased_sum_length-1 loop
			
			   --calculate temp phased sum waveforms with larger data size
				--phased_beam_waves_buff(i,j)<=resize(interp_data(0,beam_delays(i,0)+(j-15)),10)
				--	+resize(interp_data(1,beam_delays(i,1)+(j-15)),10)
				--	+resize(interp_data(2,beam_delays(i,2)+(j-15)),10)
				--	+resize(interp_data(3,beam_delays(i,3)+(j-15)),10);

				
				--saturate low and high for 7 bit LUT (more costly than limiting input bits to 5 - (32 adc) but the channels are a bit diff so maybe this is better)
				--if(to_integer(phased_beam_waves_buff(i,j))>63) then
				--	phased_beam_waves(i,j)<=b"0111111";--saturate max
				--elsif(to_integer(phased_beam_waves_buff(i,j))<-63) then
				--  phased_beam_waves(i,j)<=b"1000000"; --saturate min
				--else
				--	phased_beam_waves(i,j)<=resize(phased_beam_waves_buff(i,j),7); 
				--end if;	
				
			--end loop;
		--end loop;
	--end if;
	
		
	for i in 0 to num_beams-1 loop --loop over beams
		for j in 0 to 15 loop --for j in 16 to phased_sum_length-1 loop
			--I think I can reduce this by only calc 16 and then buffering the previous 16
			phased_beam_waves_buff(i,j)<=resize(interp_data(0,beam_delays(i,0)+(j-15)),10)
				+resize(interp_data(1,beam_delays(i,1)+(j-15)),10)
				+resize(interp_data(2,beam_delays(i,2)+(j-15)),10)
				+resize(interp_data(3,beam_delays(i,3)+(j-15)),10);
				
			if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
				--saturate low and high for 7 bit LUT (more costly than limiting input bits to 5 - (32 adc) but the channels are a bit diff so maybe this is better)
				if(to_integer(phased_beam_waves_buff(i,j))>63) then
					phased_beam_waves(i,j)<=b"0111111";--saturate max
				elsif(to_integer(phased_beam_waves_buff(i,j))<-63) then
				  phased_beam_waves(i,j)<=b"1000000"; --saturate min
				else
					phased_beam_waves(i,j)<=resize(phased_beam_waves_buff(i,j),7); 
				end if;	
			end if;
		end loop;
		for j in 16 to phased_sum_length-1 loop
			if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
				phased_beam_waves(i,j)<=phased_beam_waves(i,j-16);
			end if;
		end loop;
	end loop;
	
	
end process;

--calculate the power
--this just uses a LUT in logic to find the power from a signed value. If it synthesizes as BRAM is would be too slow but is okay as sync_ram.
DO_POWER_BEAM : for i in 0 to num_beams-1 generate
	DO_POWER_SAMPLE : for j in 0 to 15 generate --for j in 0 to phased_sum_length-1 generate
		xPOWERLUT : power_lut_7
		port map(
		clk_i => clk_data_i, --tried clock but this looks like bram (too slow), unclocked should just be LUT
		a				=> phased_beam_waves(i,j),
		z				=> phased_power(i,j));
	end generate;
end generate;

proc_move_power:process(clk_data_i)
begin
	if rising_edge(clk_data_i) and internal_phased_trig_en='1' then
		for i in 0 to num_beams-1 loop --loop over beams
			for j in 0 to 15 loop --for j in 16 to phased_sum_length-1 loop
				phased_power(i,j+16)<=phased_power(i,j);
			end loop;
		end loop;
	end if;
end process;
		

--keep for posterity
--this uses dsp's + logic to calculate the power. DSP might be needed for different interp. and uses as much logic anyway. 
--proc_square_to_power : process(clk_data_i,internal_phased_trig_en)
--begin
--	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
--		for i in 0 to num_beams-1 loop
--			for j in 0 to phased_sum_length-1 loop
--				
--				phased_power(i,j)<=unsigned(abs(phased_beam_waves(i,j)))*unsigned(abs(phased_beam_waves(i,j)));
--				
--			end loop;
--		end loop;
--	end if;
--end process;
--------------

--do the power integration
proc_avg_beam_power : process(clk_data_i)
begin		

	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		for i in 0 to num_beams-1 loop
				
			--manually type all these... not sure how to make is a loop.
			--also split in half for timing
			power_sum_lower(i)<=resize(phased_power(i,0),num_power_bits)+resize(phased_power(i,1),num_power_bits)+resize(phased_power(i,2),num_power_bits)
				+resize(phased_power(i,3),num_power_bits)+resize(phased_power(i,4),num_power_bits)+resize(phased_power(i,5),num_power_bits)
				+resize(phased_power(i,6),num_power_bits)+resize(phased_power(i,7),num_power_bits)+resize(phased_power(i,8),num_power_bits)
				+resize(phased_power(i,9),num_power_bits)+resize(phased_power(i,10),num_power_bits)+resize(phased_power(i,11),num_power_bits)
				+resize(phased_power(i,12),num_power_bits)+resize(phased_power(i,13),num_power_bits)+resize(phased_power(i,14),num_power_bits)
				+resize(phased_power(i,15),num_power_bits);
			power_sum_upper(i)<=power_sum_lower(i);
			--power_sum_upper(i)<=	resize(phased_power(i,16),num_power_bits)+resize(phased_power(i,17),num_power_bits)
			--	+resize(phased_power(i,18),num_power_bits)+resize(phased_power(i,19),num_power_bits)+resize(phased_power(i,20),num_power_bits)
			--	+resize(phased_power(i,21),num_power_bits)+resize(phased_power(i,22),num_power_bits)+resize(phased_power(i,23),num_power_bits) --all these are unsigned so add should be ok
			--	+resize(phased_power(i,24),num_power_bits)+resize(phased_power(i,25),num_power_bits)+resize(phased_power(i,26),num_power_bits) --all these are unsigned so add should be ok
			--	+resize(phased_power(i,27),num_power_bits)+resize(phased_power(i,28),num_power_bits)+resize(phased_power(i,29),num_power_bits) --all these are unsigned so add should be ok
			--	+resize(phased_power(i,30),num_power_bits)+resize(phased_power(i,31),num_power_bits); --all these are unsigned so add should be ok
			power_sum(i)<=power_sum_lower(i)+power_sum_upper(i);

		   --get the average power (bit selecting on the ones used)		
			avg_power(i)(power_sum_bits-1 downto power_sum_bits-num_div)<=unsigned(pad_zeros);
			avg_power(i)(power_sum_bits-1-num_div downto 0)<=power_sum(i)(power_sum_bits-1 downto num_div); --divide by window size
		end loop;
	end if;
end process;

--process the actual trigger
proc_get_triggering_beams : process(clk_data_i,rst_i)
begin
	if rst_i = '1' then
		phased_trigger_reg <= "00";
		phased_trigger <= '0'; -- the trigger

		phased_servo_reg <= "00";
		phased_servo <= '0';  --the servo trigger

		triggering_beam<= (others=>'0');
		servoing_beam<= (others=>'0');
		
	elsif rising_edge(clk_data_i) then
		--loop over the beams and this is a big mess
		for i in 0 to num_beams-1 loop
			
			--calculate if a beam is triggering or seroing
			if avg_power(i)>trig_beam_thresh(i) then
				triggering_beam(i)<='1';
				beam_trigger_reg(i)(0)<='1';
				latched_power_out(i)<=avg_power(i);
			else
				triggering_beam(i)<='0';
				beam_trigger_reg(i)(0)<='0';
			end if;
			
			beam_trigger_reg(i)(1)<=beam_trigger_reg(i)(0);
			if avg_power(i)>servo_beam_thresh(i) then
				servoing_beam(i)<='1';
				beam_servo_reg(i)(0)<='1';
			else
				servoing_beam(i)<='0';
				beam_servo_reg(i)(0)<='0';
			end if;
			beam_servo_reg(i)(1)<=beam_servo_reg(i)(0);

		end loop;
		
		--this is the core of figuring out if a trigger needs to happen
		if (to_integer(unsigned(triggering_beam AND internal_trigger_beam_mask))>0) and (internal_phased_trig_en='1') then
			phased_trigger_reg(0)<='1';
			power_o(num_power_bits-1 downto 0)<=std_logic_vector(latched_power_out(0)(num_power_bits-1 downto 0));
			phased_trig_metadata_o<=triggering_beam AND internal_trigger_beam_mask;
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

--process 12 bit input thresholds with some threshold offset (defined in software) if needed (likely not needed)
proc_threshold_set:process(clk_data_i)
begin
   if rising_edge(clk_data_i) then
		for i in 0 to num_beams-1 loop
			trig_beam_thresh(i)<=resize(input_trig_thresh(i),num_power_bits)+threshold_offset;
			servo_beam_thresh(i)<=resize(input_servo_thresh(i),num_power_bits)+threshold_offset;
		end loop;
	end if;
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

THRESH_OFFSET: for i in 0 to 11 generate
	xTHRESHOFFSETSYNC : signal_sync
		port map(
		clkA	=> clk_i,   clkB	=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_reg_base))+1)(i), --phased threshold offset
		SignalOut_clkB	=> threshold_offset(i));
end generate;
------------
--these were causing issues. instead just used triggering and servo beams + phased trigger/servo
--trig_array_for_scalars(2*num_beams+1 downto num_beams +2)<=servo_clear(num_beams-1 downto 0);
--trig_array_for_scalars(num_beams+1)<=phased_servo;
--trig_array_for_scalars(num_beams downto 1)<=trig_clear(num_beams-1 downto 0);
--trig_array_for_scalars(0)<=phased_trigger;

--trig_array_for_scalars(2*num_beams+1 downto num_beams +2)<=servoing_beam(num_beams-1 downto 0);
--trig_array_for_scalars(num_beams+1)<=phased_servo;
--trig_array_for_scalars(num_beams downto 1)<=triggering_beam(num_beams-1 downto 0);
--trig_array_for_scalars(0)<=phased_trigger;

----TRIGGER OUT!!
phased_trig_o <= phased_trigger;-- phased trigger for 0->1 transition. phased_trigger_reg(0) for any trigger condition


-------------
--phased trigger scaler
trigscaler: flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> phased_trigger,
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(0));

--beam trigger scalers
TrigToScalers	:	 for i in 0 to num_beams-1 generate 
	xTRIGSYNC : flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> triggering_beam(i),-- and internal_trigger_beam_mask(i),
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(i+1));
end generate TrigToScalers;

--phased servo scaler
servoscaler: flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> phased_servo,
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(num_beams+1));

--beam servo scalers
ServoToScalers	:	 for i in 0 to num_beams-1 generate 
	xSERVOSYNC : flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> servoing_beam(i),-- and internal_trigger_beam_mask(i),
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(i+num_beams+2));
end generate ServoToScalers;

--------------
xTRIGENABLESYNC : signal_sync --phased trig enable bit
	port map(
	clkA				=> clk_i,
	clkB				=> clk_data_i,
	SignalIn_clkA	=> registers_i(to_integer(unsigned(trigger_enable_reg_adr)))(9), --overall phased trig enable bit
	SignalOut_clkB	=> internal_phased_trig_en);
end rtl;