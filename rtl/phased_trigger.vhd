---------------------------------------------------------------------------------
-- Penn State  
-- Dept. of Physics
--
-- PROJECT:      RNO-G lowthresh
-- FILE:         phased_trigger.vhd
-- AUTHOR:       Ryan Krebs
-- EMAIL         rjk5416@psu.edu
-- DATE:         9/12/2024
--
-- DESCRIPTION:  phased_trigger
-- IF USING 1/2 SAMPLING CLOCK, USE clk_data_2_i - doesn't meet timing
-- OTHERWISE if 1/4 SAMPLING CLOCK, USE clk_data_i
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
		--clk_data_2_i	:	in		std_logic; --data clock
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
constant sample_bit_length: integer:=8;
constant baseline: unsigned(7 downto 0) := x"80";
constant phased_sum_bits: integer := 7; --8. trying 7 bit lut
constant phased_sum_length: integer := 32; --8 real samples ... not sure if it should be 8 or 16. longer windows smooths things. shorter window gives higher peak
constant phased_sum_power_bits: integer := 14;--16 with calc. trying 7-> 14 lut
constant num_power_bits: integer := 18;
constant power_sum_bits:	integer := 18; --actually 25 but this fits into the io regs
constant input_power_thesh_bits:	integer := 12;
constant power_length: integer := 12;
constant num_div: integer := 5;--can be calculated using -> integer(log2(real(phased_sum_length)));
constant pad_zeros: std_logic_vector(num_div-1 downto 0):=(others=>'0');
constant num_channels:integer:=4;
constant step_size:integer:=4; --sample clock / processing clock here
type antenna_delays is array (num_beams-1 downto 0,num_channels-1 downto 0) of integer range 0 to 127;

--beam zero points up at 60 deg. beam 7 points down at 60. beam 8 is flat inputs
--n=1.8 
--constant beam_delays:antenna_delays:=	((32,32,32,32),(15,17,17,18),(18,18,15,15),(27,23,18,15),(36,30,22,15),(47,37,25,15),(57,43,29,15),(65,49,32,15),(72,53,34,15));
--n=1.75 
--constant beam_delays:antenna_delays:=	((32,32,32,32),(15,17,16,18),(19,18,16,15),(27,24,19,15),(37,30,22,15),(47,37,25,15),(56,43,28,15),(64,49,31,15),(71,53,33,15));
--constant sample_bit_length: integer:=5;

--9 beams between -60 and 60 deg. commandeer flat beams so one points at pulser
--constant beam_delays:antenna_delays:=((15,15,15,15),(18,18,15,15),(25,22,18,15),(33,28,21,15),(42,33,23,15),(50,39,26,15),(58,45,29,15),(65,49,32,15),(71,53,33,15));
constant beam_delays:antenna_delays:=((15,17,16,18),(18,18,15,15),(25,22,18,15),(33,28,21,15),(42,33,23,15),(50,39,26,15),(58,45,29,15),(65,49,32,15),(71,53,33,15));
--type specific_delays is array(num_beams-1 downto 0, 3 downto 0) of signed(3 downto 0);
--signal spec_delays: specific_delays:=(others=>(others=>(x"0")));

--short streaming buffer for linear interp
type streaming_data_array is array(3 downto 0, streaming_buffer_length-1 downto 0) of signed(7 downto 0);
signal streaming_data : streaming_data_array := (others=>(others=>(others=>'0'))); --pipeline data

--temp buffers to assign input to and output from the fir filters
signal input_data: std_logic_vector(num_channels*step_size*8-1 downto 0);
signal output_data: std_logic_vector(num_channels*step_size*interp_factor*sample_bit_length-1 downto 0);

--buffer to store the interpolated sample for being pulled when doing the beamforming / summation
type interpolated_data_array is array(3 downto 0, interp_data_length-1 downto 0) of signed(sample_bit_length-1 downto 0);
signal interp_data: interpolated_data_array;

--temp buffer to calculate coherent sum waveforms to check for saturation
type phased_arr_buff is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(9 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves_buff: phased_arr_buff;

--7 bit limited coherent sum for power LUT
type phased_arr is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(phased_sum_bits-1 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves: phased_arr;

--instantaneous power
type square_waveform is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of unsigned(phased_sum_power_bits-1 downto 0);-- range 0 to 2**phased_sum_power_bits-1;--std_logic_vector(phased_sum_power_bits-1 downto 0);
signal phased_power : square_waveform;

--big arrays for thresholds/ average power
type power_array is array (num_beams-1 downto 0) of unsigned(num_power_bits-1 downto 0);-- range 0 to 2**num_power_bits-1;--std_logic_vector(num_power_bits-1 downto 0); --log2(6*(16*6)^2) max power possible
signal trig_beam_thresh : power_array:=(others=>(others=>'0')) ; --trigger thresholds for all beams
signal servo_beam_thresh : power_array:=(others=>(others=>'0')) ;--(others=>(others=>'0')) --servo thresholds for all beams
signal power_sum : power_array; --power integration using all 32 samples
signal power_sum_overlap : power_array; --power integration using all 32 samples
signal power_sum_0 : power_array; --partial power integration (lower 8 samples)
signal power_sum_1 : power_array; --partial power integration (upper 8 samples)
signal power_sum_2 : power_array; --partial power integration (last clock's power_sum_0)
signal power_sum_3 : power_array; --partial power integration (last clock's power_sum_1)
signal power_sum_4 : power_array; --partial power integration (last clock's power_sum_2)

signal avg_power: power_array; --average power (power_sum shifted down by log2(32)=5 bits)
signal avg_power_overlap: power_array; --average power (power_sum shifted down by log2(32)=5 bits)
signal latched_power_out: power_array; 

--input thresholds, 12 bits from registers then increased to ~16 by the threshold offset 
type thresh_input is array (num_beams-1 downto 0) of unsigned(input_power_thesh_bits-1 downto 0);
signal input_trig_thresh : thresh_input;
signal input_servo_thresh : thresh_input;

--threshold offset in case thresholds saturate over 4095 (they shouldn't)
signal threshold_offset: unsigned(11 downto 0):=x"000";

--mask of which beam triggers/servos to use in the trigger
signal triggering_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');
signal servoing_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');

--signal bits_for_trigger : std_logic_vector(num_beams-1 downto 0);

--actual output from the phased trigger and servo
signal phased_trigger : std_logic;
signal phased_trigger_reg : std_logic_vector(1 downto 0);
signal phased_servo : std_logic;
signal phased_servo_reg : std_logic_vector(1 downto 0);

--copy of simple trigger channel regs (probably not needed)
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
--signal internal_trigger_channel_mask : std_logic_vector(num_channels-1 downto 0); if masking channel from coh sum (not implemented)
signal internal_trigger_beam_mask : std_logic_vector(num_beams-1 downto 0);

--full regs for ouput to scalers
signal trig_array_for_scalars : std_logic_vector (2*(num_beams+1)-1 downto 0);

--output for triggering beams for metadata
signal trig_bits_metadata: std_logic_vector(num_beams-1 downto 0);

--if using clock of fs/2, you need to pull in 2 samples per clock to keep up with fs/4 sample block
--signal input_phase:std_logic:='0';

signal phased_trig_metadata: std_logic_vector(num_beams-1 downto 0); --for triggering beams

-------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------
--components, modules, etc

--thing for clock domain crossing
component signal_sync is
port(
		clkA			: in	std_logic;
		clkB			: in	std_logic;
		SignalIn_clkA	: in	std_logic;
		SignalOut_clkB	: out	std_logic);
end component;

--thing for clock domain crossing
component flag_sync is
port(
	clkA			: in	std_logic;
   clkB			: in	std_logic;
   in_clkA		: in	std_logic;
   busy_clkA	: out	std_logic;
   out_clkB		: out	std_logic);
end component;

--look up table to calculate power. 7 bit input signed to 14 bit unsigned
component power_lut_7 is --7 bit lut for calculating power
port(
		clk_i    : in std_logic;
		a			: in	signed(6 downto 0);
		z			: out	unsigned(13 downto 0));
end component;

--initialize fir upsampling block
component fir_upsampling is
port(
		clk: std_logic;
		reset_n: in std_logic;
		ast_sink_data: in std_logic_vector(input_data'length-1 downto 0);
		ast_sink_valid:in std_logic;
		ast_sink_error:in std_logic_vector(1 downto 0);
		ast_source_data: out std_logic_vector(output_data'length-1 downto 0);
		ast_source_valid: out std_logic;
		ast_source_error: out std_logic_vector(1 downto 0)
		);
end component;

-------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------
--begin rtl

begin
	
--buffer samples into the phased trigger module
--streaming data should really be used much other than storing the latest 4 sample locally
proc_pipeline_data: process(clk_data_i,internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
	
		--pull new data in
		for i in 0 to step_size-1 loop
			--if input_phase='0' then
			--	input_phase<='1';
				streaming_data(0,i)<=signed(unsigned(ch0_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
				streaming_data(1,i)<=signed(unsigned(ch1_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
				streaming_data(2,i)<=signed(unsigned(ch2_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
				streaming_data(3,i)<=signed(unsigned(ch3_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			--else --fs/2 was too fast. leaving for posterity
			--	input_phase<='0';
			--	streaming_data(0,i)<=signed(unsigned(ch0_data_i(8*(i+1+2)-1 downto 8*(i+2)))-baseline);
			--	streaming_data(1,i)<=signed(unsigned(ch1_data_i(8*(i+1+2)-1 downto 8*(i+2)))-baseline);
			--	streaming_data(2,i)<=signed(unsigned(ch2_data_i(8*(i+1+2)-1 downto 8*(i+2)))-baseline);
			--	streaming_data(3,i)<=signed(unsigned(ch3_data_i(8*(i+1+2)-1 downto 8*(i+2)))-baseline);
				
			--end if;
			
			--basic pull in samplles from data stream
			--pull in samples and subtract off 0 to make signed ints
			--streaming_data(0,i)<=signed(unsigned(ch0_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			--streaming_data(1,i)<=signed(unsigned(ch1_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			--streaming_data(2,i)<=signed(unsigned(ch2_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
			--streaming_data(3,i)<=signed(unsigned(ch3_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
		end loop;
				
		--shift the data
		for i in step_size to streaming_buffer_length-1 loop
			streaming_data(0,i)<=streaming_data(0,i-step_size);
			streaming_data(1,i)<=streaming_data(1,i-step_size);
			streaming_data(2,i)<=streaming_data(2,i-step_size);
			streaming_data(3,i)<=streaming_data(3,i-step_size);
		end loop;
	end if;
end process;


--generate FIR filter
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

		
--process inputs and outputs to the FIR filter module by reassigning vector to signed ints
proc_process_fir_upsampling: process(clk_data_i,rst_i,internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) then --not sure if these go into regs in the fir filter, doesn't hurt too much in case
		for ch in 0 to 3 loop
		
			for sample in 0 to step_size-1 loop
				input_data(ch*(step_size*8)+sample*8+8-1 downto ch*(step_size*8)+sample*8)<=std_logic_vector(streaming_data(ch,sample));
			end loop;
			
			for up_sample in 0 to interp_factor*step_size-1 loop
				interp_data(ch,up_sample)<=signed(output_data(ch*(interp_factor*step_size*sample_bit_length)+up_sample*sample_bit_length+sample_bit_length-1 downto ch*(interp_factor*step_size*sample_bit_length)+up_sample*sample_bit_length));
			end loop;
			
		end loop;
	end if;
end process;
			
			
--move interpolated samples along buffer. used with the fir filter. see old versions of code for linear interpolation
proc_interpolate: process(clk_data_i, internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then

		for ch in 0 to 3 loop --loop over channels
			
			--shift the interpolated samples so we don't need to recalculate
			for j in step_size*interp_factor to interp_data_length-1 loop
				interp_data(ch,j)<=interp_data(ch,j-step_size*interp_factor);
			end loop;
		
		end loop;
	end if;
end process;


--do phasing to calculate the coherently summed waveforms
proc_phasing: process(clk_data_i,internal_phased_trig_en)
begin
	
				
	--if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
	--
	--	for i in 0 to num_beams-1 loop --loop over beams
	--		for j in 0 to step_size*interp_factor-1 loop
	--			
	--			--calculate coh summed waveforms using just 5 bit upsampled samples
	--			phased_beam_waves(i,j)<=resize(interp_data(0,beam_delays(i,0)+(j-15)),7)+resize(interp_data(1,beam_delays(i,1)+(j-15)),7)
	--				+resize(interp_data(2,beam_delays(i,2)+(j-15)),7)+resize(interp_data(3,beam_delays(i,3)+(j-15)),7); 
	--			
	--			--below is if we can adjust beam shifts from software - tried and this blew up and wouldn't fit
	--			--phased_beam_waves(i,j)<=resize(interp_data(0,beam_delays(i,0)+to_integer(spec_delays(i,0))+(j-15)),7)+resize(interp_data(1,beam_delays(i,1)+to_integer(spec_delays(i,1))+(j-15)),7)
	--			--	+resize(interp_data(2,beam_delays(i,2)+to_integer(spec_delays(i,2))+(j-15)),7)+resize(interp_data(3,beam_delays(i,3)+to_integer(spec_delays(i,3))+(j-15)),7); 
	--		end loop;
   --	
	--		--move already calculated coh sum waves along the buffer
	--		for j in step_size*interp_factor to phased_sum_length-1 loop
	--			phased_beam_waves(i,j)<=phased_beam_waves(i,j-step_size*interp_factor);	
	--		end loop;
	--	
	--	end loop;
	--end if;
	
	
	for i in 0 to num_beams-1 loop --loop over beams
		for j in 0 to step_size*interp_factor-1 loop
		
			--assign the temporary as async, but then place it into a reg... cleaner looking code
			phased_beam_waves_buff(i,j)<=resize(interp_data(0,beam_delays(i,0)+(j-15)),10)
				+resize(interp_data(1,beam_delays(i,1)+(j-15)),10)
				+resize(interp_data(2,beam_delays(i,2)+(j-15)),10)
				+resize(interp_data(3,beam_delays(i,3)+(j-15)),10);
				
			if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
				--saturate low and high for 7 bit LUT
				if(to_integer(phased_beam_waves_buff(i,j))>63) then
					phased_beam_waves(i,j)<=b"0111111";--saturate max
				elsif(to_integer(phased_beam_waves_buff(i,j))<-63) then
				  phased_beam_waves(i,j)<=b"1000000"; --saturate min
				else
					phased_beam_waves(i,j)<=resize(phased_beam_waves_buff(i,j),7); 
				end if;	
			end if;
		end loop;
		
		for j in step_size*interp_factor to phased_sum_length-1 loop
			if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
				phased_beam_waves(i,j)<=phased_beam_waves(i,j-step_size*interp_factor);
			end if;
		end loop;
	end loop;
	
end process;


--calculate the power
--this just uses a LUT in logic to find the power from a signed value. If it synthesizes as BRAM is would be too slow. should be disributed memory
DO_POWER_BEAM : for i in 0 to num_beams-1 generate
	DO_POWER_SAMPLE : for j in 0 to step_size*interp_factor-1 generate --for j in 0 to phased_sum_length-1 generate
		xPOWERLUT : power_lut_7
		port map(
		clk_i => clk_data_i,
		a				=> phased_beam_waves(i,j),
		z				=> phased_power(i,j));
	end generate;
end generate;


--instead of recalculating the power for samples 16-31, just copy the previous 0-15 here
proc_move_power:process(clk_data_i,internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) and internal_phased_trig_en='1' then
		for i in 0 to num_beams-1 loop --loop over beams
			for j in 0 to step_size*interp_factor-1 loop --for j in 16 to phased_sum_length-1 loop
				phased_power(i,j+step_size*interp_factor)<=phased_power(i,j);
			end loop;
		end loop;
	end if;
end process;
		
		
--keep for posterity for how the arithmatic looks
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


--block to do the power integration
proc_avg_beam_power : process(clk_data_i,internal_phased_trig_en)
begin		

	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		for i in 0 to num_beams-1 loop
				
			--manually type all these... not sure how to make is a loop.
			--also split in half for timing...
			power_sum_0(i)<=resize(phased_power(i,0),num_power_bits)+resize(phased_power(i,1),num_power_bits)+resize(phased_power(i,2),num_power_bits)
				+resize(phased_power(i,3),num_power_bits)+resize(phased_power(i,4),num_power_bits)+resize(phased_power(i,5),num_power_bits)
				+resize(phased_power(i,6),num_power_bits)+resize(phased_power(i,7),num_power_bits);
			power_sum_1(i)<=resize(phased_power(i,8),num_power_bits)
				+resize(phased_power(i,9),num_power_bits)+resize(phased_power(i,10),num_power_bits)+resize(phased_power(i,11),num_power_bits)
				+resize(phased_power(i,12),num_power_bits)+resize(phased_power(i,13),num_power_bits)+resize(phased_power(i,14),num_power_bits)
				+resize(phased_power(i,15),num_power_bits);
				
			--one clock cycle calculates samples 0-num_step*interp_factor-1 (or 0-15), so shift these along
			power_sum_2(i)<=power_sum_0(i); 
			power_sum_3(i)<=power_sum_1(i); 
			power_sum_4(i)<=power_sum_2(i);

			
         --add together all smaller sums
			power_sum(i)<=power_sum_0(i)+power_sum_1(i)+power_sum_2(i)+power_sum_3(i);
			power_sum_overlap(i)<=power_sum_1(i)+power_sum_2(i)+power_sum_3(i)+power_sum_4(i);
			

		   --get the average power (bit selecting above bit 5, (divide by 32))		
			avg_power(i)(power_sum_bits-1 downto power_sum_bits-num_div)<=unsigned(pad_zeros);
			avg_power(i)(power_sum_bits-1-num_div downto 0)<=power_sum(i)(power_sum_bits-1 downto num_div); --divide by window size
			
			avg_power_overlap(i)(power_sum_bits-1 downto power_sum_bits-num_div)<=unsigned(pad_zeros);
			avg_power_overlap(i)(power_sum_bits-1-num_div downto 0)<=power_sum_overlap(i)(power_sum_bits-1 downto num_div); --divide by window size
			
		end loop;
	end if;
end process;


--compare calculated powers and compare to masks and thresholds for the actual trigger
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
			if avg_power(i)>trig_beam_thresh(i) or avg_power_overlap(i)>trig_beam_thresh(i) then
				triggering_beam(i)<='1';
				beam_trigger_reg(i)(0)<='1';
				latched_power_out(i)<=avg_power(i);
			else
				triggering_beam(i)<='0';
				beam_trigger_reg(i)(0)<='0';
			end if;
			
			beam_trigger_reg(i)(1)<=beam_trigger_reg(i)(0);
			if avg_power(i)>servo_beam_thresh(i) or avg_power_overlap(i)>servo_beam_thresh(i) then
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
			phased_trig_metadata<=triggering_beam AND internal_trigger_beam_mask;
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

-------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------

--//sync some software commands from the slow reg clock to the data clock

--enable for the entire trigger block to start running (consuming power)
xTRIGENABLESYNC : signal_sync --phased trig enable bit
	port map(
	clkA				=> clk_i,
	clkB				=> clk_data_i,
	SignalIn_clkA	=> registers_i(to_integer(unsigned(trigger_enable_reg_adr)))(9), --overall phased trig enable bit
	SignalOut_clkB	=> internal_phased_trig_en);
	
	
--sync the trigger thresholds to clk_data_i from slow reg clock
TRIG_THRESHOLDS : for bm in 0 to num_beams-1 generate
	INDIV_TRIG_BITS : for i in 0 to input_power_thesh_bits-1 generate
		xTRIGTHRESHSYNC : signal_sync
		port map(
		clkA				=> clk_i,
		clkB				=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_param_reg))+bm)(i), --threshold from software
		SignalOut_clkB	=> input_trig_thresh(bm)(i));
	end generate;
end generate;


--sync the servo thresholds to clk_data_i from slow reg clock
SERVO_THRESHOLDS : for bm in 0 to num_beams-1 generate
	INDIV_SERVO_BITS : for i in 0 to input_power_thesh_bits-1 generate
		xSERVOTHRESHSYNC : signal_sync
		port map(
		clkA				=> clk_i,
		clkB				=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_param_reg))+bm)(i+12), --threshold from software
		SignalOut_clkB	=> input_servo_thresh(bm)(i));
	end generate;
end generate;


--sync the threhsold offset (like a prescaler) to clk_data_i from slow reg clock
THRESH_OFFSET: for i in 0 to 11 generate
	xTHRESHOFFSETSYNC : signal_sync
		port map(
		clkA	=> clk_i,   clkB	=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_reg_base))+1)(i), --phased threshold offset
		SignalOut_clkB	=> threshold_offset(i));
end generate;


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


--specific delays for channels/stations didn't work. leaving in case someone tries
--SPECDELAYS : for bm in 0 to num_beams-1 generate
--	SPECDELAYSCHANNELS: for ch in 0 to 3 generate
--		SPECDELAYSBITS : for b in 0 to 3 generate
--			xSPECDELAYS : signal_sync
--			port map(
--			clkA				=> clk_i,
--			clkB				=> clk_data_i,
--			SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_param_reg)+12)+bm)(b+4*ch), --threshold from software
--			SignalOut_clkB	=> spec_delays(bm,ch)(b));
--		end generate;
--	end generate;
--end generate;


--sync the trigger beam mask to clk_data_i from slow reg clock
TRIGBEAMMASK : for bm in 0 to num_beams-1 generate --beam masks. 1 == on
	xTRIGBEAMMASKSYNC : signal_sync
	port map(
	clkA	=> clk_i,   clkB	=> clk_data_i,
	SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_reg_base)))(bm), --trig channel mask
	SignalOut_clkB	=> internal_trigger_beam_mask(bm));
end generate;


-------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------

--send/sync things from the fast clk_data_i clock to the slow reg clock

----ASYNCTRIGGER OUT!!
phased_trig_o <= phased_trigger;-- phased trigger for 0->1 transition. phased_trigger_reg(0) for any trigger condition


--phased trigger scaler
trigscaler: flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> phased_trigger,
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(0));

		
--beam trigger scalers
TrigToScalers	:	 for bm in 0 to num_beams-1 generate 
	xTRIGSYNC : flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> triggering_beam(bm),-- and internal_trigger_beam_mask(i),
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(bm+1));
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
ServoToScalers	:	 for bm in 0 to num_beams-1 generate 
	xSERVOSYNC : flag_sync
	port map(
		clkA 			=> clk_data_i,
		clkB			=> clk_i,
		in_clkA		=> servoing_beam(bm),-- and internal_trigger_beam_mask(i),
		busy_clkA	=> open,
		out_clkB		=> trig_bits_o(bm+num_beams+2));
end generate ServoToScalers;


--only needed if running at a clock different than 1/4 fs
--meta_bits	:	 for i in 0 to num_beams-1 generate 
--	xPHASEDMETA : flag_sync
--	port map(
--		clkA 			=> clk_data_i,
--		clkB			=> clk_data_i, -- to 1/4 fs
--		in_clkA		=> phased_trig_metadata(i),
--		busy_clkA	=> open,
--		out_clkB		=> phased_trig_metadata_o(i));
--end generate meta_bits;

end rtl;