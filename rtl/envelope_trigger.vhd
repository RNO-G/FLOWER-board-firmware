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

entity envelope_trigger is
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
		phased_trig_metadata_o: out std_logic_vector(num_beams-1 downto 0) --for triggering beams
		);
end envelope_trigger;

architecture rtl of envelope_trigger is

--definitions + constants -- I realize I can now just use 'length too
constant streaming_buffer_length: integer := 24;
constant interp_factor: integer := 1;
constant interp_data_length: integer := 50*interp_factor;--40;--longer by 3 clocks cycles to hold samples until imaj becomes available
constant sample_bit_length: integer:=8;
constant baseline: unsigned(7 downto 0) := x"80";
constant phased_sum_bits: integer := 7; --8. trying 7 bit lut
constant phased_sum_length: integer := 8; --8 real samples ... not sure if it should be 8 or 16. longer windows smooths things. shorter window gives higher peak
constant phased_sum_power_bits: integer := 14;--16 with calc. trying 7-> 14 lut
constant step_size: integer:=4;
constant input_thesh_bits:	integer := 12;
constant num_channels:integer:=4;
constant input_thresh_bits:integer:=8;
constant delay_offset:integer:=4*interp_factor-1;
constant real_im_sync_delay:integer:=3*step_size+3;--15;


type antenna_delays is array (num_beams-1 downto 0,num_channels-1 downto 0) of integer range 0 to 127;
--9 beams at 4x upsampling
--constant beam_delays:antenna_delays:=((15,17,16,18),(18,18,15,15),(25,22,18,15),(33,28,21,15),(42,33,23,15),(50,39,26,15),(58,45,29,15),(65,49,32,15),(71,53,33,15));

--12 beams at 2x upsampling
--constant beam_delays:antenna_delays:=((7,8,8,8),(8,8,7,7),(10,10,8,7),(13,11,9,7),(16,13,10,7),(19,15,11,7),(22,17,12,7),(25,19,13,7),(28,21,14,7),(31,23,15,7),(33,25,15,7),(35,26,16,7));

--12 beams 1x
constant beam_delays:antenna_delays:=((3,3,3,3),(4,4,3,3),(5,5,4,3),(6,6,4,3),
													(8,6,5,3),(9,7,5,3),(10,8,5,3),(12,9,6,3),
													(13,10,6,3),(14,11,7,3),(15,12,7,3),(17,12,7,3));

--short streaming buffer
type streaming_data_array is array(3 downto 0, streaming_buffer_length-1 downto 0) of signed(7 downto 0);
signal streaming_data : streaming_data_array := (others=>(others=>(others=>'0'))); --pipeline data

constant upsample_filter_length: integer:=23 ;
type upsample_coeffs_t is array (upsample_filter_length-1 downto 0) of integer range -127 to 127;
constant upsample_coeffs: upsample_coeffs_t:=(-1,0,2,-0,-3,0,6,-0,-12,0,40,64,40,0,-12,-0,6,0,-3,-0,2,0,-1);


--temp buffers to assign input to and output from the upsampling filter
signal upsampling_input_data: std_logic_vector(num_channels*step_size*sample_bit_length-1 downto 0):=(others=>'0');
signal upsampling_output_data: std_logic_vector(num_channels*interp_factor*step_size*sample_bit_length-1 downto 0):=(others=>'0');

--temp buffers to assign input to and output from the hilbert transformer
constant hilbert_filter_length: integer:= 15;
type filt_coeffs is array (hilbert_filter_length-1 downto 0) of integer range -127 to 127;
constant hilbert_coeffs: filt_coeffs:=(12,0,16,0,27,0,81,0,-81,0,-27,0,-16,0,-12);

type temp_hilbert is array (3 downto 0, step_size*interp_factor-1 downto 0, 14 downto 0) of signed(15 downto 0);
signal temp_hilbert_vals: temp_hilbert;

type padded_t is array(3 downto 0, step_size*interp_factor+upsample_filter_length-1 downto 0) of signed(sample_bit_length-1 downto 0);
signal padded_sig: padded_t:=(others=>(others=>x"00"));

--buffers to store the interpolated samples for being pulled when doing the beamforming
type interpolated_data_array is array(3 downto 0, interp_data_length-1 downto 0) of signed(sample_bit_length-1 downto 0);
signal real_analytic: interpolated_data_array:=(others=>(others=>x"00"));
signal imaginary_analytic: interpolated_data_array:=(others=>(others=>x"00"));

type fir_temp is array(3 downto 0, step_size*interp_factor-1 downto 0) of signed(15 downto 0);
signal int_hilbert: fir_temp:=(others=>(others=>x"0000"));
signal int_hilbert0: fir_temp:=(others=>(others=>x"0000"));
signal int_hilbert1: fir_temp:=(others=>(others=>x"0000"));
signal int_hilbert2: fir_temp:=(others=>(others=>x"0000"));
signal int_hilbert3: fir_temp:=(others=>(others=>x"0000"));


signal int_up: fir_temp:=(others=>(others=>x"0000"));
signal int_up0: fir_temp:=(others=>(others=>x"0000"));
signal int_up1: fir_temp:=(others=>(others=>x"0000"));
signal int_up2: fir_temp:=(others=>(others=>x"0000"));
signal int_up3: fir_temp:=(others=>(others=>x"0000"));
signal int_up4: fir_temp:=(others=>(others=>x"0000"));
signal int_up5: fir_temp:=(others=>(others=>x"0000"));
signal int_up6: fir_temp:=(others=>(others=>x"0000"));
signal int_up7: fir_temp:=(others=>(others=>x"0000"));

--temp wire to calculate coherent sum waveforms and check for saturation
type phased_arr_buff is array (num_beams-1 downto 0,step_size*interp_factor-1 downto 0) of unsigned(9 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_real_wire: phased_arr_buff;
signal phased_imaginary_wire: phased_arr_buff;

--7 bit limited coherent sum
type phased_arr is array (num_beams-1 downto 0, step_size*interp_factor-1 downto 0) of unsigned(6 downto 0);
signal phased_real_waves: phased_arr:=(others=>(others=>b"0000000"));
signal phased_imaginary_waves: phased_arr:=(others=>(others=>b"0000000"));
signal alpha_temp: phased_arr:=(others=>(others=>b"0000000"));
signal alpha_temp_temp: phased_arr:=(others=>(others=>b"0000000"));


type temp_beta is array (num_beams-1 downto 0, step_size*interp_factor-1 downto 0) of unsigned(8 downto 0);
signal beta_temp:temp_beta:=(others=>(others=>b"000000000"));
signal beta_temp_temp:temp_beta:=(others=>(others=>b"000000000"));

--8 bit envelope
type envelope_arr is array (num_beams-1 downto 0,step_size*interp_factor-1 downto 0) of unsigned(7 downto 0);-- range 0 to 2**phased_sum_power_bits-1;--std_logic_vector(phased_sum_power_bits-1 downto 0);
signal envelope_signal : envelope_arr:=(others=>(others=>x"00"));

--input thresholds, 12 bits from registers... max of 8 bits unless decimal point shifts for fractional accuracy
type thresh_input is array (num_beams-1 downto 0) of unsigned(input_thesh_bits-1 downto 0);
signal input_trig_thresh : thresh_input;
signal input_servo_thresh : thresh_input;

--internal thresholds to compare envelope to
type thresholds is array (num_beams-1 downto 0) of unsigned(7 downto 0);
signal trig_beam_thresh: thresholds;
signal servo_beam_thresh:thresholds;

--which beams are currently over threshold
signal triggering_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');
signal servoing_beam: std_logic_vector(num_beams-1 downto 0):=(others=>'0');

--which beam and which samples are over threshold
type trig_thing is array (num_beams-1 downto 0) of std_logic_vector(step_size*interp_factor-1 downto 0);
signal triggering_window:trig_thing;
signal servoing_window:trig_thing;

--actual output from the phased trigger and servo. holds last trigger state in case want low-high transition for triggers
signal phased_trigger : std_logic;
signal phased_trigger_reg : std_logic_vector(1 downto 0);
signal phased_servo : std_logic;
signal phased_servo_reg : std_logic_vector(1 downto 0);

--copy of simple trigger channel regs to hold previous beam trigger/servo states
type trigger_regs is array(num_beams-1 downto 0) of std_logic_vector(1 downto 0);
signal beam_trigger_reg : trigger_regs;
signal beam_servo_reg : trigger_regs;

--probably not needed
type trigger_counter is array (num_beams-1 downto 0) of unsigned(15 downto 0);
signal trig_clear				: std_logic_vector(num_beams-1 downto 0);
signal servo_clear			: std_logic_vector(num_beams-1 downto 0);
signal trig_counter			: trigger_counter:= (others=>(others=>'0'));
signal servo_counter			: trigger_counter:= (others=>(others=>'0'));

--previous trig beam bits for data manager output
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
signal phased_trig_metadata: std_logic_vector(num_beams-1 downto 0); --for triggering beams

-------------------------------------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------------------------------------
--components, modules, etc

--module for signal clock domain crossing
component signal_sync is
port(
		clkA			: in	std_logic;
		clkB			: in	std_logic;
		SignalIn_clkA	: in	std_logic;
		SignalOut_clkB	: out	std_logic);
end component;

--module for flag clock domain crossing
component flag_sync is
port(
	clkA			: in	std_logic;
   clkB			: in	std_logic;
   in_clkA		: in	std_logic;
   busy_clkA	: out	std_logic;
   out_clkB		: out	std_logic);
end component;

--module to do multiplication on logic cells only
component fabric_mult is 
port(
	dataa:in signed(7 downto 0);
	datab:in signed(7 downto 0);
	result:out signed(15 downto 0)
	);
end component;

/*
--2x upsampling fir filter
component upsampling_2x is
port(
		clk: std_logic;
		reset_n: in std_logic;
		ast_sink_data: in std_logic_vector(upsampling_input_data'length-1 downto 0);
		ast_sink_valid:in std_logic;
		ast_sink_error:in std_logic_vector(1 downto 0);
		ast_source_data: out std_logic_vector(upsampling_output_data'length-1 downto 0);
		ast_source_valid: out std_logic;
		ast_source_error: out std_logic_vector(1 downto 0)
		);
end component;
*/



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
				streaming_data(0,i)<=signed(unsigned(ch0_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
				streaming_data(1,i)<=signed(unsigned(ch1_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
				streaming_data(2,i)<=signed(unsigned(ch2_data_i(8*(i+1)-1 downto 8*(i)))-baseline);
				streaming_data(3,i)<=signed(unsigned(ch3_data_i(8*(i+1)-1 downto 8*(i)))-baseline);

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

/*
--generate FIR filter
xUpsampling:upsampling_2x
port map(
		clk=>clk_data_i,
		reset_n=>(not rst_i),
		ast_sink_data=>upsampling_input_data,
		ast_sink_valid=>internal_phased_trig_en,
		ast_sink_error=>b"00",
		ast_source_data=>upsampling_output_data,
		ast_source_valid=>open,
		ast_source_error=>open
		);
*/
--assign inputs and outputs to the FIR filter module by reassigning vector to signed ints
proc_process_fir_upsampling: process(streaming_data,upsampling_output_data,real_analytic, clk_data_i,rst_i,internal_phased_trig_en)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1')then --not sure if these go into regs in the fir filter, doesn't hurt too much in case
	--if (internal_phased_trig_en='1') then 		
		for ch in 0 to 3 loop
		
			--for sample in 0 to step_size-1 loop
			--	upsampling_input_data(ch*(step_size*8)+sample*8+8-1 downto ch*(step_size*8)+sample*8)<=std_logic_vector(streaming_data(ch,sample));
			--end loop;
			
			for up_sample in 0 to interp_factor*step_size-1 loop
				--real_analytic(ch,up_sample)<=signed(upsampling_output_data(ch*(interp_factor*step_size*sample_bit_length)+up_sample*sample_bit_length+sample_bit_length-1 downto ch*(interp_factor*step_size*sample_bit_length)+up_sample*sample_bit_length));
				real_analytic(ch,up_sample)<=streaming_data(ch,up_sample);
			end loop;
			
			for j in step_size*interp_factor to interp_data_length-1 loop
				real_analytic(ch,j)<=real_analytic(ch,j-step_size*interp_factor);
			end loop;
			
		end loop;
	end if;
end process;


/*
proc_upsample_by_hand:process(clk_data_i,rst_i,streaming_data, internal_phased_trig_en)
begin

	if rising_edge(clk_data_i) and (internal_phased_trig_en='1')then
		for  ch in 0 to 3 loop
			for sam in 0 to step_size*interp_factor-1 loop
			
				
				if (sam mod interp_factor) = 0 then
					padded_sig(ch,sam)<=streaming_data(ch,sam / interp_factor);
				else
					padded_sig(ch,sam)<=x"00";
				end if;

				int_up0(ch,sam)<=upsample_coeffs(0)*padded_sig(ch,0+sam)+upsample_coeffs(2)*padded_sig(ch,2+sam);
				int_up1(ch,sam)<=upsample_coeffs(4)*padded_sig(ch,4+sam)+upsample_coeffs(6)*padded_sig(ch,6+sam);
				int_up2(ch,sam)<=upsample_coeffs(8)*padded_sig(ch,8+sam)+upsample_coeffs(10)*padded_sig(ch,10+sam);
				int_up3(ch,sam)<=upsample_coeffs(11)*padded_sig(ch,11+sam)+upsample_coeffs(10)*padded_sig(ch,12+sam);
				int_up4(ch,sam)<=upsample_coeffs(8)*padded_sig(ch,14+sam)+upsample_coeffs(6)*padded_sig(ch,16+sam);
				int_up5(ch,sam)<=upsample_coeffs(4)*padded_sig(ch,18+sam)+upsample_coeffs(2)*padded_sig(ch,20+sam);
				int_up6(ch,sam)<=upsample_coeffs(0)*padded_sig(ch,22+sam);
				
				
				int_up(ch,sam)<=int_up0(ch,sam)+int_up1(ch,sam)+int_up2(ch,sam)+int_up3(ch,sam)+int_up4(ch,sam)+int_up5(ch,sam)+int_up6(ch,sam);

				--rounding
				if (int_up(ch,sam)(15)='0') and (unsigned(int_up(ch,sam)(5 downto 0))>=x"20") then
					real_analytic(ch,sam)<=resize(signed(int_up(ch,sam)(15 downto 6)),8)+1;
					
				elsif (int_up(ch,sam)(15)='0') and (unsigned(int_up(ch,sam)(5 downto 0))<x"20") then
					real_analytic(ch,sam)<=resize(signed(int_up(ch,sam)(15 downto 6)),8);
					
				elsif (int_up(ch,sam)(15)='1') and (unsigned(int_up(ch,sam)(5 downto 0))<=x"20") then
					real_analytic(ch,sam)<=resize(signed(int_up(ch,sam)(15 downto 6)),8);
					
				else --(int_hilbert(ch,sam)(15)='1') and (int_hilbert(ch,sam)(6 downto 0)>x"40") then
					real_analytic(ch,sam)<=resize(signed(int_up(ch,sam)(15 downto 6)),8)-1;
				end if;
				
			end loop;
			
			for j in step_size*interp_factor to interp_data_length-1 loop
				real_analytic(ch,j)<=real_analytic(ch,j-step_size*interp_factor);
			end loop;
			
			for j in step_size*interp_factor to step_size*interp_factor+upsample_filter_length-1 loop
				padded_sig(ch,j)<=padded_sig(ch,j-8);
			end loop;
			
		end loop;

	end if;
end process;
*/
/*
--temp multiplier results for hilbert transformer
mult_ch:for ch in 0 to 3 generate
	mult_samples: for sample in 0 to step_size*interp_factor-1 generate
		mult_filter: for fil in 0 to hilbert_filter_length-1 generate
		
			xhilbertmult: fabric_mult port map(
				dataa=>real_analytic(ch,fil+sample),
				datab=>to_signed(hilbert_coeffs(fil),8),
				result=>temp_hilbert_vals(ch,sample,fil)
				);
		end generate;
	end generate;
end generate;
*/
--bc simulating the fir ip's is being a pain, I can't easily line up the real and imag parts...
--convolution in stages for timing
--let's just do it by hand since it's small. includes rounding	
proc_hilbert_by_hand:process(clk_data_i,rst_i,streaming_data, internal_phased_trig_en)
begin

	if rising_edge(clk_data_i) and (internal_phased_trig_en='1')then
		for  ch in 0 to 3 loop
			for sam in 0 to step_size*interp_factor-1 loop
			
				--these bits for mult on logic cells
				--int_hilbert0(ch,sam)<=temp_hilbert_vals(ch,sam,0)+temp_hilbert_vals(ch,sam,2);
				--int_hilbert1(ch,sam)<=temp_hilbert_vals(ch,sam,4)+temp_hilbert_vals(ch,sam,6);
				--int_hilbert2(ch,sam)<=temp_hilbert_vals(ch,sam,8)+temp_hilbert_vals(ch,sam,10);
				--int_hilbert3(ch,sam)<=temp_hilbert_vals(ch,sam,12)+temp_hilbert_vals(ch,sam,14);
				
				--this guy goes on multipliers by default
				int_hilbert0(ch,sam)<=hilbert_coeffs(0)*real_analytic(ch,0+sam)+hilbert_coeffs(2)*real_analytic(ch,2+sam);
				int_hilbert1(ch,sam)<=hilbert_coeffs(4)*real_analytic(ch,4+sam)+hilbert_coeffs(6)*real_analytic(ch,6+sam);
				int_hilbert2(ch,sam)<=hilbert_coeffs(8)*real_analytic(ch,8+sam)+hilbert_coeffs(10)*real_analytic(ch,10+sam);
				int_hilbert3(ch,sam)<=hilbert_coeffs(12)*real_analytic(ch,12+sam)+hilbert_coeffs(14)*real_analytic(ch,14+sam);
				
				int_hilbert(ch,sam)<=int_hilbert0(ch,sam)+int_hilbert1(ch,sam)+int_hilbert2(ch,sam)+int_hilbert3(ch,sam);

				--rounding
				if (int_hilbert(ch,sam)(15)='0') and (unsigned(int_hilbert(ch,sam)(6 downto 0))>=x"40") then
					imaginary_analytic(ch,sam)<=resize(signed(int_hilbert(ch,sam)(15 downto 7)),8)+1;
					
				elsif (int_hilbert(ch,sam)(15)='0') and (unsigned(int_hilbert(ch,sam)(6 downto 0))<x"40") then
					imaginary_analytic(ch,sam)<=resize(signed(int_hilbert(ch,sam)(15 downto 7)),8);
					
				elsif (int_hilbert(ch,sam)(15)='1') and (unsigned(int_hilbert(ch,sam)(6 downto 0))<=x"40") then
					imaginary_analytic(ch,sam)<=resize(signed(int_hilbert(ch,sam)(15 downto 7)),8);
					
				else --(int_hilbert(ch,sam)(15)='1') and (int_hilbert(ch,sam)(6 downto 0)>x"40") then
					imaginary_analytic(ch,sam)<=resize(signed(int_hilbert(ch,sam)(15 downto 7)),8)-1;
				end if;
				
			end loop;
			
			for j in step_size*interp_factor to interp_data_length-1 loop
				imaginary_analytic(ch,j)<=imaginary_analytic(ch,j-step_size*interp_factor);
			end loop;
			
		end loop;

	end if;
end process;


--THIS WORKS... could saturate to 8 bits instead
--do phasing to calculate the coherently summed waveforms of real and imag components of analytic signal
proc_phasing: process(clk_data_i,internal_phased_trig_en, real_analytic, imaginary_analytic)
begin
	
	for i in 0 to num_beams-1 loop --loop over beams
		for j in 0 to step_size*interp_factor-1 loop
		
			--assign the temporary as async, but then place it into a reg... cleaner(?) looking code
			phased_real_wire(i,j)<=unsigned(abs(resize(real_analytic(0,real_im_sync_delay+beam_delays(i,0)+(j-delay_offset)),10)
				+resize(real_analytic(1,real_im_sync_delay+beam_delays(i,1)+(j-delay_offset)),10)
				+resize(real_analytic(2,real_im_sync_delay+beam_delays(i,2)+(j-delay_offset)),10)
				+resize(real_analytic(3,real_im_sync_delay+beam_delays(i,3)+(j-delay_offset)),10))); --8? to sync real and imag from filter
				
			phased_imaginary_wire(i,j)<=unsigned(abs(resize(imaginary_analytic(0,beam_delays(i,0)+(j-delay_offset)),10)
				+resize(imaginary_analytic(1,beam_delays(i,1)+(j-delay_offset)),10)
				+resize(imaginary_analytic(2,beam_delays(i,2)+(j-delay_offset)),10)
				+resize(imaginary_analytic(3,beam_delays(i,3)+(j-delay_offset)),10)));
				
			--if the saturation is too small I will need to make this unsigned 7 bits instread of 6
			
			if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then 
			
				--saturate low and high for 7 bit LUT
				if(to_integer(phased_real_wire(i,j))>127) then
					phased_real_waves(i,j)<=b"1111111";--saturate max
				--elsif (to_integer(phased_real_wire(i,j))<-63) then
				--	phased_real_waves(i,j)<=b"1000000";--saturate max --if using cordic these need to be signed
				else
					phased_real_waves(i,j)<=resize(phased_real_wire(i,j),7); 
				end if;	

				if(to_integer(phased_imaginary_wire(i,j))>127) then
					phased_imaginary_waves(i,j)<=b"1111111";--saturate max
				--elsif (to_integer(phased_imaginary_wire(i,j))<-63) then
				--	phased_imaginary_waves(i,j)<=b"1000000";--saturate max  --if using cordic these need to be signed
				else
					phased_imaginary_waves(i,j)<=resize(phased_imaginary_wire(i,j),7); 
				end if;	
			end if;
			
		end loop;
	end loop;
end process;


--THIS WORKS
--calculate the envelope using alpha*max+beta*min algorithm (little less performant than cordic but (should) use less resources)
--most of this process might be able to be unclocked with the output being put into regs, but I can't make it work... clocked and pipelined it is
proc_envelope:process(clk_data_i)
begin
	if rising_edge(clk_data_i) and (internal_phased_trig_en='1') then
		for i in 0 to num_beams-1 loop
			for j in 0 to step_size*interp_factor-1 loop
			
				if phased_real_waves(i,j)>phased_imaginary_waves(i,j) then
				
					alpha_temp(i,j)<=phased_real_waves(i,j);
					beta_temp(i,j)<=resize(phased_imaginary_waves(i,j)*3,9);
									
				else
				
					alpha_temp(i,j)<=phased_imaginary_waves(i,j);
					beta_temp(i,j)<=resize(phased_real_waves(i,j)*3,9);
			
				end if;
				
				alpha_temp_temp(i,j)<=alpha_temp(i,j);
				
				if beta_temp(i,j)(2 downto 0)>b"100" then
					beta_temp_temp(i,j)<=resize(unsigned(beta_temp(i,j)(8 downto 3))+4,9);
				else
					beta_temp_temp(i,j)<=resize(beta_temp(i,j)(8 downto 3),9);
				end if;
			
				envelope_signal(i,j)<=resize(alpha_temp_temp(i,j)+beta_temp_temp(i,j),8);
					
			end loop;
		end loop;
	end if;
end process;

--cordic magnitude algorithm
--mag_beams:for beam in 0 to num_beams-1 generate
--	mag_samples: for sample in 0 to 7 generate
--		xMags: work.magnitude port map(
--			clk=> clk_data_i,
--			x_in=> std_logic_vector(phased_real_waves(beam,sample)),
--			y_in=> std_logic_vector(phased_imaginary_waves(beam,sample)),
--			x_out=> open,
--			y_out=> open,
--			magnitude_out => envelope_signal(beam,sample));
--	end generate;
--end generate;
	

--this just uses a LUT in logic to find the power from a signed value. \
--DO_ENVELOPE_BEAM : for i in 0 to num_beams-1 generate
--	DO_ENVELOPE_SAMPLE : for j in 0 to step_size*interp_factor-1 generate --for j in 0 to phased_sum_length-1 generate
--		xENVELOPE : envelope
--		port map(
--		clk_i => clk_data_i,
--		a				=> phased_real_waves(i,j),
--		b				=> phased_imaginary_waves(i,j),
--		z				=> envelope_signal(i,j));
--	end generate;
--end generate;


--then compare envelope to threshold for triggers and apply beam masks
--
proc_get_triggering_beams : process(clk_data_i,rst_i)
begin
	if rst_i = '1' then
		phased_trigger_reg <= "00";
		phased_trigger <= '0'; -- the trigger

		phased_servo_reg <= "00";
		phased_servo <= '0';  --the servo trigger

		triggering_beam<= (others=>'0');
		servoing_beam<= (others=>'0');
		
	elsif rising_edge(clk_data_i) and (internal_phased_trig_en='1')then
	
		--loop over the beams
		for i in 0 to num_beams-1 loop
		
			--loops over new samples per clock cycle
			for j in 0 to step_size*interp_factor-1 loop
			
				--check if any sample is over trigger threshold
				if envelope_signal(i,j)>trig_beam_thresh(i) then
					triggering_window(i)(j)<='1';
				else
					triggering_window(i)(j)<='0';
				end if;

				--check if any value of over servo threshold
				if envelope_signal(i,j)>servo_beam_thresh(i) then
					servoing_window(i)(j)<='1';
				else
					servoing_window(i)(j)<='0';
				end if;
			
			end loop;
			
			--reduce samples on each beam to a single bit
			if unsigned(triggering_window(i))>0 then
				triggering_beam(i)<='1';
				beam_trigger_reg(i)(0)<='1';
			else
				triggering_beam(i)<='0';
				beam_trigger_reg(i)(0)<='0';
			end if;
			
			beam_trigger_reg(i)(1)<=beam_trigger_reg(i)(0);
			
			if unsigned(servoing_window(i))>0 then
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
	INDIV_TRIG_BITS : for i in 0 to input_thesh_bits-1 generate
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
	INDIV_SERVO_BITS : for i in 0 to input_thesh_bits-1 generate
		xSERVOTHRESHSYNC : signal_sync
		port map(
		clkA				=> clk_i,
		clkB				=> clk_data_i,
		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_param_reg))+bm)(i+12), --threshold from software
		SignalOut_clkB	=> input_servo_thresh(bm)(i));
	end generate;
end generate;


--sync the threhsold offset (like a prescaler) to clk_data_i from slow reg clock
--THRESH_OFFSET: for i in 0 to 11 generate
--	xTHRESHOFFSETSYNC : signal_sync
--		port map(
--		clkA	=> clk_i,   clkB	=> clk_data_i,
--		SignalIn_clkA	=> registers_i(to_integer(unsigned(phased_trig_reg_base))+1)(i), --phased threshold offset
--		SignalOut_clkB	=> threshold_offset(i));
--end generate;


--process 12 bit input thresholds with some threshold offset (defined in software) if needed (likely not needed)
proc_threshold_set:process(clk_data_i)
begin
   if rising_edge(clk_data_i) then
		for i in 0 to num_beams-1 loop
			trig_beam_thresh(i)<=resize(input_trig_thresh(i),8);
			servo_beam_thresh(i)<=resize(input_servo_thresh(i),8);
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

end rtl;