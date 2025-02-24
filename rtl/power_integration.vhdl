library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.log2;
use work.defs.all;

entity power_integration is
    generic(
            ENABLE_PHASED_TRIG : std_logic := '1'
            );
    
    port(
            rst_i			:	in		std_logic;
            clk_data_i	:	in		std_logic; --data clock
            enable : in std_logic;
            beam_data_i : in std_logic_vector(num_beams*8*step_size*interp_factor-1 downto 0); --bms*samples*size
            power_o : out std_logic_vector(18*2*num_beams-1 downto 0) --beams*2_pows*size
    
            );
    end power_integration;
    
architecture rtl of power_integration is

    constant phased_sum_bits: integer := 8; --8. trying 7 bit lut
    constant phased_sum_length: integer := 32; --8 real samples ... not sure if it should be 8 or 16. longer windows smooths things. shorter window gives higher peak
    constant phased_sum_power_bits: integer := 16;--16 with calc. trying 7-> 14 lut
    constant num_power_bits: integer := 18;
    constant power_sum_bits:	integer := 18; --actually 25 but this fits into the io regs
    constant input_power_thesh_bits:	integer := 12;
    constant power_length: integer := 12;
    constant num_div: integer := 5;--can be calculated using -> integer(log2(real(phased_sum_length)));
    constant pad_zeros: std_logic_vector(num_div-1 downto 0):=(others=>'0');

    type phased_arr is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(phased_sum_bits-1 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
    signal phased_beam_waves: phased_arr:= (others=>(others=>(others=>'0')));
    
    --instantaneous power
    type square_waveform is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of unsigned(phased_sum_power_bits-1 downto 0);-- range 0 to 2**phased_sum_power_bits-1;--std_logic_vector(phased_sum_power_bits-1 downto 0);
    signal phased_power : square_waveform:= (others=>(others=>(others=>'0')));
    
    --big arrays for thresholds/ average power
    type power_array is array (num_beams-1 downto 0) of unsigned(num_power_bits-1 downto 0);-- range 0 to 2**num_power_bits-1;--std_logic_vector(num_power_bits-1 downto 0); --log2(6*(16*6)^2) max power possible
    signal trig_beam_thresh : power_array:=(others=>(others=>'0')) ; --trigger thresholds for all beams
    signal servo_beam_thresh : power_array:=(others=>(others=>'0')) ;--(others=>(others=>'0')) --servo thresholds for all beams
    signal power_sum : power_array:=(others=>(others=>'0')); --power integration using all 32 samples
    signal power_sum_overlap : power_array:=(others=>(others=>'0')); --power integration using all 32 samples
    signal power_sum_0 : power_array:=(others=>(others=>'0')); --partial power integration (lower 8 samples)
    signal power_sum_1 : power_array:=(others=>(others=>'0')); --partial power integration (upper 8 samples)
    signal power_sum_2 : power_array:=(others=>(others=>'0')); --partial power integration (last clock's power_sum_0)
    signal power_sum_3 : power_array:=(others=>(others=>'0')); --partial power integration (last clock's power_sum_1)
    signal power_sum_4 : power_array:=(others=>(others=>'0')); --partial power integration (last clock's power_sum_2)
    
    signal avg_power: power_array:=(others=>(others=>'0')); --average power (power_sum shifted down by log2(32)=5 bits)
    signal avg_power_overlap: power_array:=(others=>(others=>'0')); --average power (power_sum shifted down by log2(32)=5 bits)

    component power_lut_8 is --7 bit lut for calculating power
    port(
            clk_i    : in std_logic;
            a			: in	std_logic_vector(7 downto 0);
            z			: out	unsigned(15 downto 0));
    end component;
begin


assign_beam_i: for bm in 0 to num_beams-1 generate
	assign_sams_i: for sam in 0 to 15 generate
		phased_beam_waves(bm,sam)<= signed(beam_data_i(bm*16*8+8*(sam+1)-1 downto bm*16*8+8*sam));
	end generate;
end generate;

assing_power_o: for bm in 0 to num_beams-1 generate
	power_o(2*18*(bm+1)-18-1 downto 2*18*bm)<=std_logic_vector(avg_power(bm));
	power_o(2*18*(bm+1)-1 downto 2*18*bm+18)<=std_logic_vector(avg_power_overlap(bm));
end generate;


--calculate the power
--this just uses a LUT in logic to find the power from a signed value. 8 bits synth as bram, 7 bits as norm luts

DO_POWER_BEAM : for i in 0 to num_beams-1 generate
	DO_POWER_SAMPLE : for j in 0 to step_size*interp_factor-1 generate --for j in 0 to phased_sum_length-1 generate
		xPOWERLUT : power_lut_8
		port map(
		clk_i => clk_data_i,
		a				=> std_logic_vector(phased_beam_waves(i,j)),
		z				=> phased_power(i,j));
	end generate;
end generate;


--instead of recalculating the power for samples 16-31, just copy the previous 0-15 here
proc_move_power:process(clk_data_i,enable)
begin
	if rising_edge(clk_data_i) and enable='1' then
		--sample_o<=std_logic_vector(phased_beam_waves(0,0));

		for i in 0 to num_beams-1 loop --loop over beams
			for j in 0 to step_size*interp_factor-1 loop --for j in 16 to phased_sum_length-1 loop
				--phased_power(i,j)<=unsigned(phased_beam_waves(i,j)*phased_beam_waves(i,j));
				--phased_power(i,j+step_size*interp_factor)<=phased_power(i,j);
			end loop;
		end loop;

	end if;
end process;


--block to do the power integration
proc_avg_beam_power : process(clk_data_i,enable)
begin		

	if rising_edge(clk_data_i) and (enable='1') then
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
			power_sum(i)<=power_sum_0(i)+power_sum_1(i)+power_sum_2(i);--+power_sum_3(i);
			power_sum_overlap(i)<=power_sum_1(i)+power_sum_2(i)+power_sum_3(i);--+power_sum_4(i);
			
			--round
			if (power_sum(i)(4 downto 0))>=x"10" then
				avg_power(i)<=resize(unsigned(power_sum(i)(num_power_bits-1 downto 5)),num_power_bits)+1;
			else
				avg_power(i)<=resize(unsigned(power_sum(i)(num_power_bits-1 downto 5)),num_power_bits);
			end if;
			
			if (power_sum_overlap(i)(4 downto 0))>=x"10" then
				avg_power_overlap(i)<=resize(unsigned(power_sum_overlap(i)(num_power_bits-1 downto 5)),num_power_bits)+1;
			else
				avg_power_overlap(i)<=resize(unsigned(power_sum_overlap(i)(num_power_bits-1 downto 5)),num_power_bits);
			end if;
				
		   --get the average power (bit selecting above bit 5, (divide by 32))		
			--avg_power(i)(power_sum_bits-1 downto power_sum_bits-num_div)<=unsigned(pad_zeros);
			--avg_power(i)(power_sum_bits-1-num_div downto 0)<=power_sum(i)(power_sum_bits-1 downto num_div); --divide by window size
			
			--avg_power_overlap(i)(power_sum_bits-1 downto power_sum_bits-num_div)<=unsigned(pad_zeros);
			--avg_power_overlap(i)(power_sum_bits-1-num_div downto 0)<=power_sum_overlap(i)(power_sum_bits-1 downto num_div); --divide by window size
			
		end loop;	
	end if;
end process;


end rtl;