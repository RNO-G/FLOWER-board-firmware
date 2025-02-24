library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.log2;
use work.defs.all;

entity beamforming is
    generic(
            ENABLE_PHASED_TRIG : std_logic := '1'
            );
    
    port(
            rst_i			:	in		std_logic;
            clk_data_i	:	in		std_logic; --data clock
            enable : in std_logic;
            ch_data_i : in std_logic_vector(num_channels*step_size*interp_factor*8-1 downto 0);
            beam_data_o : out std_logic_vector(num_beams*step_size*interp_factor*8-1 downto 0)
            );
    end beamforming;
    
architecture rtl of beamforming is

constant interp_data_length: integer := interp_factor*(24)+1;--interp_factor*(streaming_buffer_length-1)+1;
constant sample_bit_length: integer:=8;
constant baseline: unsigned(7 downto 0) := x"80";
constant phased_sum_bits: integer := 8; --8. trying 7 bit lut
constant phased_sum_length: integer := 16;

--buffer to store the interpolated sample for being pulled when doing the beamforming / summation
type interpolated_data_array is array(3 downto 0, interp_data_length-1 downto 0) of signed(sample_bit_length-1 downto 0);
signal interp_data: interpolated_data_array:= (others=>(others=>(others=>'0')));

--temp buffer to calculate coherent sum waveforms to check for saturation
type phased_arr_buff is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(9 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves_buff: phased_arr_buff:= (others=>(others=>(others=>'0')));

--7 bit limited coherent sum for power LUT
type phased_arr is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(phased_sum_bits-1 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves: phased_arr:= (others=>(others=>(others=>'0')));

type antenna_delays is array (num_beams-1 downto 0,num_channels-1 downto 0) of integer range 0 to 127;
--12 beams equally spaced between -60 and 60 generated with make_beams.py
constant beam_delays:antenna_delays:=((0,2,1,3),(3,3,0,0),(8,6,2,0),(13,10,4,0),(19,13,6,0),(24,17,8,0),(29,20,9,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,17,0),(56,38,18,0));

begin


proc_pipeline_data: process(clk_data_i,enable)
begin


        if rising_edge(clk_data_i) and (enable='1') then
                for ch in 0 to 3 loop
                        for sam in 0 to step_size*interp_factor-1 loop
                                interp_data(ch,sam)<=signed(ch_data_i(ch*8*16+(sam+1)*8-1 downto ch*8*16+sam*8));
                        end loop;
                end loop;
                --shift reg data
                for ch in 0 to 3 loop
                        for sam in step_size*interp_factor to interp_data_length-1 loop
                                interp_data(ch,sam)<=interp_data(ch,sam-step_size*interp_factor);
                        end loop;
                end loop;
        end if;
end process;
        
--do phasing to calculate the coherently summed waveforms
proc_phasing: process(clk_data_i,enable)
begin
	
	for i in 0 to num_beams-1 loop --loop over beams
		for j in 0 to step_size*interp_factor-1 loop
		
			--assign the temporary as async, but then place it into a reg... cleaner looking code
                        phased_beam_waves_buff(i,j)<=resize(interp_data(0,beam_delays(i,0)+(j)),10)
                        +resize(interp_data(1,beam_delays(i,1)+(j)),10)
                        +resize(interp_data(2,beam_delays(i,2)+(j)),10)
                        +resize(interp_data(3,beam_delays(i,3)+(j)),10);
                        
			if rising_edge(clk_data_i) and (enable='1') then 
    
				--saturate low and high for 8 bit LUT. max=2^(8-1)-1, min=-2^(8-1)
				if((phased_beam_waves_buff(i,j))>127) then
					phased_beam_waves(i,j)<=b"01111111";--saturate max
				elsif((phased_beam_waves_buff(i,j))<-128) then
				        phased_beam_waves(i,j)<=b"10000000"; --saturate min
				else
				        phased_beam_waves(i,j)<=resize(phased_beam_waves_buff(i,j),8); 
				end if;	
			end if;
		end loop;
		
		for j in step_size*interp_factor to phased_sum_length-1 loop
			if rising_edge(clk_data_i) and (enable='1') then 
				phased_beam_waves(i,j)<=phased_beam_waves(i,j-step_size*interp_factor);
			end if;
		end loop;
	end loop;
end process;

assign_beams_o: for bm in 0 to 11 generate
        assign_samples_o: for sam in 0 to 15 generate
                beam_data_o(bm*8*16+(sam+1)*8-1 downto bm*8*16+sam*8)<=std_logic_vector(phased_beam_waves(bm,sam));
        end generate;
end generate;

end rtl;