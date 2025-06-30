library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.defs.all;

entity beamforming is
    generic(
            ENABLE_PHASED_TRIG : std_logic := '1';
            station_number_i : in std_logic_vector(7 downto 0)
            );
    
    port(
            rst_i : in std_logic;
            clk_data_i : in	std_logic; --data clock
            enable : in std_logic;
            ch_data_i : in std_logic_vector(num_channels*step_size*interp_factor*8-1 downto 0);
            beam_data_o : out std_logic_vector(num_beams*step_size*interp_factor*8-1 downto 0)
            --specific_dels: in specific_delays_t
            );
    end beamforming;
    
architecture rtl of beamforming is

constant interp_data_length: integer := interp_factor*(20); -- atleast 16 larger than highest delay
constant sample_bit_length: integer:=8;
constant baseline: unsigned(7 downto 0) := x"80";
constant phased_sum_bits: integer := 8; --8. trying 7 bit lut
constant phased_sum_length: integer := 16;
constant num_stations: integer:=8;

--buffer to store the interpolated sample for being pulled when doing the beamforming / summation
type interpolated_data_array is array(3 downto 0, interp_data_length-1 downto 0) of signed(sample_bit_length-1 downto 0);
signal interp_data: interpolated_data_array:= (others=>(others=>(others=>'0')));

--temp buffer to calculate coherent sum waveforms to check for saturation
type phased_arr_buff is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(9 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves_buff: phased_arr_buff:= (others=>(others=>(others=>'0')));

--7 bit limited coherent sum for power LUT
type phased_arr is array (num_beams-1 downto 0,phased_sum_length-1 downto 0) of signed(phased_sum_bits-1 downto 0);-- range 0 to 2**phased_sum_bits-1; --phased sum... log2(16*8)=7bits
signal phased_beam_waves: phased_arr:= (others=>(others=>(others=>'0')));

type antenna_delays is array (num_stations-1 downto 0, num_beams-1 downto 0,num_channels-1 downto 0) of integer range 0 to 127;
--12 beams equally spaced between -60 and 60 generated with make_beams.py


function convert_station_to_index(number:std_logic_vector)
    return integer is
    begin
        if number = x"0b" then return 0;
        elsif number = x"0c" then return 1;
        elsif number = x"0d" then return 2;
        elsif number = x"0e" then return 3;
        elsif number = x"15" then return 4;
        elsif number = x"16" then return 5;
        elsif number = x"17" then return 6;
        elsif number = x"18" then return 7;
        else return -1;
        end if;
    end function;

constant station_index: integer :=convert_station_to_index(station_number_i);
--station indexed in this order = [24 (7 ind), 23, 22, 21, 14, 13, 12, 11 (0 ind)]
--beams 11 to 0 w/ beam 11 pointing down, and 0 pointing up


/*
--up to v16
--using rno_season_2024.json but stations are ordered backwards (11,12,13,14,21,22,23,24) here
constant beam_delays:antenna_delays :=
    (((0,2,1,3),(3,3,0,0),(8,6,2,0),(13,10,4,0),(19,13,6,0),(24,17,8,0),(29,20,9,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,17,0),(56,38,18,0)),
    ((0,2,3,1),(4,4,4,0),(9,7,6,0),(14,11,7,0),(19,14,9,0),(24,17,11,0),(29,21,12,0),(35,24,14,0),(40,28,16,0),(45,31,17,0),(50,34,19,0),(55,38,21,0)),
    ((0,2,1,2),(3,3,1,0),(8,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,18,0)),
    ((0,0,0,2),(4,2,0,0),(9,6,2,0),(14,9,4,0),(20,13,5,0),(25,16,7,0),(30,20,9,0),(36,23,11,0),(41,27,13,0),(46,30,14,0),(51,34,16,0),(57,37,18,0)),
    ((0,2,1,2),(3,3,1,0),(8,7,3,0),(14,10,4,0),(19,14,6,0),(24,17,8,0),(29,20,9,0),(34,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,16,0),(55,38,18,0)),
    ((0,1,1,2),(4,3,1,0),(9,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,18,0)),
    ((0,1,1,2),(4,3,1,0),(9,7,3,0),(14,10,5,0),(19,14,6,0),(24,17,8,0),(29,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,16,0),(55,38,18,0)),
    ((0,1,1,2),(3,3,1,0),(9,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,18,0)));
*/

/*
--v0p17
-- using rno_season_2024.json but keep the right station ordering
constant beam_delays:antenna_delays :=
    (((0,1,1,2),(3,3,1,0),(9,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,18,0)),
    ((0,1,1,2),(4,3,1,0),(9,7,3,0),(14,10,5,0),(19,14,6,0),(24,17,8,0),(29,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,16,0),(55,38,18,0)),
    ((0,1,1,2),(4,3,1,0),(9,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,18,0)),
    ((0,2,1,2),(3,3,1,0),(8,7,3,0),(14,10,4,0),(19,14,6,0),(24,17,8,0),(29,20,9,0),(34,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,16,0),(55,38,18,0)),
    ((0,0,0,2),(4,2,0,0),(9,6,2,0),(14,9,4,0),(20,13,5,0),(25,16,7,0),(30,20,9,0),(36,23,11,0),(41,27,13,0),(46,30,14,0),(51,34,16,0),(57,37,18,0)),
    ((0,2,1,2),(3,3,1,0),(8,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,18,0)),
    ((0,2,3,1),(4,4,4,0),(9,7,6,0),(14,11,7,0),(19,14,9,0),(24,17,11,0),(29,21,12,0),(35,24,14,0),(40,28,16,0),(45,31,17,0),(50,34,19,0),(55,38,21,0)),
    ((0,2,1,3),(3,3,0,0),(8,6,2,0),(13,10,4,0),(19,13,6,0),(24,17,8,0),(29,20,9,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,17,0),(56,38,18,0)));
*/

--v0p18
--using db detector with group delays. the lab-measured signal chain keeps the cal pulser at most within 0.5ns when comparing to measured events
constant beam_delays: antenna_delays :=
    (((0,1,1,2),(3,2,1,0),(9,6,2,0),(14,9,4,0),(19,13,6,0),(24,16,8,0),(30,20,9,0),(35,23,11,0),(40,27,13,0),(45,30,15,0),(51,34,16,0),(56,37,18,0)),--station 24
    ((0,1,1,2),(4,3,1,0),(9,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(29,20,9,0),(34,24,11,0),(40,27,13,0),(45,30,14,0),(50,34,16,0),(55,37,18,0)),--station 23
    ((0,1,1,2),(3,3,1,0),(9,7,3,0),(14,10,5,0),(19,14,6,0),(24,17,8,0),(29,21,10,0),(35,24,12,0),(40,27,13,0),(45,31,15,0),(50,34,17,0),(56,38,19,0)),--station 22
    ((0,2,1,2),(3,3,1,0),(8,7,3,0),(13,10,4,0),(19,14,6,0),(24,17,8,0),(29,21,9,0),(34,24,11,0),(40,27,13,0),(45,31,15,0),(50,34,16,0),(55,38,18,0)),--station 21
    ((0,0,0,2),(4,2,0,0),(9,6,2,0),(14,9,4,0),(20,13,5,0),(25,16,7,0),(30,20,9,0),(36,23,11,0),(41,27,13,0),(46,30,14,0),(52,34,16,0),(57,37,18,0)),--station 14
    ((0,2,1,2),(3,3,1,0),(8,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(30,20,10,0),(35,24,11,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,19,0)),--station 13
    ((0,1,3,1),(4,4,3,0),(9,7,5,0),(14,11,7,0),(19,14,8,0),(24,17,10,0),(30,21,12,0),(35,24,13,0),(40,28,15,0),(45,31,17,0),(50,34,18,0),(55,38,20,0)),--station 12
    ((0,2,1,2),(3,3,1,0),(8,6,3,0),(14,10,4,0),(19,13,6,0),(24,17,8,0),(29,20,10,0),(35,24,12,0),(40,27,13,0),(45,31,15,0),(51,34,17,0),(56,38,19,0)));--station 11

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
        
            --async add then clock saturation. adjustable station specific delays - keeping in case
            --phased_beam_waves_buff(i,j)<=resize(interp_data(0,beam_delays(STATION_INDEX,i,0)+(j)+to_integer(specific_dels(i,0))),10)
            --+resize(interp_data(1,beam_delays(STATION_INDEX,i,1)+(j)+to_integer(specific_dels(i,1))),10)
            --+resize(interp_data(2,beam_delays(STATION_INDEX,i,2)+(j)+to_integer(specific_dels(i,2))),10)
            --+resize(interp_data(3,beam_delays(STATION_INDEX,i,3)+(j)+to_integer(specific_dels(i,3))),10);

            phased_beam_waves_buff(i,j)<=resize(interp_data(0,beam_delays(station_index,i,0)+(j)),10)
                                        +resize(interp_data(1,beam_delays(station_index,i,1)+(j)),10)
                                        +resize(interp_data(2,beam_delays(station_index,i,2)+(j)),10)
                                        +resize(interp_data(3,beam_delays(station_index,i,3)+(j)),10);

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
    end loop;
end process;

assign_beams_o: for bm in 0 to 11 generate
        assign_samples_o: for sam in 0 to 15 generate
                beam_data_o(bm*8*16+(sam+1)*8-1 downto bm*8*16+sam*8)<=std_logic_vector(phased_beam_waves(bm,sam));
        end generate;
end generate;

end rtl;