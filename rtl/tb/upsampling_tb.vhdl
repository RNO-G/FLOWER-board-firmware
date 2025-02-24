library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
--use ieee.std_logic_textio.all;

use work.defs.all;
use work.register_map.all;
use work.all;

entity upsampling_tb is
end upsampling_tb;

architecture behave of upsampling_tb is
---------------------------------------------------------------------------
-- Declare the Component Under Test
-----------------------------------------------------------------------------
component upsampling is 
    port(
            rst_i			:	in		std_logic;
            clk_data_i	:	in		std_logic; --data clock
            enable : in std_logic;
            
            ch0_data_i	: 	in	std_logic_vector(8*step_size-1 downto 0); --4 samples per clock
            ch1_data_i	:	in	std_logic_vector(8*step_size-1 downto 0);
            ch2_data_i	:	in	std_logic_vector(8*step_size-1 downto 0);
            ch3_data_i	:	in	std_logic_vector(8*step_size-1 downto 0);
            
            ch0_data_o : out std_logic_vector(8*step_size*interp_factor-1 downto 0); --16 samples per clock (4x upsampling)
            ch1_data_o : out std_logic_vector(8*step_size*interp_factor-1 downto 0);
            ch2_data_o : out std_logic_vector(8*step_size*interp_factor-1 downto 0);
            ch3_data_o : out std_logic_vector(8*step_size*interp_factor-1 downto 0)
    
            );
    end component;
-----------------------------------------------------------------------------
-- Testbench Internal Signals
-----------------------------------------------------------------------------
signal  clock : std_logic := '1';

signal enable: std_logic:='1';
--type input_samples_t is unsigned(31 downto 0);
type output_samples_t is array(15 downto 0) of std_logic_vector(31 downto 0);

signal ch0_samples:std_logic_vector(31 downto 0):=x"80808080";
signal ch1_samples:std_logic_vector(31 downto 0):=x"80808080";
signal ch2_samples:std_logic_vector(31 downto 0):=x"80808080";
signal ch3_samples:std_logic_vector(31 downto 0):=x"80808080";

signal upsampling_ch0_i:std_logic_vector(31 downto 0):=(others=>'0');
signal upsampling_ch1_i:std_logic_vector(31 downto 0):=(others=>'0');
signal upsampling_ch2_i:std_logic_vector(31 downto 0):=(others=>'0');
signal upsampling_ch3_i:std_logic_vector(31 downto 0):=(others=>'0');

signal ch0_output:std_logic_vector(4*8*4-1 downto 0):=(others=>'0');
signal ch1_output:std_logic_vector(4*8*4-1 downto 0):=(others=>'0');
signal ch2_output:std_logic_vector(4*8*4-1 downto 0):=(others=>'0');
signal ch3_output:std_logic_vector(4*8*4-1 downto 0):=(others=>'0');


begin

    clock <= not clock after 4 ns;

    -----------------------------------------------------------------------------
    -- Instantiate and Map UUT
    -----------------------------------------------------------------------------
    xUpsampling : upsampling 
    port map (
        rst_i => '0',
        clk_data_i => clock,
        enable => '1',
        ch0_data_i => upsampling_ch0_i,
        ch1_data_i => upsampling_ch1_i,
        ch2_data_i => upsampling_ch2_i,
        ch3_data_i => upsampling_ch3_i,
        ch0_data_o => ch0_output,
        ch1_data_o => ch1_output,
        ch2_data_o => ch2_output,
        ch3_data_o => ch3_output
    );


    process



    variable ch0_samples_tmp:std_logic_vector(31 downto 0):=(others=>'0');
    variable ch1_samples_tmp:std_logic_vector(31 downto 0):=(others=>'0');
    variable ch2_samples_tmp:std_logic_vector(31 downto 0):=(others=>'0');
    variable ch3_samples_tmp:std_logic_vector(31 downto 0):=(others=>'0');

    variable trig_tmp: std_logic:='0';

    variable v_ILINE     : line;
    variable v_OLINE     : line;
    variable v_SPACE     : character;

    file file_INPUT : text;-- open read_mode is "input_waveforms.txt";
    file file_THRESHOLDS : text;-- open read_mode is "input_thresholds.txt";
    file file_OUTPUT : text;-- open write_mode is "output_waveforms.txt";
    file file_TRIGGERS : text;-- open write_mode is "output_trigger.txt";

        begin

            --io files
            file_open(file_INPUT, "input_waveforms.txt", read_mode);
            file_open(file_OUTPUT, "output_waveforms.txt", write_mode);

            --read in thresholds and assign to regs

            --read in samples in sets of 4
            while not endfile(file_INPUT) loop
                readline(file_INPUT, v_ILINE);
                read(v_ILINE, ch0_samples_tmp);
                read(v_ILINE, v_SPACE);
                read(v_ILINE, ch1_samples_tmp);
                read(v_ILINE, v_SPACE);
                read(v_ILINE, ch2_samples_tmp);
                read(v_ILINE, v_SPACE);
                read(v_ILINE, ch3_samples_tmp);

                --assign data
                ch0_samples<=ch0_samples_tmp;
                ch1_samples<=ch1_samples_tmp;
                ch2_samples<=ch2_samples_tmp;
                ch3_samples<=ch3_samples_tmp;

                for i in 0 to 3 loop
                    upsampling_ch0_i(8*(i+1)-1 downto 8*i)<=std_logic_vector(unsigned(ch0_samples(8*(i+1)-1 downto 8*i))-128);
                    upsampling_ch1_i(8*(i+1)-1 downto 8*i)<=std_logic_vector(unsigned(ch1_samples(8*(i+1)-1 downto 8*i))-128);
                    upsampling_ch2_i(8*(i+1)-1 downto 8*i)<=std_logic_vector(unsigned(ch2_samples(8*(i+1)-1 downto 8*i))-128);
                    upsampling_ch3_i(8*(i+1)-1 downto 8*i)<=std_logic_vector(unsigned(ch3_samples(8*(i+1)-1 downto 8*i))-128);
                end loop;

                wait for 8 ns; --about 1/118e6 ns, one full clock cycle

                write(v_OLINE,ch0_samples,right,32);
                writeline(output,v_OLINE);
                write(v_OLINE,ch0_output,right,32*4);
                writeline(output,v_OLINE);
                --write(v_OLINE,temp_sample,right,8);
                --writeline(output,v_OLINE);

                
                --write(v_OLINE,trig,right,1);
                --writeline(output,v_OLINE);

                --write(v_OLINE,is_enable,right,1);
                --writeline(output,v_OLINE);
                --write output trigger state


                --write upsampled waveforms
                for i in 0 to 15 loop
                    write(v_OLINE,unsigned(ch0_output(8*(i+1)-1 downto 8*i))+128,right,8);
                    write(v_OLINE, v_SPACE);
                end loop;

                for i in 0 to 15 loop
                    write(v_OLINE,unsigned(ch1_output(8*(i+1)-1 downto 8*i))+128,right,8);
                    write(v_OLINE, v_SPACE);
                end loop;

                for i in 0 to 15 loop
                    write(v_OLINE,unsigned(ch2_output(8*(i+1)-1 downto 8*i))+128,right,8);
                    write(v_OLINE, v_SPACE);
                end loop;

                for i in 0 to 15 loop
                    write(v_OLINE,unsigned(ch3_output(8*(i+1)-1 downto 8*i))+128,right,8);
                    write(v_OLINE, v_SPACE);
                end loop;
                --writeline(output, v_OLINE);
                writeline(file_OUTPUT, v_OLINE);
            end loop;

            file_close(file_INPUT);
            file_close(file_OUTPUT);

            wait;

        end process;

end behave;