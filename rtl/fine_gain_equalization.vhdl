library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.defs.all;

--written to take in raw (signed but not signed) data shifted by 0x80
entity gain_normalization is
    port(
            rst_i			:	in		std_logic;
            clk_data_i	:	in		std_logic;
				channel_gain_mult : std_logic_vector(5*num_channels -1 downto 0);
            ch_data_i : in std_logic_vector(8*step_size*num_channels -1 downto 0);
            ch_data_o : out std_logic_vector(8*step_size*num_channels -1 downto 0)
            );
    end gain_normalization;
    
architecture rtl of gain_normalization is

    type streaming_data_array is array(3 downto 0, 3 downto 0) of signed(7 downto 0);
	 type temp_data_array is array(3 downto 0, 3 downto 0) of signed(15 downto 0);
	 
    signal streaming_data  : streaming_data_array := (others=>(others=>(others=>'0'))); --pipeline data
    signal normalized_data : streaming_data_array:= (others=>(others=>(others=>'0')));
	 signal temp_data 		: temp_data_array:= (others=>(others=>(others=>'0')));

	 type mult_factors_type is array(3 downto 0) of unsigned(4 downto 0);
	 signal mult_factors : mult_factors_type := (others=>(others=>'0'));
	 
	 type internal_factors_type is array(3 downto 0) of unsigned(6 downto 0);
	 signal internal_factors : internal_factors_type := (others=>"1000000");
	 
begin

    --assign inputs
    assign_channels_in: for ch in 0 to 3 generate
        assign_samples: for sam in 0 to step_size-1 generate
            streaming_data(ch,sam)<=signed(unsigned(ch_data_i(8*(sam+1)+ch*4*8-1 downto ch*4*8+8*sam))-x"80");
        end generate;
    end generate;
	 
	 --assign gains
	 assign_gains_in: for ch in 0 to 3 generate
        mult_factors(ch)<=unsigned(channel_gain_mult((ch+1)*5-1 downto ch*5));
    end generate;

    --assign ouputs
    assign_channels_out: for ch in 0 to 3 generate
        assign_samples_o: for sam in 0 to step_size-1 generate
            ch_data_o(8*(sam+1)+ch*4*8-1 downto ch*4*8+8*sam)<=std_logic_vector(unsigned(normalized_data(ch,sam))+x"80");
        end generate;
    end generate;

    -- do the upsampling
    proc_gain_normalization:process(clk_data_i, rst_i) --, enable)
    begin
        for ch in 0 to 3 loop
				for sam in 0 to step_size-1 loop 
					temp_data(ch,sam) <= streaming_data(ch,sam) * to_integer(internal_factors(ch)); --64 to 33 numerator
				end loop;
        end loop;

        if rising_edge(clk_data_i) then
            for  ch in 0 to 3 loop
					 internal_factors(ch) <= to_unsigned(64,7) - mult_factors(ch);
					 
                for sam in 0 to step_size-1 loop
				
						  --round
						  --normalized_data(ch,sam)<=resize(signed(temp_data(ch,sam)(15 downto 6)),8);	  

						  --divide by 64 with rounding
						  if (unsigned(temp_data(ch,sam)(5 downto 0))>=x"20") then
                        normalized_data(ch,sam)<=resize(signed(temp_data(ch,sam)(15 downto 6)),8)+1;

                    else --(unsigned(temp_data(ch,sam)(5 downto 0))<x"20") then
                        normalized_data(ch,sam)<=resize(signed(temp_data(ch,sam)(15 downto 6)),8);

                    end if;
                end loop;
            end loop;
        end if;
    end process;
end rtl;