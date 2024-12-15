--shamelessly stolen from https://forum.digilent.com/topic/18592-enevlope-detection-using-fpga-board/
--and adapted for ~6 bit accuracy on output


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity magnitude is
    Port ( 
        clk           : in std_logic;
        x_in          : in std_logic_vector(6 downto 0):= (others => '0');
        y_in          : in std_logic_vector(6 downto 0):= (others => '0');
        x_out         : out std_logic_vector(6 downto 0) := (others => '0');
        y_out         : out std_logic_vector(6 downto 0) := (others => '0');
        magnitude_out : out unsigned(7 downto 0) := (others => '0') -- Accurate to 6 bits or so
    );
end magnitude;

architecture Behavioral of magnitude is

    type a_x is array(0 to 6) of signed(x_in'high+1 downto 0);
    type a_y is array(0 to 6) of signed(y_in'high+1 downto 0);
    type a_x_delay is array(0 to 6) of std_logic_vector(x_in'high downto 0);
    type a_y_delay is array(0 to 6) of std_logic_vector(y_in'high downto 0);
    
    signal x : a_x := (others => (others => '0'));
	 signal x_temp : a_x := (others => (others => '0'));
	 
    signal y : a_y := (others => (others => '0'));
	 signal y_temp : a_y := (others => (others => '0'));
    signal x_delay : a_x_delay := (others => (others => '0'));
    signal y_delay : a_y_delay := (others => (others => '0'));
	 
	 signal magnitude_temp:signed(11 downto 0):=(others=>'0');
    
begin

    --magnitude_out <= 13*unsigned(x(6))/8;
    x_out <= x_delay(x_delay'high);
    y_out <= y_delay(y_delay'high);

process(clk)
	begin
      	 
	magnitude_temp<=13*resize(unsigned(x(6)),12);
	
	if magnitude_temp(2 downto 0)>=b"100" then
		magnitude_out<=magnitude_temp(11 downto 3)+unsigned(b"100");
	else 
		magnitude_out<=magnitude_temp(11 downto 3);
	end if;
	
       if rising_edge(clk) then
				/*
            if x(5) >= 0 then
					 x(6) <= x(5) - y(5)(y(5)'high downto 5);
               y(6) <= y(5) + x(5)(x(5)'high downto 5);
            else
					 x(6) <= x(5) + y(5)(x(5)'high downto 5);
                y(6) <= y(5) - x(5)(x(5)'high downto 5);
            end if;
            
				if y(4) >= 0 then
                x(5) <= x(4) - y(4)(y(4)'high downto 4);
                y(5) <= y(4) + x(4)(x(4)'high downto 4);
            else
                x(5) <= x(4) + y(4)(y(4)'high downto 4);
                y(5) <= y(4) - x(4)(x(4)'high downto 4);
            end if;
				
            if y(3) >= 0 then
                x(4) <= x(3) - y(3)(y(3)'high downto 3);
                y(4) <= y(3) + x(3)(x(3)'high downto 3);
            else
                x(4) <= x(3) + y(3)(y(3)'high downto 3);
                y(4) <= y(3) - x(3)(x(3)'high downto 3);
            end if;
            */
            if y(2) >= 0 then
                x(3) <= x(2) - y(2)(y(2)'high downto 2);
                y(3) <= y(2) + x(2)(x(2)'high downto 2);
            else
                x(3) <= x(2) + y(2)(y(2)'high downto 2);
                y(3) <= y(2) - x(2)(x(2)'high downto 2);
            end if;
            
            if y(1) >= 0 then
                x(2) <= x(1) - y(1)(y(1)'high downto 1);
                y(2) <= y(1) + x(1)(x(1)'high downto 1);
            else
                x(2) <= x(1) + y(1)(y(1)'high downto 1);
                y(2) <= y(1) - x(1)(x(1)'high downto 1);
            end if;
            
            if y(0) >= 0 then
                x(1) <= x(0) - y(0)(y(0)'high downto 0);
                y(1) <= y(0) + x(0)(x(0)'high downto 0);
            else
                x(1) <= x(0) + y(0)(y(0)'high downto 0);
                y(1) <= y(0) - x(0)(x(0)'high downto 0);
            end if;
            
				 x(0) <= resize(signed(x_in),7);
				 y(0) <= resize(signed(y_in),7);
            
            -- Delay to output the inputs, so they are aligned with the magnitudes
            x_delay(1 to 6) <= x_delay(0 to 5);
            y_delay(1 to 6) <= y_delay(0 to 5);
            x_delay(0) <= x_in;
            y_delay(0) <= y_in;
        end if;
    end process;
 end Behavioral;