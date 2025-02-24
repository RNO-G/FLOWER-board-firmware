#!/usr/bin/env python3

width = 8
a_width = width
z_width = a_width*2
print("library IEEE;")
print("use ieee.std_logic_1164.all;")
print("use ieee.numeric_std.all;")
print()
print("entity power_lut_8 is")
print("port(")
print("    clk_i:in std_logic;")
print("    a: in signed({} downto 0);".format(a_width - 1))
print("    z: out unsigned({} downto 0)".format(z_width - 1))
print(");")
print("end power_lut_8;")
print("architecture rtl of power_lut_8 is")
print("begin")
print("    power_lut:process(clk_i)")
print("    begin")
print("        if rising_edge(clk_i) then")
print("            case a is")

fmt = "                when \"{{2:08b}}\" => z <= \"{{1:0{0}b}}\";".format(z_width)
for a in range(2 ** a_width):
    if a>=2**7:
        sel = 2**8-a
    else:
        sel=a
    z = sel*sel
    sel=a
    print(fmt.format(a, z, sel, z_width))

print("                when others => z <= \"0000000000000000\";")
print("            end case;")
print("        end if;")
print("    end process;")
print("end rtl;")
