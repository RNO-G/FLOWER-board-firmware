#!/usr/bin/env python3


width = 7
a_width = width
z_width = a_width*2

print("module power_lut(clk_i, a, z);")
print("    input clk_i;")
print("    input [{}:0] a;".format(a_width - 1))
print("    output reg [{}:0] z;".format(z_width - 1))
print()
print("    always @(posedge clk_i) begin")
print("        case ({a})")

fmt = "            7'b{{2:07b}}: z <= {{3}}'b{{1:0{0}b}};".format(z_width)
for a in range(2 ** a_width):
    if a>=2**7:
        sel = 2**7-a
    else:
        sel=a
    z = sel*sel
    sel=a
    print(fmt.format(a, z, sel, z_width))


print("            default: z <= 0;")
print("        endcase")
print("    end")
print("endmodule")
