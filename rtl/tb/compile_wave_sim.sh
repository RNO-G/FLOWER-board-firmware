
cd ../

echo "compiliing defs"
ghdl -a --std=08 defs.vhd

echo "compiliing upsampling"
ghdl -a --std=08 upsampling.vhdl

echo "compiliing beamforming"
ghdl -a --std=08 beamforming.vhdl

echo "compiling power lut"
ghdl -a --std=08 power_lut_8.vhdl

echo "compiling power integration"
ghdl -a --std=08 power_integration.vhdl

echo "compiling power trigger"
ghdl -a --std=08 power_trigger.vhd

echo "compiliing wave tb"
ghdl -a --std=08 tb/wave_tb.vhdl
ghdl -e --std=08 wave_tb

echo "compiliing trigger testbench, scalers not implemented"
ghdl -a --std=08 tb/trigger_tb.vhdl
ghdl -e --std=08 trigger_tb

cd tb
