cd ..
echo "running trigger testbench, hiding stdout"
ghdl -r --std=08 trigger_tb --stop-time=2170ns > /dev/null
cd tb
