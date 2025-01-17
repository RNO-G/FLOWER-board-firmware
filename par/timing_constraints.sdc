#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3

#**************************************************************
# Constraints
#**************************************************************
derive_pll_clocks -create_base_clocks -use_net_name
derive_clock_uncertainty -add

create_clock -name board_clock_i 	-period 10.000MHz 	[get_ports {board_clock_i}]
create_clock -name sys_clock_i 	  	-period 10.000MHz 	[get_ports {sys_clock_i}]
create_clock -name adc0_fclk_i   	-period 472.000MHz 	[get_ports {adc0_fclk_i}]
create_clock -name adc1_fclk_i   	-period 472.000MHz 	[get_ports {adc1_fclk_i}]

#create_clock -name spi_clock_i 		-period 10.000MHz 	[get_ports {spi_clock_i}]
#create_clock -name flower_top:inst|reset_and_startup:xRESETS|fpga_reset_pwr \
 	-period 0.0001MHz 	[get_ports {flower_top:inst|reset_and_startup:xRESETS|fpga_reset_pwr}] 

#create_clock -name master_clock1 -period 100.000MHz 	[get_ports {master_clock1}]

#set_net_delay -from [get_ports {ADC_PIN15_PIN14_RST[0]}] -to [get_ports {ADC_PIN15_PIN14_RST[1]}] -max 0.050
#set_net_delay -from [get_ports {ADC_PIN15_PIN14_RST[0]}] -to [get_ports {ADC_PIN15_PIN14_RST[2]}] -max 0.050
#set_net_delay -from [get_ports {ADC_PIN15_PIN14_RST[0]}] -to [get_ports {ADC_PIN15_PIN14_RST[3]}] -max 0.050

####set false paths from async. resets
#set_false_path -from {top_level:inst|sys_reset:xGLOBAL_RESET|pulse_stretcher_sync:xUSER_RESET|pulse_o} -to *
#set_false_path -from {top_level:inst|sys_reset:xGLOBAL_RESET|pulse_stretcher_sync:xUSER_SYS_RESET|pulse_o} -to *
#set_false_path -from {reset_and_startup:xRESETS|fpga_reset_pwr} -to "*" 
#set_false_path -from {reset_and_startup:xRESETS|power_on_rst_o} -to "*"
#set_false_path -from {get_ports {board_clock_i} -to {flower_top:inst|reset_and_startup:xRESETS|power_on_rst_o}
#set_false_path -from {top_level:inst|sys_reset:xGLOBAL_RESET|pulse_stretcher_sync:xADC_RESET|pulse_o} -to *



set_max_delay -from {clock_manager:xCLOCKS|pll_block_1:xPLL_BLOCK1|pll_block_1_0002:pll_block_1_inst|altera_pll:altera_pll_i|outclk_wire[1]} -to {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[0]} 6.000

set_max_delay -from {clock_manager:xCLOCKS|pll_block_1:xPLL_BLOCK1|pll_block_1_0002:pll_block_1_inst|altera_pll:altera_pll_i|outclk_wire[1]} -to {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[1]} 6.000

set_max_delay -from {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[0]} -to {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[1]} 6.000

set_max_delay -from {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[1]} -to {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[0]} 6.000

set_max_delay -from {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[1]} -to {clock_manager:xCLOCKS|pll_block_1:xPLL_BLOCK1|pll_block_1_0002:pll_block_1_inst|altera_pll:altera_pll_i|outclk_wire[1]} 6.000

set_max_delay -from {clock_manager:xCLOCKS|pll_block_1:xPLL_BLOCK1|pll_block_1_0002:pll_block_1_inst|altera_pll:altera_pll_i|outclk_wire[0]} -to {clock_manager:xCLOCKS|pll_block_2:xPLL_BLOCK2|pll_block_2_0002:pll_block_2_inst|altera_pll:altera_pll_i|outclk_wire[0]} 6.000

#set_max_delay -from {adc_receiver:xADC1_DATA_RX|internal_rx_dat_valid[0]} -to * 16.000
#set_max_delay -from {adc_receiver:xADC0_DATA_RX|internal_rx_dat_valid[0]} -to * 16.000

#set_clock_groups -asynchronous \
   -group [get_clocks fpga_reset_pwr] 
	#-group [get_clocks spi_sclk_i]
