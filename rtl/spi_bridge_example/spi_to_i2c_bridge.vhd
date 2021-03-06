--------------------------------------------------------------------------------
--
--   FileName:         spi_to_i2c_bridge.vhd
--   Dependencies:     spi_slave.vhd (v1.1)
--                     spi_to_i2c.vhd (v1.0)
--                     i2c_master.vhd (v1.0)
--   Design Software:  Quartus II 32-bit Version 11.1 Build 173 SJ Full Version
--
--   HDL CODE IS PROVIDED "AS IS."  DIGI-KEY EXPRESSLY DISCLAIMS ANY
--   WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
--   PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL DIGI-KEY
--   BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
--   DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
--   PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
--   BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
--   ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
--
--   Version History
--   Version 1.0 12/05/2012 Scott Larson
--     Initial Public Release
--    
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;

ENTITY spi_to_i2c_bridge IS
GENERIC(
  sys_clk_frq : INTEGER   := 50_000_000; --system clock speed in Hz
  i2c_scl_frq : INTEGER   := 400_000;    --speed the i2c bus (scl) will run at in Hz
  spi_cpol    : STD_LOGIC := '0';        --spi clock polarity mode
  spi_cpha    : STD_LOGIC := '0');       --spi clock phase mode
PORT(
  clock   : IN    STD_LOGIC;  --system clock
  reset_n : IN    STD_LOGIC;  --active low reset
  sclk    : IN    STD_LOGIC;  --spi serial clock
  ss_n    : IN    STD_LOGIC;  --spi slave select
  mosi    : IN    STD_LOGIC;  --spi master out, slave in
  miso    : OUT   STD_LOGIC;  --spi master in, slave out
  trdy    : OUT   STD_LOGIC;  --spi transmit ready
  scl     : INOUT STD_LOGIC;  --i2c serial clock
  sda     : INOUT STD_LOGIC); --i2c serial data
END spi_to_i2c_bridge;

ARCHITECTURE logic OF spi_to_i2c_bridge IS
  CONSTANT spi_d_width : INTEGER := 25;  --spi data width in bits
  SIGNAL   spi_busy    : STD_LOGIC;
  SIGNAL   spi_tx_ena  : STD_LOGIC;
  SIGNAL   spi_tx_data : STD_LOGIC_VECTOR(24 DOWNTO 0);
  SIGNAL   spi_rx_req  : STD_LOGIC;
  SIGNAL   spi_rx_data : STD_LOGIC_VECTOR(24 DOWNTO 0);
  SIGNAL   spi_rrdy    : STD_LOGIC;
  SIGNAL   i2c_ena     : STD_LOGIC;
  SIGNAL   i2c_addr    : STD_LOGIC_VECTOR(6 DOWNTO 0);
  SIGNAL   i2c_rw      : STD_LOGIC;
  SIGNAL   i2c_data_wr : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL   i2c_data_rd : STD_LOGIC_VECTOR(7 DOWNTO 0);
  SIGNAL   i2c_ack_err : STD_LOGIC;
  SIGNAL   i2c_busy    : STD_LOGIC;

  --declare spi slave component
  COMPONENT spi_slave IS
    GENERIC(
      cpol    : STD_LOGIC; --spi clock polarity mode
      cpha    : STD_LOGIC; --spi clock phase mode
      d_width : INTEGER);  --data width in bits
    PORT(
      sclk         : IN     STD_LOGIC;                            --spi clk from master
      reset_n      : IN     STD_LOGIC;                            --active low reset
      ss_n         : IN     STD_LOGIC;                            --active low slave select
      mosi         : IN     STD_LOGIC;                            --master out, slave in
      rx_req       : IN     STD_LOGIC;                            --'1' while busy = '0' moves data to the rx_data output
      st_load_en   : IN     STD_LOGIC;                            --asynchronous load enable
      st_load_trdy : IN     STD_LOGIC;                            --asynchronous trdy load input
      st_load_rrdy : IN     STD_LOGIC;                            --asynchronous rrdy load input
      st_load_roe  : IN     STD_LOGIC;                            --asynchronous roe load input
      tx_load_en   : IN     STD_LOGIC;                            --asynchronous transmit buffer load enable
      tx_load_data : IN     STD_LOGIC_VECTOR(d_width-1 DOWNTO 0); --asynchronous tx data to load
      trdy         : BUFFER STD_LOGIC := '0';                     --transmit ready bit
      rrdy         : BUFFER STD_LOGIC := '0';                     --receive ready bit
      roe          : BUFFER STD_LOGIC := '0';                     --receive overrun error bit
      rx_data      : OUT    STD_LOGIC_VECTOR(d_width-1 DOWNTO 0); --receive register output to logic
      busy         : OUT    STD_LOGIC := '0';                     --busy signal to logic ('1' during transaction)
      miso         : OUT    STD_LOGIC := 'Z');                    --master in, slave out
  END COMPONENT spi_slave;

  --declare spi to i2c component
  COMPONENT spi_to_i2c IS
    PORT(
      clk         : IN   STD_LOGIC;                     --system clock
      reset_n     : IN   STD_LOGIC;                     --active low reset
      spi_rrdy    : IN   STD_LOGIC;                     --receive ready signal from spi (new message arrived)
      spi_rx_data : IN   STD_LOGIC_VECTOR(24 DOWNTO 0); --message received on spi
      spi_busy    : IN   STD_LOGIC;                     --spi slave busy signal (talking to spi master)
      spi_rx_req  : OUT  STD_LOGIC;                     --request received message from the spi slave
      spi_tx_ena  : OUT  STD_LOGIC;                     --load return message into spi slave
      spi_tx_data : OUT  STD_LOGIC_VECTOR(24 DOWNTO 0); --return message to send over spi
      i2c_busy    : IN   STD_LOGIC;                     --i2c busy signal (talking to i2c slave)
      i2c_data_rd : IN   STD_LOGIC_VECTOR(7 DOWNTO 0);  --data received from i2c slave
      i2c_ack_err : IN   STD_LOGIC;                     --i2c acknowledge error flag
      i2c_ena     : OUT  STD_LOGIC;                     --latch command into i2c master
      i2c_addr    : OUT  STD_LOGIC_VECTOR(6 DOWNTO 0);  --i2c slave address
      i2c_rw      : OUT  STD_LOGIC;                     --i2c read/write command
      i2c_data_wr : OUT  STD_LOGIC_VECTOR(7 DOWNTO 0)); --data to write over the i2c bus
  END COMPONENT spi_to_i2c;
  
  --declare i2c master component
  COMPONENT i2c_master IS
    GENERIC(
      input_clk : INTEGER;  --input clock speed from user logic in Hz
      bus_clk   : INTEGER); --speed the i2c bus (scl) will run at in Hz
    PORT(
      clk       : IN     STD_LOGIC;                    --system clock
      reset_n   : IN     STD_LOGIC;                    --active low reset
      ena       : IN     STD_LOGIC;                    --latch in command
      addr      : IN     STD_LOGIC_VECTOR(6 DOWNTO 0); --address of target slave
      rw        : IN     STD_LOGIC;                    --'0' is write, '1' is read
      data_wr   : IN     STD_LOGIC_VECTOR(7 DOWNTO 0); --data to write to slave
      busy      : OUT    STD_LOGIC;                    --indicates transaction in progress
      data_rd   : OUT    STD_LOGIC_VECTOR(7 DOWNTO 0); --data read from slave
      ack_error : BUFFER STD_LOGIC;                    --flag if improper acknowledge from slave
      sda       : INOUT  STD_LOGIC;                    --serial data output of i2c bus
      scl       : INOUT  STD_LOGIC);                   --serial clock output of i2c bus
  END COMPONENT i2c_master;
  
BEGIN

  --instantiate the spi slave
  spi_slave_0:  spi_slave
    GENERIC MAP(cpol => spi_cpol, cpha => spi_cpha, d_width => spi_d_width)
    PORT MAP(sclk => sclk, reset_n => reset_n, ss_n => ss_n, mosi => mosi,
             rx_req => spi_rx_req, st_load_en => '0', st_load_trdy => '0',
             st_load_rrdy => '0', st_load_roe => '0', tx_load_en => spi_tx_ena,
             tx_load_data => spi_tx_data, trdy => trdy, rrdy => spi_rrdy, roe => open,
             rx_data => spi_rx_data, busy => spi_busy, miso => miso);
            
  --instantiate the bridge component
  spi_to_i2c_0:  spi_to_i2c
    PORT MAP(clk => clock, reset_n => reset_n, spi_rrdy => spi_rrdy,
             spi_rx_data => spi_rx_data, spi_busy => spi_busy,
             spi_rx_req => spi_rx_req, spi_tx_ena => spi_tx_ena,
             spi_tx_data => spi_tx_data, i2c_busy => i2c_busy,
             i2c_data_rd => i2c_data_rd, i2c_ack_err => i2c_ack_err,
             i2c_ena => i2c_ena, i2c_addr => i2c_addr, i2c_rw => i2c_rw,
             i2c_data_wr => i2c_data_wr);  
  
  --instantiate the i2c master
  i2c_master_0:  i2c_master
    GENERIC MAP(input_clk => sys_clk_frq, bus_clk => i2c_scl_frq)
    PORT MAP(clk => clock, reset_n => reset_n, ena => i2c_ena, addr => i2c_addr,
             rw => i2c_rw, data_wr => i2c_data_wr, busy => i2c_busy,
             data_rd => i2c_data_rd, ack_error => i2c_ack_err, sda => sda,
             scl => scl);  

END logic;
