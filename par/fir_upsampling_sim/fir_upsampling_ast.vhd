
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.auk_dspip_lib_pkg_hpfir.all;
use work.auk_dspip_math_pkg_hpfir.all;

entity fir_upsampling_ast is
  generic (
        INWIDTH             : integer := 8;
        OUT_WIDTH_UNTRIMMED : integer := 17;
        BANKINWIDTH         : integer := 0;
        REM_LSB_BIT_g       : integer := 5;
        REM_LSB_TYPE_g      : string := "trunc";
        REM_MSB_BIT_g       : integer := 4;
        REM_MSB_TYPE_g      : string := "trunc";
        PHYSCHANIN          : integer := 16;
        PHYSCHANOUT         : integer := 64;
        CHANSPERPHYIN       : natural := 1;
        CHANSPERPHYOUT      : natural := 1;
        OUTPUTFIFODEPTH     : integer := 4;
        USE_PACKETS         : integer := 0;
        MODE_WIDTH         : integer := 0;
        ENABLE_BACKPRESSURE : boolean := false;
        LOG2_CHANSPERPHYOUT : natural := log2_ceil_one(1);
        NUMCHANS            : integer := 4;
        DEVICE_FAMILY       : string := "Cyclone V";
        COMPLEX_CONST       : integer := 1
  );
  port(
    clk                : in  std_logic;
    reset_n            : in  std_logic;
    ast_sink_ready     : out std_logic;
    ast_source_data    : out std_logic_vector(COMPLEX_CONST*(OUT_WIDTH_UNTRIMMED - REM_LSB_BIT_g - REM_MSB_BIT_g) * PHYSCHANOUT - 1  downto 0);
    ast_sink_data      : in std_logic_vector( COMPLEX_CONST*(INWIDTH + BANKINWIDTH) * PHYSCHANIN  + MODE_WIDTH - 1 downto 0);
    ast_sink_valid     : in  std_logic;
    ast_source_valid   : out std_logic;    
    ast_source_ready   : in  std_logic;
    ast_source_eop     : out std_logic;
    ast_source_sop     : out std_logic;
    ast_source_channel : out std_logic_vector (LOG2_CHANSPERPHYOUT - 1 downto 0);
    ast_sink_eop       : in  std_logic;
    ast_sink_sop       : in  std_logic;
    ast_sink_error     : in  std_logic_vector (1 downto 0);
    ast_source_error   : out std_logic_vector (1 downto 0)
    );
attribute altera_attribute : string;
attribute altera_attribute of fir_upsampling_ast:entity is "-name MESSAGE_DISABLE 15400; -name MESSAGE_DISABLE 14130; -name MESSAGE_DISABLE 12020; -name MESSAGE_DISABLE 12030; -name MESSAGE_DISABLE 12010; -name MESSAGE_DISABLE 12110; -name MESSAGE_DISABLE 14320; -name MESSAGE_DISABLE 13410; -name MESSAGE_DISABLE 10036";
end fir_upsampling_ast;

-- Warnings Suppression On
-- altera message_off 10036

architecture struct of fir_upsampling_ast is
  
  constant OUTWIDTH          : integer   := OUT_WIDTH_UNTRIMMED - REM_LSB_BIT_g - REM_MSB_BIT_g;

  signal channel_out         : std_logic_vector(LOG2_CHANSPERPHYOUT - 1 downto 0);
  
  signal core_channel_out    : std_logic_vector(2 -1 downto 0);
  signal at_source_channel   : std_logic_vector(2 -1 downto 0);
  signal sink_packet_error   : std_logic_vector(1 downto 0);
  signal data_in             : std_logic_vector((COMPLEX_CONST*INWIDTH + BANKINWIDTH) * PHYSCHANIN  + MODE_WIDTH - 1 downto 0);
  signal data_valid          : std_logic_vector(0 downto 0);
  
  signal data_out            : std_logic_vector(COMPLEX_CONST*OUTWIDTH * PHYSCHANOUT -1 downto 0);
  signal reset_fir           : std_logic;
  signal sink_ready_ctrl     : std_logic;
  signal source_packet_error : std_logic_vector(1 downto 0);
  signal source_stall        : std_logic;
  signal source_valid_ctrl   : std_logic;
  signal stall               : std_logic;
  signal valid               : std_logic;
  signal core_valid          : std_logic;
  signal enable_in           : std_logic_vector(0 downto 0);
  
  signal outp_out            : std_logic_vector(COMPLEX_CONST*OUTWIDTH * PHYSCHANOUT - 1 downto 0);
  signal outp_blk_valid      : std_logic_vector(PHYSCHANOUT - 1 downto 0);

  signal core_out            : std_logic_vector(OUT_WIDTH_UNTRIMMED * PHYSCHANOUT - 1 downto 0);
  signal core_out_valid      : std_logic_vector(0 downto 0);
  signal core_out_channel    : std_logic_vector(7 downto 0);

  signal core_out_channel_0  : std_logic_vector(7 downto 0);

     
begin
  sink : auk_dspip_avalon_streaming_sink_hpfir
    generic map (
      WIDTH_g          => (COMPLEX_CONST*INWIDTH + BANKINWIDTH) * PHYSCHANIN  + MODE_WIDTH,
      DATA_WIDTH       => (COMPLEX_CONST*INWIDTH + BANKINWIDTH) * PHYSCHANIN  + MODE_WIDTH,
      DATA_PORT_COUNT  => 1,
      PACKET_SIZE_g    => CHANSPERPHYIN)
    port map (
      clk             => clk,
      reset_n         => reset_n,
      data            => data_in,
      data_valid      => data_valid,
      sink_ready_ctrl => sink_ready_ctrl,
      packet_error    => sink_packet_error,
      at_sink_ready   => ast_sink_ready,
      at_sink_valid   => ast_sink_valid,
      at_sink_data    => ast_sink_data,
      at_sink_sop     => ast_sink_sop,
      at_sink_eop     => ast_sink_eop,
      at_sink_error   => ast_sink_error);
  
  source : auk_dspip_avalon_streaming_source_hpfir
    generic map (
      WIDTH_g           => COMPLEX_CONST*OUTWIDTH * PHYSCHANOUT,
      DATA_WIDTH        => COMPLEX_CONST*OUTWIDTH,
      DATA_PORT_COUNT   => PHYSCHANOUT,
      FIFO_DEPTH_g      => OUTPUTFIFODEPTH,
      USE_PACKETS       => USE_PACKETS,
      HAVE_COUNTER_g    => false,
      PACKET_SIZE_g     => CHANSPERPHYOUT,
      COUNTER_LIMIT_g   => CHANSPERPHYOUT,
      ENABLE_BACKPRESSURE_g => ENABLE_BACKPRESSURE)
    port map (
      clk               => clk,
      reset_n           => reset_n,
      data_in           => data_out,
      data_count        => channel_out,
      source_valid_ctrl => source_valid_ctrl,
      source_stall      => source_stall,
      packet_error      => source_packet_error,
      at_source_ready   => ast_source_ready,
      at_source_valid   => ast_source_valid,
      at_source_data    => ast_source_data,
      at_source_channel => ast_source_channel,
      at_source_sop     => ast_source_sop,
      at_source_eop     => ast_source_eop,
      at_source_error   => ast_source_error);
   
   
  intf_ctrl : auk_dspip_avalon_streaming_controller_hpfir
    port map (
      clk                 => clk,
      reset_n             => reset_n,
      sink_packet_error   => sink_packet_error,
      source_stall        => source_stall,
      valid               => valid,
      reset_design        => reset_fir,
      sink_ready_ctrl     => sink_ready_ctrl,
      source_packet_error => source_packet_error,
      source_valid_ctrl   => source_valid_ctrl,
      stall               => stall);


  
  multi_data_out: for m in PHYSCHANOUT-1 downto 0 generate  
    data_out(((m*OUTWIDTH)+OUTWIDTH-1) downto (m*OUTWIDTH)) <= outp_out(((m*OUTWIDTH)+OUTWIDTH-1) downto (m*OUTWIDTH));
  end generate multi_data_out;

  channel_pipe_lsb: if REM_LSB_TYPE_g = "round" and REM_LSB_BIT_g > 0 generate
  begin
    out_lsb_p : process (clk, reset_n)
    begin
      if reset_n = '0' then
        core_out_channel_0 <= (others => '0');
      elsif rising_edge(clk) then
        core_out_channel_0 <= core_out_channel;
      end if;
    end process out_lsb_p;
  end generate channel_pipe_lsb;
  
  channel_wire_lsb: if REM_LSB_TYPE_g = "trunc" or REM_LSB_BIT_g = 0 generate
  begin
    core_out_channel_0 <= core_out_channel;
  end generate channel_wire_lsb;  
  
  channel_pipe_msb: if REM_MSB_TYPE_g = "sat" and REM_MSB_BIT_g > 0 generate
  begin
    out_p : process (clk, reset_n)
    begin
      if reset_n = '0' then
        channel_out <= (others => '0');
      elsif rising_edge(clk) then
        channel_out <= core_out_channel_0(LOG2_CHANSPERPHYOUT-1 downto 0);
      end if;
    end process out_p;
  end generate channel_pipe_msb;

  channel_wire_msb: if REM_MSB_TYPE_g = "trunc" or REM_MSB_BIT_g = 0 generate
  begin
    channel_out <= core_out_channel_0(LOG2_CHANSPERPHYOUT-1 downto 0);
  end generate channel_wire_msb;


real_passthrough : if COMPLEX_CONST = 1 generate

      component fir_upsampling_rtl_core is
      port (
        xIn_v                 : in std_logic_vector(0 downto 0);
        xIn_c                 : in std_logic_vector(7 downto 0);
        xIn_0                : in std_logic_vector(8 - 1 downto 0);
        xIn_1                : in std_logic_vector(8 - 1 downto 0);
        xIn_2                : in std_logic_vector(8 - 1 downto 0);
        xIn_3                : in std_logic_vector(8 - 1 downto 0);
        xIn_4                : in std_logic_vector(8 - 1 downto 0);
        xIn_5                : in std_logic_vector(8 - 1 downto 0);
        xIn_6                : in std_logic_vector(8 - 1 downto 0);
        xIn_7                : in std_logic_vector(8 - 1 downto 0);
        xIn_8                : in std_logic_vector(8 - 1 downto 0);
        xIn_9                : in std_logic_vector(8 - 1 downto 0);
        xIn_10                : in std_logic_vector(8 - 1 downto 0);
        xIn_11                : in std_logic_vector(8 - 1 downto 0);
        xIn_12                : in std_logic_vector(8 - 1 downto 0);
        xIn_13                : in std_logic_vector(8 - 1 downto 0);
        xIn_14                : in std_logic_vector(8 - 1 downto 0);
        xIn_15                : in std_logic_vector(8 - 1 downto 0);
        xOut_v               : out std_logic_vector(0 downto 0);
        xOut_c               : out std_logic_vector(7 downto 0);
        xOut_0              : out std_logic_vector(17- 1 downto 0);
        xOut_1              : out std_logic_vector(17- 1 downto 0);
        xOut_2              : out std_logic_vector(17- 1 downto 0);
        xOut_3              : out std_logic_vector(17- 1 downto 0);
        xOut_4              : out std_logic_vector(17- 1 downto 0);
        xOut_5              : out std_logic_vector(17- 1 downto 0);
        xOut_6              : out std_logic_vector(17- 1 downto 0);
        xOut_7              : out std_logic_vector(17- 1 downto 0);
        xOut_8              : out std_logic_vector(17- 1 downto 0);
        xOut_9              : out std_logic_vector(17- 1 downto 0);
        xOut_10              : out std_logic_vector(17- 1 downto 0);
        xOut_11              : out std_logic_vector(17- 1 downto 0);
        xOut_12              : out std_logic_vector(17- 1 downto 0);
        xOut_13              : out std_logic_vector(17- 1 downto 0);
        xOut_14              : out std_logic_vector(17- 1 downto 0);
        xOut_15              : out std_logic_vector(17- 1 downto 0);
        xOut_16              : out std_logic_vector(17- 1 downto 0);
        xOut_17              : out std_logic_vector(17- 1 downto 0);
        xOut_18              : out std_logic_vector(17- 1 downto 0);
        xOut_19              : out std_logic_vector(17- 1 downto 0);
        xOut_20              : out std_logic_vector(17- 1 downto 0);
        xOut_21              : out std_logic_vector(17- 1 downto 0);
        xOut_22              : out std_logic_vector(17- 1 downto 0);
        xOut_23              : out std_logic_vector(17- 1 downto 0);
        xOut_24              : out std_logic_vector(17- 1 downto 0);
        xOut_25              : out std_logic_vector(17- 1 downto 0);
        xOut_26              : out std_logic_vector(17- 1 downto 0);
        xOut_27              : out std_logic_vector(17- 1 downto 0);
        xOut_28              : out std_logic_vector(17- 1 downto 0);
        xOut_29              : out std_logic_vector(17- 1 downto 0);
        xOut_30              : out std_logic_vector(17- 1 downto 0);
        xOut_31              : out std_logic_vector(17- 1 downto 0);
        xOut_32              : out std_logic_vector(17- 1 downto 0);
        xOut_33              : out std_logic_vector(17- 1 downto 0);
        xOut_34              : out std_logic_vector(17- 1 downto 0);
        xOut_35              : out std_logic_vector(17- 1 downto 0);
        xOut_36              : out std_logic_vector(17- 1 downto 0);
        xOut_37              : out std_logic_vector(17- 1 downto 0);
        xOut_38              : out std_logic_vector(17- 1 downto 0);
        xOut_39              : out std_logic_vector(17- 1 downto 0);
        xOut_40              : out std_logic_vector(17- 1 downto 0);
        xOut_41              : out std_logic_vector(17- 1 downto 0);
        xOut_42              : out std_logic_vector(17- 1 downto 0);
        xOut_43              : out std_logic_vector(17- 1 downto 0);
        xOut_44              : out std_logic_vector(17- 1 downto 0);
        xOut_45              : out std_logic_vector(17- 1 downto 0);
        xOut_46              : out std_logic_vector(17- 1 downto 0);
        xOut_47              : out std_logic_vector(17- 1 downto 0);
        xOut_48              : out std_logic_vector(17- 1 downto 0);
        xOut_49              : out std_logic_vector(17- 1 downto 0);
        xOut_50              : out std_logic_vector(17- 1 downto 0);
        xOut_51              : out std_logic_vector(17- 1 downto 0);
        xOut_52              : out std_logic_vector(17- 1 downto 0);
        xOut_53              : out std_logic_vector(17- 1 downto 0);
        xOut_54              : out std_logic_vector(17- 1 downto 0);
        xOut_55              : out std_logic_vector(17- 1 downto 0);
        xOut_56              : out std_logic_vector(17- 1 downto 0);
        xOut_57              : out std_logic_vector(17- 1 downto 0);
        xOut_58              : out std_logic_vector(17- 1 downto 0);
        xOut_59              : out std_logic_vector(17- 1 downto 0);
        xOut_60              : out std_logic_vector(17- 1 downto 0);
        xOut_61              : out std_logic_vector(17- 1 downto 0);
        xOut_62              : out std_logic_vector(17- 1 downto 0);
        xOut_63              : out std_logic_vector(17- 1 downto 0);
        clk                  : in std_logic;
        areset               : in std_logic
        );
end component fir_upsampling_rtl_core;


    --Complex data re-ordering
    signal core_channel_out_core    : std_logic_vector(2 -1 downto 0);
    signal data_in_core             : std_logic_vector((COMPLEX_CONST*INWIDTH + BANKINWIDTH) * PHYSCHANIN  + MODE_WIDTH - 1 downto 0);
    signal data_valid_core          : std_logic_vector(0 downto 0);
    signal core_out_core            : std_logic_vector(OUT_WIDTH_UNTRIMMED * PHYSCHANOUT - 1 downto 0);
    signal core_out_valid_core      : std_logic_vector(0 downto 0);
    signal core_out_channel_core    : std_logic_vector(7 downto 0);
  


  begin
        hpfircore_core: fir_upsampling_rtl_core
           port map (
            xIn_v     => data_valid_core,
            xIn_c     => "00000000",
            xIn_0     => data_in_core((0 + 8) * 0 + 8 - 1 downto (0 + 8) * 0),
            xIn_1     => data_in_core((0 + 8) * 1 + 8 - 1 downto (0 + 8) * 1),
            xIn_2     => data_in_core((0 + 8) * 2 + 8 - 1 downto (0 + 8) * 2),
            xIn_3     => data_in_core((0 + 8) * 3 + 8 - 1 downto (0 + 8) * 3),
            xIn_4     => data_in_core((0 + 8) * 4 + 8 - 1 downto (0 + 8) * 4),
            xIn_5     => data_in_core((0 + 8) * 5 + 8 - 1 downto (0 + 8) * 5),
            xIn_6     => data_in_core((0 + 8) * 6 + 8 - 1 downto (0 + 8) * 6),
            xIn_7     => data_in_core((0 + 8) * 7 + 8 - 1 downto (0 + 8) * 7),
            xIn_8     => data_in_core((0 + 8) * 8 + 8 - 1 downto (0 + 8) * 8),
            xIn_9     => data_in_core((0 + 8) * 9 + 8 - 1 downto (0 + 8) * 9),
            xIn_10     => data_in_core((0 + 8) * 10 + 8 - 1 downto (0 + 8) * 10),
            xIn_11     => data_in_core((0 + 8) * 11 + 8 - 1 downto (0 + 8) * 11),
            xIn_12     => data_in_core((0 + 8) * 12 + 8 - 1 downto (0 + 8) * 12),
            xIn_13     => data_in_core((0 + 8) * 13 + 8 - 1 downto (0 + 8) * 13),
            xIn_14     => data_in_core((0 + 8) * 14 + 8 - 1 downto (0 + 8) * 14),
            xIn_15     => data_in_core((0 + 8) * 15 + 8 - 1 downto (0 + 8) * 15),
            xOut_v    => core_out_valid_core,
            xOut_c    => core_out_channel_core,
            xOut_0   => core_out_core(17* 0 + 17- 1 downto 17* 0),
            xOut_1   => core_out_core(17* 1 + 17- 1 downto 17* 1),
            xOut_2   => core_out_core(17* 2 + 17- 1 downto 17* 2),
            xOut_3   => core_out_core(17* 3 + 17- 1 downto 17* 3),
            xOut_4   => core_out_core(17* 4 + 17- 1 downto 17* 4),
            xOut_5   => core_out_core(17* 5 + 17- 1 downto 17* 5),
            xOut_6   => core_out_core(17* 6 + 17- 1 downto 17* 6),
            xOut_7   => core_out_core(17* 7 + 17- 1 downto 17* 7),
            xOut_8   => core_out_core(17* 8 + 17- 1 downto 17* 8),
            xOut_9   => core_out_core(17* 9 + 17- 1 downto 17* 9),
            xOut_10   => core_out_core(17* 10 + 17- 1 downto 17* 10),
            xOut_11   => core_out_core(17* 11 + 17- 1 downto 17* 11),
            xOut_12   => core_out_core(17* 12 + 17- 1 downto 17* 12),
            xOut_13   => core_out_core(17* 13 + 17- 1 downto 17* 13),
            xOut_14   => core_out_core(17* 14 + 17- 1 downto 17* 14),
            xOut_15   => core_out_core(17* 15 + 17- 1 downto 17* 15),
            xOut_16   => core_out_core(17* 16 + 17- 1 downto 17* 16),
            xOut_17   => core_out_core(17* 17 + 17- 1 downto 17* 17),
            xOut_18   => core_out_core(17* 18 + 17- 1 downto 17* 18),
            xOut_19   => core_out_core(17* 19 + 17- 1 downto 17* 19),
            xOut_20   => core_out_core(17* 20 + 17- 1 downto 17* 20),
            xOut_21   => core_out_core(17* 21 + 17- 1 downto 17* 21),
            xOut_22   => core_out_core(17* 22 + 17- 1 downto 17* 22),
            xOut_23   => core_out_core(17* 23 + 17- 1 downto 17* 23),
            xOut_24   => core_out_core(17* 24 + 17- 1 downto 17* 24),
            xOut_25   => core_out_core(17* 25 + 17- 1 downto 17* 25),
            xOut_26   => core_out_core(17* 26 + 17- 1 downto 17* 26),
            xOut_27   => core_out_core(17* 27 + 17- 1 downto 17* 27),
            xOut_28   => core_out_core(17* 28 + 17- 1 downto 17* 28),
            xOut_29   => core_out_core(17* 29 + 17- 1 downto 17* 29),
            xOut_30   => core_out_core(17* 30 + 17- 1 downto 17* 30),
            xOut_31   => core_out_core(17* 31 + 17- 1 downto 17* 31),
            xOut_32   => core_out_core(17* 32 + 17- 1 downto 17* 32),
            xOut_33   => core_out_core(17* 33 + 17- 1 downto 17* 33),
            xOut_34   => core_out_core(17* 34 + 17- 1 downto 17* 34),
            xOut_35   => core_out_core(17* 35 + 17- 1 downto 17* 35),
            xOut_36   => core_out_core(17* 36 + 17- 1 downto 17* 36),
            xOut_37   => core_out_core(17* 37 + 17- 1 downto 17* 37),
            xOut_38   => core_out_core(17* 38 + 17- 1 downto 17* 38),
            xOut_39   => core_out_core(17* 39 + 17- 1 downto 17* 39),
            xOut_40   => core_out_core(17* 40 + 17- 1 downto 17* 40),
            xOut_41   => core_out_core(17* 41 + 17- 1 downto 17* 41),
            xOut_42   => core_out_core(17* 42 + 17- 1 downto 17* 42),
            xOut_43   => core_out_core(17* 43 + 17- 1 downto 17* 43),
            xOut_44   => core_out_core(17* 44 + 17- 1 downto 17* 44),
            xOut_45   => core_out_core(17* 45 + 17- 1 downto 17* 45),
            xOut_46   => core_out_core(17* 46 + 17- 1 downto 17* 46),
            xOut_47   => core_out_core(17* 47 + 17- 1 downto 17* 47),
            xOut_48   => core_out_core(17* 48 + 17- 1 downto 17* 48),
            xOut_49   => core_out_core(17* 49 + 17- 1 downto 17* 49),
            xOut_50   => core_out_core(17* 50 + 17- 1 downto 17* 50),
            xOut_51   => core_out_core(17* 51 + 17- 1 downto 17* 51),
            xOut_52   => core_out_core(17* 52 + 17- 1 downto 17* 52),
            xOut_53   => core_out_core(17* 53 + 17- 1 downto 17* 53),
            xOut_54   => core_out_core(17* 54 + 17- 1 downto 17* 54),
            xOut_55   => core_out_core(17* 55 + 17- 1 downto 17* 55),
            xOut_56   => core_out_core(17* 56 + 17- 1 downto 17* 56),
            xOut_57   => core_out_core(17* 57 + 17- 1 downto 17* 57),
            xOut_58   => core_out_core(17* 58 + 17- 1 downto 17* 58),
            xOut_59   => core_out_core(17* 59 + 17- 1 downto 17* 59),
            xOut_60   => core_out_core(17* 60 + 17- 1 downto 17* 60),
            xOut_61   => core_out_core(17* 61 + 17- 1 downto 17* 61),
            xOut_62   => core_out_core(17* 62 + 17- 1 downto 17* 62),
            xOut_63   => core_out_core(17* 63 + 17- 1 downto 17* 63),
            clk       => clk,
            areset    => reset_fir
        );




    core_channel_out <= core_channel_out_core;              
    data_in_core <= data_in;               
    data_valid_core <= data_valid;                
    core_out <= core_out_core;              
    core_out_valid(0) <= core_out_valid_core(0);                
    core_out_channel <= core_out_channel_core;                



  gen_outp_blk : for i in PHYSCHANOUT-1 downto 0 generate  
  begin
    outp_blk : auk_dspip_roundsat_hpfir
      generic map (
        IN_WIDTH_g        =>  OUT_WIDTH_UNTRIMMED      ,
        REM_LSB_BIT_g     =>  REM_LSB_BIT_g   ,
        REM_LSB_TYPE_g    =>  REM_LSB_TYPE_g  ,
        REM_MSB_BIT_g     =>  REM_MSB_BIT_g   ,
        REM_MSB_TYPE_g    =>  REM_MSB_TYPE_g
      )
      port map (
        clk               =>  clk,
        reset_n           =>  reset_n,
        enable            =>  core_out_valid(0),
        datain            =>  core_out(((i*OUT_WIDTH_UNTRIMMED)+OUT_WIDTH_UNTRIMMED-1) downto (i*OUT_WIDTH_UNTRIMMED)),
        valid             =>  outp_blk_valid(i),
        dataout           =>  outp_out(((i*OUTWIDTH)+OUTWIDTH-1) downto (i*OUTWIDTH))
      );
  end generate gen_outp_blk;
 end generate real_passthrough;



  valid <= outp_blk_valid(0);
  
  enable_in(0) <= not stall;

end struct;




