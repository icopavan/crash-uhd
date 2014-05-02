-------------------------------------------------------------------------------
--  Copyright 2013-2014 Jonathon Pendlum
--
--  This is free software: you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation, either version 3 of the License, or
--  (at your option) any later version.
--
--  This is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.
--
--
--  File: crash_ddr_intf.vhd
--  Author: Jonathon Pendlum (jon.pendlum@gmail.com)
--  Description: Interface and control for CRASH. Implements DDR LVDS I/O and
--               UART control interface.
--
--               Operates with full bandwidth (100 MSPS), full resolution
--               (14-bits for the ADC, 16-bits for the DAC) IQ data. Both ADC
--               and DAC data from CRUSH is synchronous to the input clock.
--
--               The USRP firmware has logic to correct DC offset and IQ
--               balance for both the ADC and DAC. This path is selectable.
--
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library unisim;
use unisim.vcomponents.all;

entity crash_ddr_intf is
  generic (
    CLOCK_FREQ        : integer := 100e6;
    BAUD              : integer := 1e6);
  port (
    clk               : in    std_logic;                      -- Clock (from ADC)
    reset             : in    std_logic;                      -- Active high reset
    RX_DATA_CLK_N     : out   std_logic;                      -- RX data clock (P)
    RX_DATA_CLK_P     : out   std_logic;                      -- RX data clock (N)
    RX_DATA_N         : out   std_logic_vector(4 downto 0);   -- RX data (P)
    RX_DATA_P         : out   std_logic_vector(4 downto 0);   -- RX data (N)
    RX_DATA_STB_N     : out   std_logic;                      -- RX data strobe (N)
    RX_DATA_STB_P     : out   std_logic;                      -- RX data strobe (N)
    TX_DATA_N         : in    std_logic_vector(5 downto 0);   -- TX data (P)
    TX_DATA_P         : in    std_logic_vector(5 downto 0);   -- TX data (N)
    TX_DATA_STB_N     : in    std_logic;                      -- TX data strobe (N)
    TX_DATA_STB_P     : in    std_logic;                      -- TX data strobe (N)
    UART_RX           : in    std_logic;                      -- Control interface from CRUSH (RX)
    -- CRASH RX data signals
    adc_i             : in    std_logic_vector(13 downto 0);  -- ADC data I, raw sample data
    adc_q             : in    std_logic_vector(13 downto 0);  -- ADC data Q, raw sample data
    adc_dc_off_i      : in    std_logic_vector(23 downto 0);  -- ADC data I, with DC offset correction & IQ balance
    adc_dc_off_q      : in    std_logic_vector(23 downto 0);  -- ADC data Q, with DC offset correction & IQ balance
    -- Following signals come from USRP firmware and are used when bypassing CRASH functionality
    dac_usrp_i        : in    std_logic_vector(15 downto 0);  -- DAC data I from USRP, raw sample data
    dac_usrp_q        : in    std_logic_vector(15 downto 0);  -- DAC data Q from USRP, raw sample data
    dac_usrp_dc_off_i : in    std_logic_vector(23 downto 0);  -- DAC data I from USRP, before DC offset correction & IQ balance
    dac_usrp_dc_off_q : in    std_logic_vector(23 downto 0);  -- DAC data Q from USRP, before DC offset correction & IQ balance
    -- CRASH TX data signals
    dac_i             : out   std_logic_vector(15 downto 0);  -- DAC data I, raw sample data
    dac_q             : out   std_logic_vector(15 downto 0);  -- DAC data Q, raw sample data
    dac_dc_off_i      : out   std_logic_vector(23 downto 0);  -- DAC data I, before DC offset correction & IQ balance
    dac_dc_off_q      : out   std_logic_vector(23 downto 0)); -- DAC data Q, before DC offset correction & IQ balance
end entity;

architecture RTL of crash_ddr_intf is

  -----------------------------------------------------------------------------
  -- Component Declaration
  -----------------------------------------------------------------------------
  component uart is
    generic (
      CLOCK_FREQ        : integer := 100e6;           -- Input clock frequency (Hz)
      BAUD              : integer := 1e6;             -- Baud rate (bits/sec)
      DATA_BITS         : integer := 8;               -- Number of data bits
      PARITY            : string  := "MARK";          -- EVEN, ODD, MARK (always = '1'), SPACE (always = '0'), NONE
      NO_STROBE_ON_ERR  : string  := "TRUE");         -- No rx_data_stb if error in received data.
    port (
      clk               : in    std_logic;            -- Clock
      reset             : in    std_logic;            -- Active high reset
      tx_busy           : out   std_logic;            -- Transmitting data
      tx_data_stb       : in    std_logic;            -- Transmit buffer load and begin transmission strobe
      tx_data           : in    std_logic_vector(DATA_BITS-1 downto 0);
      rx_busy           : out   std_logic;            -- Receiving data
      rx_data_stb       : out   std_logic;            -- Receive buffer data valid strobe
      rx_data           : out   std_logic_vector(DATA_BITS-1 downto 0);
      rx_error          : out   std_logic;            -- '1' = Invalid parity bit, start bit, or stop bit(s)
      tx                : out   std_logic;            -- TX output
      rx                : in    std_logic);           -- RX input
  end component;

  component fifo_16x15
    port (
      rst               : in    std_logic;
      wr_clk            : in    std_logic;
      rd_clk            : in    std_logic;
      din               : in    std_logic_vector(14 downto 0);
      wr_en             : in    std_logic;
      rd_en             : in    std_logic;
      dout              : out   std_logic_vector(14 downto 0);
      full              : out   std_logic;
      almost_full       : out   std_logic;
      empty             : out   std_logic;
      almost_empty      : out   std_logic);
  end component;

  component fifo_16x18
    port (
      rst               : in    std_logic;
      wr_clk            : in    std_logic;
      rd_clk            : in    std_logic;
      din               : in    std_logic_vector(17 downto 0);
      wr_en             : in    std_logic;
      rd_en             : in    std_logic;
      dout              : out   std_logic_vector(17 downto 0);
      full              : out   std_logic;
      almost_full       : out   std_logic;
      empty             : out   std_logic;
      almost_empty      : out   std_logic);
  end component;

  component trunc_unbiased
    generic (
      WIDTH_IN        : integer;                                                -- Input bit width
      TRUNCATE        : integer);                                               -- Number of bits to truncate
    port (
      i               : in    std_logic_vector(WIDTH_IN-1 downto 0);            -- Signed Input
      o               : out   std_logic_vector(WIDTH_IN-TRUNCATE-1 downto 0));  -- Truncated Signed Output
  end component;

  -----------------------------------------------------------------------------
  -- Constants Declaration
  -----------------------------------------------------------------------------
  -- Crash commands
  constant CMD_TX_MODE            : std_logic_vector(3 downto 0) := x"1";
  constant CMD_RX_MODE            : std_logic_vector(3 downto 0) := x"2";
  -- RX modes (lower nibble)
  constant RX_ADC_RAW_MODE        : std_logic_vector(3 downto 0) := x"0";
  constant RX_ADC_DC_OFF_MODE     : std_logic_vector(3 downto 0) := x"1";
  constant RX_TEST_SINE_MODE      : std_logic_vector(3 downto 0) := x"2";
  constant RX_TEST_PATTERN_MODE   : std_logic_vector(3 downto 0) := x"3";
  constant RX_ALL_1s_MODE         : std_logic_vector(3 downto 0) := x"4";
  constant RX_ALL_0s_MODE         : std_logic_vector(3 downto 0) := x"5";
  constant RX_I_1s_Q_0s_MODE      : std_logic_vector(3 downto 0) := x"6";
  constant RX_I_0s_Q_1s_MODE      : std_logic_vector(3 downto 0) := x"7";
  constant RX_CAL_INTF_MODE       : std_logic_vector(3 downto 0) := x"8";
  constant RX_TX_LOOPBACK_MODE    : std_logic_vector(3 downto 0) := x"9";
  -- TX modes (upper nibble)
  constant TX_PASSTHRU_MODE       : std_logic_vector(3 downto 0) := x"0";
  constant TX_DAC_RAW_MODE        : std_logic_vector(3 downto 0) := x"1";
  constant TX_DAC_DC_OFF_MODE     : std_logic_vector(3 downto 0) := x"2";
  constant TX_TEST_SINE_MODE      : std_logic_vector(3 downto 0) := x"3";

  -----------------------------------------------------------------------------
  -- Signals Declaration
  -----------------------------------------------------------------------------
  signal clk_3x                 : std_logic;
  signal clk_3x_180             : std_logic;
  signal clk_3x_dcm             : std_logic;
  signal clk_3x_180_dcm         : std_logic;
  signal dcm_locked             : std_logic;
  signal dcm_locked_n           : std_logic;

  signal crash_uart_rx_data     : std_logic_vector(7 downto 0);
  signal crash_uart_rx_data_stb : std_logic;
  signal crash_cmd              : std_logic_vector(3 downto 0);
  signal crash_data             : std_logic_vector(3 downto 0);
  signal crash_rx_mode          : std_logic_vector(3 downto 0);
  signal crash_tx_mode          : std_logic_vector(3 downto 0);

  signal adc_i_reg              : std_logic_vector(13 downto 0);
  signal adc_q_reg              : std_logic_vector(13 downto 0);
  signal adc_dc_off_i_reg       : std_logic_vector(23 downto 0);
  signal adc_dc_off_q_reg       : std_logic_vector(23 downto 0);
  signal adc_dc_off_i_trunc     : std_logic_vector(13 downto 0);
  signal adc_dc_off_q_trunc     : std_logic_vector(13 downto 0);

  signal dac_usrp_i_reg         : std_logic_vector(15 downto 0);
  signal dac_usrp_q_reg         : std_logic_vector(15 downto 0);
  signal dac_usrp_dc_off_i_reg  : std_logic_vector(23 downto 0);
  signal dac_usrp_dc_off_q_reg  : std_logic_vector(23 downto 0);

  signal tx_data_i              : std_logic_vector(15 downto 0);
  signal tx_data_q              : std_logic_vector(15 downto 0);
  signal tx_data_3x_i           : std_logic_vector(5 downto 0);
  signal tx_data_3x_q           : std_logic_vector(5 downto 0);
  signal tx_data_3x_stb         : std_logic;
  signal tx_data_3x_ddr         : std_logic_vector(5 downto 0);
  signal tx_data_3x_stb_ddr     : std_logic;
  signal tx_fifo_din_i          : std_logic_vector(17 downto 0);
  signal tx_fifo_din_q          : std_logic_vector(17 downto 0);
  signal tx_fifo_dout_i         : std_logic_vector(17 downto 0);
  signal tx_fifo_dout_q         : std_logic_vector(17 downto 0);
  signal tx_fifo_rd_en          : std_logic;
  signal tx_fifo_wr_en          : std_logic;
  signal tx_fifo_full           : std_logic;
  signal tx_fifo_empty          : std_logic;

  signal rx_data_i              : std_logic_vector(13 downto 0);
  signal rx_data_q              : std_logic_vector(13 downto 0);
  signal rx_data_3x_i           : std_logic_vector(4 downto 0);
  signal rx_data_3x_q           : std_logic_vector(4 downto 0);
  signal rx_data_3x_stb         : std_logic;
  signal rx_data_3x_ddr         : std_logic_vector(4 downto 0);
  signal rx_data_3x_stb_ddr     : std_logic;
  signal rx_fifo_din_i          : std_logic_vector(14 downto 0);
  signal rx_fifo_din_q          : std_logic_vector(14 downto 0);
  signal rx_fifo_dout_i         : std_logic_vector(14 downto 0);
  signal rx_fifo_dout_q         : std_logic_vector(14 downto 0);
  signal rx_fifo_rd_en          : std_logic;
  signal rx_fifo_wr_en          : std_logic;
  signal rx_fifo_full           : std_logic;
  signal rx_fifo_empty          : std_logic;
  signal rx_cnt                 : integer range 0 to 2;

  signal test_pattern_cnt       : integer range 0 to 5;
  signal test_pattern_i         : std_logic_vector(13 downto 0);
  signal test_pattern_q         : std_logic_vector(13 downto 0);
  signal sine_pattern_cnt       : integer range 0 to 3;
  signal sine_pattern_i         : std_logic_vector(15 downto 0);
  signal sine_pattern_q         : std_logic_vector(15 downto 0);

begin

  -- UART receives commands from crash, such as setting the TX / RX modes
  inst_uart : uart
    generic map (
      CLOCK_FREQ                => CLOCK_FREQ,
      BAUD                      => BAUD,
      DATA_BITS                 => 8,
      PARITY                    => "EVEN",
      NO_STROBE_ON_ERR          => "TRUE")
    port map (
      clk                       => clk,
      reset                     => reset,
      tx_busy                   => open,
      tx_data_stb               => '0',
      tx_data                   => x"00",
      rx_busy                   => open,
      rx_data_stb               => crash_uart_rx_data_stb,
      rx_data                   => crash_uart_rx_data,
      rx_error                  => open,
      tx                        => open,
      rx                        => UART_RX);

  crash_cmd                     <= crash_uart_rx_data(7 downto 4);
  crash_data                    <= crash_uart_rx_data(3 downto 0);

  -- Process commands from UART
  proc_crash_cmd : process(clk,reset)
  begin
    if rising_edge(clk) then
      if (reset = '1') then
        crash_rx_mode           <= (others=>'0');
        crash_tx_mode           <= (others=>'0');
      else
        -- Only update mode when valid.
        if (crash_uart_rx_data_stb = '1') then
          case (crash_cmd) is
            when (CMD_TX_MODE) =>
              crash_tx_mode     <= crash_data;
            when (CMD_RX_MODE) =>
              crash_rx_mode     <= crash_data;
            when others =>
          end case;
        end if;
      end if;
    end if;
  end process;

  -- Truncate to 14-bits
  inst_adc_i_trunc : trunc_unbiased
    generic map (
      WIDTH_IN                  => 24,
      TRUNCATE                  => 10)
    port map (
      i                         => adc_dc_off_i_reg,
      o                         => adc_dc_off_i_trunc);

  -- Truncate to 14-bits
  inst_adc_q_trunc : trunc_unbiased
    generic map (
      WIDTH_IN                  => 24,
      TRUNCATE                  => 10)
    port map (
      i                         => adc_dc_off_q_reg,
      o                         => adc_dc_off_q_trunc);

  -- Set receive mode via UART
  proc_rx_crash_mode : process(clk,reset)
  begin
    if rising_edge(clk) then
      if (reset = '1') then
        adc_i_reg               <= (others=>'0');
        adc_q_reg               <= (others=>'0');
        adc_dc_off_i_reg        <= (others=>'0');
        adc_dc_off_q_reg        <= (others=>'0');
        rx_data_i               <= (others=>'0');
        rx_data_q               <= (others=>'0');
      else
        -- Register inputs to improve timing
        adc_i_reg               <= adc_i;
        adc_q_reg               <= adc_q;
        adc_dc_off_i_reg        <= adc_dc_off_i;
        adc_dc_off_q_reg        <= adc_dc_off_q;

        -- CRUSH sets the mode through the UART interface
        -- Most modes are for testing.
        case (crash_rx_mode) is
          -- ADC data directly from the ADC interface
          when RX_ADC_RAW_MODE =>
            rx_data_i           <= adc_i_reg;
            rx_data_q           <= adc_q_reg;
          -- ADC data after DC offset correction and IQ balance in USRP firmware
          when RX_ADC_DC_OFF_MODE =>
            rx_data_i           <= adc_dc_off_i_trunc;
            rx_data_q           <= adc_dc_off_q_trunc;
          -- Test tone
          when RX_TEST_SINE_MODE =>
            rx_data_i           <= sine_pattern_i(15 downto 2);
            rx_data_q           <= sine_pattern_q(15 downto 2);
          when RX_TEST_PATTERN_MODE =>
            rx_data_i           <= test_pattern_i;
            rx_data_q           <= test_pattern_q;
          when RX_ALL_1s_MODE =>
            rx_data_i           <= "11111111111111";
            rx_data_q           <= "11111111111111";
          when RX_ALL_0s_MODE =>
            rx_data_i           <= "00000000000000";
            rx_data_q           <= "00000000000000";
          when RX_I_1s_Q_0s_MODE =>
            rx_data_i           <= "11111111111111";
            rx_data_q           <= "00000000000000";
          when RX_I_0s_Q_1s_MODE =>
            rx_data_i           <= "00000000000000";
            rx_data_q           <= "11111111111111";
          -- Loopback DAC data for debug
          when RX_TX_LOOPBACK_MODE =>
            rx_data_i           <= tx_data_i(13 downto 0);
            rx_data_q           <= tx_data_q(13 downto 0);
          when others =>
            rx_data_i           <= "01" & x"BAD";
            rx_data_q           <= "10" & x"BAD";
        end case;
      end if;
    end if;
  end process;

  -- Set transmit mode via UART
  proc_tx_crash_mode : process(clk,reset)
  begin
    if rising_edge(clk) then
      if (reset = '1') then
        dac_usrp_i_reg          <= (others=>'0');
        dac_usrp_q_reg          <= (others=>'0');
        dac_usrp_dc_off_i_reg   <= (others=>'0');
        dac_usrp_dc_off_q_reg   <= (others=>'0');
      else
        -- Register outputs to improve timing
        dac_usrp_i_reg          <= dac_usrp_i;
        dac_usrp_q_reg          <= dac_usrp_q;
        dac_usrp_dc_off_i_reg   <= dac_usrp_dc_off_i;
        dac_usrp_dc_off_q_reg   <= dac_usrp_dc_off_q;

        -- CRUSH sets the mode through the UART interface
        -- Most modes are for testing.
        case (crash_tx_mode) is
          -- Use data from host computer instead of CRASH
          when TX_PASSTHRU_MODE =>
            dac_i               <= dac_usrp_i_reg;
            dac_q               <= dac_usrp_q_reg;
            dac_dc_off_i        <= dac_usrp_dc_off_i_reg;
            dac_dc_off_q        <= dac_usrp_dc_off_q_reg;
          -- Directly drive DACs with CRASH
          when TX_DAC_RAW_MODE =>
            dac_i               <= tx_data_i;
            dac_q               <= tx_data_q;
            dac_dc_off_i        <= dac_usrp_dc_off_i_reg;
            dac_dc_off_q        <= dac_usrp_dc_off_q_reg;
          -- Pass CRASH DAC data through DC offset and IQ balance correction logic in USRP firmware
          when TX_DAC_DC_OFF_MODE =>
            dac_i               <= dac_usrp_i_reg;
            dac_q               <= dac_usrp_q_reg;
            dac_dc_off_i        <= tx_data_i & x"00";
            dac_dc_off_q        <= tx_data_q & x"00";
          -- Directly drive DACs with test tone
          when TX_TEST_SINE_MODE =>
            dac_i               <= sine_pattern_i;
            dac_q               <= sine_pattern_q;
            dac_dc_off_i        <= dac_usrp_dc_off_i_reg;
            dac_dc_off_q        <= dac_usrp_dc_off_q_reg;
          when others =>
            dac_i               <= dac_usrp_i_reg;
            dac_q               <= dac_usrp_q_reg;
            dac_dc_off_i        <= dac_usrp_dc_off_i_reg;
            dac_dc_off_q        <= dac_usrp_dc_off_q_reg;
        end case;
      end if;
    end if;
  end process;

  -- Test patterns for debugging receive and transmit
  -- interfaces
  proc_gen_test_patterns : process(clk,reset)
  begin
    if rising_edge(clk) then
      if (reset = '1') then
        test_pattern_i          <= (others=>'0');
        test_pattern_q          <= (others=>'0');
        test_pattern_cnt        <= 0;
        sine_pattern_i          <= (others=>'0');
        sine_pattern_q          <= (others=>'0');
        sine_pattern_cnt        <= 0;
      else
        -- Generate test pattern
        if (test_pattern_cnt = 1) then
          test_pattern_i        <= "01" & x"A1B";
          test_pattern_q        <= "10" & x"A2B";
          test_pattern_cnt      <= 0;
        else
          test_pattern_i        <= "01" & x"C1D";
          test_pattern_q        <= "10" & x"C2D";
          test_pattern_cnt      <= 1;
        end if;
        -- Generate sine test pattern (Fs/4) = 25MHz tone
        if (sine_pattern_cnt = 3) then
          sine_pattern_cnt      <= 0;
        else
          sine_pattern_cnt      <= sine_pattern_cnt + 1;
        end if;
        case (sine_pattern_cnt) is
          when 0 =>
            sine_pattern_i      <= x"7FFF";
            sine_pattern_q      <= x"0000";
          when 1 =>
            sine_pattern_i      <= x"0000";
            sine_pattern_q      <= x"7FFF";
          when 2 =>
            sine_pattern_i      <= x"8000";
            sine_pattern_q      <= x"0000";
          when 3 =>
            sine_pattern_i      <= x"0000";
            sine_pattern_q      <= x"8000";
          when others =>
            sine_pattern_i      <= x"7FFF";
            sine_pattern_q      <= x"0000";
        end case;
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- DCM to create 3x clocks
  -----------------------------------------------------------------------------
  inst_ddr_intf_DCM : DCM_SP
    generic map (
      CLKDV_DIVIDE              => 2.0,
      CLKFX_DIVIDE              => 1,
      CLKFX_MULTIPLY            => 3,
      CLKIN_DIVIDE_BY_2         => FALSE,
      CLKIN_PERIOD              => 10.0,
      CLKOUT_PHASE_SHIFT        => "NONE",
      CLK_FEEDBACK              => "NONE", -- When only using FX output we do not need feedback, see DCM documentation
      DESKEW_ADJUST             => "SYSTEM_SYNCHRONOUS",
      DLL_FREQUENCY_MODE        => "LOW",
      DUTY_CYCLE_CORRECTION     => TRUE,
      PHASE_SHIFT               => 0,
      STARTUP_WAIT              => FALSE)
    port map (
      CLK0                      => open,
      CLK180                    => open,
      CLK270                    => open,
      CLK2X                     => open,
      CLK2X180                  => open,
      CLK90                     => open,
      CLKDV                     => open,
      CLKFX                     => clk_3x_dcm,
      CLKFX180                  => clk_3x_180_dcm,
      LOCKED                    => dcm_locked,
      PSDONE                    => open,
      STATUS                    => open,
      CLKFB                     => open,
      CLKIN                     => clk,
      PSCLK                     => '0',
      PSEN                      => '0',
      PSINCDEC                  => '0',
      RST                       => reset);

  dcm_locked_n                  <= NOT(dcm_locked);

  inst_clk2x_BUFG : BUFG
    port map (
      I                         => clk_3x_dcm,
      O                         => clk_3x);

  inst_clk2x_180_BUFG : BUFG
    port map (
      I                         => clk_3x_180_dcm,
      O                         => clk_3x_180);

  -----------------------------------------------------------------------------
  -- LVDS DDR TX Data Interface, 3x Clock Domain (300 MHz)
  -- Receive 6-bit TX interleaved I/Q data at 300 MHz DDR and
  -- output 16-bit TX I & Q samples
  -----------------------------------------------------------------------------
  -- DDR LVDS Data Input
  gen_tx_ddr_lvds : for i in 0 to 5 generate
    inst_IDDR2 : IDDR2
      generic map (
        DDR_ALIGNMENT           => "C0",
        INIT_Q0                 => '0',
        INIT_Q1                 => '0',
        SRTYPE                  => "ASYNC")
      port map (
        D                       => tx_data_3x_ddr(i),
        C0                      => clk_3x,
        C1                      => clk_3x_180,
        CE                      => '1',
        Q0                      => tx_data_3x_i(i),
        Q1                      => tx_data_3x_q(i),
        R                       => dcm_locked_n,
        S                       => '0');

    inst_IBUFDS : IBUFDS
      generic map (
        DIFF_TERM               => TRUE,
        IOSTANDARD              => "DEFAULT")
      port map (
        I                       => TX_DATA_P(i),
        IB                      => TX_DATA_N(i),
        O                       => tx_data_3x_ddr(i));
  end generate;

  -- Strobe signal asserts on the last word
  inst_tx_stb_IDDR2 : IDDR2
  generic map (
    DDR_ALIGNMENT           => "C0",
    INIT_Q0                 => '0',
    INIT_Q1                 => '0',
    SRTYPE                  => "ASYNC")
  port map (
    D                       => tx_data_3x_stb_ddr,
    C0                      => clk_3x,
    C1                      => clk_3x_180,
    CE                      => '1',
    Q0                      => open,
    Q1                      => tx_data_3x_stb,
    R                       => dcm_locked_n,
    S                       => '0');

  inst_tx_stb_IBUFDS : IBUFDS
  generic map (
    DIFF_TERM               => TRUE,
    IOSTANDARD              => "DEFAULT")
  port map (
    I                       => TX_DATA_STB_P,
    IB                      => TX_DATA_STB_N,
    O                       => tx_data_3x_stb_ddr);

  proc_gen_tx_data : process(clk_3x,dcm_locked_n)
  begin
    if (dcm_locked_n = '1') then
      tx_fifo_wr_en                   <= '0';
      tx_fifo_din_i                   <= (others=>'0');
      tx_fifo_din_q                   <= (others=>'0');
    else
      if rising_edge(clk_3x) then
        tx_fifo_wr_en                 <= tx_data_3x_stb;
        tx_fifo_din_i(5 downto 0)     <= tx_data_3x_i;
        tx_fifo_din_i(11 downto 6)    <= tx_fifo_din_i(5 downto 0);
        tx_fifo_din_i(17 downto 12)   <= tx_fifo_din_i(11 downto 6);
        tx_fifo_din_q(5 downto 0)     <= tx_data_3x_q;
        tx_fifo_din_q(11 downto 6)    <= tx_fifo_din_q(5 downto 0);
        tx_fifo_din_q(17 downto 12)   <= tx_fifo_din_q(11 downto 6);
      end if;
    end if;
  end process;

  inst_tx_fifo_i : fifo_16x18
    port map (
      rst                       => dcm_locked_n,
      wr_clk                    => clk_3x,
      rd_clk                    => clk,
      din                       => tx_fifo_din_i,
      wr_en                     => tx_fifo_wr_en,
      rd_en                     => tx_fifo_rd_en,
      dout                      => tx_fifo_dout_i,
      full                      => tx_fifo_full,
      almost_full               => open,
      empty                     => tx_fifo_empty,
      almost_empty              => open);

  inst_tx_fifo_q : fifo_16x18
    port map (
      rst                       => dcm_locked_n,
      wr_clk                    => clk_3x,
      rd_clk                    => clk,
      din                       => tx_fifo_din_q,
      wr_en                     => tx_fifo_wr_en,
      rd_en                     => tx_fifo_rd_en,
      dout                      => tx_fifo_dout_q,
      full                      => open,
      almost_full               => open,
      empty                     => open,
      almost_empty              => open);

  tx_fifo_rd_en                 <= NOT(tx_fifo_empty);
  tx_data_i                     <= tx_fifo_dout_i(17 downto 2); -- Sample data is only 16-bit wide
  tx_data_q                     <= tx_fifo_dout_q(17 downto 2);

  -----------------------------------------------------------------------------
  -- LVDS DDR RX Data Interface, 3x Clock Domain (300 MHz)
  -- Transmit 14-bit RX I & Q samples as 5-bit interleaved I/Q data at
  -- 300 MHz DDR
  -----------------------------------------------------------------------------
  inst_rx_fifo_i : fifo_16x15
    port map (
      rst                       => dcm_locked_n,
      wr_clk                    => clk,
      rd_clk                    => clk_3x,
      din                       => rx_fifo_din_i,
      wr_en                     => rx_fifo_wr_en,
      rd_en                     => rx_fifo_rd_en,
      dout                      => rx_fifo_dout_i,
      full                      => rx_fifo_full,
      almost_full               => open,
      empty                     => rx_fifo_empty,
      almost_empty              => open);

  inst_rx_fifo_q : fifo_16x15
    port map (
      rst                       => dcm_locked_n,
      wr_clk                    => clk,
      rd_clk                    => clk_3x,
      din                       => rx_fifo_din_q,
      wr_en                     => rx_fifo_wr_en,
      rd_en                     => rx_fifo_rd_en,
      dout                      => rx_fifo_dout_q,
      full                      => open,
      almost_full               => open,
      empty                     => open,
      almost_empty              => open);

  rx_fifo_wr_en                 <= NOT(rx_fifo_full);
  rx_fifo_rd_en                 <= rx_data_3x_stb;
  rx_fifo_din_i                 <= rx_data_i & '0';
  rx_fifo_din_q                 <= rx_data_q & '0';

  proc_gen_rx_data : process(clk_3x,dcm_locked_n)
  begin
    if (dcm_locked_n = '1') then
      rx_cnt                    <= 0;
      rx_data_3x_stb            <= '0';
      rx_data_3x_i              <= (others=>'0');
      rx_data_3x_q              <= (others=>'0');
    else
      if rising_edge(clk_3x) then
        case rx_cnt is
          when 0 =>
            rx_cnt              <= rx_cnt + 1;
            rx_data_3x_stb      <= '0';
            rx_data_3x_i        <= rx_fifo_dout_i(14 downto 10);
            rx_data_3x_q        <= rx_fifo_dout_q(14 downto 10);
          when 1 =>
            rx_cnt              <= rx_cnt + 1;
            rx_data_3x_stb      <= '0';
            rx_data_3x_i        <= rx_fifo_dout_i(9 downto 5);
            rx_data_3x_q        <= rx_fifo_dout_q(9 downto 5);
          when 2 =>
            rx_cnt              <= 0;
            rx_data_3x_stb      <= NOT(rx_fifo_empty);
            rx_data_3x_i        <= rx_fifo_dout_i(4 downto 0);
            rx_data_3x_q        <= rx_fifo_dout_q(4 downto 0);
          when others =>
        end case;
      end if;
    end if;
  end process;

  -- DDR RX LVDS Data Output
  gen_rx_ddr_lvds : for i in 0 to 4 generate
    inst_ODDR2 : ODDR2
      generic map (
        DDR_ALIGNMENT           => "C0",
        INIT                    => '0',
        SRTYPE                  => "ASYNC")
      port map (
        Q                       => rx_data_3x_ddr(i),
        C0                      => clk_3x,
        C1                      => clk_3x_180,
        CE                      => '1',
        D0                      => rx_data_3x_i(i),
        D1                      => rx_data_3x_q(i),
        R                       => dcm_locked_n,
        S                       => '0');

    inst_OBUFDS : OBUFDS
      generic map (
        IOSTANDARD              => "DEFAULT")
      port map (
        I                       => rx_data_3x_ddr(i),
        O                       => RX_DATA_P(i),
        OB                      => RX_DATA_N(i));
  end generate;

  inst_rx_stb_ODDR2 : ODDR2
    generic map (
      DDR_ALIGNMENT           => "C0",
      INIT                    => '0',
      SRTYPE                  => "ASYNC")
    port map (
      Q                       => rx_data_3x_stb_ddr,
      C0                      => clk_3x,
      C1                      => clk_3x_180,
      CE                      => '1',
      D0                      => '0',
      D1                      => rx_data_3x_stb,
      R                       => dcm_locked_n,
      S                       => '0');

  inst_rx_stb_OBUFDS : OBUFDS
    generic map (
      IOSTANDARD              => "DEFAULT")
    port map (
      I                       => rx_data_3x_stb_ddr,
      O                       => RX_DATA_STB_P,
      OB                      => RX_DATA_STB_N);

  inst_OBUFDS : OBUFDS
    generic map (
      IOSTANDARD                => "DEFAULT")
    port map (
      I                         => clk_3x,
      O                         => RX_DATA_CLK_P,
      OB                        => RX_DATA_CLK_N);

end architecture;
