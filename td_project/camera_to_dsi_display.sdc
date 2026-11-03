create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 4.167 -waveform {0.000 2.083} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]


set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]


# ---- Ethernet RGMII timing (from milianke example, adapted names) ----
create_clock -name {eth_rgmii_rxc} -period 8.000 -waveform {0.000 4.000} [get_ports {eth_rx_clk}]
set_input_delay -clock [get_clocks {eth_rgmii_rxc}] -max 2.500 [get_ports {{eth_rxd[*]} eth_rx_dv}]
set_input_delay -clock [get_clocks {eth_rgmii_rxc}] -min 1.500 [get_ports {{eth_rxd[*]} eth_rx_dv}]
set_input_delay -clock [get_clocks {eth_rgmii_rxc}] -max -clock_fall -add_delay 2.500 [get_ports {{eth_rxd[*]} eth_rx_dv}]
set_input_delay -clock [get_clocks {eth_rgmii_rxc}] -min -clock_fall -add_delay 1.500 [get_ports {{eth_rxd[*]} eth_rx_dv}]

rename_clock -name {eth_a_tx_clk} [get_clocks {u_eth_pll/ph1p_phy_pll_wrapper_d2616deaaa79_Inst/u_PH1P_PHY_PLL.clkc[0]}]
create_generated_clock -name {eth_txc} -source [get_pins {u_eth_pll/ph1p_phy_pll_wrapper_d2616deaaa79_Inst/u_PH1P_PHY_PLL.clkc[0]}] -master_clock {eth_a_tx_clk} -multiply_by 1.000 -phase 0.000 [get_ports {eth_tx_clk}]

set_output_delay -clock [get_clocks eth_txc] -max 2.800 [get_ports {eth_tx_en {eth_txd[*]}}]
set_output_delay -clock [get_clocks eth_txc] -min 1.200 [get_ports {eth_tx_en {eth_txd[*]}}]
set_output_delay -clock [get_clocks eth_txc] -clock_fall -max -add_delay 2.800 [get_ports {eth_tx_en {eth_txd[*]}}]
set_output_delay -clock [get_clocks eth_txc] -clock_fall -min -add_delay 1.200 [get_ports {eth_tx_en {eth_txd[*]}}]

set_clock_groups -exclusive -group [get_clocks {eth_rgmii_rxc}] -group [get_clocks {eth_txc}]
