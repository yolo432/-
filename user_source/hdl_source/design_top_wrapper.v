

module design_top_wrapper (
    input wire        I_sys_clk,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
    output wire       O_buzzer,
    output wire       O_gimbal_pan,
    output wire       O_gimbal_tilt,
      
    input wire [3:0]  I_button,

    // ---- 以太网（YT8511, RGMII 百兆模式）----
    output wire       eth_rst_n,     // V8  PHY 复位，低有效
    output wire       eth_mdc,       // V9  （未用 MDIO，恒 0）
    input  wire       eth_mdio,      // U10 （只读悬置）
    output wire       eth_tx_clk,    // U9  RGMII TXC（FPGA 驱动 25M）
    output wire       eth_tx_en,     // R10 RGMII TX_CTL
    output wire [3:0] eth_txd,       // R11/P11/T10/R12
    input  wire       eth_rx_clk,    // V3  RGMII RXC（PHY 驱动）
    input  wire       eth_rx_dv,     // N8  RGMII RX_CTL
    input  wire [3:0] eth_rxd,       // N9/T7/T8/T9
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
    wire            S_motion_alarm;
    wire            S_track_update, S_track_valid;
    wire [8:0]      S_track_x;
    wire [7:0]      S_track_y;
    wire            S_frame_write, S_frame_select;
    wire [1:0]      S_frame_write_slot, S_frame_read_slot;
    wire [15:0]     S_latency_ms;
    wire            S_latency_valid;
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    // 720p input-format diagnostics.  One RAW10 line from the factory
    // 1280-pixel mode contains 1600 bytes, i.e. 400 valid 32-bit CSI words.
    reg         S_dbg_hs_valid_d;
    reg [10:0]  S_dbg_csi_words;
    reg [10:0]  S_dbg_good_lines;
    reg         S_dbg_frame_seen_once;
    reg         S_dbg_frame_repeat_seen;
    reg         S_dbg_width_1280_seen;
    reg         S_dbg_height_720_seen;
    reg         S_dbg_isp_frame_seen_once;
    reg         S_dbg_isp_frame_repeat_seen;
    reg [1:0]   S_dbg_frame_repeat_sync;
    reg [1:0]   S_dbg_width_1280_sync;
    reg [1:0]   S_dbg_height_720_sync;
    reg [1:0]   S_dbg_isp_frame_repeat_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;


    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    // Count CSI payload words between HS packet boundaries.  This avoids
    // probing an internal net of csi_unpacket_2lane (which ChipWatcher cannot
    // legally connect to) and verifies the received line length externally.
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) begin
            S_dbg_hs_valid_d          <= 1'b0;
            S_dbg_csi_words           <= 11'd0;
            S_dbg_good_lines          <= 11'd0;
            S_dbg_frame_seen_once     <= 1'b0;
            S_dbg_frame_repeat_seen   <= 1'b0;
            S_dbg_width_1280_seen     <= 1'b0;
            S_dbg_height_720_seen     <= 1'b0;
            S_dbg_isp_frame_seen_once <= 1'b0;
            S_dbg_isp_frame_repeat_seen <= 1'b0;
        end
        else begin
            S_dbg_hs_valid_d <= S_hs_rx_valid;

            if(!S_dbg_hs_valid_d && S_hs_rx_valid)
                S_dbg_csi_words <= 11'd0;
            else if(S_csi_valid)
                S_dbg_csi_words <= S_dbg_csi_words + 11'd1;

            // A packet with CSI payload is an image line.  Ignore short
            // frame-start/frame-end packets, whose payload count is zero.
            if(S_dbg_hs_valid_d && !S_hs_rx_valid && (S_dbg_csi_words != 0)) begin
                if(S_dbg_csi_words == 11'd400) begin
                    S_dbg_width_1280_seen <= 1'b1;
                    S_dbg_good_lines <= S_dbg_good_lines + 11'd1;
                end
            end

            if(S_csi_frame_start) begin
                if(S_dbg_frame_seen_once) begin
                    S_dbg_frame_repeat_seen <= 1'b1;
                    if(S_dbg_good_lines == 11'd720)
                        S_dbg_height_720_seen <= 1'b1;
                end
                else
                    S_dbg_frame_seen_once <= 1'b1;
                S_dbg_good_lines <= 11'd0;
            end

            if(S_ISP_O_tuser) begin
                if(S_dbg_isp_frame_seen_once)
                    S_dbg_isp_frame_repeat_seen <= 1'b1;
                else
                    S_dbg_isp_frame_seen_once <= 1'b1;
            end
        end
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
            S_dbg_frame_repeat_sync <= 2'b00;
            S_dbg_width_1280_sync <= 2'b00;
            S_dbg_height_720_sync <= 2'b00;
            S_dbg_isp_frame_repeat_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            S_dbg_frame_repeat_sync <= {S_dbg_frame_repeat_sync[0], S_dbg_frame_repeat_seen};
            S_dbg_width_1280_sync <= {S_dbg_width_1280_sync[0], S_dbg_width_1280_seen};
            S_dbg_height_720_sync <= {S_dbg_height_720_sync[0], S_dbg_height_720_seen};
            S_dbg_isp_frame_repeat_sync <= {S_dbg_isp_frame_repeat_sync[0], S_dbg_isp_frame_repeat_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    // On-screen bring-up status, from left to right in DEBUG_MODE=3:
    // I2C configuration completed, MIPI HS activity seen, repeated CSI frames,
    // and a 1280-pixel RAW10 line (400 x 32-bit CSI payload words) seen.
    assign S_hdmi_debug_status = {
        S_dbg_width_1280_sync[1],
        S_dbg_frame_repeat_sync[1],
        S_dbg_hs_seen_sync[1],
        S_dbg_cam_cfg_seen_sync[1]
    };
	
	assign O_cam_rst 	= 1'b1;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( 1'b0              ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn(4'b1111),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );
  
    uicfgcs500_720p #(
      .CLK_DIV(24000000 / 100000 - 1)
  ) u_uicfgcs500 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_rst_n),  //系统复位输入
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done)  //AE配置完成
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "0dB"   ),//"0dB","1.5dB","3dB","4.5dB","6dB","7.5dB","9dB","10.5dB"
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// SC500 factory HD mode already outputs native 1280x720 RAW10.
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_button        (I_button),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.O_motion_alarm  (S_motion_alarm),
.O_track_update  (S_track_update),
.O_track_valid   (S_track_valid),
.O_track_x       (S_track_x),
.O_track_y       (S_track_y),
.I_tready        ()
  );

gimbal_servo_pwm u_gimbal (
    .clk(I_sys_clk), .src_clk(S_csi_rx_clk), .rst_n(S_rst_n),
    .target_update(S_track_update), .target_valid(S_track_valid),
    .target_x(S_track_x), .target_y(S_track_y),
    .pan_pwm(O_gimbal_pan), .tilt_pwm(O_gimbal_tilt)
);

motion_buzzer #(
    .CLOCK_HZ(60000000),
    .TONE_HZ(500),
    .ALARM_FRAMES(18)
) u_motion_buzzer (
    .clk(S_csi_rx_clk),
    .rst_n(S_rst_n),
    // The packer forwards SOF with valid low on its first input word.
    .frame_start(S_ISP_O_tuser),
    .motion_alarm(S_motion_alarm),
    .buzzer(O_buzzer)
);


// ================= 以太网子系统（UDP 心跳点亮，2026-10-09） =================
// 架构：eth_pll(125M/125M_90/25M) → uiudp_stack(用户口25M) → rgmii_interface(百兆)
//       → YT8511 PHY；uiphyrst 产生 PHY 复位；eth_heartbeat 每秒发 HBEAT 包
wire        eth_pll_lock;
wire        eth_clk_125, eth_clk_125_90, eth_clk_25;
wire [7:0]  eth_gmii_tdata, eth_gmii_rdata;
wire        eth_gmii_tvalid, eth_gmii_rvalid, eth_gmii_rclk;

wire        eth_udp_busy;
wire        eth_hb_req, eth_hb_valid;
wire [7:0]  eth_hb_data;
wire [15:0] eth_hb_len;

pll u_eth_pll(
    .refclk   ( I_sys_clk      ),
    .reset    ( 1'b0           ),
    .lock     ( eth_pll_lock   ),
    .clk0_out ( eth_clk_125    ),
    .clk1_out ( eth_clk_125_90 ),
    .clk2_out ( eth_clk_25     )
);

wire eth_phy_rst_n_r;
uiphyrst #(.CLK_FREQ(32'd25_000_000)) u_uiphyrst(
    .I_CLK   ( eth_clk_25    ),
    .I_rstn  ( eth_pll_lock  ),
    .I_phyrst( eth_pll_lock  ),
    .O_phyrst( eth_phy_rst_n_r )
);
assign eth_rst_n = eth_phy_rst_n_r;
assign eth_mdc   = 1'b0;               // 不配置 MDIO，PHY 自动协商

uiudp_stack #(
    .CRC_GEN_EN        (1'b1),
    .INTER_FRAME_GAP   (4'd12)
) u_uiudp_stack (
    .I_uclk            ( eth_clk_125                 ),
    .I_reset           ( ~eth_phy_rst_n_r            ),
    .I_mac_local_addr  ( 48'h0123456789ab            ),
    .I_udp_local_port  ( 16'd6002                    ),
    .I_ip_local_addr   ( 32'hc0a88902                ), // 192.168.137.2
    .I_udp_dest_port   ( 16'd6001                    ),
    .I_ip_dest_addr    ( 32'hc0a88901                ), // 192.168.137.1
    .O_W_udp_busy      ( eth_udp_busy                ),
    .I_W_udp_req       ( eth_hb_req                  ),
    .I_W_udp_valid     ( eth_hb_valid                ),
    .I_W_udp_data      ( eth_hb_data                 ),
    .I_W_udp_len       ( eth_hb_len                  ),
    .O_R_udp_valid     (                             ),
    .O_R_udp_data      (                             ),
    .O_R_udp_len       (                             ),
    .O_R_udp_src_port  (                             ),
    .I_gmii_rclk       ( eth_gmii_rclk               ),
    .I_gmii_rvalid     ( eth_gmii_rvalid             ),
    .I_gmii_rdata      ( eth_gmii_rdata              ),
    .I_gmii_tclk       ( eth_clk_125                 ),
    .O_gmii_tvalid     ( eth_gmii_tvalid             ),
    .O_gmii_tdata      ( eth_gmii_tdata              ),
    .O_ip_rerror       (                             ),
    .O_mac_rerror      (                             )
);

rgmii_interface u_rgmii_interface(
    .speed_10_100  ( 1'b0            ),   // 千兆模式（YT8511 实为千兆PHY，链路协商1G实测）
    .gmii_tx_clk_d ( eth_clk_125     ),
    .gmii_tx_reset_d( ~eth_phy_rst_n_r ),
    .gmii_tx_reset ( ~eth_phy_rst_n_r ),
    .gmii_tx_clk   ( eth_clk_125     ),
    .gmii_txd      ( eth_gmii_tdata  ),
    .gmii_tx_en    ( eth_gmii_tvalid ),
    .gmii_tx_er    (                 ),
    .gmii_rx_reset ( ~eth_phy_rst_n_r ),
    .gmii_rx_clk   ( eth_gmii_rclk   ),
    .gmii_rxd      ( eth_gmii_rdata  ),
    .gmii_rx_dv    ( eth_gmii_rvalid ),
    .gmii_rx_er    (                 ),
    .rgmii_txd     ( eth_txd         ),
    .rgmii_tx_ctl  ( eth_tx_en       ),
    .rgmii_txc     ( eth_tx_clk      ),
    .rgmii_rxd     ( eth_rxd         ),
    .rgmii_rx_ctl  ( eth_rx_dv       ),
    .rgmii_rxc     ( eth_rx_clk      ),
    .gmii_crs      (                 ),
    .gmii_col      (                 ),
    .link_status   (                 ),
    .clock_speed   (                 ),
    .duplex_status (                 )
);

// Stable network path: sample the already-rendered HDMI RGB stream into a
// small on-chip frame mailbox. The experimental DDR snapshot path is disabled.
camera_udp_preview #(
    .WIDTH      (320),
    .HEIGHT     (180),
    .STEP       (4),
    .INPUT_WIDTH(1280),
    .FRAME_SKIP (5),
    .GAP_CYCLES (12500),
    .BOX_FILTER (0)
) u_camera_udp_preview(
    .I_pixel_clk(S_hdmi_pixel_clk),
    .I_vsync    (S_hdmi_out_vsync),
    .I_de       (S_hdmi_out_de),
    .I_rgb      (S_hdmi_out_data),
    .I_clk      (eth_clk_125),
    .I_reset    (~S_rst_n | ~eth_phy_rst_n_r),
    .I_udp_busy (eth_udp_busy),
    .O_udp_req  (eth_hb_req),
    .O_udp_valid(eth_hb_valid),
    .O_udp_len  (eth_hb_len),
    .O_udp_data (eth_hb_data)
);

wire old_mem_ready, old_mem_valid;

// ---- M2 插入点验证：debug_insert（直通 + 绿框）----
wire [127:0] S_ins_tdata;
wire         S_ins_tvalid, S_ins_tlast, S_ins_tuser;

debug_insert u_debug_insert (
    .I_clk    ( S_csi_rx_clk     ),
    .I_rst_n  ( S_rst_n          ),
    .I_tdata  ( S_ISP_O_tdata    ),
    .I_tvalid ( S_ISP_O_tvalid   ),
    .I_tlast  ( S_ISP_O_tlast    ),
    .I_tuser  ( S_ISP_O_tuser    ),
    .O_tdata  ( S_ins_tdata      ),
    .O_tvalid ( S_ins_tvalid     ),
    .O_tlast  ( S_ins_tlast      ),
    .O_tuser  ( S_ins_tuser      )
);

    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ins_tuser   ),
        .I_camera_valid       ( S_ins_tvalid  ),
        .I_camera_data        ( S_ins_tdata   ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),
        .O_frame_write       ( S_frame_write         ),
        .O_frame_slot        ( S_frame_write_slot    ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );


    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;
    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;
    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;
    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;
    assign old_mem_ready       = S_ddr_user_ready;
    assign old_mem_valid       = S_ddr_user_rd_valid;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
        .O_frame_select     ( S_frame_select       ),
        .O_frame_slot       ( S_frame_read_slot    ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready      ),
        .I_ddr_user_rd_valid ( old_mem_valid        ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );


    video_frame_latency u_video_frame_latency (
        .rst_n(S_rst_n),
        .time_clk(I_sys_clk),
        .camera_clk(S_csi_rx_clk),
        .capture_start(S_csi_frame_start),
        .ddr_clk(S_ddr_clk),
        .frame_write(S_frame_write),
        .write_slot(S_frame_write_slot),
        .frame_select(S_frame_select),
        .read_slot(S_frame_read_slot),
        .display_clk(S_hdmi_pixel_clk),
        .display_rst_n(S_hdmi_rst_n),
        .display_vsync(S_hdmi_vsync),
        .display_start(S_hdmi_user && S_hdmi_de),
        .latency_ms(S_latency_ms),
        .latency_valid(S_latency_valid)
    );


    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    ) // Normal SC500 1280x720 camera image
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_latency_ms    ( S_latency_ms        ),
        .I_latency_valid ( S_latency_valid     ),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_video_rd_data    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule

module motion_buzzer #(
    parameter integer CLOCK_HZ = 60000000,
    parameter integer TONE_HZ = 500,
    parameter integer ALARM_FRAMES = 18
) (
    input wire clk, input wire rst_n, input wire frame_start,
    input wire motion_alarm, output reg buzzer
);
    localparam integer HALF_PERIOD = CLOCK_HZ / (TONE_HZ * 2);
    localparam integer DIV_WIDTH = (HALF_PERIOD <= 2) ? 1 : $clog2(HALF_PERIOD);
    localparam integer FRAME_WIDTH = (ALARM_FRAMES <= 2) ? 1 : $clog2(ALARM_FRAMES + 1);
    reg [DIV_WIDTH-1:0] tone_count;
    reg [FRAME_WIDTH-1:0] frame_count;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            buzzer <= 1'b1; tone_count <= 0; frame_count <= 0;
        end else begin
            if(motion_alarm) frame_count <= ALARM_FRAMES;
            else if(frame_start && frame_count != 0)
                frame_count <= frame_count - 1'b1;
            if(frame_count != 0 || motion_alarm) begin
                if(tone_count >= HALF_PERIOD-1) begin
                    tone_count <= 0; buzzer <= ~buzzer;
                end else tone_count <= tone_count + 1'b1;
            end else begin
                // C12 drives an active-low PNP stage; high is the idle level.
                tone_count <= 0; buzzer <= 1'b1;
            end
        end
    end
endmodule

// ============================================================
// M2 插入点验证：debug_insert（直通 + 绿框）
// 完整源码在 cart2026/rtl/ctrl/debug_insert.v（此为并入副本）
// ============================================================
module debug_insert #(
    parameter BEATS_PER_LINE = 320,
    parameter LINES          = 720,
    parameter BORDER_X        = 16,            // 左右边框宽（拍）：1 拍=4px
    parameter BORDER_Y        = 40,            // 上下边框高（行）
    parameter [31:0] GREEN   = 32'h0000FF00
)(
    input  wire         I_clk,
    input  wire         I_rst_n,
    input  wire [127:0] I_tdata,
    input  wire         I_tvalid,
    input  wire         I_tlast,
    input  wire         I_tuser,
    output wire [127:0] O_tdata,
    output wire         O_tvalid,
    output wire         O_tlast,
    output wire         O_tuser
);
    reg [127:0] d_tdata;
    reg         d_tvalid, d_tlast, d_tuser;
    reg [8:0]  x_reg;
    reg [9:0]  y_reg;
    reg        prev_line_end;

    wire [8:0] x_cur = d_tuser ? 9'd0
                     : (x_reg == BEATS_PER_LINE - 1) ? 9'd0 : x_reg + 1'b1;
    wire line_end = (x_cur == BEATS_PER_LINE - 1) && d_tlast;
    wire [9:0] y_cur = d_tuser ? 10'd0
                     : (prev_line_end ? ((y_reg == LINES - 1) ? 10'd0 : y_reg + 1'b1)
                                      : y_reg);
    wire in_border = (y_cur <= BORDER_Y) || (y_cur > LINES - BORDER_Y)            // 顶部/底部
                  || ((y_cur > LINES/2 - 10) && (y_cur <= LINES/2 + 10));      // 中央横带
    // 注：x 方向左右边框在显示端不可见（疑似裁边），已弃用；改用中央横带验证注入

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            d_tdata  <= 128'd0;
            d_tvalid <= 1'b0;
            d_tlast  <= 1'b0;
            d_tuser  <= 1'b0;
            x_reg    <= BEATS_PER_LINE - 1;
            y_reg    <= LINES - 1;
            prev_line_end <= 1'b0;
        end else begin
            d_tdata  <= I_tdata;
            d_tvalid <= I_tvalid;
            d_tlast  <= I_tlast;
            d_tuser  <= I_tuser;
            if (d_tvalid) begin
                x_reg <= x_cur;
                y_reg <= y_cur;
                prev_line_end <= line_end;
            end
        end
    end

    // 全屏绿测试（悬案 H1/H3 判定实验）：有数据流 = 整屏纯绿
    // 实验后恢复上一行注释的原版判定
    assign O_tdata  = d_tdata;
    // assign O_tdata  = (d_tvalid && in_border) ? {4{GREEN}} : d_tdata;
    assign O_tvalid = d_tvalid;
    assign O_tlast  = d_tlast;
    assign O_tuser  = d_tuser;
endmodule


// ============================================================
// eth_heartbeat（并入副本，规范源在 cart2026/rtl/ctrl/eth_heartbeat.v）
// ============================================================
module eth_heartbeat #(
    parameter CLK_HZ   = 125_000_000,
    parameter PAYLOAD  = 8
)(
    input  wire         I_clk,
    input  wire         I_reset,
    output reg          O_udp_req,
    output reg  [15:0]  O_udp_len,
    output reg          O_udp_valid,
    output reg  [7:0]   O_udp_data,
    input  wire         I_udp_busy
);
    localparam CNT_MAX = CLK_HZ - 1;
    localparam ST_WAIT = 2'd0, ST_PULSE = 2'd1, ST_BUSY = 2'd2, ST_SEND = 2'd3;
    reg [26:0] tick;
    reg [15:0] hb_cnt;
    reg [3:0]  byte_idx;
    reg [1:0]  st;

    function [7:0] payload_byte(input [2:0] idx, input [15:0] cnt);
        case (idx)
            3'd0: payload_byte = "H";
            3'd1: payload_byte = "B";
            3'd2: payload_byte = "E";
            3'd3: payload_byte = "A";
            3'd4: payload_byte = "T";
            3'd5: payload_byte = cnt[7:0];
            3'd6: payload_byte = cnt[15:8];
            default: payload_byte = 8'h0A;
        endcase
    endfunction

    always @(posedge I_clk or posedge I_reset) begin
        if (I_reset) begin
            tick <= 27'd0; hb_cnt <= 16'd0; byte_idx <= 4'd0; st <= ST_WAIT;
            O_udp_req <= 1'b0; O_udp_len <= 16'd0;
            O_udp_valid <= 1'b0; O_udp_data <= 8'd0;
        end else begin
            case (st)
            ST_WAIT: begin
                if (tick >= CNT_MAX[26:0]) begin
                    tick <= 27'd0; st <= ST_PULSE;
                end else tick <= tick + 27'd1;
            end
            ST_PULSE: begin
                O_udp_req <= 1'b1; O_udp_len <= PAYLOAD[15:0]; st <= ST_BUSY;
            end
            ST_BUSY: begin
                O_udp_req <= 1'b0;
                if (I_udp_busy) begin
                    O_udp_valid <= 1'b1;
                    O_udp_data  <= payload_byte(3'd0, hb_cnt);
                    byte_idx <= 4'd1; st <= ST_SEND;
                end
            end
            ST_SEND: begin
                if (I_udp_busy) begin
                    if (byte_idx < PAYLOAD) begin
                        O_udp_data <= payload_byte(byte_idx[2:0], hb_cnt);
                        byte_idx <= byte_idx + 4'd1;
                    end else begin
                        O_udp_valid <= 1'b0; hb_cnt <= hb_cnt + 16'd1; st <= ST_WAIT;
                    end
                end else begin
                    O_udp_valid <= 1'b0; st <= ST_WAIT;
                end
            end
            default: st <= ST_WAIT;
            endcase
        end
    end
endmodule
