

module video_in (
    input wire         I_rst_n,

    input wire         I_camera_clk,
    input wire         I_camera_frame_start,
    input wire         I_camera_valid,
    input wire[127:0]  I_camera_data,
    input wire         I_mipi_rx_error,

    input wire         I_ddr_clk,

    input wire         I_display_pause,
    input wire         I_video_out_rd_busy,
    output wire        O_video_in_wr_busy,
    output reg[1:0]    O_video_out_rp,     ///读指针
    output wire       O_frame_write,
    output wire[1:0]  O_frame_slot,

    output reg         O_ddr_user_wr_en,
    output reg[24:0]   O_ddr_user_addr,
    output wire[127:0] O_ddr_user_wr_data,
    input wire         I_ddr_user_ready
);

    wire        S_fifo_rst;
    reg         S_camera_frame_start_extend; ///脉冲展宽，同步到ddr_clk时钟域
    reg[3:0]    S_extend_cnt;
    reg         S_camera_frame_start_extend_1d;
    reg         S_camera_frame_start_extend_2d;
    reg         S_camera_frame_start_extend_3d;
    wire        S_frame_start;
    reg[1:0]    S_video_in_wp;       ///写指针
    wire[8:0]   S_fifo_rd_num;      
    wire        S_fifo_rd_en;   
    wire        S_fifo_rd_valid;
    wire[127:0] S_fifo_rd_data; 
    reg         S_ddr_wr_valid;
    reg[9:0]    S_ddr_wr_cnt;

    localparam IMAGE_BASE_ADDR_0 = 25'd0;
    localparam IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam IMAGE_BASE_ADDR_3 = 25'd21000000;


    // The 96->128 packer emits its first complete word on the second
    // valid input beat.  Resetting this asynchronous FIFO for several
    // camera clocks at frame start therefore discards the first words of
    // every frame and shifts the DDR line phase.  The stream is continuous
    // and each 1280-pixel line is exactly 240 words, so the FIFO does not
    // need a per-frame flush; the frame marker is used only for buffer
    // rotation below.
    assign S_fifo_rst = 1'b0;

    // S_ddr_wr_valid is the ownership flag for the complete DDR write burst.
    // O_ddr_user_wr_en is registered one cycle after S_fifo_rd_en; including it
    // here bridges the only idle cycle between consecutive 240-beat bursts.
    // With a continuous 1080p stream that kept wr_busy permanently asserted and
    // starved video_out, so no DDR read request could ever start.
    assign O_video_in_wr_busy = S_ddr_wr_valid;

    w128_d512_fifo U_w128_d512_fifo(
        .rst        ( S_fifo_rst         ),   

        .clkw       ( I_camera_clk       ),     
        .we         ( I_camera_valid     ),
        .di         ( I_camera_data      ),
        .afull      (),
        .full_flag  (),
        .wrusedw    (),
 
        .clkr       ( I_ddr_clk          ),
        .re         ( S_fifo_rd_en       ), 
        .dout       ( O_ddr_user_wr_data ),  
        .rdusedw    ( S_fifo_rd_num      ),
        .valid      (), 
        .empty_flag (),
        .aempty     ()
    );


    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_camera_frame_start_extend <= 1'b0;
        else
            if(I_camera_frame_start)
                S_camera_frame_start_extend <= 1'b1;
            else if(S_extend_cnt == 'd7)
                S_camera_frame_start_extend <= 1'b0;
            else
                S_camera_frame_start_extend <= S_camera_frame_start_extend;
    end

	
    always @ (posedge I_camera_clk) begin
    	if(S_camera_frame_start_extend)
        	S_extend_cnt <= S_extend_cnt + 'd1;
      	else
          	S_extend_cnt <= 'd0;
    end


    always @(posedge I_ddr_clk) begin
        S_camera_frame_start_extend_1d <= S_camera_frame_start_extend;
        S_camera_frame_start_extend_2d <= S_camera_frame_start_extend_1d;
        S_camera_frame_start_extend_3d <= S_camera_frame_start_extend_2d;
    end

    assign S_frame_start = ~S_camera_frame_start_extend_3d & S_camera_frame_start_extend_2d;
    // Export the exact buffer decision used by the address generator.
    assign O_frame_write = S_frame_start && !I_mipi_rx_error && !I_display_pause;
    assign O_frame_slot = S_video_in_wp;

	/*
		如果前一帧图像有误少行了，那么地址回退为之前的，覆盖有问题的那帧图像
	*/
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_video_in_wp <= 'd0;
        else
            if(I_display_pause)
                S_video_in_wp <= S_video_in_wp;
            else if(S_frame_start)
                begin
                    if(I_mipi_rx_error)
                        S_video_in_wp <= S_video_in_wp;
                    else
                        S_video_in_wp <= S_video_in_wp + 'd1;
                end
            else
                S_video_in_wp <= S_video_in_wp;
    end


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_video_out_rp <= 'd0;
        else
            if(I_display_pause)
                O_video_out_rp <= O_video_out_rp;
            else if(S_frame_start)
                begin
                    case(S_video_in_wp)
                        'd0: O_video_out_rp <= 'd2;
                        'd1: O_video_out_rp <= 'd3;
                        'd2: O_video_out_rp <= 'd0;
                        'd3: O_video_out_rp <= 'd1;
                    endcase
                end
            else
                O_video_out_rp <= O_video_out_rp;
    end

    /*
        fifo的深度是512，实际的深度是510，因此需要注意每次读出的长度避免为256，否则连续两次读写会引起fifo读空，丢失数据
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_ddr_wr_valid <= 1'b0;
        else
            if(S_ddr_wr_valid && I_ddr_user_ready && S_ddr_wr_cnt == 'd239)
                S_ddr_wr_valid <= 1'b0;
            else if(S_fifo_rd_num >= 'd240 && (!I_video_out_rd_busy) && (!S_fifo_rst))
                S_ddr_wr_valid <= 1'b1;
            else
                S_ddr_wr_valid <= S_ddr_wr_valid;
    end

    always @(posedge I_ddr_clk) begin
        if(S_ddr_wr_valid)
            begin
                if(I_ddr_user_ready)
                    S_ddr_wr_cnt <= S_ddr_wr_cnt + 'd1;
                else
                    S_ddr_wr_cnt <= S_ddr_wr_cnt;
            end
        else
            S_ddr_wr_cnt <= 'd0;
    end

    assign S_fifo_rd_en = S_ddr_wr_valid & I_ddr_user_ready;

    always @(posedge I_ddr_clk) begin
        O_ddr_user_wr_en <= S_fifo_rd_en;
    end

	/*
		如果前一帧图像有误少行了，那么地址回退为之前的，覆盖有问题的那帧图像
	*/
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_ddr_user_addr <= 1'b0;
        else
            if(S_frame_start && (!I_mipi_rx_error))
                begin
                    case(S_video_in_wp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                    endcase
                end
			else if(S_frame_start && I_mipi_rx_error)
				begin
                	case(S_video_in_wp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                    endcase
                end
            else if(O_ddr_user_wr_en)
                O_ddr_user_addr <= O_ddr_user_addr + 'd8;
            else    
                O_ddr_user_addr <= O_ddr_user_addr;
    end

    
endmodule

// Capture timestamp -> exact DDR slot -> selected HDMI frame. Timestamps are
// bundled data held for a whole camera/display frame; only the 1 ms timebase
// crosses continuously, in registered Gray code. Range is modulo 65536 ms.
module video_frame_latency #(
    parameter integer TIME_CLOCK_HZ = 50000000
) (
    input wire rst_n, time_clk, camera_clk, capture_start, ddr_clk,
    input wire frame_write,
    input wire [1:0] write_slot,
    input wire frame_select,
    input wire [1:0] read_slot,
    input wire display_clk, display_rst_n, display_vsync, display_start,
    output reg [15:0] latency_ms,
    output reg latency_valid
);
    localparam integer MS_DIV = TIME_CLOCK_HZ / 1000;
    localparam integer DIV_BITS = (MS_DIV <= 2) ? 1 : $clog2(MS_DIV);
    reg [DIV_BITS-1:0] divider;
    reg [15:0] clock_ms, clock_gray;
    wire [15:0] next_ms = clock_ms + 16'd1;
    always @(posedge time_clk or negedge rst_n) begin
        if(!rst_n) begin
            divider <= 0; clock_ms <= 0; clock_gray <= 0;
        end else if(divider == MS_DIV-1) begin
            divider <= 0;
            clock_ms <= next_ms;
            clock_gray <= next_ms ^ (next_ms >> 1);
        end else divider <= divider + 1'b1;
    end

    function [15:0] gray_binary;
        input [15:0] gray;
        integer i;
        begin
            gray_binary[15] = gray[15];
            for(i=14;i>=0;i=i-1)
                gray_binary[i] = gray_binary[i+1] ^ gray[i];
        end
    endfunction

    (* async_reg = "true" *) reg [15:0] cam_gray_1, cam_gray_2;
    reg [15:0] capture_ms;
    reg capture_seen;
    always @(posedge camera_clk or negedge rst_n) begin
        if(!rst_n) begin
            cam_gray_1 <= 0; cam_gray_2 <= 0;
            capture_ms <= 0; capture_seen <= 0;
        end else begin
            cam_gray_1 <= clock_gray;
            cam_gray_2 <= cam_gray_1;
            if(capture_start) begin
                capture_ms <= gray_binary(cam_gray_2);
                capture_seen <= 1;
            end
        end
    end

    (* async_reg = "true" *) reg [15:0] capture_ms_1, capture_ms_2;
    (* async_reg = "true" *) reg [1:0] capture_seen_sync;
    reg [15:0] slot_ms [0:3];
    reg [3:0] slot_valid;
    reg previous_write;
    reg [1:0] previous_slot;
    reg [15:0] selected_ms;
    reg selected_valid, selected_toggle;
    integer j;
    always @(posedge ddr_clk or negedge rst_n) begin
        if(!rst_n) begin
            capture_ms_1 <= 0; capture_ms_2 <= 0;
            capture_seen_sync <= 0;
            slot_valid <= 0; previous_write <= 0; previous_slot <= 0;
            selected_ms <= 0; selected_valid <= 0; selected_toggle <= 0;
            for(j=0;j<4;j=j+1) slot_ms[j] <= 0;
        end else begin
            capture_ms_1 <= capture_ms;
            capture_ms_2 <= capture_ms_1;
            capture_seen_sync <= {capture_seen_sync[0],capture_seen};
            if(frame_write) begin
                // Publish only after the next frame boundary. A slot currently
                // being written must never be advertised as a completed image.
                if(previous_write) slot_valid[previous_slot] <= 1;
                slot_valid[write_slot] <= 0;
                slot_ms[write_slot] <= capture_ms_2;
                previous_slot <= write_slot;
                previous_write <= capture_seen_sync[1];
            end
            if(frame_select) begin
                selected_ms <= slot_ms[read_slot];
                selected_valid <= slot_valid[read_slot] &&
                                  !(frame_write && write_slot == read_slot);
                selected_toggle <= ~selected_toggle;
            end
        end
    end

    (* async_reg = "true" *) reg [15:0] display_gray_1, display_gray_2;
    (* async_reg = "true" *) reg [1:0] select_sync;
    reg select_seen, vsync_previous;
    reg [15:0] pending_ms;
    reg pending_valid;
    wire display_reset_n = rst_n && display_rst_n;
    always @(posedge display_clk or negedge display_reset_n) begin
        if(!display_reset_n) begin
            display_gray_1 <= 0; display_gray_2 <= 0;
            select_sync <= 0; select_seen <= 0; vsync_previous <= 0;
            pending_ms <= 0; pending_valid <= 0;
            latency_ms <= 0; latency_valid <= 0;
        end else begin
            display_gray_1 <= clock_gray;
            display_gray_2 <= display_gray_1;
            select_sync <= {select_sync[0],selected_toggle};
            vsync_previous <= display_vsync;
            // Discard last frame's metadata at the same VS that starts a
            // new DDR read. The new bundle arrives during vertical blanking.
            if(display_vsync && !vsync_previous) pending_valid <= 0;
            if(select_sync[1] != select_seen) begin
                select_seen <= select_sync[1];
                pending_ms <= selected_ms;
                pending_valid <= selected_valid;
            end
            if(display_start) begin
                latency_ms <= gray_binary(display_gray_2) - pending_ms;
                latency_valid <= pending_valid;
                pending_valid <= 0;
            end
        end
    end
endmodule
