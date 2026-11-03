module edge_detect_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH    = 11'd80,
    parameter KEY_DEBOUNCE_CYCLES = 1200000
)(
    input               I_clk,
    input               I_rst_n,
    input      [3:0]    I_button,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    output              O_tready,
    output reg          O_motion_alarm,
    output              O_track_update,
    output              O_track_valid,
    output [8:0]        O_track_x,
    output [7:0]        O_track_y
);

assign O_tready = I_tready;

wire [1:0] display_mode;
wire tracking_mode;
video_key_modes #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) u_video_key_modes (
    .I_clk(I_clk), .I_rst_n(I_rst_n), .I_button(I_button),
    .I_frame_start(I_tvalid && I_tuser), .O_mode(display_mode),
    .O_tracking(tracking_mode)
);

wire [23:0] pix0 = I_tdata[95:72];
wire [23:0] pix1 = I_tdata[71:48];
wire [23:0] pix2 = I_tdata[47:24];
wire [23:0] pix3 = I_tdata[23:0];

// Detect color on the AWB-corrected pixels before Sobel, quantization, and
// edge overlay alter the original chroma.
// Select colored lamp pixels rather than the nearly neutral warm background.
// The clipped white core need not match: the colored rim supplies the box.
function lamp_red;
    input [23:0] p;
    begin
        lamp_red = (p[23:16] >= 8'd64) &&
                   ({1'b0,p[23:16]} >= {1'b0,p[15:8]} + 9'd12) &&
                   ({1'b0,p[23:16]} >= {1'b0,p[7:0]} + 9'd24);
    end
endfunction
wire red0 = lamp_red(pix0);
wire red1 = lamp_red(pix1);
wire red2 = lamp_red(pix2);
wire red3 = lamp_red(pix3);

function lamp_green;
    input [23:0] p;
    begin
        lamp_green = (p[15:8] >= 8'd60) &&
                     ({1'b0,p[15:8]} > {1'b0,p[23:16]} + 9'd15) &&
                     ({1'b0,p[15:8]} > {1'b0,p[7:0]} + 9'd15) &&
                     // Reject bright near-neutral pixels without raising the
                     // dark-lamp brightness or three-pixel support threshold.
                     ({1'b0,p[15:8]} >= {1'b0,p[23:16]} + {3'b0,p[15:10]}) &&
                     ({1'b0,p[15:8]} >= {1'b0,p[7:0]} + {3'b0,p[15:10]});
    end
endfunction

// Disjoint R/G intervals prevent one pixel from voting for two colors.
// Preserve the validated red gate; yellow occupies the interval before green.
function lamp_yellow;
    input [23:0] p;
    begin
        lamp_yellow = (p[23:16] >= 8'd64) && (p[15:8] >= 8'd60) &&
                      ({1'b0,p[23:16]} <= {1'b0,p[15:8]} + 9'd11) &&
                      ({1'b0,p[15:8]} <= {1'b0,p[23:16]} + 9'd15) &&
                      ({1'b0,p[23:16]} >= {1'b0,p[7:0]} + 9'd24) &&
                      ({1'b0,p[15:8]} >= {1'b0,p[7:0]} + 9'd24);
    end
endfunction
wire green0 = lamp_green(pix0);
wire green1 = lamp_green(pix1);
wire green2 = lamp_green(pix2);
wire green3 = lamp_green(pix3);
wire yellow0 = lamp_yellow(pix0);
wire yellow1 = lamp_yellow(pix1);
wire yellow2 = lamp_yellow(pix2);
wire yellow3 = lamp_yellow(pix3);

// Keep the fractional terms until after addition, including neutral grays.
function [7:0] luminance;
    input [23:0] p;
    reg [9:0] weighted;
    begin
        weighted = {2'b0,p[23:16]} + {1'b0,p[15:8],1'b0} +
                   {2'b0,p[7:0]};
        luminance = weighted[9:2];
    end
endfunction

wire [7:0] gray0 = luminance(pix0);
wire [7:0] gray1 = luminance(pix1);
wire [7:0] gray2 = luminance(pix2);
wire [7:0] gray3 = luminance(pix3);

localparam IMG_WIDTH_GRP = (IMG_WIDTH >> 2);

// Preserve all four pixels: pair averaging previously erased thin contours.
(* ram_style = "block" *) reg [31:0] line1_mem [0:IMG_WIDTH_GRP-1];
(* ram_style = "block" *) reg [31:0] line2_mem [0:IMG_WIDTH_GRP-1];

reg [10:0] x_grp;
reg [10:0] y_pos;

wire       red_acc_valid;
wire [11:0] red_acc_xmin, red_acc_xmax;
wire [11:0] red_acc_ymin, red_acc_ymax;
wire [31:0] red_acc_pixels;
wire        green_acc_valid;
wire [11:0] green_acc_xmin, green_acc_xmax;
wire [11:0] green_acc_ymin, green_acc_ymax;
wire [31:0] green_acc_pixels;
wire        yellow_acc_valid;
wire [11:0] yellow_acc_xmin, yellow_acc_xmax;
wire [11:0] yellow_acc_ymin, yellow_acc_ymax;
wire [31:0] yellow_acc_pixels;

reg        red_box_valid;
reg [11:0] red_box_xmin, red_box_xmax;
reg [11:0] red_box_ymin, red_box_ymax;
reg [3:0]  red_hold_frames;
wire        green_box_valid;
wire [11:0] green_box_xmin, green_box_xmax;
wire [11:0] green_box_ymin, green_box_ymax;
wire        yellow_box_valid;
wire [11:0] yellow_box_xmin, yellow_box_xmax;
wire [11:0] yellow_box_ymin, yellow_box_ymax;

wire [11:0] beat_x0 = I_tuser ? 12'd0 : {x_grp[9:0],2'b00};
wire [11:0] beat_x1 = beat_x0 + 12'd1;
wire [11:0] beat_x2 = beat_x0 + 12'd2;
wire [11:0] beat_x3 = beat_x0 + 12'd3;
wire [11:0] beat_y  = I_tuser ? 12'd0 : {1'b0,y_pos};

wire track_valid, track_fresh;
wire [8:0] track_x, track_width;
wire [7:0] track_y;
localparam MOTION_THRESHOLD_SAMPLES = 12; // 12 * 4 = 48 source pixels
reg        motion_have_previous;
reg [8:0]  motion_previous_x;
reg [7:0]  motion_previous_y;
reg [4:0]  motion_hold_frames;
wire [8:0] motion_dx = (track_x >= motion_previous_x) ?
                       track_x - motion_previous_x :
                       motion_previous_x - track_x;
wire [7:0] motion_dy = (track_y >= motion_previous_y) ?
                       track_y - motion_previous_y :
                       motion_previous_y - track_y;
red_circle_stream #(.WIDTH(IMG_WIDTH/4), .HEIGHT(IMG_HEIGHT/4)) u_circle_track (
    .clk(I_clk), .rst_n(I_rst_n), .enable(tracking_mode),
    .valid(I_tvalid && (beat_y[1:0] == 0)),
    .sof(I_tuser), .eol(I_tlast),
    .x(beat_x0[10:2]), .y(beat_y[9:2]), .rgb(pix0),
    .box_valid(track_valid), .fresh(track_fresh),
    .center_x(track_x), .center_y(track_y), .diameter(track_width)
);
// The tracker publishes at SOF. Sample one cycle later so the decimal text
// uses that same frame's center, expressed in 1280x720 source coordinates.
reg coordinate_start;
assign O_track_update = coordinate_start;
assign O_track_valid = tracking_mode && track_valid && track_fresh;
assign O_track_x = track_x;
assign O_track_y = track_y;
always @(posedge I_clk or negedge I_rst_n)
    if(!I_rst_n) coordinate_start <= 1'b0;
    else coordinate_start <= I_tvalid && I_tuser;

// The reference advances only after an alarm, so gradual large movement
// also triggers. Held boxes cannot move it; a fully lost track clears it.
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_motion_alarm <= 1'b0;
        motion_have_previous <= 1'b0;
        motion_previous_x <= 9'd0;
        motion_previous_y <= 8'd0;
    end else begin
        O_motion_alarm <= 1'b0;
        if(!tracking_mode) begin
            motion_have_previous <= 1'b0;
        end else if(coordinate_start) begin
            if(!track_valid) begin
                motion_have_previous <= 1'b0;
            end else if(track_fresh) begin
                if(!motion_have_previous) begin
                    motion_previous_x <= track_x;
                    motion_previous_y <= track_y;
                    motion_have_previous <= 1'b1;
                end else if((motion_dx >= MOTION_THRESHOLD_SAMPLES) ||
                            (motion_dy >= MOTION_THRESHOLD_SAMPLES)) begin
                    O_motion_alarm <= 1'b1;
                    motion_previous_x <= track_x;
                    motion_previous_y <= track_y;
                end
            end
        end
    end
end

// A visible movement indication is an event hold, not a target count.
// Only the existing fresh-observation displacement test can start it.
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) motion_hold_frames <= 0;
    else if(!tracking_mode) motion_hold_frames <= 0;
    else if(O_motion_alarm) motion_hold_frames <= 5'd18;
    else if(coordinate_start) begin
        if(!track_valid) motion_hold_frames <= 0;
        else if(motion_hold_frames != 0)
            motion_hold_frames <= motion_hold_frames - 1'b1;
    end
end

wire [15:0] track_x_bcd, track_y_bcd;
wire coordinate_ready;
track_coordinate_decimal u_track_coordinate_decimal (
    .clk(I_clk), .rst_n(I_rst_n), .start(coordinate_start),
    .x({track_x,2'b00}), .y({track_y,2'b00}),
    .x_bcd(track_x_bcd), .y_bcd(track_y_bcd), .ready(coordinate_ready)
);
wire [9:0] track_radius = {1'b0,track_width} >> 1;
wire [9:0] track_left = (track_x < track_radius) ? 10'd0 :
                        {1'b0,track_x} - track_radius;
wire [9:0] track_right = ({1'b0,track_x} + track_radius >= IMG_WIDTH/4) ?
                         IMG_WIDTH/4-1 : {1'b0,track_x} + track_radius;
wire [9:0] track_top = (track_y < track_radius) ? 10'd0 :
                       {2'b0,track_y} - track_radius;
wire [9:0] track_bottom = ({2'b0,track_y} + track_radius >= IMG_HEIGHT/4) ?
                          IMG_HEIGHT/4-1 : {2'b0,track_y} + track_radius;
wire red_render_valid = tracking_mode ? track_valid : red_box_valid;
wire [11:0] red_render_xmin = tracking_mode ? {track_left,2'b00} : red_box_xmin;
wire [11:0] red_render_xmax = tracking_mode ? {track_right,2'b11} : red_box_xmax;
wire [11:0] red_render_ymin = tracking_mode ? {track_top,2'b00} : red_box_ymin;
wire [11:0] red_render_ymax = tracking_mode ? {track_bottom,2'b11} : red_box_ymax;
wire [23:0] red_render_color = (tracking_mode && !track_fresh) ? 24'hffb020 :
                               (tracking_mode && motion_hold_frames != 0) ?
                               24'hffff00 : 24'hff2020;

// The fixed FPGA logo occupies a 160x160 upper-left OSD area and contains
// red pixels. Keep a small margin so the logo cannot merge with the target.
wire red_detection_roi = !((beat_x0 < 12'd180) && (beat_y < 12'd180));
wire red_beat_valid = red_detection_roi &&
                      (red0 || red1 || red2 || red3);
wire [11:0] red_beat_xmin = red0 ? beat_x0 :
                             red1 ? beat_x1 :
                             red2 ? beat_x2 : beat_x3;
wire [11:0] red_beat_xmax = red3 ? beat_x3 :
                             red2 ? beat_x2 :
                             red1 ? beat_x1 : beat_x0;
wire [2:0] red_beat_count = {2'd0,red0} + {2'd0,red1} +
                             {2'd0,red2} + {2'd0,red3};
// Rank local row-connected candidates instead of merging all frame hits.
red_lamp_row_tracker u_red_lamp_row_tracker (
    .clk(I_clk), .rst_n(I_rst_n),
    .valid(I_tvalid), .sof(I_tuser), .eol(I_tlast),
    .y(beat_y), .hit_count(red_beat_valid ? red_beat_count : 3'd0),
    .hit_xmin(red_beat_xmin), .hit_xmax(red_beat_xmax),
    .candidate_valid(red_acc_valid),
    .candidate_xmin(red_acc_xmin), .candidate_xmax(red_acc_xmax),
    .candidate_ymin(red_acc_ymin), .candidate_ymax(red_acc_ymax),
    .candidate_pixels(red_acc_pixels)
);
wire green_beat_valid = red_detection_roi && (green0 || green1 || green2 || green3);
wire [11:0] green_beat_xmin = green0 ? beat_x0 :
                               green1 ? beat_x1 :
                               green2 ? beat_x2 : beat_x3;
wire [11:0] green_beat_xmax = green3 ? beat_x3 :
                               green2 ? beat_x2 :
                               green1 ? beat_x1 : beat_x0;
wire [2:0] green_beat_count = {2'd0,green0} + {2'd0,green1} +
                               {2'd0,green2} + {2'd0,green3};
wire yellow_beat_valid = red_detection_roi && (yellow0 || yellow1 || yellow2 || yellow3);
wire [11:0] yellow_beat_xmin = yellow0 ? beat_x0 :
                                yellow1 ? beat_x1 :
                                yellow2 ? beat_x2 : beat_x3;
wire [11:0] yellow_beat_xmax = yellow3 ? beat_x3 :
                                yellow2 ? beat_x2 :
                                yellow1 ? beat_x1 : beat_x0;
wire [2:0] yellow_beat_count = {2'd0,yellow0} + {2'd0,yellow1} +
                                {2'd0,yellow2} + {2'd0,yellow3};

red_lamp_row_tracker u_yellow_lamp_row_tracker (
    .clk(I_clk), .rst_n(I_rst_n),
    .valid(I_tvalid), .sof(I_tuser), .eol(I_tlast),
    .y(beat_y), .hit_count(yellow_beat_valid ? yellow_beat_count : 3'd0),
    .hit_xmin(yellow_beat_xmin), .hit_xmax(yellow_beat_xmax),
    .candidate_valid(yellow_acc_valid),
    .candidate_xmin(yellow_acc_xmin), .candidate_xmax(yellow_acc_xmax),
    .candidate_ymin(yellow_acc_ymin), .candidate_ymax(yellow_acc_ymax),
    .candidate_pixels(yellow_acc_pixels)
);
red_lamp_row_tracker u_green_lamp_row_tracker (
    .clk(I_clk), .rst_n(I_rst_n),
    .valid(I_tvalid), .sof(I_tuser), .eol(I_tlast),
    .y(beat_y), .hit_count(green_beat_valid ? green_beat_count : 3'd0),
    .hit_xmin(green_beat_xmin), .hit_xmax(green_beat_xmax),
    .candidate_valid(green_acc_valid),
    .candidate_xmin(green_acc_xmin), .candidate_xmax(green_acc_xmax),
    .candidate_ymin(green_acc_ymin), .candidate_ymax(green_acc_ymax),
    .candidate_pixels(green_acc_pixels)
);
lamp_box_hold #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT)) u_yellow_box (
    .clk(I_clk), .rst_n(I_rst_n), .publish(I_tvalid && I_tuser),
    .candidate_valid(yellow_acc_valid),
    .candidate_xmin(yellow_acc_xmin), .candidate_xmax(yellow_acc_xmax),
    .candidate_ymin(yellow_acc_ymin), .candidate_ymax(yellow_acc_ymax),
    .box_valid(yellow_box_valid),
    .box_xmin(yellow_box_xmin), .box_xmax(yellow_box_xmax),
    .box_ymin(yellow_box_ymin), .box_ymax(yellow_box_ymax)
);
lamp_box_hold #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT),
                .REQUIRE_CONFIRM(1)) u_green_box (
    .clk(I_clk), .rst_n(I_rst_n), .publish(I_tvalid && I_tuser),
    .candidate_valid(green_acc_valid),
    .candidate_xmin(green_acc_xmin), .candidate_xmax(green_acc_xmax),
    .candidate_ymin(green_acc_ymin), .candidate_ymax(green_acc_ymax),
    .box_valid(green_box_valid),
    .box_xmin(green_box_xmin), .box_xmax(green_box_xmax),
    .box_ymin(green_box_ymin), .box_ymax(green_box_ymax)
);

wire [10:0] line_addr = I_tuser ? 11'd0 : x_grp;
wire [31:0] top_word = line2_mem[line_addr];
wire [31:0] mid_word = line1_mem[line_addr];
wire [31:0] bot_word = {gray0,gray1,gray2,gray3};

wire [7:0] top0 = top_word[31:24];
wire [7:0] top1 = top_word[23:16];
wire [7:0] top2 = top_word[15:8];
wire [7:0] top3 = top_word[7:0];
wire [7:0] mid0 = mid_word[31:24];
wire [7:0] mid1 = mid_word[23:16];
wire [7:0] mid2 = mid_word[15:8];
wire [7:0] mid3 = mid_word[7:0];

reg [7:0] top_prev2, top_prev1;
reg [7:0] mid_prev2, mid_prev1;
reg [7:0] bot_prev2, bot_prev1;

function [13:0] sobel_mag;
    input [7:0] a00, a01, a02;
    input [7:0] a10, a11, a12;
    input [7:0] a20, a21, a22;
    reg signed [12:0] gx_f;
    reg signed [12:0] gy_f;
    reg [12:0] abs_gx_f;
    reg [12:0] abs_gy_f;
    begin
        gx_f = -$signed({1'b0,a00}) + $signed({1'b0,a02})
             - ($signed({1'b0,a10}) <<< 1) + ($signed({1'b0,a12}) <<< 1)
             - $signed({1'b0,a20}) + $signed({1'b0,a22});
        gy_f =  $signed({1'b0,a00}) + ($signed({1'b0,a01}) <<< 1) + $signed({1'b0,a02})
             - $signed({1'b0,a20}) - ($signed({1'b0,a21}) <<< 1) - $signed({1'b0,a22});
        abs_gx_f = gx_f[12] ? (~gx_f + 1'b1) : gx_f;
        abs_gy_f = gy_f[12] ? (~gy_f + 1'b1) : gy_f;
        sobel_mag = abs_gx_f + abs_gy_f;
    end
endfunction

wire [13:0] mag0 = sobel_mag(top_prev2,top_prev1,top0,
                             mid_prev2,mid_prev1,mid0,
                             bot_prev2,bot_prev1,gray0);
wire [13:0] mag1 = sobel_mag(top_prev1,top0,top1,
                             mid_prev1,mid0,mid1,
                             bot_prev1,gray0,gray1);
wire [13:0] mag2 = sobel_mag(top0,top1,top2,
                             mid0,mid1,mid2,
                             gray0,gray1,gray2);
wire [13:0] mag3 = sobel_mag(top1,top2,top3,
                             mid1,mid2,mid3,
                             gray1,gray2,gray3);

wire row_valid = (y_pos >= 11'd2) && !I_tuser;
wire window_valid = row_valid && (x_grp != 11'd0);
wire edge0 = window_valid && (mag0 >= EDGE_TH);
wire edge1 = window_valid && (mag1 >= EDGE_TH);
wire edge2 = row_valid && (mag2 >= EDGE_TH);
wire edge3 = row_valid && (mag3 >= EDGE_TH);

// Nearest of eight luminance levels. Independent RGB truncation made small
// neutral-channel differences jump into visibly different hue bins.
function [7:0] quant8;
    input [7:0] c;
    begin
        if(c < 8'd18) quant8 = 8'd0;
        else if(c < 8'd55) quant8 = 8'd36;
        else if(c < 8'd91) quant8 = 8'd73;
        else if(c < 8'd128) quant8 = 8'd109;
        else if(c < 8'd164) quant8 = 8'd146;
        else if(c < 8'd201) quant8 = 8'd182;
        else if(c < 8'd237) quant8 = 8'd219;
        else quant8 = 8'd255;
    end
endfunction

function [7:0] add_luma_delta;
    input [7:0] c;
    input signed [8:0] delta;
    reg signed [9:0] shifted;
    begin
        shifted = $signed({2'b0,c}) + delta;
        if(shifted < 0) add_luma_delta = 8'd0;
        else if(shifted > 255) add_luma_delta = 8'd255;
        else add_luma_delta = shifted[7:0];
    end
endfunction

function [23:0] quant_rgb;
    input [23:0] p;
    reg [7:0] y;
    reg signed [8:0] delta;
    begin
        y = luminance(p);
        delta = $signed({1'b0,quant8(y)}) - $signed({1'b0,y});
        // The same offset preserves R-G and G-B until a channel clips.
        quant_rgb = {add_luma_delta(p[23:16],delta),
                     add_luma_delta(p[15:8],delta),
                     add_luma_delta(p[7:0],delta)};
    end
endfunction

// Lift the displayed camera image without changing the detection coordinates.
// Saturating 1.5x is cheap in fabric and recovers dark scenes without wrapping.
function [7:0] brighten8;
    input [7:0] c;
    reg [8:0] scaled;
    begin
        scaled = {1'b0,c} + ({1'b0,c} >> 1);
        brighten8 = scaled[8] ? 8'hff : scaled[7:0];
    end
endfunction

function [23:0] brighten_rgb;
    input [23:0] p;
    begin
        brighten_rgb = {brighten8(p[23:16]),
                        brighten8(p[15:8]),
                        brighten8(p[7:0])};
    end
endfunction

// Detection uses the original RGB above; display modes share the same
// tested brightness lift and do not feed rendered colors back to detection.
wire [23:0] bright0 = brighten_rgb(pix0);
wire [23:0] bright1 = brighten_rgb(pix1);
wire [23:0] bright2 = brighten_rgb(pix2);
wire [23:0] bright3 = brighten_rgb(pix3);
wire [23:0] base0 = bright0;
wire [23:0] base1 = bright1;
wire [23:0] base2 = bright2;
wire [23:0] base3 = bright3;

// Small 5x7 "red" label rendered above the red box.
function [4:0] red_font_row;
    input [1:0] ch;
    input [2:0] row;
    begin
        case(ch)
            2'd0: begin
                case(row)
                    3'd0: red_font_row = 5'b00000;
                    3'd1: red_font_row = 5'b11110;
                    3'd2: red_font_row = 5'b10001;
                    3'd3: red_font_row = 5'b11110;
                    3'd4: red_font_row = 5'b10100;
                    3'd5: red_font_row = 5'b10010;
                    3'd6: red_font_row = 5'b10001;
                    default: red_font_row = 5'b00000;
                endcase
            end
            2'd1: begin
                case(row)
                    3'd0: red_font_row = 5'b00000;
                    3'd1: red_font_row = 5'b01110;
                    3'd2: red_font_row = 5'b10001;
                    3'd3: red_font_row = 5'b11111;
                    3'd4: red_font_row = 5'b10000;
                    3'd5: red_font_row = 5'b10001;
                    3'd6: red_font_row = 5'b01110;
                    default: red_font_row = 5'b00000;
                endcase
            end
            default: begin
                case(row)
                    3'd0: red_font_row = 5'b00000;
                    3'd1: red_font_row = 5'b11110;
                    3'd2: red_font_row = 5'b10001;
                    3'd3: red_font_row = 5'b10001;
                    3'd4: red_font_row = 5'b10001;
                    3'd5: red_font_row = 5'b10001;
                    3'd6: red_font_row = 5'b11110;
                    default: red_font_row = 5'b00000;
                endcase
            end
        endcase
    end
endfunction

function red_label_pixel;
    input        valid;
    input [11:0] px;
    input [11:0] py;
    input [11:0] xmin;
    input [11:0] ymin;
    reg [11:0] label_y;
    reg [11:0] local_x;
    reg [11:0] local_y;
    reg [1:0] ch;
    reg [2:0] col;
    reg [4:0] bits;
    begin
        label_y = (ymin >= 12'd8) ? (ymin - 12'd8) : ymin;
        red_label_pixel = 1'b0;
        if(valid && (px >= xmin) && (px < xmin + 12'd15) &&
           (py >= label_y) && (py < label_y + 12'd7)) begin
            local_x = px - xmin;
            local_y = py - label_y;
            if(local_x < 12'd5) begin
                ch = 2'd0;
                col = local_x[2:0];
            end else if(local_x < 12'd10) begin
                ch = 2'd1;
                col = local_x - 12'd5;
            end else begin
                ch = 2'd2;
                col = local_x - 12'd10;
            end
            bits = red_font_row(ch, local_y[2:0]);
            red_label_pixel = bits[4-col];
        end
    end
endfunction

function [4:0] color_font_row;
    input [7:0] ch;
    input [2:0] row;
    reg [34:0] glyph;
    begin
        case(ch)
            "y": glyph = {5'b00000,5'b10001,5'b10001,5'b01111,5'b00001,5'b10001,5'b01110};
            "e": glyph = {5'b00000,5'b01110,5'b10001,5'b11111,5'b10000,5'b10001,5'b01110};
            "l": glyph = {5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
            "o": glyph = {5'b00000,5'b00000,5'b01110,5'b10001,5'b10001,5'b10001,5'b01110};
            "w": glyph = {5'b00000,5'b00000,5'b10001,5'b10001,5'b10101,5'b10101,5'b01010};
            "g": glyph = {5'b00000,5'b01111,5'b10001,5'b10001,5'b01111,5'b00001,5'b01110};
            "r": glyph = {5'b00000,5'b00000,5'b10110,5'b11001,5'b10000,5'b10000,5'b10000};
            "n": glyph = {5'b00000,5'b00000,5'b11110,5'b10001,5'b10001,5'b10001,5'b10001};
            default: glyph = 35'd0;
        endcase
        color_font_row = (row < 3'd7) ? glyph[34-row*5 -: 5] : 5'd0;
    end
endfunction

wire [2:0] status_mode_code = tracking_mode ? 3'd4 :
                              display_mode == 2'd3 ? 3'd3 :
                              {1'b0,display_mode};
function [7:0] status_character;
    input [2:0] mode;
    input [2:0] index;
    begin
        status_character = 8'd0;
        case(mode)
            3'd0: case(index)
                0: status_character="R";
                1: status_character="A";
                2: status_character="W";
                default: status_character=0;
            endcase
            3'd1: case(index)
                0: status_character="E";
                1: status_character="D";
                2: status_character="G";
                3: status_character="E";
                default: status_character=0;
            endcase
            3'd2: case(index)
                0: status_character="Q";
                1: status_character="U";
                2: status_character="A";
                3: status_character="N";
                4: status_character="T";
                default: status_character=0;
            endcase
            3'd3: case(index)
                0: status_character="C";
                1: status_character="O";
                2: status_character="L";
                3: status_character="O";
                4: status_character="R";
                default: status_character=0;
            endcase
            3'd4: case(index)
                0: status_character=(motion_hold_frames != 0) ? "M" : "T";
                1: status_character=(motion_hold_frames != 0) ? "O" : "R";
                2: status_character=(motion_hold_frames != 0) ? "V" : "A";
                3: status_character=(motion_hold_frames != 0) ? "E" : "C";
                4: status_character=(motion_hold_frames != 0) ? 0 : "K";
                default: status_character=0;
            endcase
            default: status_character=0;
        endcase
    end
endfunction

function [4:0] status_font_row;
    input [7:0] ch;
    input [2:0] row;
    reg [34:0] glyph;
    begin
        case(ch)
            "A": glyph={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
            "C": glyph={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
            "D": glyph={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
            "E": glyph={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
            "G": glyph={5'b01111,5'b10000,5'b10000,5'b10111,5'b10001,5'b10001,5'b01111};
            "K": glyph={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
            "L": glyph={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
            "M": glyph=35'b10001_11011_10101_10101_10001_10001_10001;
            "N": glyph={5'b10001,5'b11001,5'b11001,5'b10101,5'b10011,5'b10011,5'b10001};
            "O": glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
            "Q": glyph={5'b01110,5'b10001,5'b10001,5'b10001,5'b10101,5'b10010,5'b01101};
            "R": glyph={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
            "T": glyph={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
            "U": glyph={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
            "V": glyph=35'b10001_10001_10001_10001_10001_01010_00100;
            "W": glyph={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010};
            "X": glyph=35'b10001_10001_01010_00100_01010_10001_10001;
            "Y": glyph=35'b10001_10001_01010_00100_00100_00100_00100;
            ":": glyph=35'b00000_00100_00100_00000_00100_00100_00000;
            "-": glyph=35'b00000_00000_00000_11111_00000_00000_00000;
            "0": glyph=35'b01110_10001_10011_10101_11001_10001_01110;
            "1": glyph=35'b00100_01100_00100_00100_00100_00100_01110;
            "2": glyph=35'b01110_10001_00001_00010_00100_01000_11111;
            "3": glyph=35'b11110_00001_00001_01110_00001_00001_11110;
            "4": glyph=35'b00010_00110_01010_10010_11111_00010_00010;
            "5": glyph=35'b11111_10000_10000_11110_00001_00001_11110;
            "6": glyph=35'b01110_10000_10000_11110_10001_10001_01110;
            "7": glyph=35'b11111_00001_00010_00100_01000_01000_01000;
            "8": glyph=35'b01110_10001_10001_01110_10001_10001_01110;
            "9": glyph=35'b01110_10001_10001_01111_00001_00001_01110;
            default: glyph=35'd0;
        endcase
        status_font_row = row < 7 ? glyph[34-row*5 -: 5] : 5'd0;
    end
endfunction

function [7:0] coordinate_character;
    input is_y;
    input [2:0] index;
    reg [15:0] digits;
    reg [3:0] digit;
    begin
        digits = is_y ? track_y_bcd : track_x_bcd;
        case(index)
            2: digit=digits[15:12];
            3: digit=digits[11:8];
            4: digit=digits[7:4];
            default: digit=digits[3:0];
        endcase
        if(index==0) coordinate_character=is_y ? "Y" : "X";
        else if(index==1) coordinate_character=":";
        else if(index<6)
            coordinate_character=(track_valid && coordinate_ready) ? {4'h3,digit} : "-";
        else coordinate_character=0;
    end
endfunction

wire [12:0] track_center_source_x = ({4'b0,track_x} << 2);
wire [12:0] coordinate_panel_left_unclamped = track_center_source_x >= 13'd104 ?
                                               track_center_source_x-13'd104 : 13'd0;
wire [11:0] coordinate_panel_left = coordinate_panel_left_unclamped > 13'd1072 ?
                                    12'd1072 : coordinate_panel_left_unclamped[11:0];
// Leave four top rows for the SOF publication and decimal conversion.
wire [11:0] coordinate_panel_top = red_render_ymin >= 12'd84 ?
                                   red_render_ymin-12'd80 : 12'd4;
wire coordinate_panel_active = tracking_mode && red_render_valid &&
    beat_x0 >= coordinate_panel_left &&
    beat_x0 < coordinate_panel_left+12'd208 &&
    beat_y >= coordinate_panel_top && beat_y < coordinate_panel_top+12'd80;

function [23:0] with_coordinate_panel;
    input [23:0] p;
    input [11:0] px,py;
    input active;
    reg [11:0] local_x,local_y,glyph_x;
    reg [2:0] glyph_index,glyph_col,glyph_row;
    reg [7:0] ch;
    reg [4:0] bits;
    begin
        with_coordinate_panel=p;
        if(active) begin
            local_x=px-coordinate_panel_left;
            local_y=py-coordinate_panel_top;
            glyph_x=local_x-12'd8;
            glyph_index=glyph_x[7:5];
            glyph_col=glyph_x[4:2];
            ch=0;
            if(local_x>=12'd8 && local_x<12'd200 &&
               local_y>=12'd4 && local_y<12'd36)
                ch=coordinate_character(1'b0,glyph_index);
            else if(local_x>=12'd8 && local_x<12'd200 &&
                    local_y>=12'd44 && local_y<12'd76)
                ch=coordinate_character(1'b1,glyph_index);
            glyph_row=(local_y>=12'd44 ? local_y-12'd44 :
                                              local_y-12'd4) >> 2;
            bits=status_font_row(ch,glyph_row);
            if(glyph_col<3'd5 && bits[4-glyph_col])
                with_coordinate_panel=(track_valid && !track_fresh) ?
                                      24'hffb020 : 24'hffffff;
            else if(local_x<12'd4)
                with_coordinate_panel=(track_valid && !track_fresh) ?
                                      24'hffb020 : 24'hff3030;
            else
                with_coordinate_panel=24'h101820;
        end
    end
endfunction

// Four source pixels per glyph pixel survive the 4:1 Ethernet decimation.
function status_panel_pixel;
    input [11:0] px, py;
    reg [11:0] local_x, local_y, glyph_x, glyph_y;
    reg [2:0] glyph_index, glyph_col, glyph_row;
    reg [7:0] ch;
    reg [4:0] bits;
    begin
        local_x = px - 12'd1048;
        local_y = py - 12'd12;
        glyph_x = local_x - 12'd16;
        glyph_y = local_y - 12'd4;
        glyph_index = glyph_x[7:5];
        glyph_col = glyph_x[4:2];
        ch = 0;
        if(local_x >= 12'd16 && local_x < 12'd176 &&
           local_y >= 12'd4 && local_y < 12'd36)
            ch = status_character(status_mode_code,glyph_index);
        glyph_row = glyph_y[4:2];
        bits = status_font_row(ch,glyph_row);
        status_panel_pixel = glyph_col < 3'd5 ? bits[4-glyph_col] : 1'b0;
    end
endfunction

function [23:0] with_status_panel;
    input [23:0] p;
    input [11:0] px, py;
    reg [11:0] local_x;
    begin
        with_status_panel = p;
        if(px >= 12'd1048 && py >= 12'd12 &&
           px < 12'd1264 && py < 12'd52) begin
            local_x=px-12'd1048;
            if(status_panel_pixel(px,py))
                with_status_panel=24'hffffff;
            else if(local_x < 12'd8)
                with_status_panel=(status_mode_code==3'd0) ? 24'h00c8ff :
                                  (status_mode_code==3'd1) ? 24'hffffff :
                                  (status_mode_code==3'd2) ? 24'hffd020 :
                                  (status_mode_code==3'd3) ? 24'h00e060 :
                                  (motion_hold_frames != 0) ? 24'hffff00 : 24'hff3030;
            else
                with_status_panel=24'h101820;
        end
    end
endfunction

function color_label_pixel;
    input is_yellow;
    input valid;
    input [11:0] px, py, xmin, ymin;
    reg [11:0] label_x, label_y, local_x, local_y;
    reg [2:0] col;
    reg [7:0] ch;
    reg [4:0] bits;
    begin
        // Keep the complete label on screen for targets near the right edge.
        label_x = (xmin > IMG_WIDTH - 36) ? IMG_WIDTH - 36 : xmin;
        label_y = (ymin >= 12'd8) ? ymin - 12'd8 : ymin;
        local_x = px - label_x;
        local_y = py - label_y;
        ch = 8'd0;
        col = 3'd0;
        color_label_pixel = 1'b0;
        if(valid && (px >= label_x) && (local_y < 12'd7) &&
           (py >= label_y) && (local_x < (is_yellow ? 12'd36 : 12'd30))) begin
            if(local_x < 12'd6) begin
                ch = is_yellow ? "y" : "g";
                col = local_x[2:0];
            end else if(local_x < 12'd12) begin
                ch = is_yellow ? "e" : "r";
                col = local_x - 12'd6;
            end else if(local_x < 12'd18) begin
                ch = is_yellow ? "l" : "e";
                col = local_x - 12'd12;
            end else if(local_x < 12'd24) begin
                ch = is_yellow ? "l" : "e";
                col = local_x - 12'd18;
            end else if(local_x < 12'd30) begin
                ch = is_yellow ? "o" : "n";
                col = local_x - 12'd24;
            end else begin
                ch = "w";
                col = local_x - 12'd30;
            end
            bits = color_font_row(ch, local_y[2:0]);
            if(col < 3'd5) color_label_pixel = bits[4-col];
        end
    end
endfunction

// Publish one local candidate per color into the common camera stream, so
// HDMI and Ethernet display the same boxes and labels.
function box_edge;
    input        valid;
    input [11:0] px;
    input [11:0] py;
    input [11:0] xmin;
    input [11:0] xmax;
    input [11:0] ymin;
    input [11:0] ymax;
    begin
        box_edge = valid &&
                   (px >= xmin) && (px <= xmax) &&
                   (py >= ymin) && (py <= ymax) &&
                   ((px < xmin + 12'd3) ||
                    (px + 12'd3 > xmax) ||
                    (py < ymin + 12'd3) ||
                    (py + 12'd3 > ymax));
    end
endfunction

wire red_edge0 = box_edge(red_render_valid, beat_x0, beat_y,
                          red_render_xmin, red_render_xmax,
                          red_render_ymin, red_render_ymax);
wire red_edge1 = box_edge(red_render_valid, beat_x1, beat_y,
                          red_render_xmin, red_render_xmax,
                          red_render_ymin, red_render_ymax);
wire red_edge2 = box_edge(red_render_valid, beat_x2, beat_y,
                          red_render_xmin, red_render_xmax,
                          red_render_ymin, red_render_ymax);
wire red_edge3 = box_edge(red_render_valid, beat_x3, beat_y,
                          red_render_xmin, red_render_xmax,
                          red_render_ymin, red_render_ymax);
wire yellow_edge0 = box_edge(yellow_box_valid, beat_x0, beat_y,
                             yellow_box_xmin, yellow_box_xmax,
                             yellow_box_ymin, yellow_box_ymax);
wire yellow_edge1 = box_edge(yellow_box_valid, beat_x1, beat_y,
                             yellow_box_xmin, yellow_box_xmax,
                             yellow_box_ymin, yellow_box_ymax);
wire yellow_edge2 = box_edge(yellow_box_valid, beat_x2, beat_y,
                             yellow_box_xmin, yellow_box_xmax,
                             yellow_box_ymin, yellow_box_ymax);
wire yellow_edge3 = box_edge(yellow_box_valid, beat_x3, beat_y,
                             yellow_box_xmin, yellow_box_xmax,
                             yellow_box_ymin, yellow_box_ymax);
wire green_edge0 = box_edge(green_box_valid, beat_x0, beat_y,
                            green_box_xmin, green_box_xmax,
                            green_box_ymin, green_box_ymax);
wire green_edge1 = box_edge(green_box_valid, beat_x1, beat_y,
                            green_box_xmin, green_box_xmax,
                            green_box_ymin, green_box_ymax);
wire green_edge2 = box_edge(green_box_valid, beat_x2, beat_y,
                            green_box_xmin, green_box_xmax,
                            green_box_ymin, green_box_ymax);
wire green_edge3 = box_edge(green_box_valid, beat_x3, beat_y,
                            green_box_xmin, green_box_xmax,
                            green_box_ymin, green_box_ymax);
wire red_label0 = red_label_pixel(red_render_valid, beat_x0, beat_y,
                                  red_render_xmin, red_render_ymin);
wire red_label1 = red_label_pixel(red_render_valid, beat_x1, beat_y,
                                  red_render_xmin, red_render_ymin);
wire red_label2 = red_label_pixel(red_render_valid, beat_x2, beat_y,
                                  red_render_xmin, red_render_ymin);
wire red_label3 = red_label_pixel(red_render_valid, beat_x3, beat_y,
                                  red_render_xmin, red_render_ymin);
wire yellow_label0 = color_label_pixel(1'b1, yellow_box_valid, beat_x0, beat_y, yellow_box_xmin, yellow_box_ymin);
wire yellow_label1 = color_label_pixel(1'b1, yellow_box_valid, beat_x1, beat_y, yellow_box_xmin, yellow_box_ymin);
wire yellow_label2 = color_label_pixel(1'b1, yellow_box_valid, beat_x2, beat_y, yellow_box_xmin, yellow_box_ymin);
wire yellow_label3 = color_label_pixel(1'b1, yellow_box_valid, beat_x3, beat_y, yellow_box_xmin, yellow_box_ymin);
wire green_label0 = color_label_pixel(1'b0, green_box_valid, beat_x0, beat_y, green_box_xmin, green_box_ymin);
wire green_label1 = color_label_pixel(1'b0, green_box_valid, beat_x1, beat_y, green_box_xmin, green_box_ymin);
wire green_label2 = color_label_pixel(1'b0, green_box_valid, beat_x2, beat_y, green_box_xmin, green_box_ymin);
wire green_label3 = color_label_pixel(1'b0, green_box_valid, beat_x3, beat_y, green_box_xmin, green_box_ymin);

function track_cross;
    input [11:0] px, py;
    reg [11:0] cx, cy;
    begin
        cx = {1'b0,track_x,2'b00};
        cy = {2'b0,track_y,2'b00};
        track_cross = tracking_mode && track_valid &&
            (((px + 12'd12 >= cx) && (px <= cx + 12'd12) &&
              (py + 12'd1 >= cy) && (py <= cy + 12'd1)) ||
             ((py + 12'd12 >= cy) && (py <= cy + 12'd12) &&
              (px + 12'd1 >= cx) && (px <= cx + 12'd1)));
    end
endfunction

wire [23:0] box0 = track_cross(beat_x0,beat_y) ? 24'hffffff :
                   red_label0 ? 24'hffffff :
                   red_edge0 ? red_render_color :
                   tracking_mode ? base0 :
                   (yellow_label0 || green_label0) ? 24'hffffff :
                   yellow_edge0 ? 24'he0c000 :
                   green_edge0 ? 24'h00e060 : base0;
wire [23:0] box1 = track_cross(beat_x1,beat_y) ? 24'hffffff :
                   red_label1 ? 24'hffffff :
                   red_edge1 ? red_render_color :
                   tracking_mode ? base1 :
                   (yellow_label1 || green_label1) ? 24'hffffff :
                   yellow_edge1 ? 24'he0c000 :
                   green_edge1 ? 24'h00e060 : base1;
wire [23:0] box2 = track_cross(beat_x2,beat_y) ? 24'hffffff :
                   red_label2 ? 24'hffffff :
                   red_edge2 ? red_render_color :
                   tracking_mode ? base2 :
                   (yellow_label2 || green_label2) ? 24'hffffff :
                   yellow_edge2 ? 24'he0c000 :
                   green_edge2 ? 24'h00e060 : base2;
wire [23:0] box3 = track_cross(beat_x3,beat_y) ? 24'hffffff :
                   red_label3 ? 24'hffffff :
                   red_edge3 ? red_render_color :
                   tracking_mode ? base3 :
                   (yellow_label3 || green_label3) ? 24'hffffff :
                   yellow_edge3 ? 24'he0c000 :
                   green_edge3 ? 24'h00e060 : base3;

function [23:0] display_pixel;
    input [1:0] mode;
    input [23:0] natural_rgb;
    input binary_edge;
    input [23:0] detected_rgb;
    begin
        case(mode)
            2'd1: display_pixel = binary_edge ? 24'hffffff : 24'h000000;
            2'd2: display_pixel = quant_rgb(natural_rgb);
            2'd3: display_pixel = detected_rgb;
            default: display_pixel = natural_rgb;
        endcase
    end
endfunction
// Panel boundaries and glyph cells are multiples of four: share one font
// lookup across the four lanes without adding latency to the video stream.
wire status_panel_active = beat_x0 >= 12'd1048 && beat_x0 < 12'd1264 &&
                           beat_y >= 12'd12 && beat_y < 12'd52;
wire [23:0] status_panel_rgb = with_status_panel(24'd0,beat_x0,beat_y);
// The moving tag also starts on the four-pixel grid; all lanes share its
// font lookup. Give the target tag priority if it crosses the mode panel.
wire [23:0] coordinate_panel_rgb = with_coordinate_panel(24'd0,beat_x0,beat_y,
                                                        coordinate_panel_active);
wire overlay_active = coordinate_panel_active || status_panel_active;
wire [23:0] overlay_rgb = coordinate_panel_active ? coordinate_panel_rgb : status_panel_rgb;
wire [23:0] out0 = overlay_active ? overlay_rgb :
                   display_pixel(display_mode, bright0, edge0, box0);
wire [23:0] out1 = overlay_active ? overlay_rgb :
                   display_pixel(display_mode, bright1, edge1, box1);
wire [23:0] out2 = overlay_active ? overlay_rgb :
                   display_pixel(display_mode, bright2, edge2, box2);
wire [23:0] out3 = overlay_active ? overlay_rgb :
                   display_pixel(display_mode, bright3, edge3, box3);

// The target is a red circle, not any arbitrary green region. Reject thin
// red fragments and strongly elongated red regions before publishing a box.
wire [12:0] red_box_width  = {1'b0,red_acc_xmax} - {1'b0,red_acc_xmin} + 13'd1;
wire [12:0] red_box_height = {1'b0,red_acc_ymax} - {1'b0,red_acc_ymin} + 13'd1;
wire red_circle_shape_ok =
    (red_box_width  >= 13'd1) &&
    (red_box_height >= 13'd1) &&
    // Allow the larger indicator LED to occupy more of the 1280x720 frame,
    // while still rejecting a frame-wide box caused by scattered red noise.
    (red_box_width  <= 13'd720) &&
    (red_box_height <= 13'd720) &&
    (red_box_width  <= (red_box_height * 5)) &&
    (red_box_height <= (red_box_width  * 5));
wire red_circle_valid =
    red_acc_valid &&
    (red_acc_pixels >= 32'd3) &&
    red_circle_shape_ok;

// The Ethernet preview keeps one sample out of each 4x4 input block. A
// one-pixel detection box would therefore disappear after 1280x720 -> 320x180
// decimation. Keep the measured box for large targets, but enlarge only tiny
// boxes to a visible 16x16 overlay in the source coordinate space.
wire [12:0] red_center_x =
    ({1'b0,red_acc_xmin} + {1'b0,red_acc_xmax}) >> 1;
wire [12:0] red_center_y =
    ({1'b0,red_acc_ymin} + {1'b0,red_acc_ymax}) >> 1;
wire [11:0] red_display_xmin =
    (red_box_width < 13'd16) ?
        ((red_center_x < 13'd8) ? 12'd0 :
         (red_center_x > 13'd1271) ? 12'd1264 :
         red_center_x[11:0] - 12'd8) :
        red_acc_xmin;
wire [11:0] red_display_xmax =
    (red_box_width < 13'd16) ?
        ((red_center_x < 13'd8) ? 12'd15 :
         (red_center_x > 13'd1271) ? 12'd1279 :
         red_center_x[11:0] + 12'd7) :
        red_acc_xmax;
wire [11:0] red_display_ymin =
    (red_box_height < 13'd16) ?
        ((red_center_y < 13'd8) ? 12'd0 :
         (red_center_y > 13'd711) ? 12'd704 :
         red_center_y[11:0] - 12'd8) :
        red_acc_ymin;
wire [11:0] red_display_ymax =
    (red_box_height < 13'd16) ?
        ((red_center_y < 13'd8) ? 12'd15 :
         (red_center_y > 13'd711) ? 12'd719 :
         red_center_y[11:0] + 12'd7) :
        red_acc_ymax;

always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_tlast   <= 1'b0;
        O_tuser   <= 1'b0;
        O_tdata   <= 96'd0;
        O_tvalid  <= 1'b0;
        x_grp     <= 11'd0;
        y_pos     <= 11'd0;
        top_prev2 <= 8'd0;
        top_prev1 <= 8'd0;
        mid_prev2 <= 8'd0;
        mid_prev1 <= 8'd0;
        bot_prev2 <= 8'd0;
        bot_prev1 <= 8'd0;
        red_box_valid <= 1'b0;
        red_box_xmin <= 12'd0;
        red_box_xmax <= 12'd0;
        red_box_ymin <= 12'd0;
        red_box_ymax <= 12'd0;
        red_hold_frames <= 4'd0;
    end else begin
        O_tlast  <= I_tlast;
        O_tuser  <= I_tuser;
        O_tvalid <= I_tvalid;

        if(I_tvalid) begin
            O_tdata <= {out0,out1,out2,out3};

            if(I_tuser) begin
                // Keep a confirmed box for a few frames so one dim frame
                // does not make the overlay flicker or disappear.
                if(red_circle_valid) begin
                    red_box_valid   <= 1'b1;
                    red_box_xmin    <= red_display_xmin;
                    red_box_xmax    <= red_display_xmax;
                    red_box_ymin    <= red_display_ymin;
                    red_box_ymax    <= red_display_ymax;
                    red_hold_frames <= 4'd3;
                end else if(red_box_valid && (red_hold_frames != 4'd0)) begin
                    red_box_valid   <= 1'b1;
                    red_hold_frames <= red_hold_frames - 1'b1;
                end else begin
                    red_box_valid   <= 1'b0;
                    red_hold_frames <= 4'd0;
                end
            end

            line2_mem[line_addr] <= mid_word;
            line1_mem[line_addr] <= bot_word;

            top_prev2 <= top2;
            top_prev1 <= top3;
            mid_prev2 <= mid2;
            mid_prev1 <= mid3;
            bot_prev2 <= gray2;
            bot_prev1 <= gray3;

            if(I_tuser) begin
                x_grp <= 11'd1;
                y_pos <= 11'd0;
            end else if(I_tlast) begin
                x_grp <= 11'd0;
                if(y_pos == IMG_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;
                top_prev2 <= 8'd0;
                top_prev1 <= 8'd0;
                mid_prev2 <= 8'd0;
                mid_prev1 <= 8'd0;
                bot_prev2 <= 8'd0;
                bot_prev1 <= 8'd0;
            end else begin
                x_grp <= x_grp + 1'b1;
            end
        end else begin
            O_tdata <= 96'd0;
        end
    end
end

endmodule

// Keys are synchronized and debounced in the ISP clock domain. Both press
// and release must remain stable; holding a key cannot repeat an action.
// A 320x180 row-chord profile avoids a second frame buffer. Seven sequential
// reads test each possible circle center during the following sensor frame.
// Reuse one four-digit double-dabble datapath for X and Y. Conversion takes
// 22 clocks after start, well before the coordinate panel at source row 56.
module track_coordinate_decimal (
    input clk, rst_n, start,
    input [10:0] x,
    input [9:0] y,
    output reg [15:0] x_bcd, y_bcd,
    output reg ready
);
    reg busy, axis_y;
    reg [3:0] remaining;
    reg [10:0] binary;
    reg [9:0] saved_y;
    reg [15:0] work, staged_x;
    reg [15:0] adjusted;
    integer nibble;
    always @* begin
        adjusted=work;
        for(nibble=0;nibble<4;nibble=nibble+1)
            if(work[nibble*4 +:4]>=5)
                adjusted[nibble*4 +:4]=work[nibble*4 +:4]+4'd3;
    end
    wire [15:0] shifted={adjusted[14:0],binary[10]};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0; axis_y<=0; remaining<=0; binary<=0; saved_y<=0;
            work<=0; staged_x<=0; x_bcd<=0; y_bcd<=0; ready<=0;
        end else if(start) begin
            busy<=1; axis_y<=0; remaining<=10; binary<=x; saved_y<=y;
            work<=0; ready<=0;
        end else if(busy) begin
            work<=shifted;
            binary<={binary[9:0],1'b0};
            if(remaining!=0) remaining<=remaining-1'b1;
            else if(!axis_y) begin
                staged_x<=shifted; binary<={1'b0,saved_y};
                work<=0; remaining<=10; axis_y<=1;
            end else begin
                x_bcd<=staged_x; y_bcd<=shifted;
                busy<=0; ready<=1;
            end
        end
    end
endmodule

module red_circle_stream #(
    parameter WIDTH = 320, HEIGHT = 180
) (
    input clk, rst_n, enable, valid, sof, eol,
    input [8:0] x,
    input [7:0] y,
    input [23:0] rgb,
    output reg box_valid, fresh,
    output reg [8:0] center_x, diameter,
    output reg [7:0] center_y
);
    reg [17:0] profile [0:HEIGHT-1];
    reg [8:0] run_start, run_end, run_hits;
    reg [8:0] row_start, row_end, row_hits;
    reg [2:0] gap;
    wire [8:0] r = {1'b0,rgb[23:16]};
    wire [8:0] g = {1'b0,rgb[15:8]};
    wire [8:0] b = {1'b0,rgb[7:0]};
    wire hit = r >= 9'd12 && r >= g+9'd3 && r >= b+9'd3 &&
               r >= g+(g >> 3) && !(x < 45 && y < 45);
    reg [8:0] ns, ne, nh, ps, pe, ph;
    reg [2:0] ng;
    wire [9:0] picked_width = {1'b0,pe}-{1'b0,ps}+10'd1;
    wire [9:0] picked_center = ({1'b0,ps}+{1'b0,pe}) >> 1;
    always @* begin
        ns = run_start; ne = run_end; nh = run_hits; ng = gap;
        ps = row_start; pe = row_end; ph = row_hits;
        if(sof) begin
            ns=0; ne=0; nh=0; ng=0; ps=0; pe=0; ph=0;
        end
        if(hit) begin
            if(nh == 0) ns=x;
            ne=x; nh=nh+1'b1; ng=0;
        end else if(nh != 0) begin
            if(ng == 1) begin
                if(nh > ph) begin ps=ns; pe=ne; ph=nh; end
                nh=0; ng=0;
            end else ng=ng+1'b1;
        end
        if(eol && nh > ph) begin ps=ns; pe=ne; ph=nh; end
    end
    always @(posedge clk) begin
        if(enable && valid && eol)
            profile[y] <= (ph >= 4 && {ph,1'b0} >= picked_width) ?
                {picked_center[8:0],picked_width[8:0]} : 18'd0;
    end

    reg profile_ready;
    reg [2:0] missed;
    reg scanning, good;
    reg [3:0] phase;
    reg [7:0] scan_y;
    reg [8:0] cx, width;
    reg [11:0] error_sum;
    reg best_valid;
    reg [8:0] best_x, best_width;
    reg [7:0] best_y;
    reg [11:0] best_error;
    reg [7:0] address;
    wire [17:0] entry = profile[address];
    wire [8:0] chord = entry[8:0];
    wire [8:0] chord_x = entry[17:9];
    reg [9:0] offset;
    reg [12:0] expected_chord;
    reg [12:0] actual_chord;
    reg [12:0] difference;
    reg check_ok;
    wire [9:0] center_dx = (cx > center_x) ? cx-center_x : center_x-cx;
    wire [8:0] center_dy = (scan_y > center_y) ? scan_y-center_y : center_y-scan_y;
    wire near_old = !box_valid ||
        (center_dx <= (diameter >> 1)+10'd24 &&
         center_dy <= (diameter >> 1)+10'd24 &&
         {width,1'b0} >= {1'b0,diameter} &&
         {1'b0,width} <= {1'b0,diameter}+({1'b0,diameter} >> 1));
    wire [12:0] total_error = {1'b0,error_sum}+difference;
    // Quantized normalized error is compared without division or multipliers.
    wire [1:0] score = total_error <= {4'b0,width} ? 2'd0 :
                      total_error <= {3'b0,width,1'b0} ? 2'd1 :
                      total_error <= {2'b0,width,2'b00} ? 2'd2 : 2'd3;
    wire [1:0] best_score = best_error <= {3'b0,best_width} ? 2'd0 :
                           best_error <= {2'b0,best_width,1'b0} ? 2'd1 :
                           best_error <= {1'b0,best_width,2'b00} ? 2'd2 : 2'd3;
    wire better = !best_valid || score < best_score ||
                  (score == best_score && width > best_width);
    always @* begin
        offset=0;
        case(phase)
            1,2: offset={1'b0,width} >> 2;
            3,4: offset=({1'b0,width} >> 2)+({1'b0,width} >> 3);
            5,6: offset={1'b0,width} >> 1;
            default: offset=0;
        endcase
        address = scan_y;
        if(phase != 0)
            address = phase[0] ? scan_y-offset : scan_y+offset;
        actual_chord={1'b0,chord,3'b000};
        expected_chord=0;
        if(phase==1 || phase==2) expected_chord=({1'b0,width,3'b000}-{4'b0,width});
        if(phase==3 || phase==4) expected_chord=({2'b0,width,2'b00}+{4'b0,width});
        difference=(actual_chord>expected_chord) ?
            actual_chord-expected_chord : expected_chord-actual_chord;
        check_ok=1;
        if(phase==5 || phase==6)
            check_ok=({chord,2'b00} <= ({2'b0,width}+{1'b0,width,1'b0}));
        else if(phase != 0)
            check_ok=({chord,1'b0} >= {1'b0,width}) &&
                     ({chord,2'b00} <= {2'b0,width}+{width,2'b00}) &&
                     ({1'b0,chord_x}+(width >> 3)+10'd2 >= {1'b0,cx}) &&
                     ({1'b0,cx}+(width >> 3)+10'd2 >= {1'b0,chord_x});
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            run_start<=0; run_end<=0; run_hits<=0;
            row_start<=0; row_end<=0; row_hits<=0; gap<=0;
            profile_ready<=0; missed<=0;
            box_valid<=0; fresh<=0; center_x<=0; center_y<=0; diameter<=0;
            scanning<=0; scan_y<=0; phase<=0; good<=0;
            cx<=0; width<=0; error_sum<=0;
            best_valid<=0; best_x<=0; best_y<=0; best_width<=0; best_error<=0;
        end else if(!enable) begin
            run_hits<=0; row_hits<=0; gap<=0;
            profile_ready<=0; box_valid<=0; fresh<=0;
            scanning<=0; best_valid<=0; missed<=0;
        end else begin
            if(valid) begin
                run_start<=ns; run_end<=ne; run_hits<=eol ? 9'd0 : nh;
                row_start<=ps; row_end<=pe; row_hits<=eol ? 9'd0 : ph;
                gap<=eol ? 3'd0 : ng;
            end
            if(valid && sof) begin
                if(profile_ready) begin
                    fresh<=best_valid;
                    if(best_valid) begin
                        box_valid<=1; missed<=0;
                        center_x<=best_x; center_y<=best_y; diameter<=best_width;
                    end else if(box_valid && missed < 3) begin
                        missed<=missed+1'b1;
                    end else begin
                        box_valid<=0; missed<=0;
                    end
                end
                best_valid<=0;
            end else if(valid && eol && y==HEIGHT-1) begin
                profile_ready<=1;
                scanning<=1; scan_y<=0; phase<=0;
            end else if(scanning) begin
                if(phase==0) begin
                    cx<=chord_x; width<=chord; error_sum<=0;
                    good<=1;
                    if(chord >= 8 && chord <= 170 &&
                       scan_y >= (chord >> 1) && scan_y+(chord >> 1)<HEIGHT) begin
                        phase<=7;
                    end else if(scan_y==HEIGHT-1) scanning<=0;
                    else scan_y<=scan_y+1'b1;
                end else if(phase==7) begin
                    good<=near_old; phase<=1;
                end else begin
                    good<=good && check_ok;
                    error_sum<=total_error[11:0];
                    if(phase==6) begin
                        if(good && check_ok && better) begin
                            best_valid<=1; best_x<=cx; best_y<=scan_y;
                            best_width<=width; best_error<=total_error[11:0];
                        end
                        phase<=0;
                        if(scan_y==HEIGHT-1) scanning<=0;
                        else scan_y<=scan_y+1'b1;
                    end else phase<=phase+1'b1;
                end
            end
        end
    end
endmodule

module video_key_press #(
    parameter DEBOUNCE_CYCLES = 1200000
) (
    input I_clk, I_rst_n, I_key_n,
    output reg O_press
);
    localparam COUNT_WIDTH = (DEBOUNCE_CYCLES > 1) ? $clog2(DEBOUNCE_CYCLES) : 1;
    (* async_reg = "true" *) reg key_meta, key_sync;
    reg stable_n;
    reg [COUNT_WIDTH-1:0] stable_count;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
            stable_n <= 1'b1;
            stable_count <= 0;
            O_press <= 1'b0;
        end else begin
            key_meta <= I_key_n;
            key_sync <= key_meta;
            O_press <= 1'b0;
            if(key_sync == stable_n) begin
                stable_count <= 0;
            end else if(stable_count == DEBOUNCE_CYCLES - 1) begin
                stable_n <= key_sync;
                stable_count <= 0;
                O_press <= !key_sync;
            end else begin
                stable_count <= stable_count + 1'b1;
            end
        end
    end
endmodule

module video_key_modes #(
    parameter DEBOUNCE_CYCLES = 1200000
) (
    input I_clk, I_rst_n,
    input [3:0] I_button,
    input I_frame_start,
    output [1:0] O_mode,
    output O_tracking
);
    wire key1_press, key2_press, key3_press;
    reg [1:0] base_mode;
    reg detection_enabled;
    reg [1:0] active_mode;
    reg tracking_enabled, active_tracking;
    wire [1:0] requested_mode = (detection_enabled || tracking_enabled) ? 2'd3 : base_mode;
    video_key_press #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_key1 (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_key_n(I_button[0]),
        .O_press(key1_press)
    );
    video_key_press #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_key2 (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_key_n(I_button[1]),
        .O_press(key2_press)
    );
    video_key_press #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) u_key3 (
        .I_clk(I_clk), .I_rst_n(I_rst_n), .I_key_n(I_button[2]),
        .O_press(key3_press)
    );
    // SOF's first beat must already use the newly committed selection.
    assign O_mode = I_frame_start ? requested_mode : active_mode;
    assign O_tracking = I_frame_start ? tracking_enabled : active_tracking;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            base_mode <= 2'd0;
            detection_enabled <= 1'b0;
            active_mode <= 2'd0;
            tracking_enabled <= 1'b0;
            active_tracking <= 1'b0;
        end else begin
            if(key1_press) begin
                base_mode <= (base_mode == 2'd2) ? 2'd0 : base_mode + 1'b1;
                detection_enabled <= 1'b0;
                tracking_enabled <= 1'b0;
            end else if(key2_press) begin
                detection_enabled <= tracking_enabled ? 1'b1 : !detection_enabled;
                tracking_enabled <= 1'b0;
            end else if(key3_press) begin
                tracking_enabled <= !tracking_enabled;
                detection_enabled <= 1'b0;
            end
            if(I_frame_start) begin
                active_mode <= requested_mode;
                active_tracking <= tracking_enabled;
            end
        end
    end
endmodule

// Candidate geometry is already qualified by the row tracker.
// Expand tiny boxes for the decimated preview and bridge three missed frames.
module lamp_box_hold #(
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter REQUIRE_CONFIRM = 0
) (
    input clk, rst_n, publish,
    input candidate_valid,
    input [11:0] candidate_xmin, candidate_xmax,
    input [11:0] candidate_ymin, candidate_ymax,
    output reg box_valid,
    output reg [11:0] box_xmin, box_xmax,
    output reg [11:0] box_ymin, box_ymax
);
    reg [1:0] hold_frames;
    reg pending_valid;
    reg [11:0] pending_xmin, pending_xmax, pending_ymin, pending_ymax;
    // A green candidate must recur nearby on the next sensor frame. A small
    // native-pixel margin permits motion without accepting jumping noise.
    wire candidate_near =
        ({1'b0,candidate_xmin} <= {1'b0,pending_xmax} + 13'd16) &&
        ({1'b0,pending_xmin} <= {1'b0,candidate_xmax} + 13'd16) &&
        ({1'b0,candidate_ymin} <= {1'b0,pending_ymax} + 13'd16) &&
        ({1'b0,pending_ymin} <= {1'b0,candidate_ymax} + 13'd16);
    wire confirmed = candidate_valid &&
                     (!REQUIRE_CONFIRM || (pending_valid && candidate_near));
    wire [12:0] width = {1'b0,candidate_xmax} - {1'b0,candidate_xmin} + 13'd1;
    wire [12:0] height = {1'b0,candidate_ymax} - {1'b0,candidate_ymin} + 13'd1;
    wire [12:0] center_x = ({1'b0,candidate_xmin} + {1'b0,candidate_xmax}) >> 1;
    wire [12:0] center_y = ({1'b0,candidate_ymin} + {1'b0,candidate_ymax}) >> 1;
    wire [11:0] small_xmin = (center_x < 13'd8) ? 12'd0 :
                            (center_x > IMG_WIDTH - 9) ? IMG_WIDTH - 16 :
                            center_x[11:0] - 12'd8;
    wire [11:0] small_ymin = (center_y < 13'd8) ? 12'd0 :
                            (center_y > IMG_HEIGHT - 9) ? IMG_HEIGHT - 16 :
                            center_y[11:0] - 12'd8;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            box_valid <= 1'b0;
            box_xmin <= 12'd0;
            box_xmax <= 12'd0;
            box_ymin <= 12'd0;
            box_ymax <= 12'd0;
            hold_frames <= 2'd0;
            pending_valid <= 1'b0;
            pending_xmin <= 12'd0;
            pending_xmax <= 12'd0;
            pending_ymin <= 12'd0;
            pending_ymax <= 12'd0;
        end else if(publish) begin
            pending_valid <= candidate_valid;
            if(candidate_valid) begin
                pending_xmin <= candidate_xmin;
                pending_xmax <= candidate_xmax;
                pending_ymin <= candidate_ymin;
                pending_ymax <= candidate_ymax;
            end
            if(confirmed) begin
                box_valid <= 1'b1;
                box_xmin <= (width < 13'd16) ? small_xmin : candidate_xmin;
                box_xmax <= (width < 13'd16) ? small_xmin + 12'd15 : candidate_xmax;
                box_ymin <= (height < 13'd16) ? small_ymin : candidate_ymin;
                box_ymax <= (height < 13'd16) ? small_ymin + 12'd15 : candidate_ymax;
                hold_frames <= 2'd3;
            end else if(box_valid && (hold_frames != 2'd0)) begin
                hold_frames <= hold_frames - 1'b1;
            end else begin
                box_valid <= 1'b0;
                hold_frames <= 2'd0;
            end
        end
    end
endmodule

// Resource-bounded run tracker, not full connected-component labeling.
// Keep the strongest run on each row; join nearby runs on adjacent rows,
// and retain the largest qualifying candidate over the frame.
module red_lamp_row_tracker (
    input clk,
    input rst_n,
    input valid,
    input sof,
    input eol,
    input [11:0] y,
    input [2:0] hit_count,
    input [11:0] hit_xmin,
    input [11:0] hit_xmax,
    output reg candidate_valid,
    output reg [11:0] candidate_xmin,
    output reg [11:0] candidate_xmax,
    output reg [11:0] candidate_ymin,
    output reg [11:0] candidate_ymax,
    output reg [31:0] candidate_pixels
);
    reg run_valid, run_gap;
    reg [11:0] run_xmin, run_xmax, run_pixels;
    reg [11:0] row_xmin, row_xmax, row_pixels;
    reg next_run_valid, next_run_gap;
    reg [11:0] next_run_xmin, next_run_xmax, next_run_pixels;
    reg [11:0] pick_xmin, pick_xmax, pick_pixels;

    reg active_valid;
    reg [11:0] active_xmin, active_xmax, active_ymin, active_ymax;
    reg [11:0] last_xmin, last_xmax;
    reg [31:0] active_pixels;
    reg best_valid;
    reg [11:0] best_xmin, best_xmax, best_ymin, best_ymax;
    reg [31:0] best_pixels;

    function blob_ok;
        input [11:0] xmin, xmax, ymin, ymax;
        input [31:0] pixels;
        reg [12:0] w, h;
        begin
            w = {1'b0,xmax} - {1'b0,xmin} + 13'd1;
            h = {1'b0,ymax} - {1'b0,ymin} + 13'd1;
            blob_ok = (pixels >= 32'd3) && (w <= 13'd720) &&
                      (h <= 13'd720) && (w <= h * 5) && (h <= w * 5);
        end
    endfunction
    wire active_ok = active_valid &&
        blob_ok(active_xmin, active_xmax, active_ymin, active_ymax, active_pixels);
    wire join_row = active_valid &&
        ({1'b0,y} <= {1'b0,active_ymax} + 13'd2) &&
        ({1'b0,pick_xmin} <= {1'b0,last_xmax} + 13'd8) &&
        ({1'b0,last_xmin} <= {1'b0,pick_xmax} + 13'd8);

    // One empty group may bridge a small hole in the colored lamp rim.
    always @* begin
        next_run_valid = sof ? 1'b0 : run_valid;
        next_run_gap = sof ? 1'b0 : run_gap;
        next_run_xmin = sof ? 12'd0 : run_xmin;
        next_run_xmax = sof ? 12'd0 : run_xmax;
        next_run_pixels = sof ? 12'd0 : run_pixels;
        pick_xmin = sof ? 12'd0 : row_xmin;
        pick_xmax = sof ? 12'd0 : row_xmax;
        pick_pixels = sof ? 12'd0 : row_pixels;

        if(hit_count != 3'd0) begin
            if(!next_run_valid) begin
                next_run_valid = 1'b1;
                next_run_xmin = hit_xmin;
                next_run_pixels = {9'd0,hit_count};
            end else begin
                next_run_pixels = next_run_pixels + {9'd0,hit_count};
            end
            next_run_xmax = hit_xmax;
            next_run_gap = 1'b0;
        end else if(next_run_valid) begin
            if(next_run_gap) begin
                if(next_run_pixels > pick_pixels) begin
                    pick_xmin = next_run_xmin;
                    pick_xmax = next_run_xmax;
                    pick_pixels = next_run_pixels;
                end
                next_run_valid = 1'b0;
                next_run_pixels = 12'd0;
            end else begin
                next_run_gap = 1'b1;
            end
        end
        if(eol && next_run_valid && (next_run_pixels > pick_pixels)) begin
            pick_xmin = next_run_xmin;
            pick_xmax = next_run_xmax;
            pick_pixels = next_run_pixels;
        end
    end

    // Include the unfinished candidate, including a target on the last row.
    always @* begin
        candidate_valid = best_valid;
        candidate_xmin = best_xmin;
        candidate_xmax = best_xmax;
        candidate_ymin = best_ymin;
        candidate_ymax = best_ymax;
        candidate_pixels = best_pixels;
        if(active_ok && (!best_valid || (active_pixels > best_pixels))) begin
            candidate_valid = 1'b1;
            candidate_xmin = active_xmin;
            candidate_xmax = active_xmax;
            candidate_ymin = active_ymin;
            candidate_ymax = active_ymax;
            candidate_pixels = active_pixels;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            run_valid <= 1'b0;
            run_gap <= 1'b0;
            run_xmin <= 12'd0;
            run_xmax <= 12'd0;
            run_pixels <= 12'd0;
            row_xmin <= 12'd0;
            row_xmax <= 12'd0;
            row_pixels <= 12'd0;
            active_valid <= 1'b0;
            active_xmin <= 12'd0;
            active_xmax <= 12'd0;
            active_ymin <= 12'd0;
            active_ymax <= 12'd0;
            last_xmin <= 12'd0;
            last_xmax <= 12'd0;
            active_pixels <= 32'd0;
            best_valid <= 1'b0;
            best_xmin <= 12'd0;
            best_xmax <= 12'd0;
            best_ymin <= 12'd0;
            best_ymax <= 12'd0;
            best_pixels <= 32'd0;
        end else if(valid) begin
            run_valid <= eol ? 1'b0 : next_run_valid;
            run_gap <= eol ? 1'b0 : next_run_gap;
            run_xmin <= next_run_xmin;
            run_xmax <= next_run_xmax;
            run_pixels <= eol ? 12'd0 : next_run_pixels;
            row_xmin <= pick_xmin;
            row_xmax <= pick_xmax;
            row_pixels <= eol ? 12'd0 : pick_pixels;

            if(sof) begin
                active_valid <= 1'b0;
                active_pixels <= 32'd0;
                best_valid <= 1'b0;
                best_pixels <= 32'd0;
            end else if(eol) begin
                if(pick_pixels != 12'd0) begin
                    if(join_row) begin
                        if(pick_xmin < active_xmin) active_xmin <= pick_xmin;
                        if(pick_xmax > active_xmax) active_xmax <= pick_xmax;
                        active_ymax <= y;
                        active_pixels <= active_pixels + {20'd0,pick_pixels};
                    end else begin
                        if(active_ok && (!best_valid || (active_pixels > best_pixels))) begin
                            best_valid <= 1'b1;
                            best_xmin <= active_xmin;
                            best_xmax <= active_xmax;
                            best_ymin <= active_ymin;
                            best_ymax <= active_ymax;
                            best_pixels <= active_pixels;
                        end
                        active_valid <= 1'b1;
                        active_xmin <= pick_xmin;
                        active_xmax <= pick_xmax;
                        active_ymin <= y;
                        active_ymax <= y;
                        active_pixels <= {20'd0,pick_pixels};
                    end
                    last_xmin <= pick_xmin;
                    last_xmax <= pick_xmax;
                end else if(active_valid &&
                            ({1'b0,y} > {1'b0,active_ymax} + 13'd1)) begin
                    if(active_ok && (!best_valid || (active_pixels > best_pixels))) begin
                        best_valid <= 1'b1;
                        best_xmin <= active_xmin;
                        best_xmax <= active_xmax;
                        best_ymin <= active_ymin;
                        best_ymax <= active_ymax;
                        best_pixels <= active_pixels;
                    end
                    active_valid <= 1'b0;
                    active_pixels <= 32'd0;
                end
            end
        end
    end
endmodule
