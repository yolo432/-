module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0
) (
    input wire I_clk, I_rst_n,
    input wire I_video_vsync, I_video_hsync, I_video_de,
    input wire I_video_user, I_video_last,
    input wire [3:0] I_debug_status,
    input wire [15:0] I_latency_ms,
    input wire I_latency_valid,
    output wire O_video_rd_en,
    input wire [23:0] I_video_rd_data,
    output reg O_hdmi_vsync, O_hdmi_hsync, O_hdmi_de,
    output reg [23:0] O_hdmi_data
);
    reg [11:0] x, y, x_1, y_1, x_2, y_2;
    reg [1:0] vs_pipe, hs_pipe, de_pipe;
    reg [23:0] rgb_1, rgb_2;
    assign O_video_rd_en = I_video_de;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x <= 0; y <= 0; x_1 <= 0; y_1 <= 0; x_2 <= 0; y_2 <= 0;
        end else begin
            x_1 <= x; y_1 <= y; x_2 <= x_1; y_2 <= y_1;
            if(I_video_user) begin x <= 0; y <= 0; end
            else if(I_video_de) begin
                if(I_video_last) begin x <= 0; y <= y + 1'b1; end
                else x <= x + 1'b1;
            end
        end
    end

    wire [14:0] logo_address = y_2 * 15'd160 + x_2;
    wire [24:0] logo_pixel;
    anlogic_logo_rom u_anlogic_logo_rom (
        .I_addr(logo_address), .O_pixel(logo_pixel)
    );
    wire logo_hit = de_pipe[1] && x_2 < 160 && y_2 < 160 && logo_pixel[24];

    // One serial conversion per video frame replaces the old per-pixel
    // divisions and the disconnected 0/1/2 object counter.
    reg convert_start, convert_busy, digits_ready;
    reg [3:0] convert_bit;
    reg [15:0] binary_shift;
    reg [19:0] bcd_work, bcd_digits;
    reg [19:0] bcd_adjust;
    wire [19:0] bcd_next = {bcd_adjust[18:0],binary_shift[15]};
    integer i;
    always @(*) begin
        bcd_adjust = bcd_work;
        for(i=0;i<5;i=i+1)
            if(bcd_work[i*4 +: 4] >= 5)
                bcd_adjust[i*4 +: 4] = bcd_work[i*4 +: 4] + 4'd3;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            convert_start <= 0; convert_busy <= 0; digits_ready <= 0;
            convert_bit <= 0; binary_shift <= 0; bcd_work <= 0; bcd_digits <= 0;
        end else begin
            // The latency register updates on I_video_user; read it next edge.
            convert_start <= I_video_user;
            if(convert_start) begin
                digits_ready <= 0;
                convert_busy <= I_latency_valid;
                convert_bit <= 0;
                binary_shift <= I_latency_ms;
                bcd_work <= 0;
            end else if(convert_busy) begin
                bcd_work <= bcd_next;
                binary_shift <= {binary_shift[14:0],1'b0};
                convert_bit <= convert_bit + 1'b1;
                if(convert_bit == 15) begin
                    bcd_digits <= bcd_next;
                    digits_ready <= 1;
                    convert_busy <= 0;
                end
            end
        end
    end

    function [7:0] hud_character;
        input [3:0] index;
        reg [3:0] digit;
        begin
            case(index)
                4: digit=bcd_digits[19:16];
                5: digit=bcd_digits[15:12];
                6: digit=bcd_digits[11:8];
                7: digit=bcd_digits[7:4];
                default: digit=bcd_digits[3:0];
            endcase
            case(index)
                0: hud_character="L";
                1: hud_character="A";
                2: hud_character="T";
                3: hud_character=":";
                9: hud_character="M";
                10: hud_character="S";
                default: hud_character=(digits_ready && I_latency_valid) ?
                                       {4'h3,digit} : "-";
            endcase
        end
    endfunction
    function [4:0] hud_font;
        input [7:0] ch;
        input [2:0] row;
        reg [34:0] glyph;
        begin
            case(ch)
                "L": glyph=35'b10000_10000_10000_10000_10000_10000_11111;
                "A": glyph=35'b01110_10001_10001_11111_10001_10001_10001;
                "T": glyph=35'b11111_00100_00100_00100_00100_00100_00100;
                ":": glyph=35'b00000_00100_00100_00000_00100_00100_00000;
                "M": glyph=35'b10001_11011_10101_10101_10001_10001_10001;
                "S": glyph=35'b01111_10000_10000_01110_00001_00001_11110;
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
                default: glyph=0;
            endcase
            hud_font = row < 7 ? glyph[34-row*5 -: 5] : 5'd0;
        end
    endfunction
    wire hud_hit = de_pipe[1] && x_2 >= 16 && x_2 < 384 &&
                                  y_2 >= 176 && y_2 < 216;
    wire [11:0] hud_x = x_2 - 12'd24;
    wire [11:0] hud_y = y_2 - 12'd180;
    wire [4:0] hud_bits = hud_font(hud_character(hud_x[8:5]),hud_y[4:2]);
    wire hud_ink = x_2 >= 24 && x_2 < 376 && y_2 >= 180 && y_2 < 208 &&
                   hud_x[4:2] < 5 && hud_bits[4-hud_x[4:2]];

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            vs_pipe <= 0; hs_pipe <= 0; de_pipe <= 0;
            rgb_1 <= 0; rgb_2 <= 0;
            O_hdmi_vsync <= 0; O_hdmi_hsync <= 0; O_hdmi_de <= 0;
            O_hdmi_data <= 0;
        end else begin
            vs_pipe <= {vs_pipe[0],I_video_vsync};
            hs_pipe <= {hs_pipe[0],I_video_hsync};
            de_pipe <= {de_pipe[0],I_video_de};
            rgb_1 <= I_video_rd_data; rgb_2 <= rgb_1;
            O_hdmi_vsync <= vs_pipe[1];
            O_hdmi_hsync <= hs_pipe[1];
            O_hdmi_de <= de_pipe[1];
            if(!de_pipe[1]) O_hdmi_data <= 0;
            else if(DEBUG_MODE == 1) begin
                case(x_2 >> 7)
                    0: O_hdmi_data <= 24'hff0000;
                    1: O_hdmi_data <= 24'h00ff00;
                    2: O_hdmi_data <= 24'h0000ff;
                    3: O_hdmi_data <= 24'hffff00;
                    4: O_hdmi_data <= 24'h00ffff;
                    5: O_hdmi_data <= 24'hff00ff;
                    6: O_hdmi_data <= 24'hffffff;
                    default: O_hdmi_data <= 24'h202020;
                endcase
            end else if(DEBUG_MODE == 2) begin
                case(I_debug_status)
                    0: O_hdmi_data <= 24'h000000;
                    1: O_hdmi_data <= 24'hff0000;
                    3: O_hdmi_data <= 24'hffff00;
                    7: O_hdmi_data <= 24'h00ff00;
                    15: O_hdmi_data <= I_video_rd_data;
                    default: O_hdmi_data <= 24'h0000ff;
                endcase
            end else if(DEBUG_MODE == 3) begin
                if(x_2 < 256) O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                else if(x_2 < 512) O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                else if(x_2 < 768) O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                else O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
            end else if(logo_hit) O_hdmi_data <= logo_pixel[23:0];
            else if(hud_hit)
                O_hdmi_data <= hud_ink ? 24'hfff200 : 24'h101820;
            else O_hdmi_data <= rgb_2;
        end
    end
endmodule
