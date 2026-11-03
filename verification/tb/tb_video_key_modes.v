`timescale 1ns/1ps
module tb_video_key_modes;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg [3:0] keys = 4'b1111;
    reg valid = 0, sof = 0, eol = 0;
    reg [95:0] data = 0;
    wire [95:0] result;
    wire result_valid, result_sof, result_eol;
    reg [3:0] control_keys = 4'b1111;
    reg control_sof = 0;
    wire [1:0] mode;
    wire tracking;
    integer x, y, lane, white_edges, colored_edges;
    reg request_midframe = 0;
    reg [23:0] p, q, expected;
    edge_detect_96 #(
        .IMG_WIDTH(64), .IMG_HEIGHT(16), .KEY_DEBOUNCE_CYCLES(4)
    ) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_button(keys),
        .I_tlast(eol), .I_tuser(sof), .I_tvalid(valid),
        .I_tdata(data), .I_tready(1'b1),
        .O_tdata(result), .O_tvalid(result_valid),
        .O_tuser(result_sof), .O_tlast(result_eol)
    );
    video_key_modes #(.DEBOUNCE_CYCLES(4)) controller (
        .I_clk(clk), .I_rst_n(rst_n), .I_button(control_keys),
        .I_frame_start(control_sof), .O_mode(mode), .O_tracking(tracking)
    );
    task tick;
        input integer count;
        integer n;
        begin
            for(n = 0; n < count; n = n + 1) @(negedge clk);
        end
    endtask
    task check_mode;
        input [1:0] wanted;
        begin
            #1;
            if(mode !== wanted)
                $fatal(1,"mode expected=%0d actual=%0d",wanted,mode);
        end
    endtask
    task commit_mode;
        input [1:0] wanted;
        begin
            control_sof = 1;
            // Check the selection before the first pixel is sampled.
            check_mode(wanted);
            tick(1);
            control_sof = 0;
            check_mode(wanted);
        end
    endtask
    task control_press;
        input integer key;
        begin
            control_keys[key] = 0;
            tick(12);
            control_keys[key] = 1;
            tick(12);
        end
    endtask
    task video_press;
        input integer key;
        begin
            keys[key] = 0;
            tick(12);
            keys[key] = 1;
            tick(12);
        end
    endtask
    function [7:0] lifted;
        input [7:0] c;
        integer value;
        begin
            value = c * 3 / 2;
            lifted = (value > 255) ? 255 : value;
        end
    endfunction
    function [7:0] quantized;
        input [7:0] c;
        integer i, level, distance, best_distance;
        begin
            best_distance = 256;
            for(i=0; i<8; i=i+1) begin
                level = (i * 255 + 3) / 7;
                distance = (c > level) ? c-level : level-c;
                if(distance <= best_distance) begin
                    quantized = level;
                    best_distance = distance;
                end
            end
        end
    endfunction
    function [23:0] quantized_rgb;
        input [23:0] rgb;
        integer lum, delta, channel, value;
        begin
            lum = (rgb[23:16] + 2*rgb[15:8] + rgb[7:0]) / 4;
            delta = integer'(quantized(lum)) - lum;
            for(channel=0; channel<3; channel=channel+1) begin
                value = integer'(rgb[channel*8 +:8]) + delta;
                quantized_rgb[channel*8 +:8] =
                    (value < 0) ? 0 : (value > 255) ? 255 : value;
            end
        end
    endfunction
    function [23:0] pixel;
        input integer xx, yy, scene;
        begin
            case(scene)
                0: pixel = 24'h314f72;
                // Color edges must survive binary mode's Sobel threshold.
                1: pixel = (xx < 32) ? 24'h000000 : 24'hff0000;
                2: pixel = {8'(xx * 4), 8'(yy * 16), 8'(255 - xx * 4)};
                default: pixel = 24'h000000;
            endcase
        end
    endfunction
    task frame;
        input integer selected, scene;
        begin
            white_edges = 0;
            colored_edges = 0;
            for(y = 0; y < 16; y = y + 1)
                for(x = 0; x < 64; x = x + 4) begin
                    @(negedge clk);
                    valid = 1;
                    sof = (x == 0) && (y == 0);
                    eol = (x == 60);
                    if(request_midframe && x == 0 && y == 4) keys[0] = 0;
                    if(request_midframe && x == 0 && y == 8) keys[0] = 1;
                    for(lane = 0; lane < 4; lane = lane + 1)
                        data[95-lane*24 -: 24] = pixel(x+lane,y,scene);
                    @(posedge clk);
                    #1;
                    if(!result_valid || result_sof !== sof || result_eol !== eol)
                        $fatal(1,"stream sideband alignment");
                    if(dut.display_mode !== selected[1:0])
                        $fatal(1,"unexpected video mode=%0d",dut.display_mode);
                    for(lane = 0; lane < 4; lane = lane + 1) begin
                        p = data[95-lane*24 -: 24];
                        q = result[95-lane*24 -: 24];
                        expected = {lifted(p[23:16]),lifted(p[15:8]),lifted(p[7:0])};
                        if(selected == 1) begin
                            if(q !== 24'h000000 && q !== 24'hffffff)
                                $fatal(1,"binary mode contains color/unknown=%h",q);
                            if(q == 24'hffffff) begin
                                white_edges = white_edges + 1;
                                if(p == 24'hff0000) colored_edges = colored_edges + 1;
                            end
                        end else begin
                            if(selected == 2)
                                expected = quantized_rgb(expected);
                            if(q !== expected)
                                $fatal(1,"pixel mode=%0d at (%0d,%0d) expected=%h got=%h",
                                    selected,x+lane,y,expected,q);
                        end
                    end
                end
            @(negedge clk);
            valid = 0; sof = 0; eol = 0;
            if(selected == 1 && (white_edges == 0 || colored_edges == 0))
                $fatal(1,"colored object produced no white Sobel edge");
        end
    endtask
    initial begin
        tick(3);
        rst_n = 1;
        check_mode(0);
        // Neither a short press nor a short release may advance the cycle.
        control_keys[0] = 0; tick(2);
        control_keys[0] = 1; tick(12);
        commit_mode(0);
        control_keys[0] = 0; tick(20);
        check_mode(0);
        commit_mode(1);
        tick(30);
        commit_mode(1);
        control_keys[0] = 1; tick(2);
        control_keys[0] = 0; tick(12);
        commit_mode(1);
        control_keys[0] = 1; tick(12);
        control_press(0);
        check_mode(1);
        commit_mode(2);
        control_press(1); check_mode(2); commit_mode(3);
        control_press(1); check_mode(3); commit_mode(2);
        control_press(0); commit_mode(0);
        control_press(1); commit_mode(3);
        control_press(0); commit_mode(1);
        control_press(2); check_mode(1); commit_mode(3);
        if(!tracking) $fatal(1,"KEY3 did not enable tracking");
        control_press(2); commit_mode(1);
        if(tracking) $fatal(1,"KEY3 did not exit tracking");
        control_press(2); commit_mode(3);
        control_press(1); commit_mode(3);
        if(tracking) $fatal(1,"KEY2 did not restore lamp detection");
        control_press(1); commit_mode(1);
        control_press(3); commit_mode(1);
        // KEY1 has deterministic priority for simultaneous presses.
        control_keys[1:0] = 0; tick(12);
        control_keys[1:0] = 3; tick(12);
        commit_mode(2);
        frame(0,0);
        video_press(0);
        if(dut.display_mode !== 0) $fatal(1,"mode changed without a frame");
        // An invalid SOF must not commit a pending switch.
        sof = 1; tick(1); sof = 0;
        if(dut.display_mode !== 0) $fatal(1,"invalid SOF changed mode");
        frame(1,1);
        video_press(0); frame(2,2);
        video_press(1); frame(3,0);
        video_press(1); frame(2,2);
        video_press(0); frame(0,0);
        request_midframe = 1;
        frame(0,0);
        request_midframe = 0;
        frame(1,1);
        $display("PASS debounce, held/released keys, reserved keys, frame-boundary modes, natural RGB, binary colored edges, chroma-preserving 8-level luminance quantization");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1,"timeout");
    end
endmodule
