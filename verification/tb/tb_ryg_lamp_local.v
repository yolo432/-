`timescale 1ns/1ps
module tb_ryg_lamp_local;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg valid = 0, sof = 0, eol = 0;
    reg [95:0] data = 0;
    reg [3:0] keys = 4'b1111;
    integer xx, yy, lane, failures = 0;
    integer rr, gg, bb, votes;
    integer red_labels, yellow_labels, green_labels;
    integer red_borders, yellow_borders, green_borders;
    reg inspect_overlay = 0;
    reg [23:0] sample;
    edge_detect_96 #(.IMG_WIDTH(320), .IMG_HEIGHT(240), .KEY_DEBOUNCE_CYCLES(4)) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_tlast(eol), .I_tuser(sof),
        .I_button(keys),
        .I_tdata(data), .I_tvalid(valid), .I_tready(1'b1)
    );

    function [23:0] pixel;
        input integer x, y, scene;
        begin
            // Strong achromatic edges must not become green detections.
            pixel = ((x % 16) < 8) ? 24'h202020 : 24'h505050;
            if(scene == 1) begin
                if((x >= 200) && (x <= 207) && (y >= 184) && (y <= 191))
                    pixel = 24'hb05010;
                if((x >= 240) && (x <= 247) && (y >= 204) && (y <= 211))
                    pixel = 24'hb0b010;
                if((x >= 280) && (x <= 287) && (y >= 224) && (y <= 231))
                    pixel = 24'h10b010;
                // Smaller, remote components of each color must not stretch boxes.
                if((x >= 184) && (x <= 185)) begin
                    if((y >= 60) && (y <= 61)) pixel = 24'hb05010;
                    if((y >= 100) && (y <= 101)) pixel = 24'hb0b010;
                    if((y >= 140) && (y <= 141)) pixel = 24'h10b010;
                end
            end
            if((scene == 2) && (x >= 317) && (y == 239))
                pixel = 24'h606008;
            if((scene == 3) && (x >= 317) && (y == 239))
                pixel = 24'h086008;
            if((scene == 4) && (x >= 32) && (x < 40)) begin
                if((y >= 32) && (y < 40)) pixel = 24'hb05010;
                if((y >= 48) && (y < 56)) pixel = 24'hb0b010;
                if((y >= 64) && (y < 72)) pixel = 24'h10b010;
            end
            // Isolated targets exercise every four-pixel lane.
            if((scene >= 5) && (scene <= 8) &&
               (x == 240 + scene - 5) && (y >= 200) && (y <= 202))
                pixel = 24'h606008;
            if((scene >= 9) && (scene <= 12) &&
               (x == 240 + scene - 9) && (y >= 200) && (y <= 202))
                pixel = 24'h086008;
            if(scene == 13) begin
                pixel = 24'hf0fff0;
                if((x >= 190) && (x <= 210) && (y >= 190) && (y <= 210))
                    pixel = 24'h506656;
                if((x >= 260) && (x <= 280) && (y >= 190) && (y <= 210))
                    pixel = 24'ha0b8a4;
            end
        end
    endfunction

    task frame;
        input integer scene;
        begin
            for(yy = 0; yy < 240; yy = yy + 1) begin
                for(xx = 0; xx < 320; xx = xx + 4) begin
                    @(negedge clk);
                    valid = 1;
                    sof = (xx == 0) && (yy == 0);
                    eol = (xx == 316);
                    for(lane = 0; lane < 4; lane = lane + 1)
                        data[95-lane*24 -: 24] = pixel(xx+lane,yy,scene);
                    @(posedge clk);
                    #1;
                    if(inspect_overlay) begin
                        for(lane = 0; lane < 4; lane = lane + 1) begin
                            sample = dut.O_tdata[95-lane*24 -: 24];
                            if(sample == 24'hffffff) begin
                                if((yy >= 171) && (yy < 178)) red_labels = red_labels + 1;
                                if((yy >= 191) && (yy < 198)) yellow_labels = yellow_labels + 1;
                                if((yy >= 211) && (yy < 218)) green_labels = green_labels + 1;
                            end
                            if((yy == 179) && (xx+lane == 195) && (sample == 24'hff2020))
                                red_borders = red_borders + 1;
                            if((yy == 199) && (xx+lane == 235) && (sample == 24'he0c000))
                                yellow_borders = yellow_borders + 1;
                            if((yy == 219) && (xx+lane == 275) && (sample == 24'h00e060))
                                green_borders = green_borders + 1;
                        end
                    end
                end
            end
            @(negedge clk);
            valid = 0; sof = 0; eol = 0;
            #1;
        end
    endtask

    task no_candidates;
        begin
            if(dut.red_acc_valid || dut.yellow_acc_valid || dut.green_acc_valid) begin
                $display("FAIL unexpected candidate on blank/logo/edge-only input");
                failures = failures + 1;
            end
        end
    endtask

    initial begin
        repeat(3) @(negedge clk);
        rst_n = 1;
        keys[1] = 0;
        repeat(10) @(negedge clk);
        keys[1] = 1;
        repeat(10) @(negedge clk);
        // Sweep the full R/G plane at representative B levels, including
        // high values that would expose eight-bit addition overflow.
        for(bb = 0; bb <= 255; bb = bb + 85)
            for(rr = 0; rr <= 255; rr = rr + 1)
                for(gg = 0; gg <= 255; gg = gg + 1) begin
                    sample = {rr[7:0],gg[7:0],bb[7:0]};
                    votes = dut.lamp_red(sample) + dut.lamp_yellow(sample) + dut.lamp_green(sample);
                    if(votes > 1) $fatal(1,"FAIL overlapping color votes for %h",sample);
                end
        if(!dut.lamp_red(24'hb05010) || !dut.lamp_yellow(24'hb0b010) ||
           !dut.lamp_green(24'h10b010) || dut.lamp_yellow(24'hfffff0) ||
           dut.lamp_green(24'hf0fff0) || dut.lamp_green(24'h506656) ||
           dut.lamp_green(24'ha0b8a4) || !dut.lamp_green(24'h2c402c))
            $fatal(1,"FAIL representative color gates");

        frame(0);
        no_candidates;
        if(dut.red_box_valid || dut.yellow_box_valid || dut.green_box_valid)
            $fatal(1,"FAIL startup boxes");
        frame(1);
        if(!dut.red_acc_valid || (dut.red_acc_xmin != 200) ||
           (dut.red_acc_xmax != 207) || (dut.red_acc_pixels != 64) ||
           !dut.yellow_acc_valid || (dut.yellow_acc_xmin != 240) ||
           (dut.yellow_acc_xmax != 247) || (dut.yellow_acc_pixels != 64) ||
           !dut.green_acc_valid || (dut.green_acc_xmin != 280) ||
           (dut.green_acc_xmax != 287) || (dut.green_acc_pixels != 64))
            $fatal(1,"FAIL simultaneous local candidates");

        // Green requires the candidate on two consecutive sensor frames.
        frame(1);
        if(dut.green_box_valid || !dut.red_box_valid || !dut.yellow_box_valid)
            $fatal(1,"FAIL green confirmation changed red/yellow or accepted one frame");
        red_labels = 0; yellow_labels = 0; green_labels = 0;
        red_borders = 0; yellow_borders = 0; green_borders = 0;
        inspect_overlay = 1;
        frame(0);
        inspect_overlay = 0;
        if(!dut.red_box_valid || !dut.yellow_box_valid || !dut.green_box_valid ||
           (red_labels == 0) || (yellow_labels == 0) || (green_labels == 0) ||
           (red_borders != 1) || (yellow_borders != 1) || (green_borders != 1))
            $fatal(1,"FAIL rendered boxes/labels R=%0d/%0d Y=%0d/%0d G=%0d/%0d",
                red_borders,red_labels,yellow_borders,yellow_labels,green_borders,green_labels);
        no_candidates;
        repeat(4) frame(0);
        if(dut.red_box_valid || dut.yellow_box_valid || dut.green_box_valid)
            $fatal(1,"FAIL stale boxes");

        frame(2);
        if(!dut.yellow_acc_valid || (dut.yellow_acc_pixels != 3) ||
           (dut.yellow_acc_xmin != 317) || (dut.yellow_acc_ymax != 239) ||
           dut.red_acc_valid || dut.green_acc_valid)
            $fatal(1,"FAIL three-pixel yellow or color isolation");
        frame(0);
        if(!dut.yellow_box_valid || (dut.yellow_box_xmin != 304) ||
           (dut.yellow_box_xmax != 319) || (dut.yellow_box_ymin != 224) ||
           (dut.yellow_box_ymax != 239))
            $fatal(1,"FAIL yellow corner expansion");
        frame(3);
        if(!dut.green_acc_valid || (dut.green_acc_pixels != 3) ||
           (dut.green_acc_xmin != 317) || (dut.green_acc_ymax != 239) ||
           dut.red_acc_valid || dut.yellow_acc_valid)
            $fatal(1,"FAIL three-pixel green or color isolation");
        frame(3);
        frame(0);
        if(!dut.green_box_valid || (dut.green_box_xmin != 304) ||
           (dut.green_box_xmax != 319) || (dut.green_box_ymin != 224) ||
           (dut.green_box_ymax != 239))
            $fatal(1,"FAIL green corner expansion");
        frame(4);
        no_candidates;
        frame(13);
        no_candidates;
        for(rr = 5; rr <= 12; rr = rr + 1) begin
            frame(rr);
            if(rr < 9) begin
                if(!dut.yellow_acc_valid || (dut.yellow_acc_pixels != 3) ||
                   (dut.yellow_acc_xmin != 240+rr-5) || dut.green_acc_valid || dut.red_acc_valid)
                    $fatal(1,"FAIL yellow lane %0d",rr-5);
            end else begin
                if(!dut.green_acc_valid || (dut.green_acc_pixels != 3) ||
                   (dut.green_acc_xmin != 240+rr-9) || dut.yellow_acc_valid || dut.red_acc_valid)
                    $fatal(1,"FAIL green lane %0d",rr-9);
            end
        end
        if(failures) $fatal(1,"FAIL %0d",failures);
        $display("PASS: disjoint masks, three local candidates, rendered color boxes and labels,");
        $display("      no edge-overlay feedback, corner/three-pixel targets, four lanes, ROI, hold expiry,");
        $display("      tinted white rejection, dim saturated green, green two-frame confirmation");
        $finish;
    end
endmodule
